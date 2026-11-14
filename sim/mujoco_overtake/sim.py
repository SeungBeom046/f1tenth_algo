"""추월 시나리오 시뮬 (Mac, ROS 없음). 판단 로직은 my_algo.aur을 그대로 쓴다.

scan마다(10 Hz): MuJoCo 레이캐스트 → pointcloud_to_laserscan 규칙의 scan → tracker / corridor / FSM / 횡 제어
→ ERPM·서보 명령 → 운동학 차량 모델(구동 지연 t_r, 1차 지연, 가감속 한계).
AEB(autonomous_drive)는 이 시뮬에 없다: 충돌은 기하로 판정하고 N_AEB는 나오지 않는다.
"""
import math
import os
import sys

import mujoco
import numpy as np
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, REPO)
sys.path.insert(0, HERE)

from my_algo.aur.corridor import Corridor                      # noqa: E402
from my_algo.aur.ego_fsm import EgoFSM, Obs                    # noqa: E402
from my_algo.aur.lateral import Lateral                        # noqa: E402
from my_algo.aur.leader_logic import Leader                    # noqa: E402
from my_algo.aur.params import load_params                     # noqa: E402
from my_algo.aur.tracker import VehicleTracker, scan_to_points  # noqa: E402
from my_algo.vehicle_utils import sanitize_rpm, steering_to_servo  # noqa: E402

from lidar import Lidar, cloud_to_scan                         # noqa: E402
from vehicle import Kinematic                                  # noqa: E402
from world import build_xml                                    # noqa: E402

COMMON_YAML = os.path.join(REPO, 'config', 'exp_common.yaml')


def load_cfg(path=None, **overrides):
    """scenario.yaml을 읽고 'track.W=1.3' 같은 점 표기 덮어쓰기를 적용한다."""
    with open(path or os.path.join(HERE, 'scenario.yaml'), encoding='utf-8') as f:
        cfg = yaml.safe_load(f)
    for key, value in overrides.items():
        sec, name = key.split('.')
        if name not in cfg[sec]:
            raise KeyError(key)
        cfg[sec][name] = value
    return cfg


def corners(x, y, yaw, cfg):
    v = cfg['vehicle']
    c, s = math.cos(yaw), math.sin(yaw)
    hw = cfg['w_car'] / 2.0
    return [(x + c * px - s * py, y + s * px + c * py)
            for px, py in ((-v['rear_overhang'], -hw), (v['length'] - v['rear_overhang'], -hw),
                           (v['length'] - v['rear_overhang'], hw), (-v['rear_overhang'], hw))]


def rects_overlap(a, b):
    """볼록 사각형 2개의 겹침 (분리축)."""
    for poly in (a, b):
        for i in range(4):
            nx, ny = poly[i][1] - poly[i - 1][1], poly[i - 1][0] - poly[i][0]
            pa = [nx * px + ny * py for px, py in a]
            pb = [nx * px + ny * py for px, py in b]
            if max(pa) < min(pb) or max(pb) < min(pa):
                return False
    return True


class Car:
    def __init__(self, sim, name, x_rear_axle, view):
        cfg, p, v = sim.cfg, sim.p, sim.cfg['vehicle']
        self.name = name
        self.kin = Kinematic(x_rear_axle, 0.0, 0.0, v['wheelbase'], v['t_r'], v['tau'], v['a_max'], v['a_b'],
                             v['steer_rate'], erpm_gain=p.erpm_gain, max_steer=p.max_steering)
        self.body = mujoco.mj_name2id(sim.m, mujoco.mjtObj.mjOBJ_BODY, name)
        mount = {'lidar_xyz': cfg['lidar']['mount_xyz'], 'lidar_rpy': [0.0, 0.0, 0.0]}
        self.lidar = Lidar(sim.m, cfg['lidar'], mount, self.body, seed=sim.seed + (1 if name == 'leader' else 0))
        self.tracker = VehicleTracker(p, view=view)
        self.shadow = VehicleTracker(p, view=view)   # 기록용: FSM 상태와 무관하게 매 scan 정면 규칙만 돌린다
        self.front = None
        self.corridor = Corridor(p)
        self.lateral = Lateral(p)
        self.track = None
        self.points = []
        self.points_world = np.empty((0, 3))
        self.v_cmd = 0.0
        self.y_ref = 0.0
        self.scripted = None        # (y, yaw): 선행차 yaw 고정 양보 중일 때의 자세

    def perceive(self, sim, side):
        cloud, self.points_world, _ = self.lidar.scan(sim.d, self.kin.pose())
        scan = cloud_to_scan(cloud, sim.cfg['scan'])
        self.points = scan_to_points(scan['ranges'].tolist(), scan['angle_min'], scan['angle_increment'],
                                     scan['range_min'], scan['range_max'])
        cor, p = self.corridor, sim.p
        if cor.W_est is None:
            cor.update(self.points)
        W = p.W if p.W is not None else cor.W_est
        args = (self.points, sim.t, self.kin.v, cor.ego_lateral, W)
        self.track = self.tracker.update_side(*args) if side else self.tracker.update(*args)
        self.front = self.shadow.update(*args)
        cor.update(self.points, self.track.idx)
        return W

    def drive(self, sim, v_cmd, y_ref):
        self.v_cmd, self.y_ref = v_cmd, y_ref
        steer = self.lateral.step(self.corridor.ego_lateral, y_ref, sim.frame_dt, self.corridor.heading)
        self.kin.command(sim.t, sanitize_rpm(v_cmd * sim.p.erpm_gain), steering_to_servo(steer))


class Sim:
    def __init__(self, method='d', cfg=None, seed=0, run_id=None, reuse=None, **param_overrides):
        """reuse = (MjModel, MjData): 뷰어 재시작 때 창이 붙어 있는 모델·데이터를 다시 쓴다 (같은 설정이어야 함)."""
        self.cfg = cfg or load_cfg()
        self.p = load_params(COMMON_YAML, **dict(self.cfg.get('params') or {}, **param_overrides))
        self.cfg['w_car'] = self.p.w_car
        self.seed = seed
        self.method = method
        if reuse is None:
            self.m = mujoco.MjModel.from_xml_string(build_xml(self.cfg))
            self.d = mujoco.MjData(self.m)
        else:
            self.m, self.d = reuse
            mujoco.mj_resetData(self.m, self.d)
        v, lead = self.cfg['vehicle'], self.cfg['leader']
        self.front = v['length'] - v['rear_overhang']
        self.ego = Car(self, 'ego', -self.front, 'front')
        self.leader = Car(self, 'leader', self.cfg['start']['gap0'] + v['rear_overhang'], 'rear')
        self.fsm = EgoFSM(self.p, method)
        behavior = lead['behavior']
        if behavior not in ('Y', 'NY', 'YR'):
            raise ValueError('leader.behavior는 Y|NY|YR: %r' % (behavior,))
        self.logic = Leader(self.p, 'NY' if behavior == 'NY' else 'Y', run_id if run_id is not None else seed,
                            return_after=lead['return_after'] if behavior == 'YR' else None)
        if lead['d_y'] is not None:
            self.logic.d_y = float(lead['d_y'])
        self.t = 0.0
        self.frame_dt = 1.0 / self.cfg['lidar']['hz']
        self.collision = None       # 'cars' | 'ego_wall' | 'leader_wall'
        self.log = []
        self.place_bodies()

    # ------------------------------------------------------------ 참값
    def true_gap(self):
        """ego 앞범퍼 → 선행차 뒷면 (x 방향)."""
        return (self.leader.kin.x - self.cfg['vehicle']['rear_overhang']) - (self.ego.kin.x + self.front)

    def place_bodies(self):
        for car in (self.ego, self.leader):
            x, y, yaw = car.kin.pose()
            i = self.m.body_mocapid[car.body]
            self.d.mocap_pos[i] = [x, y, 0.0]
            self.d.mocap_quat[i] = [math.cos(yaw / 2), 0, 0, math.sin(yaw / 2)]
        mujoco.mj_forward(self.m, self.d)

    def check_collision(self):
        e = corners(*self.ego.kin.pose(), self.cfg)
        lead = corners(*self.leader.kin.pose(), self.cfg)
        half = self.cfg['track']['W'] / 2.0
        if rects_overlap(e, lead):
            return 'cars'
        if any(abs(y) > half for _, y in e):
            return 'ego_wall'
        # yaw 고정 양보는 차체를 뒤축 기준으로 돌려 두기만 하므로 벽 쪽 모서리가 벽을 넘을 수 있다 (판정 제외)
        if self.leader.scripted is None and any(abs(y) > half for _, y in lead):
            return 'leader_wall'
        return None

    # ------------------------------------------------------------ scan마다
    def frame(self):
        cfg, p, ego, lead = self.cfg, self.p, self.ego, self.leader
        W = ego.perceive(self, side=self.fsm.state in ('PASS', 'MERGE'))
        trk = ego.track
        v_cmd, y_ref, state, reason = self.fsm.step(Obs(
            self.t, ego.kin.v, trk.valid, trk.d, trk.y_b, trk.v_b, ego.corridor.ego_lateral, W, trk.front_rel))
        if self.t < cfg['start']['ego_delay']:
            v_cmd = 0.0
        ego.drive(self, v_cmd, y_ref)

        Wl = lead.perceive(self, side=False)
        d_rear = lead.track.d if lead.track.valid else None
        if not lead.track.valid:    # 후행차가 옆으로 오면 이어서 추적해 그 점을 벽 계산에서 뺀다
            side = lead.tracker.update_side(lead.points, self.t, lead.kin.v, lead.corridor.ego_lateral, Wl)
            lead.corridor.update(lead.points, side.idx)
        lv, ly_ref = self.logic.step(self.t, lead.kin.v, d_rear, Wl)
        if self.t < cfg['start']['leader_delay']:
            lv = 0.0
        yaw_deg = cfg['leader']['yield_yaw_deg']
        if yaw_deg is None:
            lead.drive(self, lv, ly_ref)
        else:                       # yaw 고정 양보: 횡위치를 일정 속도로 옮기고 그동안 차체를 yaw만큼 돌려 둔다
            y = lead.scripted[0] if lead.scripted else 0.0
            step = max(-1.0, min(1.0, (ly_ref - y) / (cfg['leader']['yield_lat_speed'] * self.frame_dt)))
            y += step * cfg['leader']['yield_lat_speed'] * self.frame_dt
            lead.scripted = (y, math.radians(yaw_deg) * (1 if step > 0 else -1) if abs(step) > 1e-9 else 0.0)
            lead.v_cmd, lead.y_ref = lv, ly_ref
            lead.kin.command(self.t, sanitize_rpm(lv * p.erpm_gain), steering_to_servo(0.0))

        self.log.append(dict(
            t=self.t, state=state, reason=reason, gap=self.true_gap(), d=trk.d, valid=trk.valid,
            width=trk.width, y_b=trk.y_b, y_b_true=lead.kin.y, v_b=trk.v_b, v_b_true=lead.kin.v,
            v_e=ego.kin.v, v_cmd=v_cmd, y_e=ego.kin.y, y_e_est=ego.corridor.ego_lateral, yaw_e=ego.kin.yaw,
            yaw_l=lead.kin.yaw, y_ref=y_ref, h=self.fsm.h, b=self.fsm.belief.prob_yield(),
            lam=self.fsm.belief.lam, cor_valid=ego.corridor.valid,
            front_valid=ego.front.valid, front_width=ego.front.width,
            yield_triggered=self.logic.yield_triggered, brake_fired=self.logic.brake_fired))

    def step(self):
        """시뮬 한 스텝 (dt). scan 주기마다 frame()을 먼저 부른다."""
        dt = self.cfg['sim']['dt']
        n = int(round(self.t / dt))
        if n % int(round(self.frame_dt / dt)) == 0:
            self.frame()
        self.ego.kin.step(self.t, dt)
        lead = self.leader
        if lead.scripted is not None:
            lead.kin.yaw = 0.0
            lead.kin.step(self.t, dt)
            lead.kin.y, lead.kin.yaw = lead.scripted
        else:
            lead.kin.step(self.t, dt)
        self.t = round((n + 1) * dt, 9)
        self.place_bodies()
        if self.collision is None:
            self.collision = self.check_collision()

    def finished(self):
        s = self.cfg['sim']
        if self.collision or self.t >= s['max_time']:
            return True
        if self.ego.kin.x > self.cfg['track']['x_end'] - 3.0 or self.leader.kin.x > self.cfg['track']['x_end'] - 3.0:
            return True
        if self.fsm.state == 'DONE':
            self._t_done = getattr(self, '_t_done', self.t)
            return self.t - self._t_done >= s['end_after_done']
        return False

    def run(self, hook=None):
        while not self.finished():
            self.step()
            if hook is not None and hook(self) is False:
                break
        return self.result()

    # ------------------------------------------------------------ 결과
    def result(self):
        log, fsm = self.log, self.fsm
        # 추적 유지: 선행차가 앞쪽 검출 범위 안(참값)에 있는 scan 중 정면 규칙이 차를 찾은 비율.
        # PASS 초반(아직 뒤에 있을 때)도 포함한다: 선행차가 틀면서 비키는 구간이 대부분 여기다.
        need = [r for r in log if r['state'] in ('APPROACH', 'PROBE', 'PASS')
                and 0.05 < r['gap'] < self.p.track_x_max - 0.2]
        lost = [r for r in need if not r['front_valid']]
        longest, run_len = 0, 0
        for r in need:
            run_len = 0 if r['front_valid'] else run_len + 1
            longest = max(longest, run_len)
        success = fsm.state == 'DONE' and not self.collision
        if success:
            cause = ''
        elif self.collision:
            cause = 'collision_' + self.collision
        elif fsm.follow_reason == 'belief_NY':
            cause = 'belief_NY'
        elif fsm.follow_reason == 'T_max':
            cause = 'T_wait_a' if self.method == 'a' else 'T_max'
        elif longest * self.frame_dt > self.p.track_hold:
            cause = 'track_lost'
        else:
            cause = 'other'
        post = [r for r in log if fsm.t0 is not None and r['t'] >= fsm.t0]
        widths = [r['front_width'] for r in need if r['front_valid']]
        return dict(
            method=self.method, behavior=self.cfg['leader']['behavior'], belief=self.p.belief_model,
            seed=self.seed, W=self.cfg['track']['W'], d_y=round(self.logic.d_y, 4),
            success=success, final_state=fsm.state, cause=cause, collision=self.collision or '',
            decided=fsm.decided or '', T_id=None if fsm.t_decided is None else round(fsm.t_decided - fsm.t0, 2),
            pass_entered=any(r['state'] == 'PASS' for r in log),
            T_pass=round(getattr(self, '_t_done', float('nan')) - fsm.t0, 2) if success else None,
            h_min=round(min((r['h'] for r in post if r['state'] in ('APPROACH', 'PROBE', 'FOLLOW')), default=float('nan')), 3),
            gap_min=round(min((r['gap'] for r in post if r['state'] in ('APPROACH', 'PROBE', 'FOLLOW')), default=float('nan')), 3),
            hold_count=fsm.hold_count, track_need=len(need), track_lost=len(lost),
            track_lost_longest_s=round(longest * self.frame_dt, 2),
            width_max=round(max(widths), 3) if widths else None,
            leader_yaw_max_deg=round(math.degrees(max((abs(r['yaw_l']) for r in log), default=0.0)), 1),
            ego_yaw_max_deg=round(math.degrees(max((abs(r['yaw_e']) for r in log), default=0.0)), 1),
            t_end=round(self.t, 2))
