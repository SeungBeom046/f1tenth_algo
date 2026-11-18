"""추월 시나리오 시뮬 (Mac, ROS 없음). 판단 로직은 my_algo.aur을 그대로 쓴다.

실차 노드 구성과 같은 순서로 돈다 (docs/PIPELINE.md):
    scan마다(10 Hz, Sim.frame):
        Car.perceive   MuJoCo 레이캐스트 → pointcloud_to_laserscan 규칙의 scan → corridor + tracker   (실차: exp_planner)
        _ego_frame     EgoFSM.step → Car.drive(횡 제어 Lateral)                                       (실차: exp_planner)
        _leader_frame  Leader.step → Car.drive                                                           (실차: exp_planner, role=leader)
    50 Hz (Car.control): AEBSafetyModel → sanitize_rpm → ERPM·서보 명령                                 (실차: autonomous_drive)
    dt마다: 운동학 차량 모델(구동 지연 t_r, 1차 지연, 가감속 한계), 충돌 판정(차체 사각형)
N_AEB는 /aeb/active 상승 에지 수와 같은 방식으로 센다. AEB 모델·폭·TTC는 scenario.yaml의 aeb.
"""
from types import SimpleNamespace
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

from my_algo.baseline.aeb import AEBSafetyModel                         # noqa: E402
from my_algo.aur.corridor import Corridor                      # noqa: E402
from my_algo.aur.ego_fsm import EgoFSM, Obs                    # noqa: E402
from my_algo.aur.lateral import Lateral                        # noqa: E402
from my_algo.aur.leader_logic import Leader                    # noqa: E402
from my_algo.aur.params import load_params                     # noqa: E402
from my_algo.aur.tracker import VehicleTracker, scan_to_points  # noqa: E402
from my_algo.baseline.vehicle_utils import (ReactiveDriveCommand, SERVO_CENTER, SERVO_GAIN,  # noqa: E402
                                           sanitize_rpm, steering_to_servo)

from lidar import Lidar, cloud_to_scan                         # noqa: E402
from vehicle import Kinematic                                  # noqa: E402
from world import build_xml                                    # noqa: E402

COMMON_YAML = os.path.join(REPO, 'config', 'exp_common.yaml')
AEB_MOVING_SPEED = 0.1      # [m/s] 이 속도 이하에서 걸린 AEB는 N_AEB_moving에 넣지 않는다 (정지 중 잡음 깜빡임)


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


def rmin(values):
    """결과 표용 최솟값 (없으면 None: NaN은 같은 결과끼리도 ==가 거짓이라 결정성 비교가 깨진다)."""
    values = list(values)
    return round(min(values), 3) if values else None


def corners(x, y, yaw, cfg):
    """차체 사각형 네 모서리 (월드). (x, y) = 뒤축 중심."""
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
    """차 한 대: 운동학 모델 + 라이다 + 인지(tracker·corridor) + 횡 제어 + AEB. view = 'front'(ego) | 'rear'(선행차)."""

    def __init__(self, sim, name, x_rear_axle, view):
        cfg, p, v = sim.cfg, sim.p, sim.cfg['vehicle']
        self.name = name
        # steer_offset: 서보 중심이 어긋나 명령 0 rad에도 바퀴가 이만큼 꺾인 차 (실차 트림 오차 모사)
        self.kin = Kinematic(x_rear_axle, 0.0, 0.0, v['wheelbase'], v['t_r'], v['tau'], v['a_max'], v['a_b'],
                             v['steer_rate'], erpm_gain=p.erpm_gain, max_steer=p.max_steering,
                             servo_center=SERVO_CENTER + v.get('steer_offset', 0.0) * SERVO_GAIN)
        self.body = mujoco.mj_name2id(sim.m, mujoco.mjtObj.mjOBJ_BODY, name)
        mount = {'lidar_xyz': cfg['lidar']['mount_xyz'], 'lidar_rpy': cfg['lidar'].get('mount_rpy', [0.0, 0.0, 0.0])}
        self.lidar = Lidar(sim.m, cfg['lidar'], mount, self.body, seed=sim.seed + (1 if name == 'leader' else 0))
        self.tracker = VehicleTracker(p, view=view)
        self.shadow = VehicleTracker(p, view=view)   # 기록용: FSM 상태와 무관하게 매 scan 정면 규칙만 돌린다
        self.front_track = None     # shadow 결과 (추적 유지율·클러스터 폭 기록)
        self.corridor = Corridor(p)
        self.lateral = Lateral(p)
        self.track = None
        self.points = []
        self.points_world = np.empty((0, 3))
        self.v_cmd = 0.0
        self.y_ref = 0.0
        self.scripted = None        # (y, yaw): 선행차 yaw 고정 양보 중일 때의 자세
        a = cfg['aeb']      # 나머지 기본값 = launch/lidar_drive.launch.py 값
        aeb_type, extra = AEBSafetyModel, {}
        if p.experiment_safety:
            from my_algo.aur.emergency import ExperimentAEB
            aeb_type, extra = ExperimentAEB, {'latch_release_sec': a.get('latch_release_sec', 0.0)}
        self.aeb = aeb_type(**extra, path_margin=a['path_margin'], bumper_emergency_distance=a['bumper_emergency_distance'],
                                  vehicle_half_width=a.get('vehicle_half_width', 0.15),
                                  lidar_to_bumper_dist=p.lidar_to_front,
                                  model=a['model'], ttc_threshold=a['ttc_threshold'], curved_band=a['curved_band'],
                                  curve_max_steering=a.get('curve_max_steering', 0.0),
                                  band_intersect=a.get('band_intersect', False),
                                  wheelbase=v['wheelbase'], lidar_x_from_rear_axle=cfg['lidar']['mount_xyz'][0])
        self.W_fixed = p.W_leader if name == 'leader' else None
        self.scan_ns = None         # AEB 입력 (LaserScan 필드)
        self.cmd = (0.0, 0.0)       # planner가 낸 (rpm, 조향) = /reactive/lidar_command
        self.aeb_active = False
        self.n_aeb = 0
        self.n_aeb_moving = 0

    def perceive(self, sim, side):
        """scan 한 프레임: 레이캐스트 → scan → corridor(벽) + tracker(상대 차). side = 측면 추적(PASS·MERGE). 반환: W."""
        cloud, self.points_world, _ = self.lidar.scan(sim.d, self.kin.pose())
        if sim.cfg['scan'].get('level', False):
            # 실차 launch level_scan과 같다: 라이다 장착 기울기를 되돌린 수평 프레임(livox_level)에서 높이 띠를 자른다
            cloud = cloud @ self.lidar.R_mount.T
        scan_params = sim.cfg['scan']
        if sim.p.experiment_safety:
            scan_params = dict(scan_params, min_height=-0.20)
        scan = cloud_to_scan(cloud, scan_params)
        self.scan_ns = SimpleNamespace(**dict(scan, ranges=scan['ranges'].tolist()))
        self.points = scan_to_points(scan['ranges'].tolist(), scan['angle_min'], scan['angle_increment'],
                                     scan['range_min'], scan['range_max'])
        cor, p = self.corridor, sim.p
        fresh_wall = self.name == 'ego' and p.track_recovery and p.experiment_safety
        if cor.W_est is None and not (fresh_wall and p.guardrail_local_view):
            cor.update(self.points, W_fixed=self.W_fixed, stamp=sim.t)
        W = self.W_fixed or (p.W if p.W is not None else cor.W_est)
        tracking_points = self.points
        if fresh_wall:
            cor.update(self.points, stamp=sim.t)
            tracking_points = [q for q in self.points if q.i not in cor.wall_idx]
            if p.guardrail_local_view and not cor.valid:
                tracking_points = []
        args = (tracking_points, sim.t, self.kin.v, cor.ego_lateral, W, cor.heading)
        self.track = self.tracker.update_side(*args) if side else self.tracker.update(*args)
        self.front_track = self.shadow.update(*args)
        if not (fresh_wall and p.guardrail_local_view) or cor.sector_idx.intersection(self.track.idx):
            cor.update(self.points, self.track.idx, W_fixed=self.W_fixed, stamp=sim.t)
        return W

    def drive(self, sim, v_cmd, y_ref):
        """종방향 명령 v_cmd와 횡 목표 y_ref → (rpm, 조향) = /reactive/lidar_command."""
        self.v_cmd, self.y_ref = v_cmd, y_ref
        if (sim.p.corridor_guardrail and not self.corridor.valid) or self.tracker.pending_d is not None:
            self.v_cmd = v_cmd = 0.0
            steer = self.lateral.steer
        else:
            steer = self.lateral.step(self.corridor.ego_lateral, y_ref, sim.frame_dt, self.corridor.heading,
                                      integrate=not sim.p.guardrail_local_view or self.kin.v > sim.p.t0_speed_threshold)
        self.cmd = (v_cmd * sim.p.erpm_gain, steer)

    def control(self, sim):
        """autonomous_drive.control_timer_callback와 같은 순서: AEB → 0 RPM(조향 유지) 또는 sanitize_rpm."""
        rpm, steer = self.cmd
        a = sim.cfg['aeb']
        emergency = False
        if a['enabled'] and self.scan_ns is not None:
            speed = {'measured': self.kin.v, 'zero': 0.0, 'unknown': None}[a['speed_source']]
            emergency = self.aeb.check_emergency_braking(self.scan_ns, speed, ReactiveDriveCommand(steer, rpm))
        if emergency and not self.aeb_active:
            self.n_aeb += 1
            # 서 있는 동안 잡음으로 AEB 경계를 오가는 것은 제동이 아니므로 따로 센다
            moving = self.kin.v > AEB_MOVING_SPEED
            self.n_aeb_moving += moving
            sim.event(self.name, 'AEB 작동 (%s)%s' % (self.aeb.last_reason, '' if moving else ' [정지 중]'))
        self.aeb_active = emergency
        self.kin.command(sim.t, 0.0 if emergency else sanitize_rpm(rpm), steering_to_servo(steer))


class Sim:
    """한 회차. run()이 끝나면 result()가 회차 지표 dict를 낸다 (batch.py·baseline.py가 CSV로 저장)."""

    def __init__(self, method='d', cfg=None, seed=0, run_id=None, reuse=None, **param_overrides):
        """reuse = (MjModel, MjData): 뷰어 재시작 때 창이 붙어 있는 모델·데이터를 다시 쓴다 (같은 설정이어야 함)."""
        self.cfg = cfg or load_cfg()
        extra = {'W_leader': float(self.cfg['track']['W'])} if self.cfg['leader'].get('fix_W') else {}
        self.p = load_params(COMMON_YAML, **dict(self.cfg.get('params') or {}, **extra, **param_overrides))
        if self.p.corridor_guardrail and self.p.W is None:
            self.p.W = float(self.cfg['track']['W'])
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
        self.front_overhang = v['length'] - v['rear_overhang']     # 뒤축 → 앞 끝
        self.ego = Car(self, 'ego', -self.front_overhang, 'front')
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
        self.on_event = None        # 함수(str)를 넣으면 상태 전이·양보·AEB를 한 줄씩 받는다 (run.py가 터미널에 출력)
        self.events = []
        self._lead_yielding = False
        self._aborts = 0
        self.place_bodies()

    def event(self, who, msg):
        line = '[%6.2f s] %-6s %s' % (self.t, who, msg)
        self.events.append(line)
        if self.on_event:
            self.on_event(line)

    # ------------------------------------------------------------ 참값
    def true_gap(self):
        """ego 앞범퍼 → 선행차 뒷면 (x 방향)."""
        return (self.leader.kin.x - self.cfg['vehicle']['rear_overhang']) - (self.ego.kin.x + self.front_overhang)

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
        """scan 한 프레임: ego 판단 → 선행차 판단 → 기록 (순서를 바꾸면 결과가 달라진다)."""
        v_cmd, y_ref, state, reason = self._ego_frame()
        self._leader_frame()
        self._log_frame(v_cmd, y_ref, state, reason)

    def _ego_frame(self):
        ego = self.ego
        W = ego.perceive(self, side=self.fsm.state in ('PASS', 'MERGE', 'RETREAT'))
        trk = ego.track
        if self.p.corridor_guardrail and not ego.corridor.valid:
            ego.drive(self, 0.0, ego.lateral.y_ref)
            reason = 'corridor_initializing' if ego.corridor.initializing else 'corridor_invalid'
            return 0.0, ego.lateral.y_ref, self.fsm.state, reason
        if ego.tracker.pending_d is not None:
            ego.drive(self, 0.0, ego.lateral.y_ref)
            return 0.0, ego.lateral.y_ref, self.fsm.state, 'front_track_pending'
        v_cmd, y_ref, state, reason = self.fsm.step(Obs(
            self.t, ego.kin.v, trk.valid, trk.d, trk.y_b, trk.v_b, ego.corridor.ego_lateral, W, trk.front_rel, trk.y_l))
        if self.t < self.cfg['start']['ego_delay']:
            v_cmd = 0.0
        ego.drive(self, v_cmd, y_ref)
        return v_cmd, y_ref, state, reason

    def _leader_frame(self):
        cfg, lead = self.cfg, self.leader
        Wl = lead.perceive(self, side=False)
        d_rear = lead.track.d if lead.track.valid else None
        if not lead.track.valid:    # 후행차가 옆으로 오면 이어서 추적해 그 점을 벽 계산에서 뺀다
            side = lead.tracker.update_side(lead.points, self.t, lead.kin.v, lead.corridor.ego_lateral, Wl,
                                            lead.corridor.heading)
            lead.corridor.update(lead.points, side.idx, W_fixed=lead.W_fixed, stamp=self.t)
        if self.p.corridor_guardrail and not lead.corridor.valid:
            lead.drive(self, 0.0, lead.lateral.y_ref)
            return
        lv, ly_ref = self.logic.step(self.t, lead.kin.v, d_rear, Wl)
        if self.t < cfg['start']['leader_delay']:
            lv = 0.0
        if cfg['leader']['yield_yaw_deg'] is None:
            lead.drive(self, lv, ly_ref)
        else:
            self._scripted_yield(lv, ly_ref)

    def _scripted_yield(self, lv, ly_ref):
        """yaw 고정 양보 (tracker 강건성 테스트용): 횡위치를 일정 속도로 옮기고 그동안 차체를 yaw만큼 돌려 둔다."""
        cfg, lead = self.cfg, self.leader
        yaw_deg = cfg['leader']['yield_yaw_deg']
        y = lead.scripted[0] if lead.scripted else 0.0
        step = max(-1.0, min(1.0, (ly_ref - y) / (cfg['leader']['yield_lat_speed'] * self.frame_dt)))
        y += step * cfg['leader']['yield_lat_speed'] * self.frame_dt
        # 목표에 닿으면(남은 거리 1 mm 미만) yaw 0. 예전에는 부동소수 잔차로 ±yaw가 프레임마다 뒤집혔다
        moving = abs(ly_ref - y) > 1e-3 or abs(step) >= 1.0
        lead.scripted = (y, math.copysign(math.radians(yaw_deg), step) if moving else 0.0)
        lead.v_cmd, lead.y_ref = lv, ly_ref
        lead.cmd = (lv * self.p.erpm_gain, 0.0)

    def _frame_events(self, state, reason):
        """이전 scan과 비교해 바뀐 것만 event로 낸다 (판단 로직이 낸 행동인지 터미널에서 보이게)."""
        prev = self.log[-1] if self.log else None
        fsm, logic = self.fsm, self.logic
        if fsm.abort_count != self._aborts:     # 중단과 RETREAT 끝이 한 scan에 겹치면 상태 줄에는 retreat_done만 보인다
            self._aborts = fsm.abort_count
            self.event('ego', '추월 중단 (선행차가 다시 들어오거나 왼쪽 여유 부족)')
        if prev is None or prev['state'] != state:
            self.event('ego', '%s → %s (%s)' % (prev['state'] if prev else '-', state, reason))
        if fsm.decided and fsm.t_decided == self.t:
            self.event('ego', 'belief 확정 %s (b=%.2f)' % (fsm.decided, fsm.belief.prob_yield()))
        yielding = self.leader.y_ref != 0.0
        if yielding != self._lead_yielding:
            self.event('leader', '양보 시작: 오른쪽 차선 y_ref=%.2f (후행차 거리 ≤ d_y=%.2f)' % (self.leader.y_ref, logic.d_y)
                       if yielding else '양보 끝: 중앙 복귀')
            self._lead_yielding = yielding
        if logic.brake_fired and not (prev and prev['brake_fired']):
            self.event('leader', '급정지 (NY)')

    def _log_frame(self, v_cmd, y_ref, state, reason):
        self._frame_events(state, reason)
        ego, lead, trk = self.ego, self.leader, self.ego.track
        self.log.append(dict(
            t=self.t, state=state, reason=reason, gap=self.true_gap(), d=trk.d, valid=trk.valid,
            width=trk.width, y_b=trk.y_b, y_b_true=lead.kin.y, v_b=trk.v_b, v_b_true=lead.kin.v,
            v_e=ego.kin.v, v_cmd=v_cmd, y_e=ego.kin.y, y_e_est=ego.corridor.ego_lateral, yaw_e=ego.kin.yaw,
            yaw_l=lead.kin.yaw, y_ref=y_ref, h=self.fsm.h, b=self.fsm.belief.prob_yield(),
            lam=self.fsm.belief.lam, cor_valid=ego.corridor.valid,
            front_valid=ego.front_track.valid, front_width=ego.front_track.width,
            yield_triggered=self.logic.yield_triggered, brake_fired=self.logic.brake_fired,
            aeb=ego.aeb_active, n_aeb=ego.n_aeb))

    def step(self):
        """시뮬 한 스텝 (dt): scan 주기면 frame(), AEB 주기면 control(), 그다음 차량 모델 적분과 충돌 판정."""
        dt = self.cfg['sim']['dt']
        n = int(round(self.t / dt))
        if n % int(round(self.frame_dt / dt)) == 0:
            self.frame()
        if n % max(1, int(round(1.0 / (self.cfg['aeb']['hz'] * dt)))) == 0:
            self.ego.control(self)
            self.leader.control(self)
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
    def _return_after_overlap(self):
        lg = self.logic
        if lg.return_after is None or lg.t_yield is None:
            return False
        t_ret = lg.t_yield + lg.return_after
        t_ov = next((r['t'] for r in self.log if r['gap'] < 0.0), None)
        return t_ov is not None and t_ov <= t_ret <= self.t

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
            # 안전 판정 (기획안 변경): 급정지 전 h_min ≥ 0, 급정지 후 정지 간격 ≥ m
            h_min=rmin(r['h'] for r in post if r['state'] in ('APPROACH', 'PROBE', 'FOLLOW') and not r['brake_fired']),
            stop_gap=rmin(r['gap'] for r in post if r['brake_fired']),
            stop_gap_ok=all(r['gap'] >= self.p.m_stop for r in post if r['brake_fired']),
            brake_fired=self.logic.brake_fired,
            deadlock=(not self.collision and fsm.state in ('APPROACH', 'PROBE', 'PASS', 'MERGE')
                      and self.t >= self.cfg['sim']['max_time'] and self.ego.kin.v < 0.05),
            # YR: 후행차가 이미 옆에 온 뒤(앞뒤 겹침 후) 복귀했는가 = 시뮬 전용 시나리오
            return_after_overlap=self._return_after_overlap(),
            gap_min=rmin(r['gap'] for r in post if r['state'] in ('APPROACH', 'PROBE', 'FOLLOW')),
            hold_count=fsm.hold_count, N_AEB=self.ego.n_aeb, N_AEB_leader=self.leader.n_aeb,
            N_AEB_moving=self.ego.n_aeb_moving, N_AEB_leader_moving=self.leader.n_aeb_moving,
            aeb_speed_source=self.cfg['aeb']['speed_source'] if self.cfg['aeb']['enabled'] else 'off',
            n_abort=fsm.abort_count, track_need=len(need), track_lost=len(lost),
            track_lost_longest_s=round(longest * self.frame_dt, 2),
            width_max=round(max(widths), 3) if widths else None,
            leader_yaw_max_deg=round(math.degrees(max((abs(r['yaw_l']) for r in log), default=0.0)), 1),
            ego_yaw_max_deg=round(math.degrees(max((abs(r['yaw_e']) for r in log), default=0.0)), 1),
            t_end=round(self.t, 2))
