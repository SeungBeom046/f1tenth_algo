#!/usr/bin/env python3
"""MuJoCo 시뮬 회차 실행. 실차 exp 노드 클래스를 가짜 rclpy(sim/rosstub.py) 위에서 그대로 돌린다.

    sim/.venv/bin/python sim/runner.py --method P --scenario S1 --d-e 1.5 --seed 0          # 배치 (실시간보다 빠름)
    sim/.venv/bin/mjpython sim/runner.py --method P --scenario S3 --viewer                   # 대화형 뷰어 (macOS는 mjpython)
    sim/.venv/bin/python sim/runner.py --method P --scenario S3 --w-min 0.30 --video         # [EXP-TUNE] + 위에서 본 영상

노드 (실차와 같은 클래스·파라미터 파일): occupancy_grid_node, phantom_node, speed_layer_node, exp_aeb_node,
exp_steering_node(기본, --steering simple이면 시뮬 전용 단순 조향), exp_drive_mux, trigger_node, (S1) popout_vehicle_node.
시뮬이 만드는 것: 라이다 점군(/livox/lidar), 2D scan(/scan), odom(/vesc/odom, #2는 odom), /autonomous_mode, 시간.
시뮬 전용 차이: 조이스틱 데드맨·발행자 점검 끔 (sim.yaml mux_overrides), odom은 참값, 차량은 운동학 모델.
"""
import argparse
import datetime
import hashlib
import json
import math
import os
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path[:0] = [HERE, ROOT]

import rosstub  # noqa: E402

rn = rosstub.install()

import mujoco  # noqa: E402
import numpy as np  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from sensor_msgs.msg import LaserScan, PointCloud2, PointField  # noqa: E402
from std_msgs.msg import Bool  # noqa: E402

import lidar as lidar_mod  # noqa: E402
import simconfig  # noqa: E402
import vehicle  # noqa: E402
import world as world_mod  # noqa: E402
from my_algo.geometry import load_geometry, polygon_distance  # noqa: E402
from my_algo.track_layout import check as check_layout  # noqa: E402

TAP_TOPICS = ('/exp/final_cmd', '/exp/v_cmd', '/exp/v_raw', '/exp/v_cap', '/exp/phantom_d', '/exp/rho',
              '/aeb/active', '/exp/stop_reason', '/exp/min_speed_flag', '/exp/steering_command')
EVENT_TOPICS = ('/exp/aeb_event', '/exp/abort', '/trigger', '/popout/status', '/trigger/info')


def stamp_of(t):
    from builtin_interfaces.msg import Time
    s = Time()
    s.sec, s.nanosec = int(t), int(round((t - int(t)) * 1e9)) % 1_000_000_000
    return s


def mat(x, y, yaw, z=0.0):
    c, s = math.cos(yaw), math.sin(yaw)
    m = np.eye(4)
    m[:2, :2] = [[c, -s], [s, c]]
    m[:3, 3] = [x, y, z]
    return m


class SimpleSteering:
    """시뮬 전용 단순 조향 (옵션): 좌측 벽 목표 거리 유지 P 제어. 기본은 실차 exp_steering_node."""

    def __init__(self, target, gain, max_steer):
        self.target, self.gain, self.max_steer = target, gain, max_steer

    def steering(self, scan):
        r = scan['ranges']
        a = scan['angle_min'] + np.arange(len(r)) * scan['angle_increment']
        side = np.isfinite(r) & (a > math.radians(60)) & (a < math.radians(120))
        if not side.any():
            return None
        left = float(np.quantile(r[side] * np.sin(a[side]), 0.35))
        return max(-self.max_steer, min(self.max_steer, self.gain * (left - self.target)))


class Sim:
    def __init__(self, args):
        self.args = args
        tune = {k: getattr(args, k) for k in simconfig.TUNE_KEYS}
        self.cfg = cfg = simconfig.RunConfig(args.method, args.scenario, args.d_e, args.seed, tune)
        if getattr(args, 'test_step_v', None):        # Exp0 i 속도 계단 (bringup test_step_v:= 와 같음)
            cfg.run_extra.update(test_step_v=float(args.test_step_v), test_step_x=float(args.test_step_x))
        for w in check_layout(cfg.scenario_p):
            print(f'경고: 트랙 배치 불일치 (계산값 사용): {w}', file=sys.stderr)
        self.geom = load_geometry(cfg.geometry_path)
        self.world, ego_h = world_mod.build(cfg)
        xml = self.world.xml.replace('{EGO_GEOM}', world_mod.ego_geom_xml(self.geom.footprint, ego_h))
        self.m = mujoco.MjModel.from_xml_string(xml)
        self.d = mujoco.MjData(self.m)
        self.ego_body = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_BODY, 'ego')
        self.car2_body = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_BODY, 'car2')
        s = cfg.sim
        self.dt = float(s['sim']['dt'])
        c = cfg.common
        front = self.geom.rear_axle_to_front
        self.start = (-front, 0.0, 0.0)          # 앞 끝이 트랙 x = 0
        vcfg = s['vehicle']
        self.ego = vehicle.Kinematic(*self.start, self.geom.wheelbase, float(c['t_r']), vcfg['tau'],
                                     vcfg['a_max'], float(c['a_b']), vcfg['steer_rate'],
                                     max_steer=float(c['max_steering']))
        self.car2 = None
        if self.world.car2_start:
            c2 = s['car2']
            self.car2 = vehicle.Kinematic(*self.world.car2_start, self.geom.wheelbase, c2['t_r'], vcfg['tau'],
                                          vcfg['a_max'], float(c['a_b']), vcfg['steer_rate'],
                                          erpm_gain=float(cfg.popout['erpm_gain']),
                                          servo_center=float(cfg.popout['servo_center']))
            L, W, ro = c2['length'], c2['width'], c2['rear_overhang']
            self.car2_fp = [(L - ro, W / 2), (-ro, W / 2), (-ro, -W / 2), (L - ro, -W / 2)]
        self.lidar = lidar_mod.Lidar(self.m, s['lidar'], cfg.mount, self.ego_body, seed=args.seed)
        self.p2l = cfg.pointcloud_to_laserscan()
        self.static_polys = [[(b.center[0] - b.half[0], b.center[1] - b.half[1]),
                              (b.center[0] + b.half[0], b.center[1] - b.half[1]),
                              (b.center[0] + b.half[0], b.center[1] + b.half[1]),
                              (b.center[0] - b.half[0], b.center[1] + b.half[1])] for b in self.world.boxes]
        self.T_base_lidar = mat(cfg.mount['lidar_xyz'][0], cfg.mount['lidar_xyz'][1], cfg.mount['lidar_rpy'][2],
                                cfg.mount['lidar_xyz'][2])
        self.frame = None   # 현재 라이다 프레임의 TF
        if self.car2 and s.get('timing_overrides', {}).get('popout_from_model'):
            import popout_timing
            cfg.timing = popout_timing.enter_time(cfg, self.dt)
        self.make_nodes()

    # ------------------------------------------------------------ 노드
    def make_nodes(self):
        import my_algo.exp_aeb_node as aeb
        import my_algo.exp_drive_mux as mux
        import my_algo.exp_steering_node as steer
        import my_algo.occupancy_grid_node as og
        import my_algo.phantom_node as ph
        import my_algo.speed_layer_node as sp
        import my_algo.trigger_node as tr
        cfg = self.cfg
        og.lookup_matrix = self.lookup_matrix
        ph.lookup_pose2d = self.lookup_pose2d
        steer.load_geometry = lambda path: self.geom
        mk = lambda mod, cls, name: rosstub.make_node(rn, getattr(mod, cls), cfg.node_params(name))  # noqa: E731
        rosstub.set_time(rn, 0.0)
        self.nodes = {
            'occupancy_grid_node': mk(og, 'OccupancyGridNode', 'occupancy_grid_node'),
            'phantom_node': mk(ph, 'PhantomNode', 'phantom_node'),
            'speed_layer_node': mk(sp, 'SpeedLayerNode', 'speed_layer_node'),
            'exp_aeb_node': mk(aeb, 'ExpAebNode', 'exp_aeb_node'),
            'exp_drive_mux': mk(mux, 'ExpDriveMux', 'exp_drive_mux'),
            'trigger_node': mk(tr, 'TriggerNode', 'trigger_node'),
        }
        self.simple = None
        if self.args.steering == 'node':
            n = mk(steer, 'ExpSteeringNode', 'exp_steering_node')
            n.command_pub.topic = '/exp/steering_command'      # bringup의 remap과 같음
            self.nodes['exp_steering_node'] = n
        else:
            sec = cfg.section('exp_steering_node')
            self.simple = SimpleSteering(float(sec['target_wall_distance']), float(sec['wall_follow_gain']),
                                         float(cfg.common['max_steering']))
        if self.car2:
            import my_algo.popout_vehicle_node as po
            self.run_dir_tmp = os.path.join(self.args.out_dir or '.', '_popout')
            self.nodes['popout_vehicle_node'] = rosstub.make_node(
                rn, po.PopoutVehicleNode, cfg.popout_params(self.run_dir_tmp))
        self.tap = {}
        self.events = []
        self._routed = []
        rosstub.take(rn)

    def lookup_matrix(self, buffer, target, source, stamp, timeout_sec=0.05):
        f = self.frame
        return {('odom', 'livox_frame'): f['odom_lidar'], ('base_link', 'livox_frame'): self.T_base_lidar,
                ('odom', 'base_link'): f['odom_base']}.get((target, source))

    def lookup_pose2d(self, buffer, target, source, stamp, timeout_sec=0.05):
        return self.frame['pose_odom'] if (target, source) == ('odom', 'base_link') else None

    def route(self):
        """BUS에 쌓인 메시지를 구독 노드에 전달 (연쇄 발행이 끝날 때까지)."""
        out = []
        while rn.BUS:
            batch = rosstub.take(rn)
            for topic, msg in batch:
                out.append((topic, msg))
                if topic in TAP_TOPICS:
                    self.tap[topic] = msg.data
                if topic in EVENT_TOPICS:
                    self.events.append({'t': self.t, 'topic': topic,
                                        'data': getattr(msg, 'data', None) or getattr(msg, 'frame_id', '')})
                for node in self.nodes.values():
                    cb = node.subs.get(topic)
                    if cb:
                        cb(msg)
        self._routed.extend(out)     # apply_commands가 이번 스텝의 VESC 명령을 읽는다
        return out

    def publish(self, topic, msg):
        rn.BUS.append((topic, msg))

    # ------------------------------------------------------------ 메시지
    def to_odom(self, x, y, yaw):
        sx, sy, syaw = self.start
        c, s = math.cos(-syaw), math.sin(-syaw)
        dx, dy = x - sx, y - sy
        return c * dx - s * dy, s * dx + c * dy, yaw - syaw

    def odom_msg(self, veh, t, ego=True):
        o = Odometry()
        o.header.stamp = stamp_of(t)
        x, y, yaw = self.to_odom(*veh.pose()) if ego else veh.pose()
        o.pose.pose.position.x, o.pose.pose.position.y = x, y
        o.pose.pose.orientation.z, o.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        o.twist.twist.linear.x = veh.v
        return o

    def cloud_msg(self, pts, t):
        msg = PointCloud2()
        msg.header.stamp = stamp_of(t)
        msg.header.frame_id = 'livox_frame'
        msg.fields = [PointField('x', 0, 7), PointField('y', 4, 7), PointField('z', 8, 7)]
        msg.point_step, msg.width = 12, len(pts)
        msg.row_step = 12 * len(pts)
        msg.data = np.asarray(pts, dtype='<f4').tobytes()
        return msg

    def scan_msg(self, scan, t):
        m = LaserScan()
        m.header.stamp = stamp_of(t)
        m.header.frame_id = 'livox_frame'
        for k in ('angle_min', 'angle_max', 'angle_increment', 'range_min', 'range_max'):
            setattr(m, k, float(scan[k]))
        m.ranges = scan['ranges'].tolist()
        return m

    # ------------------------------------------------------------ 실행
    def place_bodies(self):
        x, y, yaw = self.ego.pose()
        self.d.mocap_pos[self.m.body_mocapid[self.ego_body]] = [x, y, 0.0]
        self.d.mocap_quat[self.m.body_mocapid[self.ego_body]] = [math.cos(yaw / 2), 0, 0, math.sin(yaw / 2)]
        if self.car2:
            x, y, yaw = self.car2.pose()
            self.d.mocap_pos[self.m.body_mocapid[self.car2_body]] = [x, y, 0.0]
            self.d.mocap_quat[self.m.body_mocapid[self.car2_body]] = [math.cos(yaw / 2), 0, 0, math.sin(yaw / 2)]
        mujoco.mj_kinematics(self.m, self.d)

    def distances(self):
        ego_poly = vehicle.transform_polygon(self.geom.footprint, *self.ego.pose())
        d_car2 = (polygon_distance(ego_poly, vehicle.transform_polygon(self.car2_fp, *self.car2.pose()))
                  if self.car2 else math.inf)
        d_static = min(polygon_distance(ego_poly, p) for p in self.static_polys)
        return d_car2, d_static

    def run(self, hook=None):
        """고정 스텝 루프. hook(뷰어 오버레이·배속)은 스텝이 끝난 뒤 상태를 읽기만 한다 → 배치와 같은 결과.

        hook.after_step(sim)이 False를 돌려주면(창 닫힘) 멈춘다. 일시정지·배속은 hook 안에서 벽시계로만 기다린다.
        """
        s = self.cfg.sim['sim']
        lidar_period = 1.0 / float(self.cfg.sim['lidar']['hz'])
        self.t, next_lidar, auto_sent, stopped_since = 0.0, 0.0, False, None
        log = {k: [] for k in ('t', 'x', 'y', 'yaw', 'v', 'steer', 's', 'final_cmd', 'v_cmd', 'v_raw', 'v_cap',
                               'd', 'rho', 'aeb', 'stop_reason', 'flag', 'car2_x', 'car2_y', 'car2_v',
                               'dist_car2', 'dist_static')}
        frames = {'t': [], 'phantoms': [], 'boundary': [], 'grid': [], 'grid_origin': [], 'cloud_digest': []}
        travel0 = None
        steps = int(round(float(s['max_time']) / self.dt))
        self.log, self.frames = log, frames
        for k in range(steps):
            self.t = round((k + 1) * self.dt, 9)
            rosstub.set_time(rn, self.t)
            self.ego.step(self.t, self.dt)
            if self.car2:
                self.car2.step(self.t, self.dt)
            self.place_bodies()
            if not auto_sent and self.t >= float(s['auto_start_time']):
                self.publish('/autonomous_mode', Bool(True))
                auto_sent, travel0 = True, self.ego.travel
            self.publish('/vesc/odom', self.odom_msg(self.ego, self.t))
            if self.car2:
                self.publish('odom', self.odom_msg(self.car2, self.t, ego=False))
            self.route()
            if self.t + 1e-9 >= next_lidar:
                next_lidar += lidar_period
                self.lidar_frame(frames)
            for node in self.nodes.values():
                rosstub.run_timers(rn, node, self.t)
                self.route()
            self.apply_commands()
            self.log_step(log, travel0)
            if hook is not None and hook.after_step(self) is False:
                break
            reason = self.tap.get('/exp/stop_reason', '')
            if reason in ('stop_line', 'aeb', 'joy_timeout') and self.ego.v < 0.02:
                stopped_since = stopped_since or self.t
                if self.t - stopped_since >= float(s['end_after_stop']):
                    break
        return log, frames

    def lidar_frame(self, frames):
        pose = self.ego.pose()
        pts_l, pts_w, origin = self.lidar.scan(self.d, pose)
        self.last_points_world = pts_w        # 뷰어 표시용 (읽기 전용)
        # 점군 지문: 배치·뷰어 결과가 비트 단위로 같은지 확인용 (격자는 점 몇 개가 달라도 같을 수 있다)
        frames['cloud_digest'].append(hashlib.sha1(np.ascontiguousarray(pts_l, '<f8').tobytes()).hexdigest())
        ox, oy, oyaw = self.to_odom(*pose)
        T_odom_base = mat(ox, oy, oyaw)
        self.frame = {'odom_base': T_odom_base, 'odom_lidar': T_odom_base @ self.T_base_lidar,
                      'pose_odom': (ox, oy, oyaw)}
        scan = lidar_mod.cloud_to_scan(pts_l, self.p2l)
        self.last_scan = scan
        self.publish('/livox/lidar', self.cloud_msg(pts_l, self.t))
        self.publish('/scan', self.scan_msg(scan, self.t))
        out = self.route()
        if self.simple is not None:
            st = self.simple.steering(scan)
            self.simple_steer = st if st is not None else getattr(self, 'simple_steer', 0.0)
            from std_msgs.msg import String
            from my_algo.vehicle_utils import ReactiveDriveCommand, command_to_json
            self.publish('/exp/steering_command', String(command_to_json(
                ReactiveDriveCommand(steering_rad=self.simple_steer, mode='SIM_SIMPLE'))))
            self.route()
        sx, sy, syaw = self.start
        to_world = lambda xy: np.column_stack([xy[:, 0] + sx, xy[:, 1] + sy]) if len(xy) else np.empty((0, 2))  # noqa: E731
        for topic, msg in out:
            if topic == '/exp/phantoms':
                xy = np.array([[mk.pose.position.x, mk.pose.position.y] for mk in msg.markers
                               if mk.action != 3]).reshape(-1, 2)
                frames['phantoms'].append(to_world(xy))
            elif topic == '/exp/boundary_frame':
                xy = np.array([[c.x, c.y] for c in msg.cells]).reshape(-1, 2)
                frames['boundary'].append(to_world(xy))
            elif topic == '/exp/grid':
                info = msg.info
                frames['grid'].append(np.asarray(msg.data, np.int8).reshape(info.height, info.width))
                frames['grid_origin'].append((info.origin.position.x + sx, info.origin.position.y + sy,
                                              info.resolution))
        frames['t'].append(self.t)
        for key in ('phantoms', 'boundary'):
            while len(frames[key]) < len(frames['t']):
                frames[key].append(np.empty((0, 2)))

    def apply_commands(self):
        """이번 스텝에 mux(·#2 노드)가 보낸 마지막 VESC 명령을 차량에 넣는다."""
        motor = [m.data for t, m in self.routed('/commands/motor/speed')]
        servo = [m.data for t, m in self.routed('/commands/servo/position')]
        if motor:
            self.ego.command(self.t, motor[-1], servo[-1] if servo else self.ego.servo_now)
        if self.car2:
            m2 = [m.data for t, m in self.routed('commands/motor/speed')]
            s2 = [m.data for t, m in self.routed('commands/servo/position')]
            if m2:
                self.car2.command(self.t, m2[-1], s2[-1] if s2 else self.car2.servo_now)
        self._routed = []

    def routed(self, topic):
        return [(None, msg) for tp, msg in self._routed if tp == topic]

    def log_step(self, log, travel0):
        x, y, yaw = self.ego.pose()
        d_car2, d_static = self.distances()
        tap = self.tap
        f = lambda k, default=math.nan: float(tap[k]) if k in tap and tap[k] is not None else default  # noqa: E731
        vals = {'t': self.t, 'x': x, 'y': y, 'yaw': yaw, 'v': self.ego.v, 'steer': self.ego.steer,
                's': (self.ego.travel - travel0) if travel0 is not None else 0.0,
                'final_cmd': f('/exp/final_cmd', 0.0), 'v_cmd': f('/exp/v_cmd'), 'v_raw': f('/exp/v_raw'),
                'v_cap': f('/exp/v_cap'), 'd': f('/exp/phantom_d'), 'rho': f('/exp/rho'),
                'aeb': bool(tap.get('/aeb/active', False)), 'stop_reason': tap.get('/exp/stop_reason', ''),
                'flag': tap.get('/exp/min_speed_flag', ''),
                'car2_x': self.car2.x if self.car2 else math.nan, 'car2_y': self.car2.y if self.car2 else math.nan,
                'car2_v': self.car2.v if self.car2 else math.nan, 'dist_car2': d_car2, 'dist_static': d_static}
        for k, v in vals.items():
            log[k].append(v)


def git_hash():
    try:
        h = subprocess.check_output(['git', '-C', ROOT, 'rev-parse', 'HEAD'], text=True).strip()
        dirty = bool(subprocess.check_output(['git', '-C', ROOT, 'status', '--porcelain'], text=True).strip())
        return h, dirty
    except (OSError, subprocess.CalledProcessError):
        return None, None


def run_dir_for(args):
    date = datetime.date.today().strftime('%Y%m%d')
    de = 'na' if args.d_e is None else f'{args.d_e:.1f}'
    name = f'{args.scenario}_{args.method}_de{de}_seed{args.seed:02d}'
    if args.tag:
        name += f'_{args.tag}'
    return os.path.join(args.log_root, date, name)


def save(sim, log, frames, run_dir, wall):
    os.makedirs(run_dir, exist_ok=True)
    arrays = {k: np.asarray(v) for k, v in log.items() if k not in ('stop_reason', 'flag')}
    arrays['stop_reason'] = np.asarray(log['stop_reason'], dtype='U32')
    arrays['flag'] = np.asarray(log['flag'], dtype='U32')
    for key in ('phantoms', 'boundary'):
        arrays[f'{key}_xy'] = np.concatenate(frames[key]) if frames[key] else np.empty((0, 2))
        arrays[f'{key}_frame'] = np.concatenate([np.full(len(a), i) for i, a in enumerate(frames[key])]) \
            if frames[key] else np.empty(0, int)
    arrays['frame_t'] = np.asarray(frames['t'])
    np.savez_compressed(os.path.join(run_dir, 'log.npz'), **arrays)
    np.savez_compressed(os.path.join(run_dir, 'grids.npz'),
                        **{f'g{i:04d}': g for i, g in enumerate(frames['grid'])},
                        origin=np.asarray(frames['grid_origin']))
    h, dirty = git_hash()
    meta = {'mode': 'sim', 'method': sim.args.method, 'scenario': sim.args.scenario, 'd_e': sim.args.d_e,
            'run_id': sim.args.seed, 'seed': sim.args.seed, 'steering': sim.args.steering,
            'git_hash': h, 'git_dirty': dirty, 'mujoco': mujoco.__version__, 'wall_time_s': round(wall, 2),
            'sim_time_s': log['t'][-1] if log['t'] else 0.0, 'discard': False, 'discard_reason': None,
            'events': sim.events, 'config': sim.cfg.summary(),
            'world_boxes': [{'name': b.name, 'center': b.center, 'half': b.half} for b in sim.world.boxes],
            'car2_start': sim.world.car2_start, 'start_pose': sim.start,
            'sim_differences': ['조이스틱 데드맨·발행자 점검 끔', 'odom 참값', '운동학 차량 모델',
                                f'v_cap t_r = t_r + tau = {sim.cfg.t_r_eff():.3f} (timing_overrides)',
                                f'popout_enter_time = 시뮬 #2 모델 {sim.cfg.timing.get("popout_enter_time")}, '
                                'trigger_latency = 0' if sim.cfg.timing else 'popout 시뮬 값 없음 (S1 아님)',
                                ('단순 조향(시뮬 전용)' if sim.args.steering == 'simple' else '조향 = 실차 exp_steering_node')]}
    with open(os.path.join(run_dir, 'meta.json'), 'w', encoding='utf-8') as f:
        json.dump(meta, f, ensure_ascii=False, indent=2, default=float)
    return meta


def parse(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument('--method', required=True, choices=['B0', 'B1', 'B2', 'P'])
    ap.add_argument('--scenario', required=True, choices=['S1', 'S2', 'S3', 'S4', 'E0'])
    ap.add_argument('--d-e', type=float, default=None)
    ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--steering', choices=['node', 'simple'], default='node')
    ap.add_argument('--log-root', default=os.path.join(ROOT, 'sim_logs'))
    ap.add_argument('--out-dir', default=None, help='회차 폴더를 직접 지정')
    ap.add_argument('--tag', default='')
    ap.add_argument('--viewer', action='store_true', help='대화형 뷰어 (macOS: mjpython으로 실행)')
    ap.add_argument('--test-step-v', type=float, default=None,
                    help='Exp0 i 전용: 주행 거리 --test-step-x에서 속도 명령을 이 값으로 계단처럼 내림 (보통 E0, B0)')
    ap.add_argument('--test-step-x', type=float, default=2.5)
    ap.add_argument('--viewer-max-points', type=int, default=4000,
                    help='뷰어에 그릴 라이다 점 수 상한 (표시용 솎아내기, 알고리즘 입력은 그대로)')
    ap.add_argument('--video', action='store_true', help='위에서 본 영상 mp4 저장')
    ap.add_argument('--verbose', action='store_true', help='노드 상태 출력 보이기')
    for k in simconfig.TUNE_KEYS:
        typ = int if k in ('n_frames', 'known_on', 'known_off') else float
        ap.add_argument('--' + k.replace('_', '-'), dest=k, type=typ, default=None, help='[EXP-TUNE]')
    return ap.parse_args(argv)


def main(argv=None):
    args = parse(argv)
    if args.scenario != 'S1':
        args.d_e = None
    args.out_dir = args.out_dir or run_dir_for(args)
    sim = Sim(args)
    wall0 = time.time()
    import contextlib
    quiet = contextlib.redirect_stdout(open(os.devnull, 'w')) if not args.verbose else contextlib.nullcontext()
    with quiet:   # 노드 상태 출력(planner의 [LiDAR] 줄 등)을 숨긴다
        if args.viewer:
            import mujoco.viewer
            import viewer_ui
            ui = viewer_ui.ViewerUI(sim, max_points=args.viewer_max_points)
            with mujoco.viewer.launch_passive(sim.m, sim.d, key_callback=ui.on_key) as v:
                ui.attach(v)
                log, frames = sim.run(hook=ui)
                ui.hold_until_closed()      # 회차가 끝나도 창을 닫을 때까지 마지막 장면 유지
        else:
            log, frames = sim.run()
    meta = save(sim, log, frames, args.out_dir, time.time() - wall0)
    sys.path.insert(0, os.path.join(ROOT, 'analysis'))
    import sim_reader
    m = sim_reader.compute(args.out_dir)
    if args.video:
        import video
        video.render(args.out_dir)
    print(f"{os.path.basename(args.out_dir)}: sim {meta['sim_time_s']:.1f}s (wall {meta['wall_time_s']:.1f}s) "
          f"dT_cmd={m.get('dT_cmd')} min_speed_cmd={m.get('min_speed_cmd')} "
          f"collision={m.get('collision')} aeb={m.get('aeb_triggered')} → {args.out_dir}")
    return m


if __name__ == '__main__':
    main()
