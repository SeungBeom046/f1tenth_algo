"""추월 실험 planner 노드. role=leader(검정이, 앞차 시나리오) / role=ego(파랑이, 추월하는 차).

/scan → LeaderRunner 또는 EgoRunner(my_algo/aur, 시뮬과 같은 코드) → /reactive/lidar_command (JSON)
→ autonomous_drive(AEB → VESC). gap follow(lidar_reactive_drive)는 띄우지 않는다 (launch/exp_overtake.launch.py).

leader 시나리오 (intent):
  Y : 후행차가 d_y 안으로 오면 오른쪽 차선(−δ)으로 비킨다. 속도 v_B_nominal 유지
  NY: 비키지 않는다. 후행차가 d_brake_trigger 안으로 온 뒤 brake_delay초 지나 급정지 (rpm 0 유지)
  d_y·brake_delay는 run_id로 정해지고 시작할 때 터미널에 찍힌다.
ego (method a|b|c|d): 접근 → belief로 Y/NY 판단 → 추월(PASS → MERGE → DONE) 또는 추종(FOLLOW).
  상태 전이·belief 확정·추월 중단이 터미널에 [EXP ego] 줄로 찍힌다.
straight: 서보 중심 보정용. 조향 0으로 v_B_nominal 직진만 하고, 벽으로 잰 횡위치·헤딩을 0.5 s마다 찍는다.
"""
import collections
import json
import math
import os
import time

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String

from my_algo.aur.corridor import Corridor
from my_algo.aur.ego_runner import EgoRunner
from my_algo.aur.leader_runner import LeaderRunner
from my_algo.aur.params import load_params
from my_algo.aur.tracker import scan_to_points
from my_algo.baseline.vehicle_utils import ReactiveDriveCommand, command_to_json, print_event_line

WHEELBASE = 0.33     # [m] 실측 (sim scenario.yaml vehicle.wheelbase)


def parse_intent(value):
    """launch 인자 intent → 'Y' | 'NY'. YAML이 Y를 True로 바꿔 넘기는 경우를 되돌린다."""
    if value is True:
        return 'Y'
    intent = str(value).strip().upper()
    if intent not in ('Y', 'NY'):
        raise SystemExit('exp_planner: intent는 Y 또는 NY: %r' % (value,))
    return intent


class ExpPlanner(Node):
    def __init__(self):
        super().__init__('exp_planner')
        self.declare_parameter('role', 'leader')        # leader | ego | straight
        # ROS 파라미터 YAML은 Y를 bool true로 읽는다 (intent:=Y → True). 타입을 열어 두고 아래에서 'Y'로 되돌린다.
        # dynamic_typing은 Galactic 이후에만 있다 (Foxy는 타입 검사를 안 해서 없어도 된다)
        intent_desc = ParameterDescriptor()
        if hasattr(intent_desc, 'dynamic_typing'):
            intent_desc.dynamic_typing = True
        self.declare_parameter('intent', 'Y', intent_desc)     # (leader) Y | NY
        self.declare_parameter('run_id', '0')           # (leader) d_y·급정지 지연 seed (회차표 run_id를 그대로)
        self.declare_parameter('d_y', 0.0)              # (leader) > 0이면 run_id 대신 이 양보 반응 거리 [m]
        self.declare_parameter('method', 'd')           # (ego) a | b | c | d
        self.declare_parameter('t_d', 0.0)              # (ego) > 0이면 제동 지연 t_d [s] 덮어쓰기 (보정 실험 값)
        self.declare_parameter('W', 0.0)                # > 0이면 실측 통로 폭 [m]. 0이면 벽으로 추정
        self.declare_parameter('speed_source', 'sensors_core')    # sensors_core | command
        self.declare_parameter('experiment_safety', True)
        self.declare_parameter('stationary_pass', False)
        self.declare_parameter('steer', 0.0)            # (straight) 고정 조향 [rad]. 0이 아니면 원을 그려 실제 바퀴 각을 잰다
        self.declare_parameter('params_file', os.path.join(
            get_package_share_directory('my_algo'), 'config', 'exp_common.yaml'))

        self.role = self.get_parameter('role').value
        if self.role not in ('leader', 'ego', 'straight'):
            raise SystemExit('exp_planner: role은 leader | ego | straight: %r' % self.role)
        W = float(self.get_parameter('W').value)
        t_d = float(self.get_parameter('t_d').value)
        over = {}
        over['experiment_safety'] = bool(self.get_parameter('experiment_safety').value)
        over['stationary_pass'] = bool(self.get_parameter('stationary_pass').value)
        if W > 0:
            over['W_leader' if self.role == 'leader' else 'W'] = W
        if t_d > 0:
            over['t_d'] = t_d
        self.p = load_params(self.get_parameter('params_file').value, **over)
        if self.role == 'leader':
            d_y = float(self.get_parameter('d_y').value)
            self.intent = parse_intent(self.get_parameter('intent').value)
            self.runner = LeaderRunner(self.p, self.intent, str(self.get_parameter('run_id').value),
                                       d_y=d_y if d_y > 0 else None)
        elif self.role == 'straight':
            self.runner = Corridor(self.p)
            self.straight_steer = float(self.get_parameter('steer').value)
            self.t_print = None
        else:
            self.method = str(self.get_parameter('method').value).strip().lower()
            if self.method not in ('a', 'b', 'c', 'd'):
                raise SystemExit('exp_planner: method는 a|b|c|d: %r' % self.method)
            self.runner = EgoRunner(self.p, self.method)
        self.v = 0.0
        self.v_cmd = 0.0
        self.speed_source = self.get_parameter('speed_source').value
        if self.speed_source == 'sensors_core':
            try:
                from vesc_msgs.msg import VescStateStamped
                self.create_subscription(VescStateStamped, '/sensors/core', self.core_callback, 10)
            except ImportError:
                self.get_logger().warn('vesc_msgs 없음 (VESC 워크스페이스 source?) → speed_source=command '
                                       '(명령 속도를 실제 속도로 쓴다. ego는 안전 거리 계산이 낙관적이 된다)')
                self.speed_source = 'command'

        qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT, history=QoSHistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, qos)
        self.cmd_pub = self.create_publisher(String, '/reactive/lidar_command', 10)
        self.state_pub = self.create_publisher(String, '/exp/state', 10)
        self.create_subscription(Bool, '/aeb/active', self.aeb_callback, 10)
        self.create_subscription(String, '/aeb/status', self.aeb_status_callback, 10)
        self.prev = {}
        self.stats = RunStats()
        self.tags = (None, None)    # 직전 인식 태그 (바뀔 때만 터미널에 찍는다)
        self.aeb_on = False
        self.aeb_status = ''

        Wtxt = W if W > 0 else '벽 추정'
        if self.role == 'leader':
            lg = self.runner.logic
            self.event('시작 role=leader intent=%s d_y=%.2f m brake_delay=%.2f s W=%s v_B=%.2f m/s speed_source=%s' % (
                self.intent, lg.d_y, lg.brake_delay, Wtxt, self.p.v_B_nominal, self.speed_source))
        elif self.role == 'straight':
            if self.straight_steer:
                self.event('시작 role=straight steer=%+.3f rad: %.2f m/s로 원 주행 (조향 확인). 지름 D를 재면 실제 바퀴 각 = atan(2·%.2f/D) (D는 뒤축 중심 원)' % (
                    self.straight_steer, self.p.v_B_nominal, WHEELBASE))
            else:
                self.event('시작 role=straight: 조향 0, %.2f m/s 직진 (서보 중심 보정). 횡위치 + = 왼쪽' % self.p.v_B_nominal)
        else:
            self.event('시작 role=ego method=%s W=%s t_d=%.2f s (+scan %.2f s) v_pass=%.2f m/s eta=%.2f speed_source=%s' % (
                self.method, Wtxt, self.p.t_d, self.p.scan_period, self.p.v_pass, self.p.eta, self.speed_source))

    def event(self, msg):
        print_event_line('[EXP %s] %s' % (self.role, msg))

    def core_callback(self, msg):
        # shortcut: /sensors/core speed를 ERPM으로 가정 (미확인, 보정 실험에서 확인)
        self.v = abs(msg.state.speed) / self.p.erpm_gain

    def aeb_status_callback(self, msg):
        self.aeb_status = msg.data

    def aeb_callback(self, msg):
        if msg.data and not self.aeb_on:
            self.stats.aeb(self.aeb_status)
            self.event('AEB 작동: %s' % self.aeb_status)
        self.aeb_on = msg.data

    def scan_callback(self, msg):
        t = self.get_clock().now().nanoseconds / 1e9
        v = self.v if self.speed_source == 'sensors_core' else self.v_cmd
        t_cpu = time.perf_counter()
        if self.role == 'leader':
            state, mode, steer, reason = self.leader_step(t, msg, v)
        elif self.role == 'straight':
            state, mode, steer, reason = self.straight_step(t, msg, v)
        else:
            state, mode, steer, reason = self.ego_step(t, msg, v)
        if self.role != 'straight':
            rail, lead = perception_tags(self.runner)
            if rail != self.tags[0]:
                self.event('인식: %s' % rail)
            if lead.split()[0] != self.tags[1]:
                self.event('인식: %s' % lead)
            self.tags = (rail, lead.split()[0])
            reason = '%s | %s %s' % (reason, rail, lead)
            self.stats.scan(t, v, self.v_cmd, (time.perf_counter() - t_cpu) * 1e3, state, reason.split(' |')[0],
                            rail, lead.split()[0])
        self.cmd_pub.publish(String(data=command_to_json(ReactiveDriveCommand(
            steering_rad=steer, rpm=self.v_cmd * self.p.erpm_gain, mode=mode, confidence=1.0, reason=reason))))
        self.state_pub.publish(String(data=json.dumps(state)))

    def leader_step(self, t, msg, v):
        self.v_cmd, steer, y_ref = self.runner.step(t, msg, v)
        lg, trk = self.runner.logic, self.runner.track
        mode = 'LEADER_BRAKE' if lg.brake_fired else ('LEADER_YIELD' if y_ref != 0.0 else 'LEADER_CRUISE')
        s = dict(t=t, role='leader', intent=self.intent, mode=mode, v=v, v_cmd=self.v_cmd, steer=steer, y_ref=y_ref,
                 d_rear=trk.d if trk.valid else None, ego_lateral=self.runner.corridor.ego_lateral,
                 t0=lg.t0, d_y=lg.d_y, yield_triggered=lg.yield_triggered, t_yield=lg.t_yield,
                 brake_armed=lg.t_armed is not None, brake_fired=lg.brake_fired, t_brake=lg.t_brake,
                 start_too_close=lg.start_too_close)
        cor = self.runner.corridor
        s.update(heading=cor.heading, W=cor.W_est, corridor_valid=cor.valid,
                 wall_left=cor.left, wall_right=cor.right,
                 guardrail_local_view=self.p.guardrail_local_view, wall_initializing=cor.initializing)
        prev, self.prev = self.prev, s
        d_txt = '안 보임' if s['d_rear'] is None else '%.2f m' % s['d_rear']
        if s['t0'] is not None and prev.get('t0') is None:
            self.event('출발 감지 (t0), 후행차 %s' % d_txt)
            if s['start_too_close']:
                self.event('경고: 출발 때 후행차가 너무 가깝다 (d_y_max + %.1f m 안) → 회차 무효' % self.p.start_gap_margin)
        if mode != prev.get('mode'):
            self.event({'LEADER_CRUISE': '주행 (중앙)',
                        'LEADER_YIELD': '양보 시작: 오른쪽 차선 y_ref=%.2f, 후행차 %s' % (y_ref, d_txt),
                        'LEADER_BRAKE': '급정지 (NY)'}[mode])
        if s['brake_armed'] and not prev.get('brake_armed'):
            self.event('급정지 무장: %.2f s 뒤 정지' % lg.brake_delay)
        return s, mode, steer, 'intent=%s d_rear=%s' % (self.intent, 'none' if s['d_rear'] is None else '%.2f' % s['d_rear'])

    def straight_step(self, t, msg, v):
        cor = self.runner
        cor.update(scan_to_points(list(msg.ranges), msg.angle_min, msg.angle_increment, msg.range_min, msg.range_max), stamp=t)
        self.v_cmd = self.p.v_B_nominal
        if self.p.corridor_guardrail and not cor.valid:
            self.v_cmd = 0.0
        if self.t_print is None or t - self.t_print >= 0.5:
            self.t_print = t
            self.event('횡위치 %+.3f m  헤딩 %+.1f°  (양쪽 벽 %s)' % (
                cor.ego_lateral, math.degrees(cor.heading), '보임' if cor.valid else '안 보임'))
        s = dict(t=t, role='straight', v=v, v_cmd=self.v_cmd, ego_lateral=cor.ego_lateral, heading=cor.heading,
                 W=cor.W_est, valid=cor.valid)
        return s, 'STRAIGHT', self.straight_steer, 'servo_center_check' if not self.straight_steer else 'steer_check'

    def ego_step(self, t, msg, v):
        self.v_cmd, steer, y_ref, st, reason = self.runner.step(t, msg, v)
        fsm, trk, cor = self.runner.fsm, self.runner.track, self.runner.corridor
        s = dict(t=t, role='ego', method=self.method, state=st, reason=reason, v=v, v_cmd=self.v_cmd, steer=steer,
                 y_ref=y_ref, valid=trk.valid, d=trk.d, y_b=trk.y_b, v_b=trk.v_b, ego_lateral=cor.ego_lateral,
                 heading=cor.heading, W=cor.W_est, h=fsm.h if fsm.h != float('inf') else None,
                 b=fsm.belief.prob_yield(), decided=fsm.decided,
                 t0=fsm.t0, t_decided=fsm.t_decided, n_abort=fsm.abort_count, leader_vlat=fsm.leader_vlat)
        s.update(corridor_valid=cor.valid, wall_left=cor.left, wall_right=cor.right,
                 guardrail_local_view=self.p.guardrail_local_view, wall_initializing=cor.initializing)
        s.update(track_status=self.runner.tracker.status, pending_d=self.runner.tracker.pending_d)
        prev, self.prev = self.prev, s
        if prev.get('n_abort', 0) != s['n_abort']:
            self.event('추월 중단 (선행차가 다시 들어오거나 왼쪽 여유 부족)')
        if st != prev.get('state'):
            self.event('%s → %s (%s), 선행차 %s m' % (prev.get('state', '-'), st, reason,
                                                     '%.2f' % trk.d if trk.valid else '안 보임'))
        if s['decided'] and not prev.get('decided'):
            self.event('belief 확정 %s (b=%.2f)' % (s['decided'], s['b']))
        return s, 'EGO_' + st, steer, reason


def perception_tags(runner):
    """터미널·명령 reason에 붙일 인식 상태. rail_ok | rail_hold(직전 추정 유지) | rail_init | rail_lost,
    lead_detect <d> | lead_pending(최초 3 scan 확인) | lead_jump(먼 점 거부) | lead_lost(보이다 사라짐) | lead_none."""
    cor, trk, tracker = runner.corridor, runner.track, runner.tracker
    if cor.valid:
        rail = 'rail_ok' if cor.left is not None else 'rail_hold'
    else:
        rail = 'rail_init' if cor.initializing else 'rail_lost'
    if trk is not None and trk.valid:
        lead = 'lead_detect %.2fm' % trk.d
    elif tracker.pending_d is not None:
        lead = 'lead_pending %.2fm' % tracker.pending_d
    elif tracker.status == 'distance_jump':
        lead = 'lead_jump'
    else:
        lead = 'lead_lost' if tracker._d is not None else 'lead_none'
    return rail, lead


class RunStats:
    """종료(Ctrl+C) 때 찍는 실험 요약. 기본 주행의 autonomous_metrics_logger 요약과 별도로 추월 쪽 인식·판단을 센다."""

    def __init__(self):
        self.t0 = self.t_last = None
        self.n = 0
        self.dist = 0.0
        self.v_max = self.v_sum = 0.0
        self.cpu = []
        self.dt = []
        self.states = []
        self.reasons = collections.Counter()
        self.rail = collections.Counter()
        self.lead = collections.Counter()
        self.d_min = None
        self.aeb_reasons = []

    def aeb(self, status):
        self.aeb_reasons.append(status)

    def scan(self, t, v, v_cmd, cpu_ms, state, reason, rail, lead):
        if self.t_last is not None:
            dt = t - self.t_last
            self.dt.append(dt)
            self.dist += v * dt
        self.t0 = t if self.t0 is None else self.t0
        self.t_last = t
        self.n += 1
        self.v_max = max(self.v_max, v)
        self.v_sum += v
        self.cpu.append(cpu_ms)
        st = state.get('state') or state.get('mode')
        if st and (not self.states or self.states[-1] != st):
            self.states.append(st)
        self.reasons['stop:' + reason if v_cmd <= 0.0 else 'go'] += 1
        self.rail[rail] += 1
        self.lead[lead] += 1
        # 추월 중(PASS 이후)의 측면 추적 거리는 음수가 되므로 뒤따르는 동안만 센다
        d = state.get('d') if state.get('valid') and st in ('APPROACH', 'PROBE', 'FOLLOW', 'WAIT') else state.get('d_rear')
        if d is not None:
            self.d_min = d if self.d_min is None else min(self.d_min, d)

    def summary(self, role):
        if not self.n:
            return '[EXP %s] 요약: scan 없음' % role

        def pct(counter):
            return ', '.join('%s %.0f%%' % (k, 100.0 * c / self.n) for k, c in counter.most_common())

        cpu = sorted(self.cpu)
        dt = sorted(self.dt) or [0.0]
        lines = [
            '',
            '========== EXP %s 요약 ==========' % role,
            '주행 시간            : %.1f s (scan %d개, 주기 평균 %.0f ms / 최대 %.0f ms)' % (
                self.t_last - self.t0, self.n, 1e3 * sum(dt) / len(dt), 1e3 * dt[-1]),
            '처리 시간            : 평균 %.0f ms / 95%% %.0f ms / 최대 %.0f ms (100 ms 넘으면 scan을 못 따라간다)' % (
                sum(cpu) / len(cpu), cpu[int(0.95 * (len(cpu) - 1))], cpu[-1]),
            '거리·속도            : %.2f m, 평균 %.2f m/s, 최대 %.2f m/s' % (self.dist, self.v_sum / self.n, self.v_max),
            '상태 흐름            : %s' % ' → '.join(self.states),
            '레일 인식            : %s' % pct(self.rail),
            '선행차 인식          : %s' % pct(self.lead),
            '명령                 : %s' % pct(self.reasons),
            '선행차 최소 거리     : %s' % ('없음' if self.d_min is None else '%.2f m' % self.d_min),
            'AEB 작동             : %d회%s' % (len(self.aeb_reasons), ''.join('\n  - ' + r for r in self.aeb_reasons[:5])),
            '=================================',
        ]
        return '\n'.join(lines)


def main(args=None):
    rclpy.init(args=args)
    node = ExpPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.role != 'straight':
            print(node.stats.summary(node.role), flush=True)
        node.cmd_pub.publish(String(data=command_to_json(ReactiveDriveCommand(mode='STOP', reason='exp_planner_exit'))))
        node.destroy_node()
        if rclpy.ok():      # jazzy는 Ctrl+C 때 먼저 shutdown한다
            rclpy.shutdown()
