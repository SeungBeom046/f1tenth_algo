"""추월 실험 planner 노드. 지금은 role=leader(선행차 시나리오)만 있다. ego(추월하는 차)는 아직 없다.

/scan → LeaderRunner(my_algo/aur, 시뮬과 같은 코드) → /reactive/lidar_command (JSON)
→ autonomous_drive(AEB → VESC). gap follow(lidar_reactive_drive)는 띄우지 않는다 (launch/exp_overtake.launch.py).

시나리오 (intent):
  Y : 후행차가 d_y 안으로 오면 오른쪽 차선(−δ)으로 비킨다. 속도 v_B_nominal 유지
  NY: 비키지 않는다. 후행차가 d_brake_trigger 안으로 온 뒤 brake_delay초 지나 급정지 (rpm 0 유지)
d_y·brake_delay는 run_id로 정해지고 시작할 때 터미널에 찍힌다.
"""
import json
import os

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from my_algo.aur.leader_runner import LeaderRunner
from my_algo.aur.params import load_params
from my_algo.baseline.vehicle_utils import ReactiveDriveCommand, command_to_json, print_event_line


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
        self.declare_parameter('role', 'leader')
        # ROS 파라미터 YAML은 Y를 bool true로 읽는다 (intent:=Y → True). 타입을 열어 두고 아래에서 'Y'로 되돌린다.
        # dynamic_typing은 Galactic 이후에만 있다 (Foxy는 타입 검사를 안 해서 없어도 된다)
        intent_desc = ParameterDescriptor()
        if hasattr(intent_desc, 'dynamic_typing'):
            intent_desc.dynamic_typing = True
        self.declare_parameter('intent', 'Y', intent_desc)     # Y | NY
        self.declare_parameter('run_id', '0')           # d_y·급정지 지연 seed (회차표 run_id를 그대로)
        self.declare_parameter('d_y', 0.0)              # > 0이면 run_id 대신 이 양보 반응 거리 [m]
        self.declare_parameter('W', 0.0)                # > 0이면 실측 통로 폭 [m] (W_leader). 0이면 벽으로 추정
        self.declare_parameter('speed_source', 'sensors_core')    # sensors_core | command
        self.declare_parameter('params_file', os.path.join(
            get_package_share_directory('my_algo'), 'config', 'exp_common.yaml'))

        role = self.get_parameter('role').value
        if role != 'leader':
            raise SystemExit('exp_planner: role=%r는 아직 없다 (leader만 구현)' % role)
        W = float(self.get_parameter('W').value)
        self.p = load_params(self.get_parameter('params_file').value, **({'W_leader': W} if W > 0 else {}))
        d_y = float(self.get_parameter('d_y').value)
        self.intent = parse_intent(self.get_parameter('intent').value)
        self.runner = LeaderRunner(self.p, self.intent, str(self.get_parameter('run_id').value),
                                   d_y=d_y if d_y > 0 else None)
        self.v = 0.0
        self.v_cmd = 0.0
        self.speed_source = self.get_parameter('speed_source').value
        if self.speed_source == 'sensors_core':
            try:
                from vesc_msgs.msg import VescStateStamped
                self.create_subscription(VescStateStamped, '/sensors/core', self.core_callback, 10)
            except ImportError:
                self.get_logger().warn('vesc_msgs 없음 (VESC 워크스페이스 source?) → speed_source=command')
                self.speed_source = 'command'

        qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT, history=QoSHistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, qos)
        self.cmd_pub = self.create_publisher(String, '/reactive/lidar_command', 10)
        self.state_pub = self.create_publisher(String, '/exp/state', 10)
        self.prev = {}

        lg = self.runner.logic
        self.event('시작 role=leader intent=%s d_y=%.2f m brake_delay=%.2f s W=%s v_B=%.2f m/s speed_source=%s' % (
            self.intent, lg.d_y, lg.brake_delay, W if W > 0 else '벽 추정', self.p.v_B_nominal, self.speed_source))

    def event(self, msg):
        print_event_line('[EXP leader] %s' % msg)

    def core_callback(self, msg):
        # shortcut: /sensors/core speed를 ERPM으로 가정 (미확인, 보정 실험에서 확인)
        self.v = abs(msg.state.speed) / self.p.erpm_gain

    def scan_callback(self, msg):
        t = self.get_clock().now().nanoseconds / 1e9
        v = self.v if self.speed_source == 'sensors_core' else self.v_cmd
        self.v_cmd, steer, y_ref = self.runner.step(t, msg, v)
        lg, trk = self.runner.logic, self.runner.track
        mode = 'LEADER_BRAKE' if lg.brake_fired else ('LEADER_YIELD' if y_ref != 0.0 else 'LEADER_CRUISE')
        self.cmd_pub.publish(String(data=command_to_json(ReactiveDriveCommand(
            steering_rad=steer, rpm=self.v_cmd * self.p.erpm_gain, mode=mode, confidence=1.0,
            reason='intent=%s d_rear=%s' % (self.intent, 'none' if not trk.valid else '%.2f' % trk.d)))))
        state = dict(t=t, intent=self.intent, mode=mode, v=v, v_cmd=self.v_cmd, steer=steer, y_ref=y_ref,
                     d_rear=trk.d if trk.valid else None, ego_lateral=self.runner.corridor.ego_lateral,
                     t0=lg.t0, d_y=lg.d_y, yield_triggered=lg.yield_triggered, t_yield=lg.t_yield,
                     brake_armed=lg.t_armed is not None, brake_fired=lg.brake_fired, t_brake=lg.t_brake,
                     start_too_close=lg.start_too_close)
        self.state_pub.publish(String(data=json.dumps(state)))
        self.log_changes(state)

    def log_changes(self, s):
        prev, self.prev = self.prev, s
        if s['t0'] is not None and prev.get('t0') is None:
            self.event('출발 감지 (t0), 후행차 거리 %s' % s['d_rear'])
            if s['start_too_close']:
                self.event('경고: 출발 때 후행차가 너무 가깝다 (d_y_max + %.1f m 안) → 회차 무효' % self.p.start_gap_margin)
        if s['mode'] != prev.get('mode'):
            text = {'LEADER_CRUISE': '주행 (중앙)', 'LEADER_YIELD': '양보 시작: 오른쪽 차선 y_ref=%.2f, 후행차 %s m'
                    % (s['y_ref'], s['d_rear']), 'LEADER_BRAKE': '급정지 (NY)'}[s['mode']]
            self.event(text)
        if s['brake_armed'] and not prev.get('brake_armed'):
            self.event('급정지 무장: %.2f s 뒤 정지' % self.runner.logic.brake_delay)


def main(args=None):
    rclpy.init(args=args)
    node = ExpPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(String(data=command_to_json(ReactiveDriveCommand(mode='STOP', reason='exp_planner_exit'))))
        node.destroy_node()
        if rclpy.ok():      # jazzy는 Ctrl+C 때 먼저 shutdown한다
            rclpy.shutdown()
