"""발행자 점검 노드 (exp 모드). 에고 핵심 토픽에 예상 외 발행자(#2 등 다른 호스트)가 있으면 회차를 막는다.

역할: 시작 후 settle_sec 동안 그래프 탐색을 기다린 뒤 1 Hz로 규칙(my_algo/preflight.py)을 검사한다.
파이프라인 위치: [ROS 그래프] → (publisher_guard_node) → [exp_drive_mux, record_run.sh]
구독: 없음 (rclpy 그래프 API get_publishers_info_by_topic 사용)
발행 (transient_local, 1 Hz):
    /exp/preflight_ok  std_msgs/Bool   true = 통과 (mode=warn이면 위반이 있어도 true)
    /exp/preflight     std_msgs/String JSON {ok, mode, problems, publishers}
파라미터 (config/preflight.yaml): rules (규칙 문자열 목록), mode (enforce | warn), settle_sec
실험 관련: 위반 시 exp_drive_mux가 AUTO 진입 거부(회차 중이면 정지 + 폐기), record_run.sh가 기록 거부.
    허용 노드 이름은 세션 1 check_vehicle.sh 결과로 확정 (TBD, docs/OPEN_ITEMS.md).
"""

import json

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, String

from my_algo.preflight import check, full_name, parse_rule
from my_algo.ros_util import publish_string

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


class PublisherGuardNode(Node):
    def __init__(self):
        super().__init__('publisher_guard_node')
        self.declare_parameter('rules', [''])
        self.declare_parameter('mode', 'enforce')
        self.declare_parameter('settle_sec', 3.0)
        self.rules = [parse_rule(r) for r in self.get_parameter('rules').value if r.strip()]
        self.mode = self.get_parameter('mode').value
        self.settle = float(self.get_parameter('settle_sec').value)
        self.ok_pub = self.create_publisher(Bool, '/exp/preflight_ok', LATCHED)
        self.info_pub = self.create_publisher(String, '/exp/preflight', LATCHED)
        self.started = self.get_clock().now()
        self.last_problems = None
        self.publish(False, ['그래프 탐색 대기 중'], {})
        self.create_timer(1.0, self.tick)
        self.get_logger().info(f'발행자 점검: 규칙 {len(self.rules)}개, mode={self.mode}')

    def tick(self):
        if (self.get_clock().now() - self.started).nanoseconds * 1e-9 < self.settle:
            return
        observed = {r.topic: [full_name(i.node_namespace, i.node_name)
                              for i in self.get_publishers_info_by_topic(r.topic)]
                    for r in self.rules}
        problems = check(self.rules, observed)
        if problems != self.last_problems:
            for p in problems:
                self.get_logger().error(f'발행자 점검 위반: {p}')
            if not problems:
                self.get_logger().info('발행자 점검 통과')
            self.last_problems = problems
        self.publish(not problems or self.mode == 'warn', problems, observed)

    def publish(self, ok, problems, observed):
        msg = Bool()
        msg.data = bool(ok)
        self.ok_pub.publish(msg)
        publish_string(self.info_pub, json.dumps(
            {'ok': bool(ok), 'mode': self.mode, 'problems': problems, 'publishers': observed},
            ensure_ascii=False))


def main(args=None):
    rclpy.init(args=args)
    node = PublisherGuardNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
