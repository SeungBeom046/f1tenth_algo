"""exp 모드 조향 노드. 기존 LiDAR planner를 상속해 조향만 쓰고, 방법과 무관하게 같은 조향이 나오게 고정한다.

역할: 좌측 벽 따라가기(WALL_FOLLOW) 조향각을 낸다. 속도(rpm) 출력은 exp_drive_mux가 쓰지 않는다.
    기존 planner 대비 바뀌는 것 (상속으로만 바꾸고 drive 모드 파일은 수정하지 않는다):
    1. 모드 고정: 항상 WALL_FOLLOW. OBSTACLE_AVOIDANCE 전환 없음
       (돌출 차량을 조향으로 피하면 안전 결과를 속도 계획 효과로 해석할 수 없기 때문)
    2. 좌측 벽만 사용: 오른쪽(주차 박스, 틈) 거리는 조향 오차에 넣지 않는다
    3. lookahead 고정: exp_lookahead_distance (실측 속도에 따라 바뀌지 않게)
    4. planner 정지(STOP) 때 조향은 직전 값 유지 (기존은 0으로 보냄)
    5. 앞 끝 기준: lidar_to_front_edge (vehicle_geometry.yaml)
파이프라인 위치: [/scan] → (exp_steering_node) → [exp_drive_mux]
구독: /scan, /vesc/odom, /autonomous_mode, /joy_active (부모 클래스, launch에서 remap)
발행: /reactive/lidar_command → launch에서 /exp/steering_command로 remap (JSON ReactiveDriveCommand)
파라미터 (config/common.yaml, 전부 [EXP-FIXED], 방법 간 동일):
    target_wall_distance (좌측 벽 목표 거리, README에 계산식), wall_follow_gain,
    wall_error_filter_alpha, steering_deadband, max_steering, steer_rate_limit, exp_lookahead_distance
"""

import rclpy

from my_algo.geometry import load_geometry
from my_algo.lidar_reactive_drive import LidarReactiveDriveNode
from my_algo.ros_util import default_geometry_path
from my_algo.vehicle_utils import ReactiveDriveCommand


class ExpSteeringNode(LidarReactiveDriveNode):
    def __init__(self):
        super().__init__()
        self.declare_parameter('exp_lookahead_distance', 4.4)
        self.declare_parameter('vehicle_geometry_file', '')
        self.exp_lookahead_distance = float(self.get_parameter('exp_lookahead_distance').value)
        geom = load_geometry(self.get_parameter('vehicle_geometry_file').value
                             or default_geometry_path())
        self.lidar_to_bumper_dist = geom.lidar_to_front_edge
        self.active_lookahead_distance = self.exp_lookahead_distance
        self.current_mode = 'WALL_FOLLOW'
        self.get_logger().info(
            f'exp 조향: WALL_FOLLOW 고정(좌측 벽), lookahead {self.exp_lookahead_distance} m, '
            f'좌측 벽 목표 {self.target_wall_distance} m, gain {self.wall_follow_gain}')

    def adaptive_lookahead_distance(self):
        return self.exp_lookahead_distance

    def select_reactive_mode(self, gaps, corridor, cone_track, path_blocked):
        self.current_mode = 'WALL_FOLLOW'
        return 'WALL_FOLLOW'

    def plan_wall_follow(self, corridor):
        if corridor.get('left_dist') is None:
            return ReactiveDriveCommand(steering_rad=self.previous_steering, rpm=0.0,
                                        mode='WALL_FOLLOW', confidence=0.0,
                                        reason='left_wall_lost_hold')
        left_only = dict(corridor, right_dist=None)
        return super().plan_wall_follow(left_only)

    def publish_command(self, command):
        if command.mode == 'STOP':
            command.steering_rad = self.previous_steering
        super().publish_command(command)


def main(args=None):
    rclpy.init(args=args)
    node = ExpSteeringNode()
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
