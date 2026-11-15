"""exp 노드 공통 ROS 도우미: TF 조회, 차량 치수 파일 경로, 스칼라 발행."""

import math
import os

from std_msgs.msg import Float64, String


def default_geometry_path():
    from ament_index_python.packages import get_package_share_directory
    return os.path.join(get_package_share_directory('my_algo'), 'config', 'vehicle_geometry.yaml')


def quat_to_yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def lookup_pose2d(buffer, target, source, stamp, timeout_sec=0.05):
    """target 프레임에서 본 source 원점의 (x, y, yaw). 실패하면 None."""
    from rclpy.duration import Duration
    from tf2_ros import TransformException
    try:
        tf = buffer.lookup_transform(target, source, stamp, timeout=Duration(seconds=timeout_sec))
    except TransformException:
        return None
    t = tf.transform.translation
    return (t.x, t.y, quat_to_yaw(tf.transform.rotation))


def lookup_matrix(buffer, target, source, stamp, timeout_sec=0.05):
    """target <- source 4x4 동차 변환 (numpy). 실패하면 None."""
    import numpy as np
    from rclpy.duration import Duration
    from tf2_ros import TransformException
    try:
        tf = buffer.lookup_transform(target, source, stamp, timeout=Duration(seconds=timeout_sec))
    except TransformException:
        return None
    q, t = tf.transform.rotation, tf.transform.translation
    w, x, y, z = q.w, q.x, q.y, q.z
    m = np.eye(4)
    m[:3, :3] = [[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                 [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                 [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]]
    m[:3, 3] = [t.x, t.y, t.z]
    return m


def publish_float(pub, value):
    msg = Float64()
    msg.data = float(value)
    pub.publish(msg)


def publish_string(pub, text):
    msg = String()
    msg.data = text
    pub.publish(msg)
