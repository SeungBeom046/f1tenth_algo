"""점군 deskew 순수 함수 (ROS 비의존).

한 프레임(약 100 ms) 동안 차가 움직이면 점마다 측정 위치가 다르다. 2.0 m/s면 0.2 m.
각 점을 측정 시각의 라이다 자세로 odom에 올린 뒤, 기준 시각(프레임 stamp)의 라이다 자세로 되돌린다.
자세는 프레임 처음과 끝 두 시각의 odom<-lidar 변환을 선형 보간(평면 x, y, yaw)한다.
"""

import math

import numpy as np


def interp_pose(pose0, pose1, alpha):
    """(x, y, yaw) 두 자세 사이 선형 보간. alpha는 배열 가능."""
    alpha = np.asarray(alpha)
    dyaw = math.atan2(math.sin(pose1[2] - pose0[2]), math.cos(pose1[2] - pose0[2]))
    return (pose0[0] + alpha * (pose1[0] - pose0[0]),
            pose0[1] + alpha * (pose1[1] - pose0[1]),
            pose0[2] + alpha * dyaw)


def deskew_points(xyz, t, t0, pose0, t1, pose1, t_ref):
    """xyz (N,3) 라이다 프레임 점, t (N,) 점별 시각 [s]. pose = odom에서 본 라이다 (x, y, yaw).

    반환: 기준 시각 t_ref의 라이다 프레임으로 옮긴 (N,3). z는 그대로 (평면 운동 가정).
    """
    span = t1 - t0
    if span <= 0:
        return xyz.copy()
    a = np.clip((t - t0) / span, 0.0, 1.0)
    px, py, pyaw = interp_pose(pose0, pose1, a)
    rx, ry, ryaw = interp_pose(pose0, pose1, (t_ref - t0) / span)
    c, s = np.cos(pyaw), np.sin(pyaw)
    ox = px + c * xyz[:, 0] - s * xyz[:, 1]
    oy = py + s * xyz[:, 0] + c * xyz[:, 1]
    cr, sr = math.cos(float(ryaw)), math.sin(float(ryaw))
    dx, dy = ox - float(rx), oy - float(ry)
    out = xyz.copy()
    out[:, 0] = cr * dx + sr * dy
    out[:, 1] = -sr * dx + cr * dy
    return out
