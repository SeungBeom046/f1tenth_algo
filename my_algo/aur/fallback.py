"""[1 인지] 레일 직선을 못 잡을 때의 대체 횡위치 (FTG식 빈 공간 검사).

벽 직선 모델 없이 차 옆·앞 구간에서 좌우로 가장 가까운 점까지의 거리로 횡위치를 잰다.
진행 경로 안에 가까운 점이 있으면 None (그때는 정지한다: 레일을 놓친 동안 앞차 추적도 꺼진다).
바깥 복도 벽을 레일로 잡지 않도록 폭 W 근처 안쪽 점만 쓴다.
"""


def free_space_lateral(points, W, p):
    """반환: 통로 중심 기준 횡위치 [m] (왼쪽 양수) 또는 None (경로 막힘·근거 없음)."""
    if W is None:
        return None
    half_path = p.w_car / 2.0 + p.c_lat
    reach = W / 2.0 + p.corridor_width_tol     # 차가 한쪽에 붙어도 반대 레일까지는 보되 바깥 벽은 버린다
    left, right = [], []
    for q in points:
        ahead = q.x - p.lidar_to_front
        if 0.0 < ahead < p.rail_fallback_clear and abs(q.y) < half_path:
            return None
        if -0.6 <= q.x <= 1.5 and 0.05 < abs(q.y) < W + p.corridor_width_tol:
            (left if q.y > 0 else right).append(abs(q.y))
    # 잡음 바깥값 하나가 거리를 정하지 않게 가까운 쪽 10% 분위값을 쓴다
    left = sorted(left)[len(left) // 10] if len(left) >= p.corridor_min_points else None
    right = sorted(right)[len(right) // 10] if len(right) >= p.corridor_min_points else None
    if left is not None and left > reach:
        left = None
    if right is not None and right > reach:
        right = None
    if left is not None and right is not None:
        if abs(left + right - W) > p.corridor_width_tol:
            return None
        return (right - left) / 2.0
    if right is not None:
        return right - W / 2.0
    if left is not None:
        return W / 2.0 - left
    return None
