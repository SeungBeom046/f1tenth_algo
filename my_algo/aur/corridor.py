"""다른 차 점을 제외한 벽 거리와 ego 횡위치.

lidar_reactive_drive.detect_corridor와 같은 방식(측면 섹터, 분위값)이다.
가정(미확인): 차 헤딩이 통로 방향과 거의 나란하다.
"""
import math


class Corridor:
    def __init__(self, p):
        self.p = p
        self.left = None            # 이번 scan의 벽 거리 [m], 안 보이면 None
        self.right = None
        self.ego_lateral = 0.0      # 통로 중심 기준 횡위치 (왼쪽 양수). 무효면 직전 값
        self.W_est = None
        self.valid = False          # 이번 scan에서 양쪽 벽이 다 보였는가

    def update(self, points, exclude_idx=()):
        p = self.p
        ex = set(exclude_idx)
        lo = math.radians(p.corridor_sector_min_deg)
        hi = math.radians(p.corridor_sector_max_deg)
        sides = {1: [], -1: []}
        for q in points:
            if q.i in ex or not lo <= abs(q.ang) <= hi:
                continue
            if q.x - p.lidar_to_front > p.min_point_x:
                sides[1 if q.ang > 0 else -1].append(abs(q.y))
        self.left, self.right = self._dist(sides[1]), self._dist(sides[-1])
        self.valid = self.left is not None and self.right is not None
        if self.valid:
            self.ego_lateral = (self.right - self.left) / 2.0
            self.W_est = self.left + self.right
        return self

    def _dist(self, values):
        if len(values) < self.p.corridor_min_points:
            return None
        values.sort()
        return values[int((len(values) - 1) * self.p.corridor_wall_percentile)]
