"""[1 인지] 다른 차 점을 제외한 벽 거리, ego 횡위치, 통로 기준 헤딩.

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
        self.heading = 0.0          # 통로 방향 기준 차 헤딩 [rad] (왼쪽 양수). 벽 점 직선 맞춤, 못 구하면 직전 값

    def update(self, points, exclude_idx=(), W_fixed=None):
        """W_fixed(실측 통로 폭)를 주면 오른쪽 벽만 쓴다: ego_lateral = right − W/2.
        선행차 전용이다. 후행차는 왼쪽으로 추월하므로 오른쪽 벽은 가려지지 않는다."""
        p = self.p
        ex = set(exclude_idx)
        lo = math.radians(p.corridor_sector_min_deg)
        hi = math.radians(p.corridor_sector_max_deg)
        sides = {1: [], -1: []}
        for q in points:
            if q.i in ex or not lo <= abs(q.ang) <= hi:
                continue
            if q.x - p.lidar_to_front > p.min_point_x:
                sides[1 if q.ang > 0 else -1].append(q)
        if W_fixed is not None:
            sides[1] = []
        slopes = [s for s in (self._slope(sides[1]), self._slope(sides[-1])) if s is not None]
        if slopes:
            # 차가 ψ만큼 틀어지면 차량 좌표계에서 벽은 기울기 tan(−ψ)인 직선이다.
            self.heading = -math.atan(sum(slopes) / len(slopes))
        # 벽까지 수직 거리 = 분위 |y| × cos ψ (헤딩이 0이면 기존 방식과 같다)
        c = math.cos(self.heading)
        self.left, self.right = self._dist(sides[1], c), self._dist(sides[-1], c)
        if W_fixed is not None:
            self.valid = self.right is not None
            self.W_est = W_fixed
            if self.valid:
                self.ego_lateral = self.right - W_fixed / 2.0
                self.left = W_fixed - self.right
            return self
        self.valid = self.left is not None and self.right is not None
        if self.valid:
            self.ego_lateral = (self.right - self.left) / 2.0
            self.W_est = self.left + self.right
        return self

    def _slope(self, pts):
        """벽 점의 최소제곱 직선 기울기 dy/dx. 점이 적거나 x 범위가 좁으면 None."""
        n = len(pts)
        if n < self.p.corridor_min_points:
            return None
        mx = sum(q.x for q in pts) / n
        my = sum(q.y for q in pts) / n
        sxx = sum((q.x - mx) ** 2 for q in pts)
        if sxx < 1e-4 * n:
            return None
        return sum((q.x - mx) * (q.y - my) for q in pts) / sxx

    def _dist(self, pts, cos_heading):
        if len(pts) < self.p.corridor_min_points:
            return None
        # 직선 위에서 |y − x·tan(−ψ)|가 원점에서의 y 절편이다. 점마다 절편을 구해 분위값을 쓴다.
        t = math.tan(-self.heading)
        values = sorted(abs(q.y - q.x * t) for q in pts)
        return values[int((len(values) - 1) * self.p.corridor_wall_percentile)] * cos_heading
