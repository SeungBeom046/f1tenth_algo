"""[1 인지] 다른 차 점을 제외한 벽 거리, ego 횡위치, 통로 기준 헤딩.

기본은 측면 섹터·분위값이다. corridor_guardrail은 실측 폭에 맞는 평행 직선을 선택한다.
직선 통로이며 차 헤딩이 통로 방향과 거의 나란하다고 가정한다.
"""
import math
from dataclasses import dataclass


@dataclass
class _WallLine:
    points: list
    heading: float
    distance: float
    side: int
    span: float


class Corridor:
    def __init__(self, p):
        self.p = p
        self.left = None            # 이번 scan의 벽 거리 [m], 안 보이면 None
        self.right = None
        self.ego_lateral = 0.0      # 통로 중심 기준 횡위치 (왼쪽 양수). 무효면 직전 값
        self.W_est = None
        self.valid = False          # 이번 scan에서 양쪽 벽이 다 보였는가
        self.heading = 0.0          # 통로 방향 기준 차 헤딩 [rad] (왼쪽 양수). 벽 점 직선 맞춤, 못 구하면 직전 값
        self._guardrail_locked = False
        self.wall_idx = set()
        self.sector_idx = set()
        self._stamp = None
        self._init_stamp = None
        self._init_pose = None
        self._init_count = 0
        self._t_valid = None

    def update(self, points, exclude_idx=(), W_fixed=None, stamp=None):
        """W_fixed는 선행차용이며 ego_lateral = right − W/2로 계산한다.
        가드레일 모드는 양쪽 벽으로 초기화하고, 한쪽이 가려지면 직전 위치에 맞는 벽을 쓴다.
        기존 모드는 오른쪽 벽만 쓴다."""
        p = self.p
        self._stamp = stamp
        self.wall_idx = set()
        self.sector_idx = set()
        ex = set(exclude_idx)
        lo = math.radians(p.corridor_sector_min_deg)
        sector_max = p.corridor_sector_max_deg
        if p.corridor_guardrail and p.guardrail_local_view:
            sector_max = max(sector_max, 155.0)
        hi = math.radians(sector_max)
        sides = {1: [], -1: []}
        for q in points:
            if q.i in ex or not lo <= abs(q.ang) <= hi:
                continue
            if q.x - p.lidar_to_front > p.min_point_x:
                sides[1 if q.ang > 0 else -1].append(q)
                self.sector_idx.add(q.i)
        if p.corridor_guardrail:
            return self._guardrail_update(sides, W_fixed)
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

    def _guardrail_update(self, sides, W_fixed):
        """벽 선 맞춤 + 짧은 무효 유지(corridor_hold_s) + 오래 무효면 재초기화(corridor_relock_s).

        실차 2026-11-18: 주행 중 벽이 한 프레임만 무효여도 정지했고, 선 뒤 같은 scan이라 영영 무효였다."""
        self._guardrail_fit(sides, W_fixed)
        p, t = self.p, self._stamp
        if self.valid:
            self._t_valid = t
        elif self._guardrail_locked and t is not None and self._t_valid is not None:
            gap = t - self._t_valid
            if 0.0 < p.corridor_hold_s and gap <= p.corridor_hold_s:
                self.valid = True       # 직전 횡위치·헤딩 유지 (left/right는 None)
            elif 0.0 < p.corridor_relock_s < gap:
                self._guardrail_locked = False
                self._init_count = 0
        return self

    def _guardrail_fit(self, sides, W_fixed):
        """실측 폭과 평행성이 맞는 벽만 사용한다. 다른 복도 벽으로 재초기화하지 않는다."""
        W = W_fixed if W_fixed is not None else self.p.W
        self.valid = False
        self.left = self.right = None
        if W is None or W <= 0:
            self._init_count = 0
            return self
        left = self._wall_lines(sides[1], 1, W)
        right = self._wall_lines(sides[-1], -1, W)
        parallel_tol = math.radians(5 if self.p.guardrail_local_view else 2)
        pairs = [(l, r) for l in left for r in right
                 if abs(l.distance + r.distance - W) <= self.p.corridor_width_tol
                 and abs(l.heading - r.heading) <= parallel_tol]
        local = self.p.guardrail_local_view and self._guardrail_locked
        if local:
            pairs = [(l, r) for l, r in pairs
                     if (abs(l.heading - self.heading) <= math.radians(5)
                         and abs(r.heading - self.heading) <= math.radians(5))
                     or (min(l.span, r.span) >= 3 * self.p.corridor_min_span
                         and min(len(l.points), len(r.points)) >= 3 * self.p.corridor_min_points)]
        if pairs:
            l, r = min(pairs, key=lambda pair: (
                self._continuity_cost(pair[0], W) + self._continuity_cost(pair[1], W) if local else 0.0,
                -min(len(pair[0].points), len(pair[1].points)) if self.p.guardrail_local_view and not local else 0,
                abs(pair[0].distance + pair[1].distance - W)
                + W * abs(pair[0].heading - pair[1].heading),
                -len(pair[0].points) - len(pair[1].points)))
            selected = [l, r]
        elif self._guardrail_locked:
            # 한쪽이 차에 가려지면 직전 위치·방향과 이어지는 벽만 쓴다.
            # scan 사이의 횡 이동을 허용하되 먼 복도 벽으로 넘어가지 않는다.
            selected = []
            candidates = sorted(right + left, key=lambda line: line.span, reverse=True)
            if self.p.guardrail_local_view:
                candidates = sorted(candidates, key=lambda line: (self._continuity_cost(line, W), -line.span))
            for line in candidates:
                expected = W / 2 - line.side * self.ego_lateral
                distance_ok = abs(line.distance - expected) <= self.p.corridor_width_tol
                heading_ok = abs(line.heading - self.heading) <= math.radians(5)
                if distance_ok and heading_ok:
                    selected = [line]
                    break
            if not selected:
                return self
        else:
            self._init_count = 0
            return self
        previous_heading = self.heading
        # 가까운 낮은 레일의 윗면 점보다 길게 보이는 벽 선의 방향을 우선한다.
        self.heading = max(selected, key=lambda line: line.span).heading
        c = math.cos(self.heading)
        distances = {line.side: self._dist(line.points, c) for line in selected}
        self.left, self.right = distances.get(1), distances.get(-1)
        if self.left is None:
            self.left = W - self.right
        if self.right is None:
            self.right = W - self.left
        if abs(self.left + self.right - W) > self.p.corridor_width_tol:
            self.heading = previous_heading
            self.left = self.right = None
            self._init_count = 0
            return self
        self.ego_lateral = self.right - W / 2 if W_fixed is not None else (self.right - self.left) / 2
        self.W_est = W
        if self.p.guardrail_local_view and not self._guardrail_locked:
            if self._stamp is None or self._stamp != self._init_stamp:
                previous = self._init_pose
                consistent = (self._init_count > 0 and previous is not None
                              and abs(self.heading - previous[0]) <= math.radians(5)
                              and abs(self.ego_lateral - previous[1]) <= self.p.corridor_width_tol)
                self._init_count = self._init_count + 1 if consistent else 1
                self._init_pose = (self.heading, self.ego_lateral)
                self._init_stamp = self._stamp
            if self._init_count < 3:
                return self
        self.valid = self._guardrail_locked = True
        self.wall_idx = {q.i for line in selected for q in line.points}
        return self

    @property
    def initializing(self):
        return self.p.guardrail_local_view and not self._guardrail_locked and self._init_count > 0

    def _continuity_cost(self, line, W):
        expected_distance = W / 2 - line.side * self.ego_lateral
        return W * abs(line.heading - self.heading) + abs(line.distance - expected_distance)

    def _wall_lines(self, pts, side, W):
        """가까운 소수 가드레일 점을 많은 복도 벽 점과 분리한다. 난수를 쓰지 않는다."""
        p = self.p
        if len(pts) < p.corridor_min_points:
            return []

        def fit(inliers):
            if len(inliers) < p.corridor_min_points:
                return None
            xs = sorted(q.x for q in inliers)
            span = xs[-1] - xs[0]
            if span < p.corridor_min_span:
                return None
            slope = self._slope(inliers)
            if slope is None or abs(slope) > math.tan(math.radians(35)):
                return None
            offset = sum(q.y - slope * q.x for q in inliers) / len(inliers)
            distance = side * offset / math.sqrt(1 + slope * slope)
            if not 0.05 < distance < W + p.corridor_width_tol:
                return None
            if max(b - a for a, b in zip(xs, xs[1:])) > p.corridor_min_span:
                return None
            tol = p.corridor_line_tol * math.sqrt(1 + slope * slope)
            if any(abs(q.y - slope * q.x - offset) > tol for q in inliers):
                return None
            return _WallLine(inliers, -math.atan(slope), distance, side, span)

        whole = fit(pts)
        if whole is not None:
            return [whole]
        # 각도 전체의 표본과 가까운 점의 표본을 함께 사용한다.
        indices = {k * (len(pts) - 1) // 15 for k in range(16)}
        near = sorted(range(len(pts)), key=lambda i: abs(pts[i].y - pts[i].x * math.tan(-self.heading)))
        near = near[:p.corridor_min_points * 2]
        indices.update(near[k * (len(near) - 1) // 7] for k in range(8))
        seeds = [pts[i] for i in sorted(indices)]
        lines = {}
        for i, a in enumerate(seeds):
            for b in seeds[i + 1:]:
                if abs(b.x - a.x) < p.corridor_min_span:
                    continue
                slope = (b.y - a.y) / (b.x - a.x)
                if abs(slope) > math.tan(math.radians(35)):
                    continue
                offset = a.y - slope * a.x
                tol = p.corridor_line_tol * math.sqrt(1 + slope * slope)
                inliers = [q for q in pts if abs(q.y - slope * q.x - offset) <= tol]
                line = fit(inliers)
                if line is not None:
                    key = (round(line.distance / p.corridor_line_tol), round(line.heading / math.radians(3)))
                    if key not in lines or len(inliers) > len(lines[key].points):
                        lines[key] = line
        return sorted(lines.values(), key=lambda line: line.distance)

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
