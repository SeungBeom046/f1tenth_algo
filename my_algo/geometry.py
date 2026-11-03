"""차량 치수 로더 (ROS 비의존). config/vehicle_geometry.yaml을 모든 exp 노드가 이 함수로 읽는다.

노드 안에 치수를 하드코딩하지 않기 위한 단일 진입점. [EXP-FIXED]
"""

from dataclasses import dataclass


@dataclass(frozen=True)
class VehicleGeometry:
    wheelbase: float
    rear_axle_to_front: float
    rear_axle_to_rear: float
    body_width: float
    footprint: tuple  # ((x, y), ...) base_link 기준 다각형, 반시계

    @property
    def half_width(self):
        return 0.5 * self.body_width


def load_geometry(path):
    import yaml  # ROS 환경에는 python3-yaml이 있다. 개발 PC 테스트에서는 없을 수 있어 지연 import
    with open(path, encoding='utf-8') as f:
        data = yaml.safe_load(f)
    p = data['vehicle_geometry']['ros__parameters']
    flat = [float(v) for v in p['footprint']]
    if len(flat) < 6 or len(flat) % 2:
        raise ValueError('footprint는 x, y 쌍 3개 이상이어야 한다')
    return VehicleGeometry(
        wheelbase=float(p['wheelbase']),
        rear_axle_to_front=float(p['rear_axle_to_front']),
        rear_axle_to_rear=float(p['rear_axle_to_rear']),
        body_width=float(p['body_width']),
        footprint=tuple(zip(flat[0::2], flat[1::2])),
    )


def point_polygon_distance(px, py, polygon):
    """점과 다각형(내부 포함) 사이 최소 거리 [m]. 내부면 0. 분석의 최소 거리·충돌 판정용."""
    inside = False
    n = len(polygon)
    best = float('inf')
    for i in range(n):
        x1, y1 = polygon[i]
        x2, y2 = polygon[(i + 1) % n]
        if (y1 > py) != (y2 > py):
            xc = x1 + (py - y1) * (x2 - x1) / (y2 - y1)
            if px < xc:
                inside = not inside
        dx, dy = x2 - x1, y2 - y1
        seg = dx * dx + dy * dy
        t = 0.0 if seg == 0 else max(0.0, min(1.0, ((px - x1) * dx + (py - y1) * dy) / seg))
        cx, cy = x1 + t * dx, y1 + t * dy
        best = min(best, ((px - cx) ** 2 + (py - cy) ** 2) ** 0.5)
    return 0.0 if inside else best
