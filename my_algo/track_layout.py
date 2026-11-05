"""트랙 배치 계산 (ROS 비의존). scenario yaml의 row_x_start·box_len·gap_width·n_boxes가 유일 출처다.

시뮬(월드 생성), trigger_node(충돌 지점), 분석(주차열 창)이 모두 이 함수로 row_x_end·collision_x를 얻는다.
yaml에 적힌 row_x_end·collision_x는 기록용 사본이며, 계산값과 다르면 check()가 경고 문자열을 돌려준다.

좌표: 트랙 x = AUTO 시작 시 에고 차체 앞 끝 = 0, 진행 방향 +.
    박스 i (0부터) 구간 = [row_x_start + i (box_len + gap_width), + box_len]
    row_x_end   = row_x_start + n box_len + (n − 1) gap_width          (n = 0이면 row_x_start)
    collision_x = P1–P2 틈 중앙 = row_x_start + box_len + gap_width / 2 (n < 2이면 None)
"""
from dataclasses import dataclass
from typing import List, Optional, Tuple

TOL = 1e-3   # yaml 사본과 계산값 허용 차이 [m]


@dataclass(frozen=True)
class Layout:
    boxes: Tuple[Tuple[float, float], ...]   # 박스별 (x 시작, x 끝)
    row_x_start: float
    row_x_end: float
    collision_x: Optional[float]


def layout(row_x_start, box_len, gap_width, n_boxes):
    n = int(n_boxes)
    boxes = tuple((row_x_start + i * (box_len + gap_width),
                   row_x_start + i * (box_len + gap_width) + box_len) for i in range(n))
    row_x_end = boxes[-1][1] if n else row_x_start
    collision_x = row_x_start + box_len + 0.5 * gap_width if n >= 2 else None
    return Layout(boxes, row_x_start, row_x_end, collision_x)


def from_params(p):
    """scenario yaml의 ros__parameters dict → Layout."""
    return layout(float(p['row_x_start']), float(p['box_len']), float(p['gap_width']), int(p['n_boxes']))


def check(p) -> List[str]:
    """yaml 사본(row_x_end, collision_x)과 계산값이 다르면 경고 목록."""
    lay = from_params(p)
    warnings = []
    if abs(float(p['row_x_end']) - lay.row_x_end) > TOL:
        warnings.append(f"row_x_end: yaml {p['row_x_end']} ≠ 계산 {lay.row_x_end:.3f}")
    if lay.collision_x is not None and abs(float(p['collision_x']) - lay.collision_x) > TOL:
        warnings.append(f"collision_x: yaml {p['collision_x']} ≠ 계산 {lay.collision_x:.3f}")
    return warnings
