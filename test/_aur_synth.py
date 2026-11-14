"""aur 테스트용 합성 입력: 직선 통로 scan(광선 투사)과 1차원 2대 폐루프."""
import math

from my_algo.aur import safety
from my_algo.aur.ego_fsm import EgoFSM, Obs
from my_algo.aur.leader_logic import Leader
from my_algo.aur.tracker import scan_to_points

CAR_LEN = 0.50


def box(x0, x1, y0, y1):
    return [((x0, y0), (x1, y0)), ((x1, y0), (x1, y1)), ((x1, y1), (x0, y1)), ((x0, y1), (x0, y0))]


def scan(W, ego_y, boxes=(), n=720):
    """라이다가 통로 중심에서 ego_y(왼쪽 양수)에 있고 헤딩은 통로 방향. boxes는 라이다 좌표계 선분 목록."""
    segs = [((-20.0, W / 2 - ego_y), (20.0, W / 2 - ego_y)),
            ((-20.0, -W / 2 - ego_y), (20.0, -W / 2 - ego_y))]
    for b in boxes:
        segs += b
    inc = 2 * math.pi / n
    ranges = []
    for i in range(n):
        a = -math.pi + i * inc
        dx, dy = math.cos(a), math.sin(a)
        best = float('inf')
        for (x1, y1), (x2, y2) in segs:
            ex, ey = x2 - x1, y2 - y1
            den = dx * ey - dy * ex
            if abs(den) > 1e-12:
                r = (x1 * ey - y1 * ex) / den
                u = (x1 * dy - y1 * dx) / den
                if r > 0 and 0 <= u <= 1:
                    best = min(best, r)
        ranges.append(best)
    return scan_to_points(ranges, -math.pi, inc, 0.05, 12.0)


def run(p, method, intent, run_id='t', W=1.0, gap0=1.5, d_y=None, T=30.0, dt=0.1):
    """참값 관측으로 EgoFSM과 Leader를 같이 돌린다. 속도는 ±a_brake로만 변하고 횡위치는 0.4 m/s로 y_ref를 따라간다."""
    fsm, lead = EgoFSM(p, method), Leader(p, intent, run_id)
    if d_y is not None:
        lead.d_y = d_y
    xe, ye, ve = 0.0, 0.0, 0.0                  # ego 앞범퍼
    xl, yl, vl = gap0, 0.0, 0.0                 # 선행차 뒷면
    log = []
    for k in range(int(T / dt)):
        t = k * dt
        d = xl - xe
        ahead = d > 0
        front_rel = (xl + CAR_LEN) - (xe - CAR_LEN)
        v_cmd, y_ref, state, _ = fsm.step(Obs(t, ve, ahead, d, yl, vl, ye, W, front_rel))
        vl_cmd, yl_ref = lead.step(t, vl, d if ahead else None, W)
        log.append(dict(t=t, state=state, d=d, h=fsm.h, lam=fsm.belief.lam, sep=abs(ye - yl), tracked=ahead,
                        overlap=-CAR_LEN < d < CAR_LEN))
        ve += max(-p.a_brake * dt, min(p.a_brake * dt, v_cmd - ve))
        vl += max(-p.a_brake * dt, min(p.a_brake * dt, vl_cmd - vl))
        ye += max(-0.4 * dt, min(0.4 * dt, y_ref - ye))
        yl += max(-0.4 * dt, min(0.4 * dt, yl_ref - yl))
        xe += ve * dt
        xl += vl * dt
    return fsm, lead, log
