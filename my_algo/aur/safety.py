"""안전 거리와 속도 한계, 식 (6)~(11). d는 앞범퍼 → 선행차 뒷면 거리."""
import math


def d_safe(v_e, v_b, p):
    """식 (6): 선행차가 감속도 a로 급정지해도 여유 m을 남기고 설 수 있는 최소 거리."""
    v_b = max(0.0, v_b)
    return v_e * p.t_d + (v_e * v_e - v_b * v_b) / (2.0 * p.a_brake) + p.m_stop


def h(d, v_e, v_b, p):
    """식 (7): 안전 여유. h ≥ 0이면 안전."""
    return d - d_safe(v_e, v_b, p)


def v_lim(d, v_b, p):
    """식 (8): h ≥ ε를 지키는 최대 ego 속도."""
    v_b = max(0.0, v_b)
    a, td = p.a_brake, p.t_d
    disc = a * a * td * td + 2.0 * a * (d - p.m_stop - p.eps_follow) + v_b * v_b
    return max(0.0, -a * td + math.sqrt(disc)) if disc > 0.0 else 0.0


def d_star(v_b, p):
    """식 (9): v_E = v_B로 따라갈 수 있는 최근접 거리."""
    return max(0.0, v_b) * p.t_d + p.m_stop + p.eps_follow


def probe_feasible(v_b, p):
    """식 (10): 최근접 추종 거리가 선행차 반응 범위 안에 들어오는가."""
    return d_star(v_b, p) < p.d_y_max


def pass_window(w, c):
    """식 (11): (양보해야만 추월 가능한 최소 통로 폭, 양보 없이도 추월 가능한 폭)."""
    return 2.0 * w + 3.0 * c, 3.0 * w + 4.0 * c


def lateral_pass_ok(W, y_B, w, c):
    """선행차 왼쪽에 ego 폭 + 양쪽 여유가 남는가."""
    return W / 2.0 - y_B - w / 2.0 >= w + 2.0 * c


def lane_offset(W, p):
    """차선 횡 오프셋 δ = W/2 − w/2 − c (파라미터로 고정했으면 그 값)."""
    if p.lane_offset is not None:
        return p.lane_offset
    return max(0.0, W / 2.0 - p.w_car / 2.0 - p.c_lat)
