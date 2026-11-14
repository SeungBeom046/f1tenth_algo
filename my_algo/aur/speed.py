"""m/s ↔ ERPM 변환과 최소 속도 규칙."""


def to_erpm(v, p):
    return v * p.erpm_gain


def from_erpm(erpm, p):
    return erpm / p.erpm_gain


def min_speed_rule(v_cmd, h, moving, p):
    """0 < v_cmd < v_min이면 그대로 보낼 수 없다 (sanitize_rpm이 v_min으로 올린다).

    h ≥ ε면 v_min 유지(hold), 아니면 0. 반환: (보낼 속도, 이 분기가 일어났는지).
    hold는 이미 달리는 중(moving)에만 한다. h는 현재 속도로 계산한 값이라
    정지 중에는 항상 크게 나오고, 그대로 hold하면 선 차 뒤에서 v_min으로 튀어나갔다 서기를 반복한다.
    moving은 호출하는 쪽이 히스테리시스(hold_on_speed / hold_off_speed)로 정한다.
    """
    if 0.0 < v_cmd < p.v_min:
        return (p.v_min if h >= p.eps_follow and moving else 0.0), True
    return max(0.0, v_cmd), False
