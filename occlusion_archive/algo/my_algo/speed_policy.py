"""속도 계층 순수 로직 (ROS 비의존). B0/B1/B2/P가 모두 이 모듈 하나를 쓴다.

파이프라인 위치: phantom 배치(d 또는 ρ) → (이 모듈) → AEB override → VESC
    v_cap(d)            B2·P 공통 속도 상한
    b1_speed(ρ)         B1 면적 비례 속도
    MinSpeedPolicy      플랫폼 최소 주행 속도 처리 (모든 방법 공통)

실험 태그 요약:
    [EXP-FIXED]   t_r, a_b, s_m, v_ref, 최소 속도 정책 → config/common.yaml (3단계)
    [EXP-TUNE]    v_imp (하한 = v_min), k
    [EXP0-MEASURE] t_r, a_b
"""

import math

from my_algo.vehicle_utils import ERPM_GAIN, MIN_DRIVING_RPM

# 플랫폼 최소 안정 주행 속도 [m/s]. vehicle_utils의 MIN_DRIVING_RPM에서 환산하므로 값은 한 곳에만 있다.
# 4000 ERPM / 4614 = 0.867 m/s. 이 값은 v_imp의 하한이 된다 (아래 check_v_imp).
V_MIN = MIN_DRIVING_RPM / ERPM_GAIN


def v_cap_raw(d, t_r, a_b, s_m, v_imp):
    """v*t_r + (v^2 - v_imp^2)/(2 a_b) = d - s_m 의 양의 해. d >= s_m에서만 의미가 있다."""
    return a_b * (-t_r + math.sqrt(t_r * t_r + 2.0 * (d - s_m) / a_b + (v_imp / a_b) ** 2))


def v_cap(d, t_r, a_b, s_m, v_imp):
    """최근접 phantom까지 거리 d [m, 차체 앞 끝 기준]에서의 속도 상한.

    d <= s_m이면 v_cap(s_m)으로 고정한다. v_imp로 두면 s_m 안쪽에서 오히려 속도가 올라가는
    불연속이 생기기 때문이다. d가 None(phantom 없음)이면 상한 없음(inf).
    """
    if d is None or math.isinf(d):
        return math.inf
    return v_cap_raw(max(d, s_m), t_r, a_b, s_m, v_imp)


def b1_speed(rho, v_ref, k):
    """B1: 전방 ROI 미관측 비율 rho로 v = v_ref (1 - k rho). 음수는 0으로 자른다."""
    return max(0.0, v_ref * (1.0 - k * rho))


def check_v_imp(v_imp, v_min=V_MIN):
    """v_imp < v_min이면 경고 문자열, 아니면 None.

    v_imp >= v_min이어야 '최소 속도 유지' 정책에서 충돌 지점 도달 속도 <= v_imp가 보장된다.
    """
    if v_imp < v_min - 1e-9:
        return (f'v_imp={v_imp:.3f} < v_min={v_min:.3f}: 최소 속도 유지 정책에서 '
                f'충돌 지점 도달 속도 <= v_imp 보장이 깨짐. v_imp를 v_min 이상으로 설정할 것')
    return None


class MinSpeedPolicy:
    """목표 속도를 플랫폼이 낼 수 있는 속도로 바꾼다. 모든 방법에 같은 정책을 쓴다. [EXP-FIXED]

    mode='hold' (기본): 0 < 요청 < v_min이면 v_min으로 올리고 플래그. 요청 <= 0도 v_min
        (정지는 AEB와 비상 정지만 담당. 정지·재출발 교착이 없다).
    mode='hysteresis' (비교용, 기본 off): 요청 < v_min이면 0, 정지 상태에서는
        요청 > v_min + restart_margin일 때만 재출발.
    반환: (명령 속도, 플래그). 플래그는 'clamped_to_vmin' / 'stopped_below_vmin' / ''.
    """

    def __init__(self, mode='hold', v_min=V_MIN, restart_margin=0.1):
        if mode not in ('hold', 'hysteresis'):
            raise ValueError("mode must be 'hold' or 'hysteresis'")
        self.mode = mode
        self.v_min = v_min
        self.restart_margin = restart_margin
        self.stopped = False

    def reset(self):
        self.stopped = False

    def apply(self, v_request):
        if self.mode == 'hold':
            if v_request < self.v_min:
                return self.v_min, 'clamped_to_vmin'
            return v_request, ''
        if self.stopped:
            if v_request > self.v_min + self.restart_margin:
                self.stopped = False
                return v_request, ''
            return 0.0, 'stopped_below_vmin'
        if v_request < self.v_min:
            self.stopped = True
            return 0.0, 'stopped_below_vmin'
        return v_request, ''


def command_speed(v_ref, v_limit, policy):
    """최종 목표 속도 = 정책(min(v_ref, v_limit)). v_limit은 v_cap 또는 B1 속도.

    반환: (정책 전 속도, 정책 후 명령 속도, 플래그). 정책 전 값은 '명령 기준 손실' 계산에 쓴다.
    """
    raw = min(v_ref, v_limit)
    cmd, flag = policy.apply(raw)
    return raw, cmd, flag
