"""exp 모드 최종 명령 판단 (ROS 비의존). exp_drive_mux 노드와 시뮬이 같은 함수를 쓴다.

우선순위 (높은 순): 비상 정지(AUTO 해제·조이스틱 개입) > 발행자 점검 > 조이스틱 끊김 > 정지선 > AEB(래치) >
입력 끊김 > 정상. 노드는 입력을 모아 tick()을 부르고, 결과에 따라 발행·기록만 한다.

래치 규칙(기존 노드와 같음): abort(reason)은 지금 래치된 사유와 같을 때만 무시한다. stop_line·aeb는
래치가 없을 때만 걸리고, joy_timeout·foreign_publisher는 다른 사유 위에 덮어쓸 수 있다.
"""
from dataclasses import dataclass, field
from typing import List, Optional

from my_algo.vehicle_utils import sanitize_rpm, speed_to_erpm, steering_to_servo


@dataclass
class MuxConfig:
    joy_timeout: float = 0.5
    require_joy: bool = True
    aeb_latch: bool = True
    input_timeout: float = 0.3
    stop_distance: float = 7.5
    require_preflight: bool = True
    step_x: float = 0.0        # Exp0 i 속도 계단: AUTO 후 주행 거리 >= step_x이면 속도를 step_v 이하로 (0 = 끔)
    step_v: float = 0.0


@dataclass
class MuxState:
    latched: str = ''          # 회차 끝까지 유지되는 정지 사유
    was_running: bool = False
    has_driven: bool = False


@dataclass
class MuxInputs:
    now: float
    auto_mode: bool
    joy_active: bool
    preflight_ok: bool
    last_joy: Optional[float]
    aeb: bool
    v_cmd: float
    v_time: Optional[float]
    steer: float
    steer_time: Optional[float]
    travel: float


@dataclass
class MuxOutput:
    send: bool                 # False면 아무것도 발행하지 않는다 (사람이 잡고 있음)
    speed: float = 0.0
    steering: float = 0.0
    reason: str = ''
    aborts: List[str] = field(default_factory=list)   # 이번 틱에 새로 래치된 사유 (순서대로 기록)


def tick(cfg, st, inp):
    """한 번의 판단. st(MuxState)를 갱신하고 MuxOutput을 돌려준다."""
    aborts = []

    def abort(reason):
        if st.latched == reason:
            return
        st.latched = reason
        aborts.append(reason)

    # 1. 비상 정지: 사람이 잡고 있으면 아무것도 보내지 않는다 (joy_controller가 0을 보냄)
    if inp.joy_active or not inp.auto_mode:
        if st.was_running:
            st.was_running = False
            return MuxOutput(True, 0.0, 0.0, 'manual_or_auto_off', aborts)
        return MuxOutput(False, aborts=aborts)
    st.was_running = True
    # 1-1. 발행자 점검
    if cfg.require_preflight and not inp.preflight_ok:
        if st.has_driven:
            abort('foreign_publisher')
        elif not st.latched:
            return MuxOutput(True, 0.0, 0.0, 'preflight_failed', aborts)
    # 2. 데드맨
    if cfg.require_joy and (inp.last_joy is None or inp.now - inp.last_joy > cfg.joy_timeout):
        abort('joy_timeout')
    # 3. 정지선
    if inp.travel > cfg.stop_distance and not st.latched:
        abort('stop_line')
    # 4. AEB
    if inp.aeb and cfg.aeb_latch and not st.latched:
        abort('aeb')
    if st.latched:
        return MuxOutput(True, 0.0, inp.steer, st.latched, aborts)
    if inp.aeb:
        return MuxOutput(True, 0.0, inp.steer, 'aeb', aborts)
    # 5. 입력 끊김
    if inp.v_time is None or inp.now - inp.v_time > cfg.input_timeout:
        return MuxOutput(True, 0.0, inp.steer, 'stale_speed', aborts)
    if inp.steer_time is None or inp.now - inp.steer_time > cfg.input_timeout:
        return MuxOutput(True, 0.0, 0.0, 'stale_steering', aborts)
    # 6. 정상
    st.has_driven = True
    if cfg.step_v > 0.0 and inp.travel >= cfg.step_x:
        # Exp0 i: 방법과 무관한 속도 계단 (bringup test_step_v:=, 비교 실험에서는 끔)
        return MuxOutput(True, min(inp.v_cmd, cfg.step_v), inp.steer, 'speed_step', aborts)
    return MuxOutput(True, inp.v_cmd, inp.steer, '', aborts)


def to_vesc(speed, steering, max_steering):
    """최종 속도·조향 → (ERPM, 서보 위치, 실제로 보낸 속도 m/s). 최소 RPM 규칙(sanitize_rpm) 포함."""
    steering = max(-max_steering, min(max_steering, steering))
    rpm = sanitize_rpm(speed_to_erpm(max(0.0, speed)))
    return rpm, steering_to_servo(steering), rpm / speed_to_erpm(1.0)
