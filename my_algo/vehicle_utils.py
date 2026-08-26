"""실차 F1TENTH 스택에서 공통으로 쓰는 차량 유틸리티.

역할:
    하드웨어 명령 변환과 작은 수학 유틸리티를 한 곳에 모은다.

사용 토픽:
    이 파일은 직접 subscribe/publish하지 않는다. 각 노드가
    /commands/motor/speed, /commands/servo/position으로 명령을 내기 전에
    변환 함수만 사용한다.

주요 튜닝 값:
    MIN_DRIVING_RPM:
        0이 아닌 최소 모터 명령. 출발 시 여전히 떨리면 키운다.
        낮은 RPM에서도 부드럽게 움직일 때만 줄인다.
    MAX_RPM:
        자율주행 최대 RPM. 직선 속도를 높이려면 키우고,
        더 안전하게 테스트하려면 줄인다.
"""

import json
import math
import sys
from dataclasses import asdict, dataclass


ERPM_GAIN = 4614.0
# 차량 실험으로 맞춘 속도-ERPM 변환 계수.
# speed_to_erpm: ERPM = m/s * ERPM_GAIN
# erpm_to_speed: m/s = ERPM / ERPM_GAIN
MIN_DRIVING_RPM = 2000.0
MIN_DRIVE_ERPM = MIN_DRIVING_RPM
MIN_DRIVE_SPEED_MS = MIN_DRIVING_RPM / ERPM_GAIN
DRIVE_SPEED_SCALE = 0.70
CRUISE_RPM = 5740.0
MAX_RPM = 12000.0

SERVO_CENTER = 0.5
SERVO_GAIN = 0.60
SERVO_MIN = 0.03
SERVO_MAX = 0.97

_STATUS_LINE_LEN = 0


@dataclass
class ReactiveDriveCommand:
    """autonomous_drive가 소비하는 reactive drive 출력."""

    steering_rad: float = 0.0
    rpm: float = 0.0
    mode: str = 'STOP'
    confidence: float = 0.0
    reason: str = ''

    def is_valid(self):
        """명령 값들이 유한하고 사용할 수 있는지 확인한다."""
        return (
            math.isfinite(self.steering_rad)
            and math.isfinite(self.rpm)
            and math.isfinite(self.confidence)
        )


def command_to_json(command):
    """ReactiveDriveCommand를 std_msgs/String 전송용 JSON으로 변환한다."""
    return json.dumps(asdict(command), separators=(',', ':'))


def command_from_json(payload):
    """std_msgs/String 데이터에서 ReactiveDriveCommand를 파싱한다."""
    try:
        data = json.loads(payload)
        return ReactiveDriveCommand(
            steering_rad=float(data.get('steering_rad', 0.0)),
            rpm=float(data.get('rpm', 0.0)),
            mode=str(data.get('mode', 'UNKNOWN')),
            confidence=float(data.get('confidence', 0.0)),
            reason=str(data.get('reason', '')),
        )
    except Exception:
        return ReactiveDriveCommand(mode='INVALID', reason='json_parse_failed')


def clamp(value, low, high):
    """값을 [low, high] 범위 안으로 제한한다."""
    return max(low, min(value, high))


def normalize_angle(angle):
    """각도를 [-pi, pi] 범위로 정규화한다."""
    # atan2(sin(theta), cos(theta))는 임의 각도를 같은 방향의 -pi~pi 각도로 접는다.
    return math.atan2(math.sin(angle), math.cos(angle))


def is_valid_range(scan_msg, value):
    """LaserScan 거리값이 유한하고 센서 범위 안이면 True를 반환한다."""
    return (
        math.isfinite(value)
        and value > max(0.0, scan_msg.range_min)
        and value <= scan_msg.range_max
    )


def sanitize_rpm(rpm):
    """실차 규칙을 적용한다: 명령은 0 RPM 또는 MIN_DRIVING_RPM 이상이다."""
    if not math.isfinite(rpm):
        return 0.0
    # 물리적으로 너무 큰 명령을 막기 위해 최대 RPM 범위로 제한한다.
    rpm = clamp(rpm, -MAX_RPM, MAX_RPM)
    if abs(rpm) < 1e-6:
        return 0.0
    if abs(rpm) < MIN_DRIVING_RPM:
        # 출발 토크 부족 방지: 0이 아닌 명령은 최소 2000 RPM으로 끌어올린다.
        return math.copysign(MIN_DRIVING_RPM, rpm)
    return rpm


def apply_min_drive_speed(speed_ms, deadband=0.0):
    """새 RPM 제한을 적용하면서 기존 수동 속도 동작을 유지한다."""
    if abs(speed_ms) <= deadband:
        return 0.0
    # 수동 입력 m/s를 ERPM으로 바꾼 뒤 최소 RPM 규칙을 적용하고 다시 m/s로 되돌린다.
    # 이렇게 하면 조이스틱 deadband는 유지하면서 출발 토크만 보강할 수 있다.
    rpm = speed_to_erpm(speed_ms)
    limited_rpm = sanitize_rpm(rpm)
    return erpm_to_speed(limited_rpm)


def steering_to_servo(steering_rad):
    """조향각(rad)을 VESC 서보 위치로 변환한다."""
    # 서보 변환식: servo = center - steering_rad * gain.
    # 부호는 현재 차량 조향 방향에 맞춘 값이며, 마지막에 서보 허용 범위로 제한한다.
    servo_pos = SERVO_CENTER - steering_rad * SERVO_GAIN
    return clamp(servo_pos, SERVO_MIN, SERVO_MAX)


def speed_to_erpm(speed_mps):
    """측정된 VESC gain을 사용해 m/s를 ERPM으로 변환한다."""
    # 선형 근사 공식: ERPM = 속도[m/s] * 4614.
    return speed_mps * ERPM_GAIN


def erpm_to_speed(rpm):
    """ERPM 명령을 대략적인 m/s로 변환한다."""
    # 위 식의 역변환: 속도[m/s] = ERPM / 4614.
    return rpm / ERPM_GAIN


def print_status_line(text):
    """제어 로직을 막지 않으면서 읽기 쉬운 상태 줄을 출력한다."""
    global _STATUS_LINE_LEN
    sys.stdout.write(f'{text}\n')
    sys.stdout.flush()
    _STATUS_LINE_LEN = 0


def print_event_line(text):
    """상태 출력 아래에 이벤트 줄을 출력한다."""
    global _STATUS_LINE_LEN
    if _STATUS_LINE_LEN:
        sys.stdout.write('\n')
        _STATUS_LINE_LEN = 0
    print(text, flush=True)
