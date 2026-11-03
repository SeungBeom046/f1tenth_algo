"""IMU 정규화 계산 (ROS 비의존, 단위 테스트 가능).

imu_republisher 노드가 사용한다. 하는 일:
    1. 가속도 단위 변환 (g -> m/s^2), 자이로 단위 변환과 바이어스 차감
    2. 정지 상태 비력(specific force) 벡터 g_ref 추정
       (정지 시 가속도계 값 = 위쪽을 향하는 중력 반력, 장착 기울기와 바닥 경사 포함)
    3. 자이로 적분으로 정지 시점 대비 자세 변화 q 추적
    4. 현재 자세에서의 중력 성분을 뺀 선형 가속도를 base_link 축으로 회전

용도는 분석(종방향 jerk, Exp0 a_b 측정)뿐이다. 제어 루프에 넣지 않는다.

한계:
    - IMU는 base_link 원점(뒤축 중심)에서 떨어져 있어 회전 시 구심/접선 가속도가
      섞인다. 직선 트랙에서는 무시할 수준이라 보정하지 않는다.
    - 자이로 적분 자세는 바이어스 잔차로 천천히 드리프트한다. 회차마다 정지 상태에서
      재보정(recalibrate)하면 한 회차(수 초) 동안은 충분히 작다.
"""

import math

G0 = 9.80665

ACCEL_UNIT_SCALE = {'g': G0, 'mps2': 1.0}
GYRO_UNIT_SCALE = {'rad_s': 1.0, 'deg_s': math.pi / 180.0}


def quat_multiply(a, b):
    """해밀턴 곱 (w, x, y, z)."""
    w, x, y, z = a
    v, i, j, k = b
    return (w*v - x*i - y*j - z*k, w*i + x*v + y*k - z*j,
            w*j - x*k + y*v + z*i, w*k + x*j - y*i + z*v)


def quat_conj(q):
    return (q[0], -q[1], -q[2], -q[3])


def quat_rotate(q, v):
    """벡터 v를 q로 회전한다 (q * v * q^-1)."""
    return quat_multiply(quat_multiply(q, (0.0, *v)), quat_conj(q))[1:]


def quat_to_matrix(q):
    """단위 쿼터니언 (w, x, y, z)을 3x3 회전행렬(행 리스트)로 바꾼다."""
    w, x, y, z = q
    return (
        (1 - 2*(y*y + z*z), 2*(x*y - w*z), 2*(x*z + w*y)),
        (2*(x*y + w*z), 1 - 2*(x*x + z*z), 2*(y*z - w*x)),
        (2*(x*z - w*y), 2*(y*z + w*x), 1 - 2*(x*x + y*y)),
    )


def mat_vec(m, v):
    return tuple(sum(m[r][c] * v[c] for c in range(3)) for r in range(3))


def norm(v):
    return math.sqrt(sum(x*x for x in v))


class ImuNormalizer:
    """원시 IMU 샘플을 SI 단위, 바이어스 보정, base_link 선형 가속도로 바꾼다."""

    def __init__(self, accel_unit='g', gyro_unit='rad_s', gyro_bias=(0.0, 0.0, 0.0),
                 gravity_ref=None, calibration_samples=400,
                 rest_accel_tolerance=0.5, rest_gyro_tolerance=0.05,
                 track_attitude=True, max_dt=0.1):
        if accel_unit not in ACCEL_UNIT_SCALE:
            raise ValueError(f'accel_unit must be one of {list(ACCEL_UNIT_SCALE)}')
        if gyro_unit not in GYRO_UNIT_SCALE:
            raise ValueError(f'gyro_unit must be one of {list(GYRO_UNIT_SCALE)}')
        self.accel_scale = ACCEL_UNIT_SCALE[accel_unit]
        self.gyro_scale = GYRO_UNIT_SCALE[gyro_unit]
        self.gyro_bias = tuple(float(b) for b in gyro_bias)
        self.calibration_samples = int(calibration_samples)
        self.rest_accel_tolerance = rest_accel_tolerance
        self.rest_gyro_tolerance = rest_gyro_tolerance
        self.track_attitude = track_attitude
        self.max_dt = max_dt
        self.fixed_gravity_ref = (
            tuple(float(x) for x in gravity_ref) if gravity_ref else None)
        self.reset()

    def reset(self):
        """정지 상태 재보정을 시작한다. 차가 완전히 멈춰 있어야 한다."""
        self.gravity_ref = self.fixed_gravity_ref
        self._samples = []
        self.q = (1.0, 0.0, 0.0, 0.0)
        self.last_stamp = None

    @property
    def ready(self):
        return self.gravity_ref is not None

    def to_si(self, accel_raw, gyro_raw):
        """단위 변환 + 자이로 바이어스 차감. 결과는 IMU 프레임."""
        accel = tuple(a * self.accel_scale for a in accel_raw)
        gyro = tuple(g * self.gyro_scale - b for g, b in zip(gyro_raw, self.gyro_bias))
        return accel, gyro

    def feed_calibration(self, accel_si, gyro_si):
        """정지 샘플을 모아 g_ref를 추정한다. 준비되면 True."""
        if self.ready:
            return True
        if not all(math.isfinite(x) for x in (*accel_si, *gyro_si)):
            self._samples.clear()
            return False
        # 정지가 아니거나 단위가 틀리면(|a|가 9.8에서 멀면) 처음부터 다시 모은다.
        if (abs(norm(accel_si) - G0) > self.rest_accel_tolerance
                or norm(gyro_si) > self.rest_gyro_tolerance):
            self._samples.clear()
            return False
        self._samples.append(accel_si)
        if len(self._samples) < self.calibration_samples:
            return False
        n = len(self._samples)
        self.gravity_ref = tuple(sum(s[i] for s in self._samples) / n for i in range(3))
        self._samples = []
        self.q = (1.0, 0.0, 0.0, 0.0)
        self.last_stamp = None
        return True

    def update_attitude(self, stamp, gyro_si):
        """정지 시점 IMU 프레임 기준 현재 자세 q를 자이로로 적분한다."""
        if not self.track_attitude:
            return
        if self.last_stamp is None:
            self.last_stamp = stamp
            return
        dt = stamp - self.last_stamp
        self.last_stamp = stamp
        if dt <= 0.0 or dt > self.max_dt:
            return
        rate = norm(gyro_si)
        if rate < 1e-12:
            return
        half = 0.5 * rate * dt
        s = math.sin(half) / rate
        dq = (math.cos(half), gyro_si[0]*s, gyro_si[1]*s, gyro_si[2]*s)
        q = quat_multiply(self.q, dq)
        n = norm(q)
        self.q = tuple(x / n for x in q)

    def linear_accel_imu(self, accel_si):
        """현재 자세의 중력 반력을 뺀 선형 가속도 (IMU 프레임)."""
        # q는 현재 IMU 프레임 -> 정지 시점 프레임 회전이므로,
        # 정지 시점에 잰 g_ref를 현재 프레임으로 가져오려면 q의 역회전을 쓴다.
        g_now = quat_rotate(quat_conj(self.q), self.gravity_ref)
        return tuple(a - g for a, g in zip(accel_si, g_now))

    @staticmethod
    def to_base(rotation_base_from_imu, vec_imu):
        """IMU 프레임 벡터를 base_link 축으로 회전한다 (TF의 회전만 사용)."""
        return mat_vec(rotation_base_from_imu, vec_imu)
