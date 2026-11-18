"""[공통] 실험 파라미터 (모든 단계). 기본값은 기획안 가정값이며 측정값이 아니다 (보정 실험 후 YAML로 덮어쓴다).

조정은 config/exp_common.yaml에서 한다 (TUNE 태그와 docs/TUNING.md 참고).
기존 주행(lidar_reactive_drive)과 겹치는 값의 기본값은 기존 주행 값이다. 실험에서 바꾼 값은
exp_common.yaml에서만 덮어쓴다 (EXPERIMENT_OVERRIDES). 나머지는 yaml과 같아야 하며 테스트가 확인한다.
"""
from dataclasses import dataclass, fields
from typing import Optional

from my_algo.baseline.vehicle_utils import ERPM_GAIN, MIN_DRIVING_RPM


@dataclass
class AurParams:
    # --- 안전 (식 6~11) ---
    t_d: float = 0.15               # 제동 지연 [s] (명령 → 감속 시작)
    scan_period: float = 0.1        # 인지 주기 Δ [s]. 식 (6)·(8)·(9)는 t_d + Δ를 쓴다 (기획안 변경)
    a_brake: float = 3.0            # 최대 감속도 [m/s^2]
    m_stop: float = 0.15            # 정지 후 남길 여유 [m]
    eps_follow: float = 0.05        # 추종 여유 ε [m]
    # --- belief (식 1~5) ---
    d_y_min: float = 0.6            # 선행차 양보 반응 거리 범위 [m]
    d_y_max: float = 1.2
    v_lat: float = 0.5              # 양보 시 횡속도 (관측 모델) [m/s]
    sigma_z: float = 0.02           # 횡변위 관측 노이즈 [m]
    eta: float = 0.9                # 판단 임계 확률
    belief_model: str = 'survival'  # original(기획안 식) | survival(기획안 변경)
    y_yield_thr: float = 0.08       # (survival) 양보로 보는 누적 횡변위 [m]. 4σ
    p_false_yield: float = 0.01     # (survival) NY인데 한 회차 안에 임계값을 넘을 확률
    t_react: float = 1.3            # (survival) 선행차가 양보를 시작한 뒤 횡변위가 나타나기까지의 지연 [s]
    lambda_clip: float = 20.0
    T_max: float = 8.0              # PROBE 최대 시간 [s], 넘으면 FOLLOW
    # --- 차량·통로 ---
    w_car: float = 0.27
    c_lat: float = 0.10             # 횡 여유 [m]
    W: Optional[float] = None       # 통로 폭 측정값 [m]. None이면 corridor.W_est를 쓴다
    W_leader: Optional[float] = None    # 선행차 전용 실측 통로 폭 [m]. 주면 선행차는 오른쪽 벽만으로 횡위치를 구한다
    lane_offset: Optional[float] = None   # None이면 δ = W/2 − w/2 − c
    lane_balance: bool = False      # True면 δ = W/2 − w/2 − (W − 2w)/3 (벽·차 간격 균등), False면 W/2 − w/2 − c
    lane_offset_max: Optional[float] = 0.3    # δ 상한 [m]. 넓은 통로에서 벽까지 붙지 않게 (None = 상한 없음). W 1.0의 δ 0.265보다 커서 W ≤ 1.27은 그대로
    # --- 속도 ---
    v_B_nominal: float = 0.9
    v_pass: float = 1.5
    erpm_gain: float = ERPM_GAIN    # 보정 결과로 덮어쓸 수 있다
    min_erpm: float = MIN_DRIVING_RPM
    t0_speed_threshold: float = 0.3  # 출발 시각 t0 판정 [m/s]
    hold_on_speed: float = 0.35     # v_E가 이보다 크면 최소 속도 hold 허용 [m/s]
    hold_off_speed: float = 0.25    # v_E가 이보다 작으면 hold 금지 [m/s]
    # --- 방법별 ---
    d_follow_a: float = 0.8
    T_wait_a: float = 3.0
    d_follow_b: float = 1.5
    d_probe_c: float = 0.4
    k_d: float = 1.5                # 거리 유지 P 게인 [1/s]
    probe_tol: float = 0.05         # d ≤ d_ref + probe_tol이면 PROBE
    merge_tol: float = 0.05         # MERGE 완료 판정 횡오차 [m]
    merge_gap: Optional[float] = 0.35   # 선행차 앞면이 ego 뒤범퍼보다 이만큼 뒤면 MERGE [m]. None = m_stop
    merge_v: Optional[float] = 1.5      # MERGE 중 속도 [m/s]. None = v_B_nominal
    lane_change_tol: float = 0.05   # PASS에서 추월 차선 도착 판정 횡오차 [m]. 도착 전에는 추종 속도
    track_hold: float = 0.5         # 추적이 끊겨도 직전 값을 쓰는 시간 [s]
    # --- tracker ---
    lidar_to_front: float = 0.30    # AEB의 lidar_to_bumper_dist와 같은 값 (실측 0.28). d가 AEB closest와 일치하게
    lidar_to_rear: float = 0.22
    cluster_jump: float = 0.15      # 연속 점 사이 거리가 이보다 크면 클러스터 분할 [m]
    cluster_min_points: int = 3
    track_x_max: float = 3.0
    veh_width_min: float = 0.12     # 클러스터 y 범위 [m]
    veh_width_max: float = 0.45
    veh_depth_max: float = 0.60     # 클러스터 x 범위 상한. 헤딩이 틀어졌을 때 통로 안으로 들어온 벽 조각 제거
    wall_margin: float = 0.05       # |y| ≥ W/2 − wall_margin인 점은 벽으로 보고 버린다
    rear_fov_half_deg: float = 60.0
    track_alpha: float = 0.5        # d의 α-β 필터
    track_beta: float = 0.2
    track_jump_reset: float = 0.3   # 예측과 이만큼 다르면 다른 물체로 보고 필터 초기화 [m]
    track_timeout: float = 0.5
    side_gate: float = 0.6          # 측면 추적 연관 거리 [m]
    rear_face_depth: float = 0.05   # 측면 추적에서 뒷면으로 보는 깊이 (x0부터) [m]
    leader_width: float = 0.27      # 선행차 전폭 [m] (왼쪽 모서리 추정: 왼쪽 옆면 점 선택)
    leader_length: float = 0.50     # 선행차 길이 [m] (왼쪽 모서리 추정: 옆면 직선 연장 구간)
    side_fit_min_points: int = 3    # 옆면 직선 피팅 최소 점 수
    heading_comp: bool = True       # tracker 입력 점을 corridor 헤딩으로 돌려 보정 (off = 차가 통로와 나란하다고 가정)
    abort_mode: str = 'retreat'     # 추월 중단 뒤: follow(바로 FOLLOW) | retreat(RETREAT 거쳐 FOLLOW)
    retreat_speed_ratio: float = 0.8    # RETREAT 속도 = 이 값 × v_B
    retreat_clear_gap: float = 0.15     # RETREAT 탈출: 선행차 뒷면이 앞범퍼보다 이만큼 앞 [m] (m과 같은 값)
    abort_yb_window: int = 3        # 추월 중단 판정의 y_B를 최근 n개(유효 scan) 중앙값으로. 1 = 그 scan 값 그대로
    lateral_ref: str = 'center'     # 횡 여유 판정 기준: center(y_B 중심, 기존) | left_edge(y_L 왼쪽 모서리)
    # --- corridor (lidar_reactive_drive.PLANNER_CONSTANTS와 같은 값) ---
    corridor_sector_min_deg: float = 45.0
    corridor_sector_max_deg: float = 115.0
    corridor_min_points: int = 8
    corridor_wall_percentile: float = 0.35
    corridor_guardrail: bool = False   # 실측 W에 맞는 가까운 평행 벽 선택. 폭을 주지 않거나 벽이 무효면 정지
    experiment_safety: bool = False    # 추적 소실·먼 점 점프 정지, 보수적 초기 속도, 헤딩 필터
    track_recovery: bool = False      # 현재 벽 점 제외·최초 전방 후보 확인·추적 재인식 수정
    stationary_pass: bool = False      # method a 전용, 정지차 시험임을 명시한 조기 차선 변경
    corridor_line_tol: float = 0.03    # 벽 직선에서 허용하는 점의 수직 오차 [m]
    corridor_width_tol: float = 0.10   # 실측 W와 벽 사이 거리의 허용 차이 [m]
    corridor_min_span: float = 0.20    # 벽 직선의 최소 전후 길이 [m]
    min_point_x: float = -0.05      # 실험은 −1.0 (exp_common.yaml)
    # --- 횡 제어 (y_ref_rate 외에는 lidar_reactive_drive 기본값) ---
    wall_follow_gain: float = 0.30
    wall_error_filter_alpha: float = 0.25
    steer_rate_limit: float = 0.08  # [rad/scan]
    steering_deadband: float = 0.055  # 실험은 0.02 (exp_common.yaml)
    max_steering: float = 0.78
    heading_gain: float = 0.0       # 헤딩 감쇠 [rad/rad]. 0 = 기존 방식(횡위치 P만). 실험은 0.7 (exp_common.yaml)
    y_ref_rate: float = 0.4         # 차선 변경 시 y_ref 기울기 제한 [m/s]
    lateral_ki: float = 0.3         # 횡 적분 게인 [rad/(m·s)]. 서보 중심 오차(트림) 보정. 0 = 끔
    lateral_i_max: float = 0.10     # 적분항 한계 [rad]
    lateral_i_zone: float = 0.10    # |y_ref − ego_lateral|이 이보다 작고 차선 변경 중이 아닐 때만 적분 [m]
    # --- 선행차 ---
    d_brake_trigger: float = 1.0    # NY 급정지 이벤트 무장 거리 [m]
    brake_delay_min: float = 0.5
    brake_delay_max: float = 2.0
    start_gap_margin: float = 0.2   # t0에서 d_rear < d_y_max + 이 값이면 경고

    @property
    def v_min(self):
        """0이 아닌 명령이 끌어올려지는 최소 속도 [m/s] (기본 0.867)."""
        return self.min_erpm / self.erpm_gain


# exp_common.yaml이 기본값과 다르게 두는 값 (실험 전용). 이유는 yaml의 TUNE 태그와 docs/PROGRESS.md.
EXPERIMENT_OVERRIDES = {'min_point_x': -1.0, 'steering_deadband': 0.0, 'heading_gain': 1.0, 'max_steering': 0.349, 'lidar_to_front': 0.28, 'corridor_guardrail': True, 'experiment_safety': True}


def load_params(*paths, **overrides):
    """YAML 파일들을 순서대로 덮어써 AurParams를 만든다. 최상위 `aur:` 키가 있으면 그 아래를 읽는다."""
    data = {}
    if paths:
        import yaml  # 지연 import: YAML 없이 기본값만 쓰는 환경을 위해
        for path in paths:
            with open(path) as f:
                doc = yaml.safe_load(f) or {}
            data.update(doc.get('aur', doc) or {})
    data.update(overrides)
    unknown = set(data) - {f.name for f in fields(AurParams)}
    if unknown:
        raise ValueError('알 수 없는 aur 파라미터: %s' % sorted(unknown))
    return AurParams(**data)
