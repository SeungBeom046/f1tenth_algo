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
    v_cruise: Optional[float] = None   # ego가 선행차를 한 번도 못 봤을 때 속도 [m/s]. None = v_B_nominal
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
    # race_stack(ForzaETH) spliner의 상대 차 예측(_predict_obs_movement 'constant')을 횡 방향만 보수적으로 옮긴 것.
    # 횡 여유(go·추월 중단)와 복귀 판정에 '이 시간 뒤 선행차 횡위치'를 쓴다. 왼쪽(추월 차선 쪽) 움직임만 더한다
    leader_pred_time: float = 0.0   # 선행차 횡위치 예측 시간 [s]. 0 = 끔 (race_stack fixed_pred_time 0.15)
    leader_vlat_window: int = 5     # 선행차 횡속도 = 최근 n개 유효 y_B의 최소제곱 기울기 [개]
    # race_stack state_machine의 splini_hyst_timer_sec: 추월 조건이 이 시간 동안 끊김 없이 유지돼야 PASS
    go_hold_s: float = 0.3          # [s]. 0 = 끔 (그 scan에 조건이 맞으면 바로 PASS)
    # --- 자차 위치 필터 (pose.PoseFilter): 자이로 + 속도 예측, 레일 측정으로 고정 비율 보정. 지도 없음 ---
    pose_filter: bool = False       # 끄면 매 scan 레일 측정을 그대로 쓴다 (자이로가 없어도 그렇다)
    pose_k_heading: float = 0.15    # 헤딩 보정 비율 (1이면 측정 그대로)
    pose_k_lateral: float = 0.3     # 횡위치 보정 비율
    pose_gate_deg: float = 0.0      # 레일 헤딩이 예측과 이보다 많이 다르면 보정 비율을 pose_gate_scale배로 [deg]. 0 = 끔
    pose_gate_scale: float = 0.2
    pose_reset_s: float = 1.0       # 레일을 이 시간 넘게 못 보면 다음 측정으로 바로 맞춘다 [s]
    # --- 레이스 개정안: 멈춘 앞차 확정·우회 (EgoFSM._update_static). 의도(belief)와 무관 ---
    static_bypass: bool = False     # 끄면 멈춘 차도 belief대로 (method d는 뒤에서 정지)
    static_v: float = 0.15          # 앞차 속도 v_B가 이보다 작으면 멈춘 차 후보 [m/s]
    static_lat: float = 0.05        # 후보 동안 앞차 횡이동 허용 [m]
    static_moving_v: float = 0.3    # 앞차가 이보다 빠르면 '움직이는 것을 본 차'로 기록 [m/s]
    static_t_never: float = 1.0     # 한 번도 안 움직인 차를 멈춘 차로 확정하는 시간 [s]
    static_t_moved: float = 3.0     # 달리다 선 차를 확정하는 시간 [s] (급정지 뒤 재출발보다 길어야 한다)
    static_settle_s: float = 0.5    # 앞차를 처음 본 뒤 이 시간 동안은 후보로 보지 않는다 (v_B 수렴) [s]
    static_bypass_gap: float = 2.0  # 후보 동안 이 거리 뒤에서 선다 [m] (1.5면 y −0.2 정지 차를 서서 출발해 못 돌아감)
    static_bypass_min_d: float = 1.0    # 우회를 시작할 수 있는 최소 거리 [m]
    # --- 레일을 놓쳤을 때 대체 주행 (fallback.free_space_lateral). 앞차를 아직 못 봤고 추월 전일 때만 ---
    rail_fallback_v: Optional[float] = None   # 대체 주행 속도 [m/s]. None = 끔 (레일 무효면 정지)
    rail_fallback_s: float = 3.0      # 마지막 레일 측정 뒤 이 시간까지만 [s]
    rail_fallback_clear: float = 1.5  # 앞범퍼 앞 이 거리 안 진행 경로(w/2 + c)에 점이 있으면 대체 주행 안 함 [m]
    # --- corridor (lidar_reactive_drive.PLANNER_CONSTANTS와 같은 값) ---
    corridor_sector_min_deg: float = 45.0
    corridor_sector_max_deg: float = 115.0
    corridor_min_points: int = 8
    corridor_wall_percentile: float = 0.35
    corridor_guardrail: bool = False   # 실측 W에 맞는 가까운 평행 벽 선택. 폭을 주지 않거나 벽이 무효면 정지
    experiment_safety: bool = False    # 추적 소실·먼 점 점프 정지, 보수적 초기 속도, 헤딩 필터
    track_recovery: bool = False      # 현재 벽 점 제외·최초 전방 후보 확인·추적 재인식 수정
    guardrail_local_view: bool = False  # 가까운 레일의 후측면 관측·벽 선 연속성 사용
    corridor_hold_s: float = 0.0       # 잠긴 벽이 무효여도 이 시간 동안 직전 추정으로 주행 [s], 0이면 끔
    corridor_relock_s: float = 0.0     # 잠긴 벽이 이 시간 넘게 무효면 처음부터 다시 초기화 [s], 0이면 끔
    corridor_init_window_s: float = 0.0  # 벽 초기화 3회 일치를 이 시간 창 안에서 센다 (못 찾은 scan은 무시) [s], 0이면 연속 3회
    corridor_heading_avg: bool = False   # 헤딩 = 양쪽 벽 선 길이 가중 평균 (끄면 긴 선 하나)
    stationary_pass: bool = False      # method a 전용, 정지차 시험임을 명시한 조기 차선 변경
    stationary_geom_pass: bool = False  # 정지 차 우회 시작 거리를 고정 1.5 m(멈춘 차 확정 우회 1.0 m) 대신 필요한 옆 이동으로 계산
    stationary_clear: float = 0.05      # 지나갈 때 선행차 옆면과 남길 간격 [m] (실차 추월 때 차체-벽 약 5 cm)
    stationary_steer_frac: float = 0.6  # 원호 계산에 쓰는 조향 비율 (서보 지연·횡 제어 여유)
    stationary_margin_x: float = 0.15   # 원호 거리에 더하는 종방향 여유 [m]
    wheelbase: float = 0.33             # [m] 실측
    follow_repass: bool = False         # FOLLOW에서 선행차가 더 비키고 시작 여유가 있으면 다시 추월
    belief_pre_t0: bool = False         # belief 누적 횡변위에 출발(t0) 전 비킨 변위도 넣는다 (중앙값 3개 기준)
    retreat_hold: bool = False          # 추월 중단 뒤 추월 차선 유지·감속, 거의 멈추거나 선행차가 멀어지면 FOLLOW
    retreat_exit_v: float = 0.3         # 이 속도 이하면 RETREAT 종료 [m/s]
    retreat_exit_d: float = 2.0         # 선행차가 이만큼 앞서며 ego보다 빠르면 RETREAT 종료 [m]
    pass_margin_hyst: bool = False      # 추월 시작은 횡 여유 + pass_start_margin, 중단은 3개 중앙값 y_B로 여유 0 판정
    pass_start_margin: float = 0.05     # 추월 시작에 더 요구하는 횡 여유 [m]
    lead_steady_face: bool = False      # 앞 추적에서 넓은 뒷면을 본 뒤에는 좁게 보인 scan도 같은 면 (0.22 m를 빼지 않음), 모든 방법
    stationary_partial_lead: bool = False   # stationary_pass에서 첫 확정 전 좁은(일부만 보인) 정지 차 클러스터도 후보로, 확인 중 한 scan 놓침 허용
    partial_width_min: float = 0.04     # 좁은 후보 최소 폭 [m]
    partial_depth_max: float = 0.3      # 좁은 후보 최대 깊이 [m] (레일 조각은 길다)
    partial_min_points: int = 4         # 좁은 후보 최소 점 수
    partial_wall_clear: float = 0.10    # 좁은 후보 중심이 벽 경계(W/2 − wall_margin)에서 이만큼 안쪽 [m]
    # --- 카메라 융합 (D455, my_algo/aur/camera.py). 카메라가 없거나 끊기면 라이다만 쓴다 ---
    camera_fusion: bool = False         # 깊이 장애물 거리를 /scan bin마다 최솟값으로 합친다 (벽·추적·대체 주행 공통, AEB는 라이다만)
    camera_lead: bool = False           # YOLO 차 상자와 겹치는 클러스터는 폭과 무관하게 후보, 바로 확정, 카메라 뒷면 거리 사용
    cam_stale_s: float = 0.3            # 이보다 오래된 카메라 결과는 안 쓴다 [s]
    cam_match_ang: float = 0.05         # 카메라 방위 범위와 클러스터 방위 범위 겹침 여유 [rad]
    cam_match_r: float = 0.35           # 카메라 뒷면 거리와 클러스터 최소 거리 차 허용 [m] (지연 0.2 s × 1.2 m/s + 잡음)
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
EXPERIMENT_OVERRIDES = {'min_point_x': -1.0, 'steering_deadband': 0.0, 'heading_gain': 1.0, 'max_steering': 0.349, 'lidar_to_front': 0.28, 'corridor_guardrail': True, 'experiment_safety': True, 'track_recovery': True, 'guardrail_local_view': True, 'corridor_hold_s': 0.3, 'corridor_relock_s': 1.0, 'corridor_init_window_s': 2.0, 'corridor_heading_avg': True, 'eps_follow': 0.15, 'v_cruise': 1.2, 'pose_filter': True, 'rail_fallback_v': 0.9, 'static_bypass': True, 'stationary_partial_lead': True, 'stationary_geom_pass': True, 'stationary_steer_frac': 0.8, 'lead_steady_face': True, 'pass_margin_hyst': True, 'retreat_hold': True, 'belief_pre_t0': True, 'follow_repass': True, 'pose_gate_deg': 3.0}


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
