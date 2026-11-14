"""실험 파라미터. 기본값은 기획안 가정값이며 측정값이 아니다 (보정 실험 후 YAML로 덮어쓴다).

조정은 config/exp_common.yaml에서 한다 (TUNE 태그와 docs/TUNING.md 참고).
여기 기본값은 그 파일과 같아야 하며 test_aur_safety.py가 고정한다.
"""
from dataclasses import dataclass, fields
from typing import Optional

from my_algo.vehicle_utils import ERPM_GAIN, MIN_DRIVING_RPM


@dataclass
class AurParams:
    # --- 안전 (식 6~11) ---
    t_d: float = 0.15               # 제동 지연 [s]
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
    lambda_clip: float = 20.0
    T_max: float = 8.0              # PROBE 최대 시간 [s], 넘으면 FOLLOW
    # --- 차량·통로 ---
    w_car: float = 0.27
    c_lat: float = 0.10             # 횡 여유 [m]
    W: Optional[float] = None       # 통로 폭 측정값 [m]. None이면 corridor.W_est를 쓴다
    lane_offset: Optional[float] = None   # None이면 δ = W/2 − w/2 − c
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
    # --- corridor (min_point_x 외에는 lidar_reactive_drive.PLANNER_CONSTANTS와 같은 값) ---
    corridor_sector_min_deg: float = 45.0
    corridor_sector_max_deg: float = 115.0
    corridor_min_points: int = 8
    corridor_wall_percentile: float = 0.35
    # 기존 값 −0.05(앞범퍼 기준)를 쓰면 벽 거리 < 0.25 m에서 섹터가 비어 벽을 놓친다.
    # 추월 차선에서는 가까운 벽이 w/2 + c = 0.235 m이므로 x 제한을 사실상 끈다.
    min_point_x: float = -1.0
    # --- 선행차 ---
    d_brake_trigger: float = 1.0    # NY 급정지 이벤트 무장 거리 [m]
    brake_delay_min: float = 0.5
    brake_delay_max: float = 2.0
    start_gap_margin: float = 0.2   # t0에서 d_rear < d_y_max + 이 값이면 경고

    @property
    def v_min(self):
        """0이 아닌 명령이 끌어올려지는 최소 속도 [m/s] (기본 0.867)."""
        return self.min_erpm / self.erpm_gain


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
