# 실험 태그 목록

자동 생성 파일 (`python3 scripts/gen_exp_tags.py`). 직접 고치지 말 것.
같은 줄에 태그가 여러 개면 각 표에 모두 나온다.

## [EXP-VAR] 독립변인 — 바꾸면 비교 조건이 바뀜 (18개)

| 위치 | 내용 |
|---|---|
| [config/method_B0.yaml:1](../config/method_B0.yaml#L1) | `# [EXP-VAR] 방법 B0: 공백 무시, AEB만` |
| [config/method_B1.yaml:1](../config/method_B1.yaml#L1) | `# [EXP-VAR] 방법 B1: 전방 ROI 미관측 비율 rho로 v = v_ref(1 - k rho)` |
| [config/method_B2.yaml:1](../config/method_B2.yaml#L1) | `# [EXP-VAR] 방법 B2: 모든 가림 경계 칸에 phantom` |
| [config/method_P.yaml:1](../config/method_P.yaml#L1) | `# [EXP-VAR] 방법 P: 틈 폭 >= w_min 입구와 주차열 양 끝에만 phantom (제안)` |
| [config/scenario_S1.yaml:1](../config/scenario_S1.yaml#L1) | `# [EXP-VAR] 시나리오 S1: 틈 0.45 m, 돌출 있음 (d_e = 1.0 / 1.5 / 2.0 m는 bringup 인자 d_e:=)` |
| [config/scenario_S1.yaml:6](../config/scenario_S1.yaml#L6) | `gap_width: 0.45            # [EXP-VAR] 틈 폭 [m]. 트랙 배치 계산 입력 (track_layout.py)` |
| [config/scenario_S2.yaml:1](../config/scenario_S2.yaml#L1) | `# [EXP-VAR] 시나리오 S2: 틈 0.45 m, 돌출 없음 (정당한 감속 비용)` |
| [config/scenario_S2.yaml:6](../config/scenario_S2.yaml#L6) | `gap_width: 0.45            # [EXP-VAR] 틈 폭 [m]. 트랙 배치 계산 입력 (track_layout.py)` |
| [config/scenario_S3.yaml:1](../config/scenario_S3.yaml#L1) | `# [EXP-VAR] 시나리오 S3: 틈 0.15 m (통과 불가), 돌출 없음 (B2 대비 P의 불필요 감속)` |
| [config/scenario_S3.yaml:6](../config/scenario_S3.yaml#L6) | `gap_width: 0.15            # [EXP-VAR] 틈 폭 [m]. 트랙 배치 계산 입력 (track_layout.py)` |
| [config/scenario_S4.yaml:1](../config/scenario_S4.yaml#L1) | `# [EXP-VAR] 시나리오 S4: 우측도 연속 벽, 돌출 없음 (B1 과잉 감속)` |
| [config/scenario_S4.yaml:6](../config/scenario_S4.yaml#L6) | `gap_width: 0.0            # [EXP-VAR] 틈 폭 [m]. 트랙 배치 계산 입력 (track_layout.py)` |
| [launch/bringup.launch.py:132](../launch/bringup.launch.py#L132) | `DeclareLaunchArgument('method', default_value='B0', description='[EXP-VAR] B0 \| B1 \| B2 \| P'),` |
| [launch/bringup.launch.py:133](../launch/bringup.launch.py#L133) | `DeclareLaunchArgument('scenario', default_value='S2', description='[EXP-VAR] S1 \| S2 \| S3 \| S4'),` |
| [launch/bringup.launch.py:134](../launch/bringup.launch.py#L134) | `DeclareLaunchArgument('d_e', default_value='', description='[EXP-VAR] S1 돌출 시점 1.0 \| 1.5 \| 2.0'),` |
| [my_algo/phantom_node.py:11](../my_algo/phantom_node.py#L11) | `파라미터: method ([EXP-VAR], config/method_*.yaml), ROI·w_min 등은 config/common.yaml,` |
| [my_algo/speed_layer_node.py:16](../my_algo/speed_layer_node.py#L16) | `method                      [EXP-VAR] config/method_*.yaml` |
| [my_algo/trigger_node.py:16](../my_algo/trigger_node.py#L16) | `d_e [EXP-VAR] (launch 인자), collision_x (config/scenario_S1.yaml)` |

## [EXP-FIXED] 통제변인 — 방법 간 절대 바꾸지 말 것 (92개)

| 위치 | 내용 |
|---|---|
| [config/common.yaml:6](../config/common.yaml#L6) | `#   [EXP-FIXED]    방법 간에 절대 바꾸면 안 되는 값. 바꾸면 비교가 무효가 되는 이유를 옆에 적음` |
| [config/common.yaml:14](../config/common.yaml#L14) | `v_ref: 2.0              # [EXP-FIXED] 목표 속도. 방법마다 다르면 손실 ΔT의 기준이 달라져 비교 불가` |
| [config/common.yaml:15](../config/common.yaml#L15) | `t_r: 0.15               # [EXP-FIXED][EXP0-MEASURE] 전체 지연 [s] = 관측 → 실제 감속 시작 (인지 + 통신 + 제어 + 구동,` |
| [config/common.yaml:18](../config/common.yaml#L18) | `a_b: 3.0                # [EXP-FIXED][EXP0-MEASURE] 최대 감속 [m/s^2]. v_cap 식 공통. Exp0에서 /imu로 측정` |
| [config/common.yaml:19](../config/common.yaml#L19) | `s_m: 0.2                # [EXP-FIXED] 정지 여유 [m]. v_cap 식 공통` |
| [config/common.yaml:24](../config/common.yaml#L24) | `min_speed_policy: hold  # [EXP-FIXED] hold = v_cap < V_MIN이면 V_MIN 유지 + 플래그.` |
| [config/common.yaml:27](../config/common.yaml#L27) | `restart_margin: 0.1     # [EXP-FIXED] hysteresis 정책 재출발 여유 [m/s] (hold에서는 안 씀)` |
| [config/common.yaml:28](../config/common.yaml#L28) | `perception_timeout: 0.3 # [EXP-FIXED] phantom 입력이 이보다 오래되면 0 명령 [s]` |
| [config/common.yaml:31](../config/common.yaml#L31) | `grid_x_min: -3.0        # [EXP-FIXED] odom 고정 창 [m]. odom 원점 = AUTO 시작 위치 근처` |
| [config/common.yaml:35](../config/common.yaml#L35) | `grid_resolution: 0.05   # [EXP-FIXED] 칸 크기 [m]. 틈 폭(0.15/0.45 m) 측정 해상도를 정함` |
| [config/common.yaml:36](../config/common.yaml#L36) | `max_range: 8.0          # [EXP-FIXED] 레이캐스팅 최대 거리 [m]` |
| [config/common.yaml:37](../config/common.yaml#L37) | `band_z_min: 0.05        # [EXP-FIXED] 높이 대역 하한 (base_link z) [m]. 바닥점 제외` |
| [config/common.yaml:38](../config/common.yaml#L38) | `band_z_max: 0.30        # [EXP-FIXED] 높이 대역 상한 [m]. 반드시 박스 높이(0.35)보다 낮게.` |
| [config/common.yaml:48](../config/common.yaml#L48) | `roi_x_max: 6.0          # [EXP-FIXED] phantom 탐색 앞쪽 한계 (base_link x) [m]` |
| [config/common.yaml:49](../config/common.yaml#L49) | `roi_half_y: 1.5         # [EXP-FIXED] phantom 탐색 좌우 한계 [m]` |
| [config/common.yaml:58](../config/common.yaml#L58) | `row_y_min: 0.30         # [EXP-FIXED] P 주차열로 보는 occupied 칸의 \|y\| 범위 [m]` |
| [config/common.yaml:60](../config/common.yaml#L60) | `row_link_dist: 0.10     # [EXP-FIXED] P 같은 열로 묶는 x 간격 [m]` |
| [config/common.yaml:64](../config/common.yaml#L64) | `row_end_margin: 0.5     # [EXP-FIXED] ROI 앞 경계 이 거리 안에서 끝나는 열은 끝으로 보지 않음 [m]` |
| [config/common.yaml:65](../config/common.yaml#L65) | `p_lateral_filter: 0.0   # [EXP-FIXED] P 선택 필터 (\|y\| 상한, 0 = 끔)` |
| [config/common.yaml:68](../config/common.yaml#L68) | `aeb_ttc_threshold: 0.55             # [EXP-FIXED] 방법마다 다르면 안전 결과를 비교할 수 없음` |
| [config/common.yaml:69](../config/common.yaml#L69) | `aeb_path_margin: 0.15               # [EXP-FIXED]` |
| [config/common.yaml:70](../config/common.yaml#L70) | `aeb_bumper_emergency_distance: 0.25 # [EXP-FIXED]` |
| [config/common.yaml:71](../config/common.yaml#L71) | `aeb_bumper_overlap_tolerance: 0.10  # [EXP-FIXED]` |
| [config/common.yaml:72](../config/common.yaml#L72) | `aeb_comfortable_decel: 2.8          # [EXP-FIXED] 제동거리 추정 감속도 (aeb.py 기존값)` |
| [config/common.yaml:73](../config/common.yaml#L73) | `aeb_stopping_margin: 0.20           # [EXP-FIXED]` |
| [config/common.yaml:76](../config/common.yaml#L76) | `joy_timeout: 0.5        # [EXP-FIXED] /joy 끊김 감시 [s]. 끊기면 정지 + 회차 폐기` |
| [config/common.yaml:77](../config/common.yaml#L77) | `require_joy: true       # [EXP-FIXED] 데드맨 사용` |
| [config/common.yaml:78](../config/common.yaml#L78) | `aeb_latch: true         # [EXP-FIXED] AEB 작동 후 회차 끝까지 정지 유지` |
| [config/common.yaml:79](../config/common.yaml#L79) | `input_timeout: 0.3      # [EXP-FIXED] 조향/속도 입력 끊김 [s]` |
| [config/common.yaml:80](../config/common.yaml#L80) | `max_steering: 0.78      # [EXP-FIXED] 조향 제한 [rad] (drive와 같음)` |
| [config/common.yaml:81](../config/common.yaml#L81) | `require_preflight: true # [EXP-FIXED] 발행자 점검(publisher_guard_node) 통과 전 출발 거부` |
| [config/common.yaml:88](../config/common.yaml#L88) | `deskew: false           # [EXP-FIXED] 모든 방법 동일. Exp1에서 on/off 비교 후 결정.` |
| [config/common.yaml:100](../config/common.yaml#L100) | `# [EXP-FIXED] 좌측 벽 목표 거리 = 라이다 프레임에서 본 좌측 벽 \|y\|.` |
| [config/common.yaml:107](../config/common.yaml#L107) | `exp_lookahead_distance: 4.4     # [EXP-FIXED] exp에서 lookahead 고정 (v_ref 2.0 기준 2.6+0.9*2.0)` |
| [config/common.yaml:113](../config/common.yaml#L113) | `steering_deadband: 0.055        # [EXP-FIXED] 조향 게인류는 방법마다 같아야 궤적이 같다` |
| [config/common.yaml:117](../config/common.yaml#L117) | `wall_follow_gain: 0.30          # [EXP-FIXED]` |
| [config/common.yaml:118](../config/common.yaml#L118) | `wall_error_filter_alpha: 0.25   # [EXP-FIXED]` |
| [config/common.yaml:120](../config/common.yaml#L120) | `max_steering: 0.78              # [EXP-FIXED]` |
| [config/common.yaml:122](../config/common.yaml#L122) | `# AEB와 조향이 쓰는 /scan. 값은 lidar_drive.launch.py와 같다 [EXP-FIXED].` |
| [config/popout.yaml:1](../config/popout.yaml#L1) | `# 돌출 차량 #2 (popout_vehicle_node). 모든 방법·회차 동일 [EXP-FIXED].` |
| [config/popout.yaml:8](../config/popout.yaml#L8) | `v_popout: 0.8                      # [EXP-FIXED] 돌출 속도 [m/s]. 모든 시나리오·방법 공통.` |
| [config/popout.yaml:11](../config/popout.yaml#L11) | `stop_distance: 1.0                 # [EXP-FIXED] 출발 후 이 거리에서 정지 [m] (차선 반대편 전에 멈추도록 트랙에 맞춰 교체)` |
| [config/popout.yaml:13](../config/popout.yaml#L13) | `max_time: 3.0                      # [EXP-FIXED] 무조건 정지 [s]` |
| [config/preflight.yaml:7](../config/preflight.yaml#L7) | `mode: enforce            # [EXP-FIXED] enforce = 위반 시 회차 거부 \| warn = 경고만` |
| [config/scenario_E0.yaml:15](../config/scenario_E0.yaml#L15) | `box_len: 0.55              # [EXP-FIXED] 주차 박스 진행 방향 길이 [m] (0.55 × 0.30 × 0.35). 트랙 배치 계산용` |
| [config/scenario_S1.yaml:9](../config/scenario_S1.yaml#L9) | `stop_distance: 7.5         # [EXP-FIXED] 소프트웨어 정지선: AUTO 후 주행 거리 > 이 값이면 0 명령 [m]` |
| [config/scenario_S1.yaml:11](../config/scenario_S1.yaml#L11) | `accel_end_x: 2.5           # [EXP-FIXED] 가속 구간 끝 (v_ref 도달 판정 위치) [m]` |
| [config/scenario_S1.yaml:12](../config/scenario_S1.yaml#L12) | `test_x_start: 2.5          # [EXP-FIXED] 시험 구간 (손실 ΔT 적분 범위) [m]` |
| [config/scenario_S1.yaml:16](../config/scenario_S1.yaml#L16) | `box_len: 0.55              # [EXP-FIXED] 주차 박스 진행 방향 길이 [m] (0.55 × 0.30 × 0.35). 트랙 배치 계산용` |
| [config/scenario_S2.yaml:9](../config/scenario_S2.yaml#L9) | `stop_distance: 7.5         # [EXP-FIXED] 소프트웨어 정지선: AUTO 후 주행 거리 > 이 값이면 0 명령 [m]` |
| [config/scenario_S2.yaml:11](../config/scenario_S2.yaml#L11) | `accel_end_x: 2.5           # [EXP-FIXED] 가속 구간 끝 (v_ref 도달 판정 위치) [m]` |
| [config/scenario_S2.yaml:12](../config/scenario_S2.yaml#L12) | `test_x_start: 2.5          # [EXP-FIXED] 시험 구간 (손실 ΔT 적분 범위) [m]` |
| [config/scenario_S2.yaml:16](../config/scenario_S2.yaml#L16) | `box_len: 0.55              # [EXP-FIXED] 주차 박스 진행 방향 길이 [m] (0.55 × 0.30 × 0.35). 트랙 배치 계산용` |
| [config/scenario_S3.yaml:9](../config/scenario_S3.yaml#L9) | `stop_distance: 7.5         # [EXP-FIXED] 소프트웨어 정지선: AUTO 후 주행 거리 > 이 값이면 0 명령 [m]` |
| [config/scenario_S3.yaml:11](../config/scenario_S3.yaml#L11) | `accel_end_x: 2.5           # [EXP-FIXED] 가속 구간 끝 (v_ref 도달 판정 위치) [m]` |
| [config/scenario_S3.yaml:12](../config/scenario_S3.yaml#L12) | `test_x_start: 2.5          # [EXP-FIXED] 시험 구간 (손실 ΔT 적분 범위) [m]` |
| [config/scenario_S3.yaml:16](../config/scenario_S3.yaml#L16) | `box_len: 0.55              # [EXP-FIXED] 주차 박스 진행 방향 길이 [m] (0.55 × 0.30 × 0.35). 트랙 배치 계산용` |
| [config/scenario_S4.yaml:9](../config/scenario_S4.yaml#L9) | `stop_distance: 7.5         # [EXP-FIXED] 소프트웨어 정지선: AUTO 후 주행 거리 > 이 값이면 0 명령 [m]` |
| [config/scenario_S4.yaml:11](../config/scenario_S4.yaml#L11) | `accel_end_x: 2.5           # [EXP-FIXED] 가속 구간 끝 (v_ref 도달 판정 위치) [m]` |
| [config/scenario_S4.yaml:12](../config/scenario_S4.yaml#L12) | `test_x_start: 2.5          # [EXP-FIXED] 시험 구간 (손실 ΔT 적분 범위) [m]` |
| [config/scenario_S4.yaml:16](../config/scenario_S4.yaml#L16) | `box_len: 0.55              # [EXP-FIXED] 주차 박스 진행 방향 길이 [m] (0.55 × 0.30 × 0.35). 트랙 배치 계산용` |
| [config/vehicle_geometry.yaml:3](../config/vehicle_geometry.yaml#L3) | `# 모든 방법(B0/B1/B2/P)이 이 파일 하나를 공유한다.  [EXP-FIXED]` |
| [launch/bringup.launch.py:69](../launch/bringup.launch.py#L69) | `# Exp0 odom 보정·Exp1 저속 주행 전용. v_ref는 [EXP-FIXED]라 방법 비교(Exp2·Exp3) 회차에는 쓰지 않는다.` |
| [launch/tf_static.launch.py:3](../launch/tf_static.launch.py#L3) | `base_link 원점 = 뒤축 중심, z = 0 바닥.  모든 값 [EXP-FIXED].` |
| [my_algo/aeb.py:7](../my_algo/aeb.py#L7) | `파라미터: 생성자 인자. drive는 lidar_drive.launch.py, exp는 config/common.yaml [EXP-FIXED] (같은 값).` |
| [my_algo/autonomous_drive.py:17](../my_algo/autonomous_drive.py#L17) | `AEB 임계값은 config/common.yaml [EXP-FIXED]와 같은 값이어야 한다 (test_config_consistency가 확인).` |
| [my_algo/deskew_node.py:11](../my_algo/deskew_node.py#L11) | `실험 관련: deskew on/off는 모든 방법 동일 [EXP-FIXED]. 점별 시각 필드가 없으면 노드가 경고 후 원본을 그대로 넘긴다.` |
| [my_algo/exp_aeb_node.py:15](../my_algo/exp_aeb_node.py#L15) | `[EXP-FIXED] config/common.yaml. 값은 drive 모드 launch(lidar_drive.launch.py)와 같다.` |
| [my_algo/exp_aeb_node.py:16](../my_algo/exp_aeb_node.py#L16) | `vehicle_geometry_file: lidar_to_front_edge, body_width/2 [EXP-FIXED]` |
| [my_algo/exp_drive_mux.py:7](../my_algo/exp_drive_mux.py#L7) | `주행을 시작한 뒤 위반이 생기면 정지 + 회차 폐기 ('foreign_publisher') [EXP-FIXED]` |
| [my_algo/exp_drive_mux.py:8](../my_algo/exp_drive_mux.py#L8) | `2. 조이스틱 끊김 감시(데드맨): /joy가 joy_timeout 이상 안 오면 0 명령, 회차 폐기 기록 [EXP-FIXED]` |
| [my_algo/exp_drive_mux.py:9](../my_algo/exp_drive_mux.py#L9) | `3. 소프트웨어 정지선: AUTO 시작 후 odom 주행 거리 > stop_distance 이면 0 명령 (래치) [EXP-FIXED]` |
| [my_algo/exp_drive_mux.py:10](../my_algo/exp_drive_mux.py#L10) | `4. AEB: /aeb/active=true → 0 명령 (aeb_latch=true면 회차 끝까지 유지) [EXP-FIXED]` |
| [my_algo/exp_steering_node.py:14](../my_algo/exp_steering_node.py#L14) | `파라미터 (config/common.yaml, 전부 [EXP-FIXED], 방법 간 동일):` |
| [my_algo/geometry.py:3](../my_algo/geometry.py#L3) | `노드 안에 치수를 하드코딩하지 않기 위한 단일 진입점. [EXP-FIXED]` |
| [my_algo/metrics.py:24](../my_algo/metrics.py#L24) | `JERK_CUTOFF_HZ = 5.0     # [EXP-FIXED] jerk 저역통과 차단주파수. 방법 간 상대 비교 전용, 문서에 명시` |
| [my_algo/metrics.py:26](../my_algo/metrics.py#L26) | `DECEL_RATIO = 0.95       # [EXP-FIXED] 감속 구간 판정 v < 0.95 v_ref` |
| [my_algo/metrics.py:27](../my_algo/metrics.py#L27) | `VREF_REACH_RATIO = 0.95  # [EXP-FIXED] 가속 구간 끝 v_ref 도달 판정` |
| [my_algo/metrics.py:28](../my_algo/metrics.py#L28) | `DISCARD_TOL = 1.0 - VREF_REACH_RATIO  # [EXP-FIXED] 폐기 허용치: 실측 < v_ref (1 - 0.05)` |
| [my_algo/occupancy.py:6](../my_algo/occupancy.py#L6) | `사라지기 때문이다. 대역 밖 점은 occupied와 free 어디에도 쓰지 않는다. [EXP-FIXED]` |
| [my_algo/occupancy.py:57](../my_algo/occupancy.py#L57) | `"""base_link z 기준 높이 대역 안 점 마스크. [EXP-FIXED] 대역 상한은 박스 높이보다 낮아야 한다."""` |
| [my_algo/occupancy_grid_node.py:14](../my_algo/occupancy_grid_node.py#L14) | `grid_x_min/x_max/y_min/y_max, grid_resolution, max_range   [EXP-FIXED] 격자 범위·해상도` |
| [my_algo/occupancy_grid_node.py:15](../my_algo/occupancy_grid_node.py#L15) | `band_z_min, band_z_max                                     [EXP-FIXED] 높이 대역 (상한 < 박스 높이)` |
| [my_algo/phantom.py:30](../my_algo/phantom.py#L30) | `front: float                 # rear_axle_to_front (vehicle_geometry.yaml) [EXP-FIXED]` |
| [my_algo/phantom.py:31](../my_algo/phantom.py#L31) | `roi_x_max: float = 6.0       # phantom 탐색 앞쪽 한계, base_link x [m] [EXP-FIXED]` |
| [my_algo/phantom.py:32](../my_algo/phantom.py#L32) | `roi_half_y: float = 1.5      # phantom 탐색 좌우 한계 \|y\| [m] [EXP-FIXED]` |
| [my_algo/popout_vehicle_node.py:14](../my_algo/popout_vehicle_node.py#L14) | `파라미터 ([EXP-FIXED], config/popout.yaml):` |
| [my_algo/speed_layer_node.py:12](../my_algo/speed_layer_node.py#L12) | `v_ref, t_r, a_b, s_m        [EXP-FIXED] (t_r, a_b는 [EXP0-MEASURE])` |
| [my_algo/speed_layer_node.py:14](../my_algo/speed_layer_node.py#L14) | `min_speed_policy, restart_margin   [EXP-FIXED]` |
| [my_algo/speed_policy.py:9](../my_algo/speed_policy.py#L9) | `[EXP-FIXED]   t_r, a_b, s_m, v_ref, 최소 속도 정책 → config/common.yaml (3단계)` |
| [my_algo/speed_policy.py:56](../my_algo/speed_policy.py#L56) | `"""목표 속도를 플랫폼이 낼 수 있는 속도로 바꾼다. 모든 방법에 같은 정책을 쓴다. [EXP-FIXED]` |
| [my_algo/trigger_node.py:18](../my_algo/trigger_node.py#L18) | `v_ref [EXP-FIXED] (config/common.yaml)` |

## [EXP-TUNE] 민감도 분석 대상 — bringup 인자로 덮어쓰기 허용 (27개)

| 위치 | 내용 |
|---|---|
| [config/common.yaml:7](../config/common.yaml#L7) | `#   [EXP-TUNE]     민감도 분석 대상. bringup 인자로 개별 덮어쓰기 허용. 권장 탐색 범위 기재` |
| [config/common.yaml:20](../config/common.yaml#L20) | `v_imp: 0.867            # [EXP-TUNE] 허용 충돌속도 [m/s]. 하한 = V_MIN(=4000/4614=0.867, 플랫폼 최소 안정 속도).` |
| [config/common.yaml:23](../config/common.yaml#L23) | `b1_k: 1.0               # [EXP-TUNE] B1 감속 계수 k (v = v_ref(1 - k rho)). 권장 0.5 / 1.0 / 1.5` |
| [config/common.yaml:40](../config/common.yaml#L40) | `n_frames: 3             # [EXP-TUNE] 누적 프레임 수 N. Exp1에서 결정. 권장 1–10` |
| [config/common.yaml:41](../config/common.yaml#L41) | `known_on: 2             # [EXP-TUNE] 히스테리시스: N 중 이 수 이상 관측되면 known. 권장 1–N` |
| [config/common.yaml:42](../config/common.yaml#L42) | `known_off: 1            # [EXP-TUNE] 히스테리시스: 관측 수가 이보다 적으면 unknown 복귀. 1 <= off <= on` |
| [config/common.yaml:50](../config/common.yaml#L50) | `b1_roi_length: 3.0      # [EXP-TUNE] B1 ROI 길이 (앞 끝부터) [m]. 권장 2–4` |
| [config/common.yaml:51](../config/common.yaml#L51) | `b1_roi_half_width: 0.35 # [EXP-TUNE] B1 ROI 반폭 [m]. 기본값은 결과와 무관한 트랙 기준으로 정한다:` |
| [config/common.yaml:57](../config/common.yaml#L57) | `w_min: 0.35             # [EXP-TUNE] P 최소 틈 폭 [m]. 돌출 차량 폭 0.30 + 여유. 권장 0.25–0.45` |
| [config/common.yaml:61](../config/common.yaml#L61) | `row_face_tol: 0.10      # [EXP-TUNE] P 열의 차선 쪽 면에서 이 깊이 안의 칸만 열로 묶음 [m].` |
| [launch/bringup.launch.py:7](../launch/bringup.launch.py#L7) | `# 민감도 분석: [EXP-TUNE] 값만 개별 인자로 덮어쓰기 허용` |
| [launch/bringup.launch.py:15](../launch/bringup.launch.py#L15) | `파라미터 우선순위: common.yaml < method_*.yaml < scenario_*.yaml < 회차 인자(d_e, run_id) < [EXP-TUNE] 인자.` |
| [launch/bringup.launch.py:29](../launch/bringup.launch.py#L29) | `# 개별 인자로 덮어쓸 수 있는 [EXP-TUNE] 값과 타입. 이 밖의 값은 YAML로만 바꾼다.` |
| [launch/bringup.launch.py:145](../launch/bringup.launch.py#L145) | `] + [DeclareLaunchArgument(k, default_value='', description='[EXP-TUNE] 덮어쓰기')` |
| [my_algo/occupancy.py:103](../my_algo/occupancy.py#L103) | `known_on  [EXP-TUNE]: unknown 칸이 known이 되려면 최근 N 프레임 중 이 수 이상 관측` |
| [my_algo/occupancy.py:104](../my_algo/occupancy.py#L104) | `known_off [EXP-TUNE]: known 칸이 unknown으로 돌아가는 기준 (관측 수 < known_off). 1 <= off <= on <= N` |
| [my_algo/occupancy_grid_node.py:16](../my_algo/occupancy_grid_node.py#L16) | `n_frames, known_on, known_off                              [EXP-TUNE] 누적·히스테리시스` |
| [my_algo/phantom.py:33](../my_algo/phantom.py#L33) | `b1_roi_length: float = 3.0   # B1 ROI: 앞 끝부터 길이 [m] [EXP-TUNE]` |
| [my_algo/phantom.py:34](../my_algo/phantom.py#L34) | `b1_roi_half_width: float = 0.35  # B1 ROI 반폭 [m] [EXP-TUNE] (근거: config/common.yaml)` |
| [my_algo/phantom.py:35](../my_algo/phantom.py#L35) | `w_min: float = 0.35          # P 최소 틈 폭 [m] [EXP-TUNE]` |
| [my_algo/phantom.py:40](../my_algo/phantom.py#L40) | `row_face_tol: float = 0.10   # [EXP-TUNE] P 주차열 차선 쪽 면에서 이 깊이 안의 칸만 열로 묶음 [m].` |
| [my_algo/speed_layer_node.py:13](../my_algo/speed_layer_node.py#L13) | `v_imp, b1_k                 [EXP-TUNE] (v_imp 하한 = V_MIN)` |
| [my_algo/speed_policy.py:10](../my_algo/speed_policy.py#L10) | `[EXP-TUNE]    v_imp (하한 = v_min), k` |
| [sim/runner.py:6](../sim/runner.py#L6) | `sim/.venv/bin/python sim/runner.py --method P --scenario S3 --w-min 0.30 --video         # [EXP-TUNE] + 위에서 본 영상` |
| [sim/runner.py:435](../sim/runner.py#L435) | `ap.add_argument('--' + k.replace('_', '-'), dest=k, type=typ, default=None, help='[EXP-TUNE]')` |
| [sim/simconfig.py:4](../sim/simconfig.py#L4) | `< 회차 값(d_e, run_id) < [EXP-TUNE] 인자 < 시뮬 전용(sim.yaml mux_overrides·timing_overrides, 지오메트리 파일 경로).` |
| [sim/simconfig.py:59](../sim/simconfig.py#L59) | `raise ValueError(f'[EXP-TUNE]가 아닌 값은 바꿀 수 없다: {sorted(bad)}')` |

## [EXP0-MEASURE] Exp0 실측 후 교체할 가정값 (38개)

| 위치 | 내용 |
|---|---|
| [config/common.yaml:8](../config/common.yaml#L8) | `#   [EXP0-MEASURE] Exp0 실측 후 교체할 가정값 (docs/OPEN_ITEMS.md)` |
| [config/common.yaml:15](../config/common.yaml#L15) | `t_r: 0.15               # [EXP-FIXED][EXP0-MEASURE] 전체 지연 [s] = 관측 → 실제 감속 시작 (인지 + 통신 + 제어 + 구동,` |
| [config/common.yaml:18](../config/common.yaml#L18) | `a_b: 3.0                # [EXP-FIXED][EXP0-MEASURE] 최대 감속 [m/s^2]. v_cap 식 공통. Exp0에서 /imu로 측정` |
| [config/common.yaml:84](../config/common.yaml#L84) | `trigger_latency: 0.05   # [EXP0-MEASURE] 트리거 발행 → #2 출발 지연 [s]. Exp0 측정` |
| [config/common.yaml:85](../config/common.yaml#L85) | `popout_enter_time: 0.2  # [EXP0-MEASURE] #2 출발 → 앞부분이 차선 경계에 닿는 시간 [s]. Exp0 측정` |
| [config/common.yaml:90](../config/common.yaml#L90) | `time_field: timestamp   # [EXP0-MEASURE] 점별 시각 필드 이름 (check_vehicle.sh 로그로 확정)` |
| [config/common.yaml:91](../config/common.yaml#L91) | `time_scale: 1.0e-9      # [EXP0-MEASURE] 필드 값 → 초` |
| [config/common.yaml:92](../config/common.yaml#L92) | `time_mode: absolute     # [EXP0-MEASURE] absolute \| offset` |
| [config/imu.yaml:18](../config/imu.yaml#L18) | `# [EXP0-MEASURE] 자이로 바이어스 [rad/s]. 실험일마다 imu_static_calib.py로 재측정 후 교체.` |
| [config/popout.yaml:10](../config/popout.yaml#L10) | `lane_boundary_distance: 0.3        # [EXP0-MEASURE] #2 출발 위치 → 차선 경계 거리 [m] (진입 시간 기록용)` |
| [config/popout.yaml:14](../config/popout.yaml#L14) | `erpm_gain: 4614.0                  # [EXP0-MEASURE] #2 차량의 m/s -> ERPM. 에고 값과 다를 수 있음. #2에서 실측` |
| [config/popout.yaml:15](../config/popout.yaml#L15) | `servo_center: 0.5                  # [EXP0-MEASURE] #2 직진 서보값` |
| [config/scenario_E0.yaml:9](../config/scenario_E0.yaml#L9) | `stop_distance: 4.5         # [EXP0-MEASURE] 제동 시작 위치 = v_ref 도달 후. 트랙 끝까지 정지거리(약 1 m) + 여유 확보` |
| [config/scenario_S1.yaml:8](../config/scenario_S1.yaml#L8) | `collision_x: 4.775          # [EXP0-MEASURE] P1–P2 틈 중앙. track_layout.layout()의 계산값 사본 (다르면 실행 시 경고)` |
| [config/scenario_S1.yaml:14](../config/scenario_S1.yaml#L14) | `row_x_start: 4.0           # [EXP0-MEASURE] 주차열 P1 시작 x [m]. 트랙 배치의 유일 출처 (track_layout.py)` |
| [config/scenario_S1.yaml:15](../config/scenario_S1.yaml#L15) | `row_x_end: 6.55             # [EXP0-MEASURE] track_layout 계산값 사본 (row_x_start + n box_len + (n-1) gap)` |
| [config/scenario_S2.yaml:8](../config/scenario_S2.yaml#L8) | `collision_x: 4.775          # [EXP0-MEASURE] P1–P2 틈 중앙. track_layout.layout()의 계산값 사본 (다르면 실행 시 경고)` |
| [config/scenario_S2.yaml:14](../config/scenario_S2.yaml#L14) | `row_x_start: 4.0           # [EXP0-MEASURE] 주차열 P1 시작 x [m]. 트랙 배치의 유일 출처 (track_layout.py)` |
| [config/scenario_S2.yaml:15](../config/scenario_S2.yaml#L15) | `row_x_end: 6.55             # [EXP0-MEASURE] track_layout 계산값 사본 (row_x_start + n box_len + (n-1) gap)` |
| [config/scenario_S3.yaml:8](../config/scenario_S3.yaml#L8) | `collision_x: 4.625          # [EXP0-MEASURE] P1–P2 틈 중앙. track_layout.layout()의 계산값 사본 (다르면 실행 시 경고)` |
| [config/scenario_S3.yaml:14](../config/scenario_S3.yaml#L14) | `row_x_start: 4.0           # [EXP0-MEASURE] 주차열 P1 시작 x [m]. 트랙 배치의 유일 출처 (track_layout.py)` |
| [config/scenario_S3.yaml:15](../config/scenario_S3.yaml#L15) | `row_x_end: 5.95             # [EXP0-MEASURE] track_layout 계산값 사본 (row_x_start + n box_len + (n-1) gap)` |
| [config/scenario_S4.yaml:8](../config/scenario_S4.yaml#L8) | `collision_x: 4.55          # [EXP0-MEASURE] P1–P2 틈 중앙. track_layout.layout()의 계산값 사본 (다르면 실행 시 경고)` |
| [config/scenario_S4.yaml:14](../config/scenario_S4.yaml#L14) | `row_x_start: 4.0           # [EXP0-MEASURE] 주차열 P1 시작 x [m]. 트랙 배치의 유일 출처 (track_layout.py)` |
| [config/scenario_S4.yaml:15](../config/scenario_S4.yaml#L15) | `row_x_end: 5.65             # [EXP0-MEASURE] track_layout 계산값 사본 (row_x_start + n box_len + (n-1) gap)` |
| [config/vehicle_geometry.yaml:11](../config/vehicle_geometry.yaml#L11) | `wheel_diameter: 0.0          # A5  [EXP0-MEASURE] 미측정 — odom 보정 검산용` |
| [launch/car2.launch.py:46](../launch/car2.launch.py#L46) | `# #2 계수는 popout.yaml (#2 실측 [EXP0-MEASURE]), 휠베이스·서보 게인은 에고와 같은 차종으로 가정 (TBD(세션 1))` |
| [launch/tf_static.launch.py:4](../launch/tf_static.launch.py#L4) | `실측 2026-11-03. 표시된 [EXP0-MEASURE] 값은 아직 가정값이므로 측정 후 교체.` |
| [launch/tf_static.launch.py:15](../launch/tf_static.launch.py#L15) | `ORIGIN_H = 0.03    # [EXP0-MEASURE] 장착면 -> 점군 원점 O 높이. 임시값(센서 높이 절반).` |
| [launch/tf_static.launch.py:19](../launch/tf_static.launch.py#L19) | `LIDAR_YAW = 0.0    # [EXP0-MEASURE] B5: 로고가 전방이면 0, 후방이면 3.14159, 좌측 1.5708, 우측 -1.5708` |
| [launch/tf_static.launch.py:20](../launch/tf_static.launch.py#L20) | `LIDAR_ROLL = 0.0   # [EXP0-MEASURE] C1: 정지 IMU 평균 roll = atan2(ay, az)` |
| [launch/tf_static.launch.py:21](../launch/tf_static.launch.py#L21) | `LIDAR_PITCH = 0.0  # [EXP0-MEASURE] C1: 정지 IMU 평균 pitch = atan2(-ax, sqrt(ay^2 + az^2))` |
| [my_algo/imu_republisher.py:12](../my_algo/imu_republisher.py#L12) | `파라미터 (config/imu.yaml): accel_unit (기본 g), gyro_unit, gyro_bias [EXP0-MEASURE], gravity_ref,` |
| [my_algo/speed_layer_node.py:12](../my_algo/speed_layer_node.py#L12) | `v_ref, t_r, a_b, s_m        [EXP-FIXED] (t_r, a_b는 [EXP0-MEASURE])` |
| [my_algo/speed_policy.py:11](../my_algo/speed_policy.py#L11) | `[EXP0-MEASURE] t_r, a_b` |
| [my_algo/trigger_node.py:17](../my_algo/trigger_node.py#L17) | `trigger_latency, popout_enter_time [EXP0-MEASURE] (config/common.yaml)` |
| [scripts/measure_lidar_height.py:2](../scripts/measure_lidar_height.py#L2) | `"""평평한 바닥에서 라이다 원점 높이와 장착 roll/pitch를 측정한다. [EXP0-MEASURE]` |
| [sim/config/sim.yaml:54](../sim/config/sim.yaml#L54) | `#   trigger_latency = 0 (시뮬 /trigger 전달 지연 없음). 실차 값은 Exp0까지 [EXP0-MEASURE] 유지` |
