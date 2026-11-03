# 미해결 항목

차량 세션 결과가 들어오면 갱신한다. 태그 전체 목록은 [EXP_TAGS.md](EXP_TAGS.md).

## 1. [EXP0-MEASURE] 가정값

| 항목 | 현재 가정값 | 측정 방법 | 바꿀 파일 · 키 |
|---|---|---|---|
| 라이다 원점 높이 ORIGIN_H | 0.03 (z = 0.24) | `measure_lidar_height.py` (세션 1 f) | `launch/tf_static.launch.py` `ORIGIN_H` |
| 라이다 roll, pitch | 0, 0 | `imu_static_calib.py` 0°/180° 평균 (세션 1 e), 바닥 평면과 교차 확인 | `launch/tf_static.launch.py` `LIDAR_ROLL`, `LIDAR_PITCH` |
| 라이다 yaw | 0 | 로고 방향(세션 1 g) + RViz 벽 평행 검사 | `launch/tf_static.launch.py` `LIDAR_YAW` |
| 자이로 바이어스 | 0, 0, 0 | `imu_static_calib.py` (실험일마다) | `config/imu.yaml` `gyro_bias` |
| IMU 가속도 단위 | g | `check_vehicle.sh` IMU 정지 통계 | `config/imu.yaml` `accel_unit` |
| 바퀴 지름 | 미측정 | 자 또는 5바퀴 굴림 (세션 1 g) | `config/vehicle_geometry.yaml` `wheel_diameter` |
| a_b | 3.0 m/s² | Exp0: 2.0 m/s 급제동 10회, `/imu/linear_accel_base` x | `config/common.yaml` `a_b` |
| t_r | 0.15 s | Exp0: 명령 0 발행 시각 → 감속 시작(IMU) 지연 | `config/common.yaml` `t_r` |
| 트리거 지연 | 0.05 s | Exp0: `/trigger` stamp vs #2 `latency_s`, LED+카메라 | `config/common.yaml` `trigger_latency` |
| #2 진입 시간 | 0.2 s | Exp0: #2 출발 → 앞 끝 차선 경계 (천장 카메라) | `config/common.yaml` `popout_enter_time` |
| #2 ERPM 계수, 직진 서보 | 4614, 0.5 | #2에서 odom 스케일 보정과 같은 방법 | `config/popout.yaml` `erpm_gain`, `servo_center` |
| 트랙 좌표 | collision_x 5.0, stop_distance 7.5, accel_end_x 2.5, test_x 2.5–7.0, row_x 4.0–6.3 | 트랙 마킹 후 줄자 | `config/scenario_S*.yaml` |
| 좌측 벽 목표 거리 | 0.355 | 좌측 벽 ~ 차선 왼쪽 경계 거리 실측 → 식은 `common.yaml` 주석 | `config/common.yaml` `target_wall_distance` |
| 점별 시각 필드 (deskew) | `timestamp`, 1e-9, absolute | `check_vehicle.sh` 점군 필드 목록 | `config/common.yaml` `time_field`, `time_scale`, `time_mode` |

| #2 속도 | 0.8 m/s | Exp0 e: 0.8·0.9 각 5회, 판정 규칙은 EXPERIMENT_PLAN 5절 | `config/popout.yaml` `v_popout` (+ EXPERIMENT_PLAN 8절 기록) |
| #2 출발 → 차선 경계 거리 | 0.3 m | 트랙 마킹 후 줄자 | `config/popout.yaml` `lane_boundary_distance` |
| E0 정지선 | 4.5 m | 트랙 길이에 맞춤 | `config/scenario_E0.yaml` `stop_distance` |

## 2. 속도 환산 일관성 (확인 대기)

- 명령용 `ERPM_GAIN` = **4614** (`my_algo/vehicle_utils.py`, drive·exp 공통 유일 출처)
- odom 계산용 `speed_to_erpm_gain` = **미확인** (VESC 드라이버 설정, 레포 밖)
  → 세션 1 `check_vehicle.sh` 로그의 "속도 환산 계수" 절에서 확인한다.
- 두 값이 다르면: 명령 2.0 m/s에 odom은 다른 속도를 보고한다. 실측 기준 손실, v_ref 도달 판정,
  트리거 거리(odom 누적), AEB의 에고 속도가 모두 영향을 받는다.
- **지금은 고치지 않는다.** 두 값과 `V_MIN` 환산값은 회차마다 `meta.json`에 기록된다
  (`erpm_gain`, `vesc_speed_to_erpm_gain`, `v_min`). `run_meta.py`는 값이 다르면 경고를 낸다.

### Exp0 odom 스케일 보정 절차 (계획)
1. 바닥에 5 m 직선 표시. 저속(0.9 m/s)과 2.0 m/s로 각 3회 직진.
2. 줄자 거리 / odom 주행 거리 비율로 실제 속도-ERPM 계수를 구한다.
3. 두 값을 같은 보정값으로 맞춘다 (VESC `speed_to_erpm_gain`, `vehicle_utils.ERPM_GAIN`).

> **경고:** `ERPM_GAIN`을 바꾸면 drive 모드의 속도도 바뀐다 (planner RPM은 그대로인데 m/s 해석이 달라지고,
> `MIN_DRIVING_RPM` 환산 V_MIN도 바뀐다). 승인 후 별도 커밋으로 하고 verify_refactor 기준선을 다시 잡는다.

## 3. 코드 분석 중 발견한 확인 필요 사항

1. **같은 ROS_DOMAIN_ID에서 토픽 충돌.** 에고와 #2가 `/commands/motor/speed`, `/vesc/odom`, `/tf`를 같은 이름으로
   쓰면 #2 명령이 에고 VESC로 들어갈 수 있다. #2는 namespace(`car2`)로 드라이버를 띄우고 `publish_tf: false`.
   `popout_vehicle_node`는 상대 토픽이라 namespace를 따른다. 차량에서 `ros2 topic info -v /commands/motor/speed`로
   발행자가 에고 노드뿐인지 확인할 것.
2. **#2의 0.8 m/s는 에고 최소 속도(0.867 m/s, 4000 ERPM)보다 낮다.** #2 노드는 최소 RPM 규칙을 적용하지 않는다.
   #2가 0.8 m/s에서 떨림 없이 출발·유지하는지 Exp0에서 확인.
3. **realsense 토픽 이름** (`/camera/color/...` vs `/camera/camera/...`): drive launch의 camera_reactive_drive 구독
   이름이 실제와 다르면 YOLO에 영상이 안 들어가고 있을 수 있다. 녹화 토픽(`CAMERA_TOPIC`)도 확인 후 확정.
4. **joy_node autorepeat**: 입력이 없어도 `/joy`가 약 20 Hz로 오는지 `ros2 topic hz /joy`로 확인
   (안 오면 데드맨이 AUTO 직후 정지시킨다). 무선 패드가 범위를 벗어날 때 `/joy`가 끊기는지도 확인.
5. **scan 높이 필터**: `pointcloud_to_laserscan`이 livox_frame z −0.1~0.5를 쓴다. 라이다 높이 0.24 m면
   바닥에서 0.14 m 아래 물체는 AEB·조향에 안 보인다 (drive와 exp 동일, 바꾸지 않음).
6. **점유격자 free 근거**: 높이 대역(0.05–0.30 m) 안 점만 쓰므로 바닥 점은 free를 만들지 않는다.
   빈 바닥은 벽·박스로 가는 광선이 지나갈 때만 free가 된다. 트랙이 벽으로 둘러싸여 있어야 한다.
7. **exp 조향은 좌측 벽만 본다.** 좌측 벽이 일정 구간 안 보이면 직전 조향을 유지한다.
8. **lidar_to_front_edge(0.28)**는 `vehicle_geometry.yaml`의 파생값이다. `tf_static.launch.py`의 `LIDAR_X`를
   바꾸면 이 값도 함께 바꿔야 한다.
9. **AEB는 장애물 상대속도를 쓰지 않는다** (drive와 같은 정의). 분석의 최소 TTC는 상대속도 기반과
   에고 속도 기반(`min_ego_ttc`)을 둘 다 낸다.

## 4. TBD(세션 1) — 차량 세션 1 결과로 확정할 이름·값

문서에서 "TBD(세션 1)"로 표시한 곳은 모두 여기로 모인다. 출처는 `scripts/vehicle_session_1.md`의 파일.

| 항목 | 현재 기본값 (추정) | 출처 | 바꿀 곳 |
|---|---|---|---|
| 점군 토픽 | `/livox/lidar` | b `check_vehicle_*.log` | bringup `cloud_topic`, `record_run.sh` `CLOUD_TOPIC`, `config/preflight.yaml`, `config/exp.rviz` |
| 원본 IMU 토픽 | `/livox/imu` | b | bringup `imu_topic`, `config/imu.yaml`, `RAW_IMU_TOPIC`, `config/preflight.yaml` |
| odom 토픽 | `/vesc/odom` | b | bringup `odom_topic`, `ODOM_TOPIC` |
| VESC 상태 토픽·전압 필드 | `/sensors/core`, `state.voltage_input` | b (VESC 상태 echo) | `VESC_STATE_TOPIC`, `VESC_VOLTAGE_FIELD` |
| 압축 카메라 토픽 | `/camera/camera/color/image_raw/compressed` | b (토픽 목록) | `CAMERA_TOPIC` |
| Livox 드라이버 노드 이름 | `/livox_lidar_publisher` | b, b2 | `config/preflight.yaml` |
| realsense 노드 이름 (`/tf_static` 발행) | `/camera/camera` | b2 | `config/preflight.yaml` |
| vesc_to_odom 노드 이름, `publish_tf` | 자동 탐색 (이름에 odom) | b | `VESC_ODOM_NODE`, VESC 설정 YAML |
| `speed_to_erpm_gain` | 미확인 | b | 2절 |
| 점별 시각 필드 | `timestamp` | b (점군 필드) | `config/common.yaml` `time_field` 등 |
| VESC 패키지·실행 파일 이름 (#2) | `vesc_driver/vesc_driver_node`, `vesc_ackermann/vesc_to_odom_node` | b (패키지 목록) | `car2.launch.py` 인자 기본값 |
| Livox·VESC 드라이버 실행 명령 (RUNBOOK T1) | — | 평소 쓰는 명령 | `docs/RUNBOOK.md` 1절 |
| `/joy` 재발행 주기 | 20 Hz 기대 | b2 `b2_joy_hz.txt` | 안 나오면 joy_node 설정 |
| 배터리 셀 수·교체 기준 전압 | — | 팀 결정 | `docs/RUNBOOK.md` 0.2 |
| 물리 전원 차단 위치 | — | 차량 | `docs/RUNBOOK.md` 2절 |
| 회차당 bag 크기 | 약 0.6 GB 추정 | 첫 회차 `meta.json` `bag_bytes` | `docs/LOGGING.md` 6절 |
| drive 모드 리팩터 불변 (rosbag) | 오프라인 비교만 완료 | d `d_verify.txt` | 차이 나면 3단계 중단 |

## 5. 문서 요구사항 중 남은 것

- 4.1 형식 docstring: exp 노드와 `imu_republisher`는 형식을 갖췄다. **drive 모드 노드 파일**
  (`autonomous_drive.py`, `lidar_reactive_drive.py`, `aeb.py`, `camera_reactive_drive.py`, `joy_controller.py`,
  `autonomous_metrics_logger.py`)은 수정 금지 조건 때문에 기존 docstring 그대로다 (입출력 토픽은 이미 적혀 있음).
  docstring만 고치는 것을 허용하면 같은 형식으로 맞추고 verify_refactor로 불변을 확인한다.

## 6. Foxy 점검 결과 (2026-11-03)

차량 ROS2는 Foxy (Ubuntu 20.04, Python 3.8). Foxy 브랜치 소스(rosbag2, rclpy, ros2cli, geometry2, launch_ros,
common_interfaces, joystick_drivers foxy-devel, f1tenth/vesc ros2, livox_ros_driver2)를 직접 확인했다.

| 항목 | Foxy 사실 | 조치 |
|---|---|---|
| static_transform_publisher | 이름 붙은 인자 없음. 위치 인자 `x y z yaw pitch roll parent child` (소스 `setRPY(args[6], args[5], args[4])`) | `tf_static.launch.py` 위치 인자, 값 불변 |
| `ros2 bag play` | `--clock` 없음 (`--rate`, `--topics`, `--qos-profile-overrides-path`는 있음) | `scripts/bag_player.py` (sqlite3 직접 읽기 + bag 시간 `/clock`) |
| `ros2 bag record` | `--use-sim-time` 없음, 기본 sqlite3 | `-s sqlite3`, 오프라인 재생은 `/clock`을 같이 기록해 분석에서 변환 |
| `rosbag2_py` | Foxy 레포에 없음 | 분석은 rosbags 0.9.23 (Python 3.8 지원 마지막, 0.10+는 3.10 이상) |
| `ros2 topic echo` | `--once`, `--field` 없음 | `scripts/echo_once.py` |
| `ros2 param dump` | 기본이 파일 저장, `--print`로 표준출력 | `record_run.sh`, `replay_drive.sh`, `check_vehicle.sh`에 `--print` |
| `ros2 topic info -v`, `get_publishers_info_by_topic` | 있음 | 그대로 |
| tf2_tools | `view_frames.py` | 문서 수정 |
| `sensor_msgs_py.read_points` | 튜플 생성기 (Humble은 numpy 구조화 배열), 설치 여부 불확실 | `my_algo/pc2.py` 직접 파싱, 의존성 제거 |
| rclpy QoS | `DurabilityPolicy`, `ReliabilityPolicy` 별칭 있음 | 그대로 |
| rclpy 로거 `throttle_duration_sec`, `rclpy.serialization` | 있음 | 그대로 |
| launch_ros 파라미터 치환 | 문자열을 YAML로 해석 (`'false'` → bool) | `lidar_drive.launch.py` `use_sim_time` 인자 그대로 |
| joy_node `autorepeat_rate` | 기본 20 Hz, 입력이 없어도 타임아웃 경로에서 재발행 | 데드맨 동작. 단 무선 패드가 범위를 벗어나도 수신기가 연결돼 있으면 `/joy`가 계속 올 수 있다 (차량 확인) |
| Python 3.8 문법 | 3.9+ 문법·API 없음 (3.8 문법 파서로 전 파일 확인) | 변경 없음 |
| numpy | Ubuntu 20.04 apt 1.17.4. 쓰는 API(`default_rng`, `quantile`, `minimum.at` 등)는 1.17에 있음 | 1.17 자체 실행은 미확인 (arm64 Mac용 패키지 없음, 1.21.6으로 테스트) |

### 차량 구성에서 새로 확인한 것 (세션 1에서 확정)
- **`ros2 launch vesc_driver vesc_driver_node.launch.py`는 vesc_driver만 띄운다** (f1tenth/vesc ros2 브랜치 기준).
  vesc_to_odom이 없으면 `/vesc/odom`과 odom→base_link TF가 없다. 이 경우 **기존 drive 모드는 속도 0으로 동작해 왔다**
  (AEB 동적 거리 = max(0.25, 0.20, 0) = 0.25 m 고정, planner lookahead 2.6 m 고정). `~/vesc_ws`가 수정돼 있을 수 있어
  세션 1 (b) `b_odom_info.txt`로 확정한다. exp 세션은 `vesc_odom.launch.py`로 odom을 낸다 (drive에는 쓰지 않음).
- vesc_to_odom은 `speed_to_erpm_gain`, `wheelbase`, 서보 계수를 기본값 없이 요구하는데 기본 `vesc_config.yaml`에는 없다.
  `vesc_odom.launch.py`·`car2.launch.py`는 이 값을 레포의 유일 출처(ERPM_GAIN, SERVO_*, vehicle_geometry, popout.yaml)에서 넘긴다.
- Livox: `rviz_MID360_launch.py`는 xfer_format 0(PointCloud2), 노드 이름 `livox_lidar_publisher`. Jetson RViz를 빼려고
  `livox_mid360.launch.py`를 쓴다. 점별 시각 필드 이름·단위는 세션 1 `b_cloud_fields.txt`로 확인 (deskew).

### 로컬에서 검증한 것
- Python 3.8.20 + numpy 1.21.6 + rosbags 0.9.23 격리 환경에서 단위 테스트 전부 통과, drive 오프라인 비교 일치.
- 합성 Foxy 형식 bag(rosbags로 기록) → `extract_run.py` 전체 경로 (ΔT, 감속 구간, 충돌 판정 값 확인).
- `bag_player.py`의 sqlite3 읽기·필터·순서·일정 (rosbags로 기록한 bag 포함).
- `tf_static.launch.py`가 만드는 인자 목록 (가짜 launch 모듈로 렌더링).

### 차량에서만 확인 가능한 것
- Foxy에서 모든 launch·노드가 실제로 뜨는지 (로컬에 ROS·Docker 없음).
- `bag_player.py` 재생이 `ros2 bag play`와 같은 결과를 주는지 → 세션 1 (d)가 이것까지 함께 검증한다.
- `ros2 bag record` QoS 적응 (Livox BEST_EFFORT 발행 토픽이 빠짐없이 기록되는지), 기록 중 토픽 주기 저하 없음.
- `/imu` 정지 시 z ≈ 9.8, IMU 축 방향, exp 모드 AEB 단독 정지, RViz 검증, `/joy` 주기와 무선 범위 이탈 동작.
- numpy 1.17.4에서의 실행.
- livox_ros_driver2·vesc 패키지 이름·노드 이름·토픽 이름 (4절 TBD 표).
