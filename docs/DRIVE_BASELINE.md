# 기본 주행 (main에서 온 실차 baseline)

이 문서는 `main`/`refactor/main-baseline`의 루트 README였다. `exp/aur-overtake`에서는 기본 주행 설명으로 옮겨 두었다 (내용 그대로, 관측 공백·IMU 안내만 갱신). 추월 실험은 루트 `README.md`와 `docs/PIPELINE.md`.

실차 F1TENTH용 ROS2 자율주행 패키지. LiDAR만으로 지도 없이(mapless) 주행한다.

- 차량: Jetson Orin Nano, ROS2 **Foxy**, Python 3.8
- 주행 입력: Livox 점군 → `/scan`, VESC `/vesc/odom`. **IMU·TF는 쓰지 않는다.**
- 카메라(RealSense D455)는 켜지지만 화면 표시만 한다. 제어 명령은 내지 않는다.
- 이전 연구(관측 공백)는 `occlusion_archive/README.md` (보관용, 실행 대상 아님).

## 실행

```bash
cd ~/f1tenth_ws
colcon build --packages-select my_algo
source install/setup.bash
# Livox 드라이버와 VESC 드라이버를 먼저 띄운 뒤
ros2 launch my_algo lidar_drive.launch.py
```

launch 인자 (기본값 = 평소 주행):

| 인자 | 기본 | 설명 |
|---|---|---|
| `use_camera` | `true` | RealSense 드라이버와 `camera_reactive_drive` 실행 |
| `use_metrics_logger` | `true` | trial CSV 로거 실행 |
| `use_sim_time` | `false` | rosbag 재생 검증용 |

`autonomous_drive.launch.py`는 같은 노드에 rosbridge(9090)를 더한 것이고, `camera_drive.launch.py`는 카메라만 띄운다 (제어 없음).

처음에는 바퀴를 띄운 상태에서 `/scan`, `/vesc/odom`, `/commands/motor/speed`, `/commands/servo/position`을 확인한다.

## ⚠ 조이스틱 없이 바로 자율주행을 시작한다

`lidar_drive.launch.py`를 실행하면 **바로 AUTO로 출발한다.**
`lidar_reactive_drive`와 `autonomous_drive`의 `start_autonomous`가 `True`이고, launch에서 `joy_node`·`joy_controller`가 주석 처리되어 있다.
(joy_controller를 켜면 시작 시 `/autonomous_mode=False`를 발행하므로 LB를 눌러야 출발한다.)

이 상태에서 쓸 수 있는 정지 수단:

| 수단 | 동작 |
|---|---|
| 소프트웨어 정지 | `ros2 topic pub --once /autonomous_mode std_msgs/msg/Bool "{data: false}"` → 두 노드가 AUTO를 해제하고 rpm 0·조향 중앙. autonomous_drive가 이후 False를 계속 발행해 유지된다 |
| planner 정지선 | 진행 띠(±0.25 m) 안 최근접 장애물 ≤ `0.35 + 0.45·v + 0.20` m (최대 1.5 m)이면 rpm 0 |
| gap 없음 정지 | 통과할 틈이 없고 전방 ≤ 0.75 m이면 rpm 0 |
| AEB | 띠(±0.30 m) 안 최근접 ≤ max(0.25, v²/(2·2.8)+0.20, 0.55·v) m이면 rpm 0. `/scan`이 안 들어와도 정지 |
| 명령 타임아웃 | planner 명령이 1.0 s 넘게 안 오면 rpm 0, 조향 중앙 |
| Ctrl+C | 노드 종료 시 정지 명령 발행을 시도한다. Foxy에서 실제로 나가는지는 확인되지 않았다 |

조이스틱 B 버튼(비상 정지)은 joy 노드가 꺼져 있으므로 **쓸 수 없다.** VESC 드라이버의 명령 타임아웃 동작도 확인되지 않았다. 손으로 차를 잡을 수 있는 상태에서 시험한다.

## 노드와 토픽

```text
/livox/lidar ─ pointcloud_to_laserscan ─ /scan
/scan, /vesc/odom ─ lidar_reactive_drive ─ /reactive/lidar_command (JSON)
/reactive/lidar_command, /scan, /vesc/odom ─ autonomous_drive (AEB 포함, 50 Hz)
    ─ /commands/motor/speed (ERPM), /commands/servo/position
    ─ /autonomous_mode (10 Hz heartbeat), /aeb/active, /aeb/status
/camera/color/image_raw, /camera/depth/image_rect_raw ─ camera_reactive_drive
    ─ /camera/perception/annotated_image, /camera/perception/status   (제어 없음)
autonomous_metrics_logger ─ logs/*.csv
```

| 파일 | 역할 |
|---|---|
| `my_algo/baseline/lidar_reactive_drive.py` | scan → 주행 모드 선택 → 조향·RPM 계획 |
| `my_algo/baseline/autonomous_drive.py` | planner 명령 선택, AEB 검사, VESC 명령 발행 |
| `my_algo/baseline/aeb.py` | TTC·제동거리 기반 긴급 정지 판정 |
| `my_algo/baseline/vehicle_utils.py` | RPM·서보 변환, 최소 속도 규칙, 공통 유틸 |
| `my_algo/baseline/joy_controller.py` | 수동 주행·AUTO 토글·B 비상 정지 (현재 launch에서 꺼짐) |
| `my_algo/baseline/camera_reactive_drive.py` | D455 YOLO·거리·차선·AEB overlay |
| `my_algo/baseline/autonomous_metrics_logger.py` | AUTO 구간 trial 기록 |

## 속도 상수 (`my_algo/baseline/vehicle_utils.py`)

```text
ERPM_GAIN       = 4614    ERPM = m/s × 4614
MIN_DRIVING_RPM = 4000    약 0.87 m/s
CRUISE_RPM      = 5740
MAX_RPM         = 12000
SERVO           = 0.5 − 0.60 × steering_rad, [0.03, 0.97]로 제한
```

모든 모터 명령은 `sanitize_rpm()`을 거친다: 0이면 0, 0이 아니고 |rpm| < 4000이면 ±4000으로 올림, ±12000으로 제한, NaN은 0.
출발 토크 부족을 막기 위한 값이라 바꾸지 않는다 (`test/test_drive_regression.py`가 값을 고정한다).

## 파라미터

- 차량·AEB 값은 `launch/lidar_drive.launch.py` 안 dict에 있다 (vehicle_width 0.30, safety_margin 0.25, lookahead 2.6 m, max_steering 0.78, aeb_ttc_threshold 0.55, aeb_vehicle_half_width 0.15, aeb_path_margin 0.15, lidar_to_bumper_dist 0.30 …).
- planner 내부 상수 38개는 `lidar_reactive_drive.py`의 `PLANNER_CONSTANTS`, AEB 감속도·여유는 `aeb_comfortable_decel`(2.8), `aeb_stopping_margin`(0.20) 파라미터다. launch는 이 값들을 덮어쓰지 않는다.
- **실제로 적용된 값 확인**: Foxy에서는 `--params-file`이 launch dict/`-p`보다 우선할 수 있다. 두 노드는 시작할 때 다음 한 줄을 출력한다.

  ```text
  [lidar_reactive_drive]: effective parameters: blocked_stop_distance=0.75, ...
  ```

  실행 중에는 `ros2 param dump /lidar_reactive_drive --print` (Foxy는 `--print`가 있어야 표준출력으로 나온다) 또는 `ros2 param get /autonomous_drive aeb_ttc_threshold`.

## 알고리즘 요약 (`lidar_reactive_drive.py`)

1. 좌표: 전방 ±120° 유효 빔을 `x = r·cosθ − 0.30` (앞범퍼 기준), `y = r·sinθ`로. yaw 보정 없음.
2. lookahead = clamp(2.6 + 0.9·v, 2.6, 4.5).
3. 모드: 진행 띠 안 최근접이 회피 거리(1.2 + 0.45·v)보다 가깝고 gap이 있으면 `OBSTACLE_AVOIDANCE`, 측면 벽 신뢰도 > 0.5면 `WALL_FOLLOW`, 아니면 `OPEN_SPACE`. 모드는 최소 0.45 s 유지. cone 모드는 꺼져 있다.
4. gap 점수 = 1.2·폭/1.6 + 여유/lookahead + 0.55·중앙 보너스(45°) − 0.75·|각|/120° − 0.45·|각 − 이전 조향| (각 항은 0–1로 제한).
5. gap 조향 = 중심각 × 0.70 (회피 모드 1.10), |조향| < 0.055면 0.
   RPM = (5740 + (12000 − 5740)·min(폭/1.5, 1)) × (1 − 0.35·조향비) × (0.45 + 0.55·전방 여유비).
   gap이 없으면 열린 쪽으로 최대 조향의 55%, RPM 4000.
6. 벽 추종: 오차 = 왼쪽 − 오른쪽 거리 (한쪽만 보이면 목표 0.85 m 기준), 지수 필터 α = 0.25, 조향 = 0.30·오차, RPM = 5740 × (1 − 0.2·조향비).
7. 조향 변화는 scan마다 0.08 rad (회피·정지 0.22 rad)로 제한.

## 테스트

ROS 없이 개발 PC에서 돈다.

```bash
python3 -m pytest test                          # 회귀 비교 + 최소 속도 규칙 (+ 추월 실험 단위 테스트)
python3 scripts/verify_refactor_offline.py --seeds 12   # 리팩터링 전 main(b054758)과 발행 메시지 전체 비교
```

`verify_refactor_offline.py`는 가짜 rclpy로 두 노드를 합성 입력(벽, 주차 박스, 접근 장애물, inf/NaN, 속도 변화, AUTO 토글, 명령 누락)에 돌려 메시지를 하나씩 비교한다.
합성 입력은 최소 속도 클램프 구간을 거의 지나지 않으므로 그 규칙은 별도 테스트로 고정한다.
rosbag 재생 검증(`scripts/replay_drive.sh`)은 Foxy에서 아직 돌지 않는다 (`docs/OPEN_ITEMS.md`).

## 그 밖

- IMU 단독 속도 기록 도구는 `archive/imu_tools/`로 옮겼다 (빌드·테스트에서 빠짐).
- 카메라 의존성: `realsense2_camera`, `cv_bridge`, `ultralytics`(YOLO, 기본 `yolov8n.pt`). Foxy·Python 3.8에서 맞는 버전은 차량에 설치된 것을 기준으로 한다.
