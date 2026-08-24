# my_algo

실차 F1TENTH용 ROS2 자율주행 스택이다. 현재 기본 주행은 LiDAR 기반이며,
카메라는 Intel RealSense D455로 객체/거리/차선 인식 상태를 확인하는
perception overlay 용도로 사용한다.

## 코드 구조

```text
my_algo/
├── launch/
│   ├── lidar_drive.launch.py
│   │   └── 라이다 기반 실차 자율주행 실행용 launch
│   ├── autonomous_drive.launch.py
│   │   └── 라이다/카메라 노드를 같이 띄우는 통합 launch
│   └── camera_drive.launch.py
│       └── 카메라 주행 인터페이스 확인용 launch
├── my_algo/
│   ├── vehicle_utils.py
│   │   └── RPM, 조향, JSON command, 공통 수학 유틸
│   ├── lidar_reactive_drive.py
│   │   └── LaserScan 기반 gap/corridor/cone/open-space 주행 판단
│   ├── autonomous_drive.py
│   │   └── planner command 선택, AEB 검사, VESC 명령 발행
│   ├── aeb.py
│   │   └── TTC와 제동거리 기반 긴급제동 안전 모델
│   ├── joy_controller.py
│   │   └── MANUAL/AUTO 토글과 수동 조이스틱 제어
│   ├── camera_reactive_drive.py
│   │   └── D455 RGB/depth, YOLO 객체, 거리, 차선, AEB 경고 overlay
│   └── autonomous_metrics_logger.py
│       └── trial 기반 성능 측정 및 CSV 로깅
├── package.xml
├── setup.py
└── README.md
```

## 런타임 토픽

```text
입력:
  /livox/lidar                         Livox PointCloud2 원본
  /scan                                pointcloud_to_laserscan 변환 결과
  /vesc/odom                           현재 차량 odometry
  /joy                                 조이스틱 입력
  /camera/camera/color/image_raw       D455 RGB 이미지
  /camera/camera/aligned_depth_to_color/image_raw
                                       RGB와 정렬된 D455 depth 이미지

내부 상태:
  /autonomous_mode                     AUTO/MANUAL 상태
  /joy_active                          사람이 조이스틱을 조작 중인지 여부
  /reactive/lidar_command              라이다 reactive drive 결과 JSON
  /reactive/camera_command             추후 카메라 제어용 예약 토픽
  /aeb/active                          AEB 작동 여부
  /aeb/status                          AEB 상태/작동 이유
  /camera/perception/status            카메라 인식 결과 JSON

출력:
  /commands/motor/speed                VESC 모터 ERPM 명령
  /commands/servo/position             VESC 조향 servo position 명령
  /camera/perception/annotated_image   객체/거리/차선/AEB overlay 이미지
```

## 노드 흐름

```text
/livox/lidar
  -> pointcloud_to_laserscan
  -> /scan
  -> lidar_reactive_drive
  -> /reactive/lidar_command
  -> autonomous_drive
     ├── drive_source에 맞는 command 선택
     ├── /vesc/odom 속도로 AEB 검사
     └── /commands/motor/speed + /commands/servo/position 발행

/joy
  -> joy_controller
     ├── /autonomous_mode 발행
     ├── /joy_active 발행
     └── MANUAL 모드에서는 VESC 명령 직접 발행

/camera/camera/color/image_raw + /camera/camera/aligned_depth_to_color/image_raw
  -> camera_reactive_drive
     ├── YOLO 객체 인식
     ├── depth median 기반 객체 거리 추정
     ├── 하단 ROI 차선 후보 검출
     ├── /aeb/active를 받아 경고 overlay 표시
     └── /camera/perception/annotated_image 발행
```

## 실행

라이다 기반 주행은 아래 launch가 기본이다.

```bash
ros2 launch my_algo lidar_drive.launch.py
```

단, 이 launch는 자율주행 알고리즘 launch다. 실차에서 실제로 움직이려면
Livox 라이다 드라이버와 VESC 드라이버가 먼저 떠 있어야 한다.

```text
필수 하드웨어 토픽:
  /livox/lidar
  /vesc/odom
  /commands/motor/speed
  /commands/servo/position
```

통합 launch는 라이다와 카메라 reactive drive 노드를 같이 띄우지만, 현재 기본
선택은 `drive_source=lidar`이다. 아직 카메라-라이다 센서퓨전은 아니다.

```bash
ros2 launch my_algo autonomous_drive.launch.py
```

카메라 launch는 현재 주행 제어용이 아니다. D455 드라이버와 카메라 perception
overlay만 실행해서 객체/거리/차선/AEB 표시가 정상인지 확인한다. 차량 제어는
하지 않는다.

```bash
ros2 launch my_algo camera_drive.launch.py
```

## D455 카메라 perception

현재 목표는 카메라가 실차 제어에 개입하지 않고, 주행 중 주변 정보를 제대로
받고 처리하는지 확인하는 것이다.

```text
camera_drive.launch.py
├── realsense2_camera_node
│   ├── color image publish
│   ├── depth image publish
│   └── align_depth.enable: True
└── camera_reactive_drive
    ├── RGB frame 수신
    ├── aligned depth frame 수신
    ├── YOLO 객체 인식
    ├── bbox 중앙부 depth median으로 객체 거리 계산
    ├── 하단 ROI에서 흰색/노란색 차선 후보 검출
    ├── AEB active topic을 받아 화면 경고 표시
    ├── OpenCV 창 표시
    └── /camera/perception/annotated_image publish
```

카메라 제어는 현재 의도적으로 꺼져 있다.

```text
camera_reactive_drive.py는 /commands/motor/speed를 발행하지 않는다.
camera_reactive_drive.py는 /commands/servo/position을 발행하지 않는다.
camera_drive.launch.py는 autonomous_drive를 실행하지 않는다.
LiDAR 주행 중 카메라는 perception 확인용 보조 노드로만 둔다.
```

YOLO와 OpenCV overlay가 정상 동작하면 화면에는 다음 정보가 표시된다.

```text
D455 PERCEPTION | YOLO:ON/OFF | DEPTH:ON/WAIT | CONTROL:DISABLED
객체 bbox
객체 class
confidence
distance
decision
LANE: YES/NO
AEB WARNING
```

객체별 decision은 현재 표시용이다. 예를 들어 작은 부드러운 물체는
`IGNORE_CANDIDATE`, 사람/자전거 등은 `STOP_CANDIDATE`, 차량류는 `WATCH`로
표시한다. 이 값은 추후 카메라 기반 판단 정책을 만들 때 확장한다.

필요한 Jetson 패키지:

```bash
sudo apt install ros-humble-realsense2-camera ros-humble-cv-bridge
pip3 install ultralytics
```

YOLO 모델은 기본값 `yolov8n.pt`를 사용한다. 네트워크가 없는 차량 환경에서는
미리 모델 파일을 받아 두고 launch의 `yolo_model` 값을 해당 경로로 바꾼다.

## 차량 치수 파라미터

실차 크기와 라이다 위치는 launch 파일에서 바로 볼 수 있게 노출했다.

```text
lidar_drive.launch.py
├── lidar_reactive_drive
│   ├── vehicle_width: 0.30
│   ├── safety_margin: 0.18
│   ├── target_wall_distance: 0.85
│   ├── lookahead_distance: 2.2
│   ├── max_steering: 0.78
│   └── lidar_to_bumper_dist: 0.30
└── autonomous_drive
    ├── aeb_ttc_threshold: 0.55
    ├── aeb_vehicle_half_width: 0.15
    ├── aeb_path_margin: 0.10
    ├── lidar_to_bumper_dist: 0.30
    └── max_steering: 0.78
```

차량 전체 길이는 현재 공식에 직접 쓰지 않는다. 전방 충돌과 장애물 거리는
라이다 원점 기준이 아니라 앞범퍼 기준으로 보정한다.

```text
앞범퍼 기준 x = 라이다 range*cos(theta) - lidar_to_bumper_dist
lidar_to_bumper_dist = 0.30 m
```

## 적용된 알고리즘

### 1. 라이다 좌표 변환

`lidar_reactive_drive.py`와 `aeb.py`에서 LaserScan의 극좌표를 차량 좌표계로 바꾼다.

```text
x = r*cos(theta) - lidar_to_bumper_dist
y = r*sin(theta)
```

- `r`: LaserScan range
- `theta`: LaserScan angle
- `x`: 앞범퍼 기준 전방 거리
- `y`: 차량 중심선 기준 좌우 거리

라이다 x축이 차량 정면을 향한다는 현재 장착 상태를 기준으로 한다. 별도 yaw
offset 보정은 넣지 않았다.

### 2. Gap 기반 주행

`lidar_reactive_drive.py`는 전방 +/-120도 안의 scan point를 보고 차량이 지나갈 수
있는 틈새를 찾는다.

```text
통과 필요 폭 = vehicle_width + 2*safety_margin
기본값: 0.30 + 2*0.18 = 0.66 m
```

장애물이 전방 `lookahead_distance` 안에 있고, 차량 중심선 기준 좌우 허용 폭
안에 들어오면 그 방향은 막힌 것으로 본다.

```text
lateral_limit = max((vehicle_width + 2*safety_margin)/2, 0.12)
blocked = 0 <= x <= lookahead_distance and abs(y) <= lateral_limit
```

틈새 폭은 좌우 경계점 사이의 유클리드 거리로 계산한다.

```text
gap_width = sqrt(dx^2 + dy^2)
```

틈새 중심각은 좌우 경계각 평균이다.

```text
center_angle = (left_angle + right_angle) / 2
```

### 3. Gap 점수 계산

여러 틈새가 있으면 폭, 전방 여유거리, 조향 부담, 조향 변화량을 같이 본다.

```text
width_score = clamp(gap_width / 1.6, 0, 1)
clearance_score = clamp(clearance / lookahead_distance, 0, 1)
steering_penalty = abs(center_angle) / radians(120)
change_penalty = abs(center_angle - previous_steering)

score =
  1.4*width_score
  + clearance_score
  - 0.45*steering_penalty
  - 0.25*change_penalty
```

이 점수식은 넓고 전방 여유가 큰 틈새를 선호하면서, 갑작스러운 좌우 전환을
줄이도록 만든 것이다.

gap 점수 높음 = 폭도 넓고, 앞도 비어 있고, 너무 급하게 꺾지 않아도 되고, 이전 조향과도 크게 안 달라서 안정적임을 의미
gap 점수는 폭 + 거리 + 조향 안정성을 고려하여 판단한 종합 점수임.

### 4. 조향과 RPM 계산

선택된 gap의 중심각을 조향 목표로 쓴다. 장애물 회피 모드에서는 조금 더 강하게
돌도록 1.25배를 적용한다.

```text
steering = clamp(center_angle * gain, -max_steering, max_steering)
gain = 1.25 if obstacle_avoidance else 1.0
```

틈새가 넓을수록 RPM을 높이고, 조향을 많이 할수록 감속한다.

```text
steer_ratio = clamp(abs(steering) / max_steering, 0, 1)
width_ratio = clamp(gap_width / 1.5, 0, 1)

rpm = CRUISE_RPM + (MAX_RPM - CRUISE_RPM)*width_ratio
rpm = rpm * (1 - 0.35*steer_ratio)
```

현재 기본값:

```text
CRUISE_RPM = 8200
MAX_RPM = 12000
MIN_DRIVING_RPM = 2000
```

명령 RPM은 `sanitize_rpm()`을 거쳐 0 또는 최소 2000 RPM 이상으로 제한된다.
출발 토크가 부족해서 안 움직이는 문제를 줄이기 위한 규칙이다.

### 5. Corridor 기반 wall-follow

현재 코드의 wall-follow는 이미지에 있는 전통적인 두 점 기반 공식과 다르다.
현재 적용된 방식은 좌우 측면 point의 `abs(y)` 분위값으로 벽과의 거리를 추정하고,
단순 P 제어로 조향한다.

```text
양쪽 벽이 보일 때:
  error = left_dist - right_dist

왼쪽 벽만 보일 때:
  error = left_dist - target_wall_distance

오른쪽 벽만 보일 때:
  error = target_wall_distance - right_dist

steering = clamp(0.55*error, -max_steering, max_steering)
rpm = CRUISE_RPM * (1 - 0.2*abs(steering)/max_steering)
```

네가 올린 공식은 아래 형태의 고전적인 wall-follow다.

```text
alpha = atan((a*sin(theta) - b) / (a*cos(theta)))
D_t = b*cos(alpha)
D_t+1 = D_t + L*sin(alpha)
```

이 공식은 현재 코드에 그대로 적용되어 있지 않다. 현재 코드는 고전 wall-follow
하나만 쓰는 방식이 아니라, gap 주행을 기본으로 하고 corridor가 안정적으로
보일 때만 벽 추종 모드로 들어간다.

### 6. Cone track

좌우 cone처럼 보이는 point가 충분히 있으면 좌우 평균 y 좌표로 중앙선을 만든다.

```text
track_width = left_y - right_y
center_y = (left_y + right_y) / 2
target_angle = atan2(center_y, lookahead_distance)
```

cone 추종에서도 많이 꺾을수록 감속한다.

```text
rpm = CRUISE_RPM * (1 - 0.25*abs(steering)/max_steering)
```

### 7. AEB 긴급제동

AEB는 차량 전체 주변이 아니라 현재 차량이 지나갈 전방 띠만 검사한다.

```text
corridor_half_width = aeb_vehicle_half_width + aeb_path_margin
기본값: 0.15 + 0.10 = 0.25 m
```

이 띠 안에서 가장 가까운 앞 장애물 거리 `closest_x`를 찾고, TTC와 제동거리로
긴급정지 여부를 판단한다.

```text
TTC = closest_x / current_speed
stopping_distance = speed^2 / (2*comfortable_decel) + stopping_margin
```

현재 기본값:

```text
aeb_ttc_threshold = 0.55 s
comfortable_decel = 2.8 m/s^2
stopping_margin = 0.20 m
```

아래 둘 중 하나라도 참이면 AEB가 모터 RPM을 0으로 보낸다.

```text
TTC <= aeb_ttc_threshold
closest_x <= stopping_distance
```

### 8. 속도와 조향 변환

VESC 모터 명령은 m/s가 아니라 ERPM이다.

```text
ERPM = speed_mps * 4614
speed_mps = ERPM / 4614
```

조향각은 VESC servo position으로 변환한다.

```text
servo_position = SERVO_CENTER - steering_rad*SERVO_GAIN
```

현재 기본값:

```text
SERVO_CENTER = 0.5
SERVO_GAIN = 0.60
SERVO_MIN = 0.03
SERVO_MAX = 0.97
```

## Trial 로깅 공식

`autonomous_metrics_logger.py`는 AUTO 구간을 하나의 trial로 보고 CSV를 저장한다.

```text
주행거리 step = sqrt(dx^2 + dy^2)
총 주행거리 = step 누적합
평균속도 = 총 주행거리 / AUTO 지속시간
평면속도 = sqrt(vx^2 + vy^2)
조향 변화량 = abs(current_steering - previous_steering)
조향 변화율 = 조향 변화량 / dt
성공률 = success_count / evaluated_count * 100
```

## Jetson 적용 순서

```bash
cd ~/f1tenth_ws
git pull
colcon build --packages-select my_algo
source install/setup.bash
ros2 launch my_algo lidar_drive.launch.py
```

실차 첫 테스트에서는 바퀴를 띄운 상태에서 `/commands/motor/speed`,
`/commands/servo/position`, `/scan`, `/vesc/odom` 토픽이 정상인지 먼저 확인한다.
