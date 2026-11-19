# 차 실행: 터미널별 명령

**역할: 파랑이(Foxy) = 추월하는 차(ego, 뒤), 검정이(jazzy) = 앞차(leader).** 출발할 때 검정이를 앞에, 파랑이를 뒤에 둔다.

차 한 대에 터미널 6개를 띄운다. 아래 블록을 터미널마다 **그대로 복사해** 붙인다.
각 블록 안에 환경 설정(`ROS_DOMAIN_ID`, `source`)이 들어 있으므로 터미널을 새로 열 때마다 블록 전체를 붙인다.

| 터미널 | 하는 일 | 끄는 순서 |
|---|---|---|
| 0 | 코드 받기·빌드 (실험 날 처음 한 번, 끝나면 닫아도 됨) | – |
| 1 | 라이다 (Livox) | 4 |
| 2 | 모터·서보 (VESC) | 3 |
| 3 | 확인 + 정지 명령 (계속 열어 둔다) | 5 |
| 4 | 기록 (ros2 bag) | 2 |
| 5 | 주행 알고리즘 | 1 |

띄우는 순서는 0 → 1 → 2 → 3(확인) → 4 → 5, 끄는 순서는 5 → 4 → 2 → 1 → 3이다.

> **터미널 5를 실행하는 순간 차가 바로 출발한다** (조이스틱 없음). 처음에는 바퀴를 띄우고 실행한다.
> 이 명령들은 Mac에서 정리했고 차에서 그대로 실행해 확인하지 않았다. 다르면 이 문서를 고친다.

### 무엇을 하느냐에 따라 터미널 5만 다르다

| 하려는 것 | 검정이 터미널 5 (앞차) | 파랑이 터미널 5 (추월하는 차) |
|---|---|---|
| **추월 실험: 앞차가 비켜 줌 (Y)** | `exp_overtake.launch.py role:=leader intent:=Y ...` | `exp_overtake.launch.py role:=ego method:=d ...` |
| **추월 실험: 앞차가 안 비키고 급정지 (NY)** | `exp_overtake.launch.py role:=leader intent:=NY ...` | `exp_overtake.launch.py role:=ego method:=d ...` |
| 방법 비교 (a·b·c·d) | 위와 같음 | `method:=a` / `b` / `c` / `d` |
| **서보 중심 보정 (차마다 처음 한 번, 직진성)** | `exp_overtake.launch.py role:=straight servo_center:=0.50` | 같음 |
| 기본 주행 확인, 보정 실험 | `lidar_drive.launch.py` | `lidar_drive.launch.py` |

**`lidar_drive.launch.py`는 추월 코드가 아니다.** 기본 주행(gap follow: 막히면 빈 공간으로 피해 감)이고, 값은 main 브랜치와 같다. 추월 실험에서는 두 차 모두 `exp_overtake.launch.py`를 쓴다.
각 명령의 인자와 터미널 로그는 맨 아래 "추월 실험" 절에 있다.

---

## 검정이: 앞차 leader (`ROS_DOMAIN_ID=37`, jazzy)

이 코드는 Foxy(Python 3.8) 기준으로 만들었고 jazzy(Python 3.12)에서는 빌드·실행해 보지 않았다. 터미널 0의 빌드 결과를 먼저 본다.

### 터미널 0: 코드 받기·빌드

```bash
export ROS_DOMAIN_ID=37
source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws/src/f1tenth_algo
git status                                   # 고친 파일이 있으면 먼저: git stash push -m car-local
git fetch origin
git switch exp/aur-overtake                  # 이미 이 브랜치면 "Already on" 이라고 나온다
git pull --ff-only
git log --oneline -1                         # 이 커밋을 회차표에 적는다
cd ~/ros2_ws
rm -rf build/my_algo install/my_algo         # 브랜치를 처음 바꾼 날만
colcon build --packages-select my_algo
```

### 터미널 1: 라이다

```bash
export ROS_DOMAIN_ID=37
source /opt/ros/jazzy/setup.bash
source ~/f1_tenth_ws/install/setup.bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

### 터미널 2: VESC

```bash
export ROS_DOMAIN_ID=37
source /opt/ros/jazzy/setup.bash
ros2 launch vesc_driver vesc_driver_node.launch.py
```

### 터미널 3: 확인 + 정지 (바퀴를 띄운 상태에서)

```bash
export ROS_DOMAIN_ID=37
source /opt/ros/jazzy/setup.bash
ros2 topic info /livox/lidar                 # Type: sensor_msgs/msg/PointCloud2 이어야 한다 (CustomMsg면 아래 참고)
ros2 topic hz /sensors/core                  # 숫자가 나오면 VESC 정상. Ctrl+C로 멈춘다
```

터미널 5를 띄운 뒤 같은 터미널에서:

```bash
ros2 topic hz /scan                          # 약 10 Hz. Ctrl+C
ros2 topic echo /aeb/status --once
```

그다음 아래 정지 명령을 미리 쳐 두고, 정지할 때 Enter만 누른다.

```bash
ros2 topic pub --once /autonomous_mode std_msgs/msg/Bool "{data: false}"
```

`/livox/lidar`가 `CustomMsg`로 나오면 `/scan`이 생기지 않는다. 터미널 1을 `rviz_MID360_launch.py`로 바꾸거나, Livox 설정의 `xfer_format`을 0으로 바꾼다.

### 터미널 4: 기록

`test1`을 회차 이름으로 바꾼다 (예: `calib1`, `dY2`). **꺾쇠 `< >`는 쓰지 않는다** (bash에서 파일 입력으로 해석돼 명령이 실행되지 않는다).

```bash
export ROS_DOMAIN_ID=37
source /opt/ros/jazzy/setup.bash
mkdir -p ~/runs && cd ~/runs
ros2 bag record -o $(date +%m%d_%H%M)_test1_black \
  /scan /sensors/core /commands/motor/speed /commands/servo/position \
  /reactive/lidar_command /aeb/active /aeb/status /autonomous_mode /exp/state /livox/imu
```

### 터미널 5: 주행 알고리즘 (실행하면 바로 출발)

**둘 중 하나만** 띄운다.

(가) 앞차 시나리오: 추월 실험용. 비키기(Y) 또는 급정지(NY). 인자 설명은 아래 "추월 실험" 절.

```bash
export ROS_DOMAIN_ID=37
source /opt/ros/jazzy/setup.bash
source ~/f1_tenth_ws/install/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch my_algo exp_overtake.launch.py role:=leader intent:=Y run_id:=test1 W:=1.0 servo_center:=0.50    # 회차 이름·실측 폭·보정한 서보 값으로
```

(나) 기본 주행 (gap follow): 보정 실험용.

```bash
export ROS_DOMAIN_ID=37
source /opt/ros/jazzy/setup.bash
source ~/f1_tenth_ws/install/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch my_algo lidar_drive.launch.py use_camera:=false
```

예전에 쓰던 `ros2 topic pub /autonomous_mode std_msgs/msg/Bool "data: true" -r 10`은 **치지 않는다.** 이 명령은 true를 계속 보내서 터미널 3의 정지 명령을 덮어쓴다. 이 명령 없이도 출발한다.

---

## 파랑이: 추월하는 차 ego (`ROS_DOMAIN_ID=38`, Foxy)

### 터미널 0: 코드 받기·빌드

```bash
export ROS_DOMAIN_ID=38
source /opt/ros/foxy/setup.bash
source ~/vesc_ws/install/setup.bash
cd ~/f1tenth_ws/src/f1tenth_algo
git status                                   # 고친 파일이 있으면 먼저: git stash push -m car-local
git fetch origin
git switch exp/aur-overtake
git pull --ff-only
git log --oneline -1                         # 이 커밋을 회차표에 적는다
cd ~/f1tenth_ws
rm -rf build/my_algo install/my_algo         # 브랜치를 처음 바꾼 날만
colcon build --packages-select my_algo
```

### 터미널 1: 라이다

```bash
export ROS_DOMAIN_ID=38
source /opt/ros/foxy/setup.bash
source ~/f1tenth_ws/install/setup.bash
ros2 launch livox_ros_driver2 rviz_MID360_launch.py
```

### 터미널 2: VESC

```bash
export ROS_DOMAIN_ID=38
source /opt/ros/foxy/setup.bash
source ~/vesc_ws/install/setup.bash
ros2 launch vesc_driver vesc_driver_node.launch.py
```

### 터미널 3: 확인 + 정지 (바퀴를 띄운 상태에서)

```bash
export ROS_DOMAIN_ID=38
source /opt/ros/foxy/setup.bash
source ~/vesc_ws/install/setup.bash          # 없으면 /sensors/core에서 No module named 'vesc_msgs'
ros2 topic list                              # /livox/lidar, /sensors/core 가 보여야 한다. 안 보이면 터미널마다 ROS_DOMAIN_ID 확인
ros2 topic info /livox/lidar                 # Type: sensor_msgs/msg/PointCloud2
ros2 topic hz /sensors/core                  # 숫자가 나오면 VESC 정상. Ctrl+C
```

터미널 5를 띄운 뒤:

```bash
ros2 topic hz /scan                          # 약 10 Hz. Ctrl+C
ros2 topic echo /aeb/status                  # 한두 줄 보고 Ctrl+C (Foxy의 echo에는 --once가 없다)
ros2 topic pub --once /autonomous_mode std_msgs/msg/Bool "{data: false}"    # 정지할 때 Enter (pub의 --once는 Foxy도 된다)
```

### 터미널 4: 기록

```bash
export ROS_DOMAIN_ID=38
source /opt/ros/foxy/setup.bash
source ~/vesc_ws/install/setup.bash          # /sensors/core(vesc_msgs) 기록에 필요
mkdir -p ~/runs && cd ~/runs
ros2 bag record -o $(date +%m%d_%H%M)_test1_blue \
  /scan /sensors/core /commands/motor/speed /commands/servo/position \
  /reactive/lidar_command /aeb/active /aeb/status /autonomous_mode /exp/state /livox/imu
```

### 터미널 5: 주행 알고리즘 (실행하면 바로 출발)

**둘 중 하나만** 띄운다.

(가) 추월 알고리즘: 추월 실험용. 인자 설명은 아래 "추월 실험" 절.

```bash
export ROS_DOMAIN_ID=38
source /opt/ros/foxy/setup.bash
source ~/vesc_ws/install/setup.bash
source ~/f1tenth_ws/install/setup.bash
ros2 launch my_algo exp_overtake.launch.py role:=ego method:=d W:=1.0 servo_center:=0.50    # 실측 통로 폭·보정한 서보 값으로
```

(나) 기본 주행 (gap follow): 보정 실험용. **추월 코드가 아니다.**

```bash
export ROS_DOMAIN_ID=38
source /opt/ros/foxy/setup.bash
source ~/vesc_ws/install/setup.bash
source ~/f1tenth_ws/install/setup.bash
ros2 launch my_algo lidar_drive.launch.py use_camera:=false
```

---

## 정지

| 상황 | 할 것 |
|---|---|
| 평소 정지 | 터미널 3에서 정지 명령 Enter → 차가 서면 터미널 5에서 Ctrl+C |
| 급할 때 | 차를 손으로 잡는다. 동시에 터미널 5 Ctrl+C |
| 회차 끝 | 터미널 5 → 4 Ctrl+C (bag 저장됨). 다음 회차는 4 → 5만 다시 띄운다 (1·2·3은 그대로) |

`ROS_DOMAIN_ID`가 다르면 정지 명령이 그 차에 가지 않는다. 검정이는 37, 파랑이는 38 터미널에서 보낸다.

## 자주 막히는 곳

| 증상 | 원인·조치 |
|---|---|
| 터미널 5에서 `package 'realsense2_camera' not found` | 이 launch는 카메라를 꺼도 이 패키지 경로를 먼저 찾는다. 차에 RealSense 패키지를 설치하거나 그 환경을 source한다 |
| `package 'my_algo' not found` | 터미널 0의 빌드가 실패했거나, 터미널 5에서 워크스페이스 `install/setup.bash`를 source하지 않았다 |
| `package 'pointcloud_to_laserscan' not found` | `sudo apt install ros-<foxy|jazzy>-pointcloud-to-laserscan` (패키지 이름 미확인) |
| `executable 'exp_planner' not found` | 빌드가 예전 것. 터미널 0의 `git pull` → `colcon build`를 다시 |
| `/scan`, `/aeb/status`가 없다 (터미널 5 전) | 정상. 둘 다 터미널 5가 만든다 |
| `[Drive] mode=FAILSAFE ... missing_command`만 계속 | `exp_planner`가 죽었다. 그 위의 `[exp_planner-2]` 에러 줄을 본다 |
| `ros2 topic list`에 방금 띄운 토픽이 없다 | ros2 daemon이 늦게 갱신한다. `ros2 daemon stop` 후 `ros2 topic list --no-daemon --spin-time 5` |
| `No module named 'vesc_msgs'` | 그 터미널에서 VESC 워크스페이스를 source하지 않았다 (파랑이 `source ~/vesc_ws/install/setup.bash`) |
| RViz에는 점군이 보이는데 `Unknown topic '/livox/lidar'` | 터미널끼리 `ROS_DOMAIN_ID`가 다르다. 각 터미널에서 `echo $ROS_DOMAIN_ID`로 비교한다 (빈칸 = 0) |
| `/scan`이 안 나온다 | 터미널 1이 꺼졌거나 `/livox/lidar`가 PointCloud2가 아니다 (터미널 3 확인) |
| 바퀴가 안 돈다 | 터미널 2 VESC 오류, 또는 차 앞 0.25 m 안에 무언가가 있어 AEB가 걸렸다 (`/aeb/status`) |
| 다른 차가 같이 움직인다 / 토픽이 섞인다 | 두 차 `ROS_DOMAIN_ID`가 같다. 37·38로 나눈다 |

## 자이로 확인 (pose_filter, 차마다 처음 한 번)

추월 launch는 `/livox/imu`(Mid-360 내장 IMU)의 z 각속도로 자차 헤딩을 예측한다 (지도는 안 쓴다). 단위(rad/s)와 부호는 **실차 미확인**이다.

```bash
ros2 topic hz /livox/imu                                   # 보통 200 Hz 근처
ros2 topic echo /livox/imu --field angular_velocity.z      # 차를 손으로 왼쪽(반시계)으로 돌린다
```

| 보이는 것 | 할 것 |
|---|---|
| 왼쪽으로 돌릴 때 양수, 90°를 1 s에 돌리면 약 1.5 | 그대로 |
| 왼쪽으로 돌릴 때 음수 | launch에 `imu_yaw_sign:=-1.0` |
| 값이 약 57배 크다 (deg/s) | 단위가 다르다. 알려 주면 코드에서 바꾼다 |
| 토픽이 없다 | Livox 드라이버 설정에서 IMU 발행을 켠다. 없어도 주행은 되지만 레일 측정만 쓴다 (요약 `IMU 자이로: 없음`) |

Ctrl+C 요약의 `IMU 자이로` 줄에 받은 개수, scan 중 자이로를 쓴 비율, yaw rate 평균이 나온다. 서 있을 때 평균이 0.01 rad/s보다 크면 바이어스가 큰 것이다 (레일 측정이 계속 보정한다).

## 로그·요약 읽기 (추월 launch)

`[Drive]` 줄 끝의 reason은 `판단 이유 | 레일 선행차` 순서다. 인식이 바뀔 때마다 `[EXP ego] 인식: ...` 줄이 찍힌다.

| 태그 | 뜻 |
|---|---|
| `rail_ok` | 양쪽(또는 한쪽) 가드레일 선을 잡았다 |
| `rail_hold` | 이번 scan은 못 잡았지만 0.3 s 안이라 직전 위치·방향으로 간다 |
| `rail_init` | 처음(또는 1 s 넘게 놓친 뒤) 다시 잡는 중, 2 s 안 3회 일치가 필요하다. 이 동안 정지 |
| `rail_lost` | 못 잡았다. 앞차를 아직 못 봤고 추월 전이면 3 s까지 빈 공간 가운데로 0.9 m/s (`[Drive]` reason `rail_fallback`), 아니면 정지 |
| `lead_detect 1.23m` | 선행차를 추적 중, 앞범퍼 → 선행차 뒷면 거리 |
| `lead_pending` | 처음 본 후보를 3 scan 확인 중 → 정지 |
| `lead_lost` / `lead_jump` | 보던 선행차를 놓쳤다 / 갑자기 먼 점이라 거부했다 → 정지 |
| `lead_none` | 아직 한 번도 못 봤다 → `v_cruise` 1.2 m/s로 간다 |

AEB가 걸리면 `[EXP ego] AEB 작동: <이유>` 줄이 찍힌다. Ctrl+C로 끄면 `EXP ego 요약`(상태 흐름, 인식 비율, 정지 이유 비율, scan 주기·처리 시간, AEB 횟수와 이유)과 기본 주행과 같은 `AUTONOMOUS TRIAL` 요약이 찍힌다. 처리 시간이 100 ms를 넘으면 scan을 못 따라가는 것이다.

정지한 차는 method d로 추월하지 않는다 (앞차가 비키는 것을 봐야 추월한다, 설계). 정지 차 우회는 `method:=a stationary_pass:=true`로, 출발 간격 1.5 m 이상에서 한다.

## 서보 중심 보정 (차마다 처음 한 번)

**직진성이 떨어지는 가장 큰 원인은 서보 중심(트림) 오차다.** 조향 명령이 0인데 바퀴가 조금 꺾여 있으면 차가 한쪽으로 흐른다.
시뮬에서 트림 0.03 rad(1.7°)만 넣어도 앞차가 비키는 반응이 늦어져 비켜 주는(Y) 회차의 절반이 "안 비킨다"로 오판됐다. 횡 제어의 적분항이 남은 작은 오차(±0.01 rad 정도)는 잡지만, 큰 트림은 먼저 이 보정으로 없앤다.

1. 양쪽 벽이 있는 직선 통로 중앙, 앞이 5 m 이상 빈 곳에 차를 놓는다. 터미널 1~4는 평소대로.
2. 터미널 5 (그 차의 환경 설정 줄을 먼저 붙인 뒤):

```bash
ros2 launch my_algo exp_overtake.launch.py role:=straight servo_center:=0.50
```

3. 차가 조향 0, 0.9 m/s로 직진하면서 0.5 s마다 이렇게 찍는다 (횡위치 + = 왼쪽, 통로 중앙 기준):

```text
[EXP straight] 횡위치 +0.012 m  헤딩 +0.4°  (양쪽 벽 보임)
[EXP straight] 횡위치 +0.035 m  헤딩 +1.1°  (양쪽 벽 보임)
```

4. 3~4 m 가서 손으로 잡거나 정지 명령. **횡위치가 계속 커지면(왼쪽으로 흐름) `servo_center`를 키우고, 작아지면(오른쪽) 줄인다.** 0.01씩 바꿔 다시 한다. (바꿨더니 더 흐르면 차의 서보 방향이 코드 가정과 반대다. 반대로 바꾼다.)
5. 3 m 동안 횡위치 변화가 ±0.05 m 안, 헤딩이 ±1° 안이면 그 값을 그 차의 `servo_center`로 정하고 회차표 맨 위에 적는다. 이후 모든 `exp_overtake.launch.py` 명령에 `servo_center:=<그 값>`을 붙인다.

`lidar_drive.launch.py`(기본 주행)는 main과 같게 두기 위해 이 값을 쓰지 않는다.

## 조향 확인 (차마다 처음 한 번, 서보 중심 보정 다음)

앞바퀴의 물리 최대 조향은 **±20° (0.349 rad)**다. 기본 주행 코드는 서보 끝(0.5 ± 0.47)을 0.78 rad로 가정해(게인 0.60) 바퀴가 명령의 45%만 꺾였다 → 조향이 둔하고 명령이 끝까지 쌓였다.
`exp_overtake.launch.py`는 2026-11-17부터 **`servo_gain:=1.35`(= 0.47 / 0.349), `max_steering:=0.349`**가 기본이다. "서보 끝 = 바퀴 20°"라는 가정이 맞는지 한 번 확인한다.

**방법 A (바퀴 띄우고, 1분)**: 차를 받침대에 올리고 터미널 1~4는 평소대로.
```bash
ros2 launch my_algo exp_overtake.launch.py role:=straight steer:=0.2 servo_center:=0.50   # 0.50 자리에 보정한 값
```
앞바퀴가 왼쪽으로 꺾인 각을 각도기(또는 휴대폰 각도 앱)로 잰다. **약 11.5°면 맞다.** (바퀴가 돌아가니 손 조심. 끝나면 Ctrl+C)

**방법 B (원 주행, 공간 4 m)**: 같은 명령을 바닥에 놓고 실행하면 0.9 m/s로 왼쪽 원을 돈다. 뒤축 중심이 그리는 원의 지름 D를 잰다 (바닥 테이프 두 점).
- **D ≈ 3.3 m면 맞다.**
- D ≈ 1.4 m면 서보가 게인 0.60대로 움직이는 차다 → 모든 명령에 `servo_gain:=0.6` (최대 조향은 0.349 그대로).
- 그 밖이면 실제 각 δ = atan(2 × 0.33 / D), **`servo_gain` = 1.35 × 0.2 / δ**로 고쳐 쓴다.

정해진 `servo_gain`은 `servo_center`처럼 회차표 맨 위에 적고, 다른 값이면 모든 `exp_overtake.launch.py` 명령에 붙인다.

## 추월 실험

터미널 1~4는 그대로이고 **터미널 5 명령만 바뀐다.** `exp_overtake.launch.py`는 라이다 변환 + 추월 실험 노드(`exp_planner`) + AEB·VESC 노드(`autonomous_drive`, 좁은 검사 띠)를 띄운다. gap follow와 카메라는 띄우지 않는다.
**실행하면 바로 출발한다.** 차 앞 0.2 m에 출발 게이트(상자)를 두면 AEB로 서 있다가, 게이트를 치우면 출발한다.

명령 끝에 `2>&1 | tee ...`를 붙이면 터미널 내용이 파일로도 남는다 (터미널이 꺼져도 확인 가능). 아래 명령은 모두 이 형태다.
`run_id`(회차 이름)와 `W`(줄자로 잰 통로 폭)는 회차마다 바꾼다. **꺾쇠 `< >`는 쓰지 않는다.**
통로 폭은 시뮬에서 1·2·3 m 모두 확인했다 (넓으면 비키는·추월 차선을 중앙에서 0.3 m로 제한). 다만 "비켜야만 추월할 수 있다"는 실험 조건은 W ≤ 1.21 m에서만 성립하므로 본 실험은 1.0 m로 한다.

### 낮은 가드레일 안에서 벽 선택 확인

`config/exp_common.yaml`의 `corridor_guardrail`은 기본 true이며 실측 `W`에 맞는 가까운 평행 벽을 선택한다.
처음에는 양쪽 벽이 필요하다. 주행 중 한쪽이 차량 등에 가려지면 직전 위치·방향과 이어지는 반대 벽으로 위치를 구한다.
유효한 가드레일 선을 찾지 못하면 속도 명령을 0으로 내린다. 이때 `/exp/state`의 `corridor_valid`는 false이고 ego의 `reason`은 `corridor_invalid`다.

두 차 모두 줄자로 잰 가드레일 안쪽 폭을 `W:=1.0`처럼 준다. 복도 전체 폭을 넣으면 주행 기준이 달라진다.
`/exp/state`에서 `wall_left`, `wall_right`, `heading`, `ego_lateral`을 확인한다.
가드레일 중앙에 똑바로 세웠을 때 두 벽 거리 약 0.5 m, 헤딩·횡위치 약 0이 기준이다.
라이다의 오른쪽 장착 오프셋 약 5 mm는 이 값에 포함될 수 있다.

높이 20 cm 가드레일·바깥 복도 벽이 함께 있는 MuJoCo 회귀로 확인한 기능이다. **실차 미확인**.

### 정지차 충돌 방지와 정지차 우회 시험 (2026-11-18)

`experiment_safety:=true`는 기본값이며 앞차 추적 소실 시 정지하고, 가까운 앞차가 먼 점으로 바뀌는 재초기화를 차단한다.
헤딩 감쇠 입력에 0.5 저역 필터를 적용한다. scan 높이 하한은 라이다 원점 기준 −0.20 m로 내려 낮은 차체를 포함한다.
장착 높이 0.24 m·수평 프레임 가정에서 지면 약 4 cm 위부터다. 실제 바닥 제거와 직진성은 **실차 미확인**이다.

AEB는 `/vesc/odom` 속도가 0이어도 요청 ERPM으로 환산한 속도 이상을 사용한다.
긴급 정지 후에는 `/scan` 점이 사라져도 정지를 유지하며 `/aeb/status`에 `latched`가 표시된다.
해제는 장애물을 치우고 AUTO off → on으로 한다. Foxy·Jazzy 모두 아래 명령을 지원한다.

```bash
ros2 topic pub --once /autonomous_mode std_msgs/msg/Bool "{data: false}"
ros2 topic pub --once /autonomous_mode std_msgs/msg/Bool "{data: true}"
```

터미널의 `선행차 d`는 ego 앞범퍼에서 관측한 앞차 표면까지의 거리다. 라이다→앞범퍼 0.28 m를 뺀 값이다.
앞차도 같은 라이다 위치라면 뒤범퍼→라이다는 약 0.22 m다. 라이다 본체만 보이는 반환은 실제 차간 거리를 더 크게 보일 수 있다.
새 모드는 기존 앞차와 이어지는 좁은 반환에 0.22 m를 추가 차감하며, AEB도 정면의 좁은 부분 반환에 0.22 m 여유를 둔다.
이는 부분 관측에 대한 보수적 처리이며, 앞차 뒷범퍼를 직접 측정했다고 보장하지 않는다.

방법 d는 앞차의 양보 움직임을 확인해야 추월한다. 한쪽에 세워 둔 정지차를 지나는 시험에는 아래의 별도 옵션을 사용한다.
앞차를 오른쪽에 정지시키고 앞뒤 간격 2.5 m에서 시작한 MuJoCo 조건을 확인했다. 초기 관측 간격이 1.5 m 미만이면 조기 추월에 진입하지 않는다.

```bash
ros2 launch my_algo exp_overtake.launch.py role:=ego method:=a W:=1.0 experiment_safety:=true stationary_pass:=true
```

두 차가 움직이는 원래 의도 추정 실험은 `method:=d stationary_pass:=false`를 쓴다.
정지차 우회 모드는 초기 관측에 횡 여유가 있을 때 멀리서 차선을 바꾸며, 양보 의도 추정 비교 결과와 별도로 기록한다.

### front_track_lost 확인

`front_track_lost`는 이전에 확인한 앞차를 현재 scan에서 추적하지 못해 정지한 상태다.
가드레일 유효 여부는 `/exp/state`의 `corridor_valid`로 별도로 확인한다.
`track_recovery`는 기본 true다. 현재 프레임에서 찾은 가드레일 점을 앞차 후보에서 제외하고, 최초 앞차는 연속 3 scan 동안 확인한다.
확인 중에는 `front_track_pending`·`track_status=confirming`과 속도 명령 0이 나오며, 일시 후보가 사라지면 이전 앞차로 저장하지 않는다.
확인된 앞차가 끊긴 경우는 정지를 유지한다. 이후 비슷한 거리에 앞차가 다시 나타나면 재인식할 수 있지만 갑자기 먼 점으로 바뀌는 것은 차단한다.

`/exp/state`의 `track_status`는 `no_candidate`(앞차 후보 없음), `confirming`(최초 후보 확인 중), `tracking`(추적), `distance_jump`(먼 거리 점프 차단)이다.
가드레일이 안 잡히면 `reason=corridor_invalid`다. 이 기능의 실제 환경 효과는 **실차 미확인**이다.

### 실제 bag에서 확인한 레일 초기화 실패와 재기록

2026-11-18 제공 bag은 `/exp/state` 850개가 전부 `corridor_invalid`였다. 사용자가 확인한 실제 폭은 1.0 m·라이다 좌우 각각 0.5 m다.
후측면의 짧은 가까운 면과 앞측면의 더 먼 면이 함께 반환되는 입력에서 기존 45~115° 관측은 가까운 레일 선을 초기화하지 못했다.
`guardrail_local_view`는 기본 true다. 측면 범위를 최소 155°까지 넓히고 기존 전후 거리 제한을 적용한다. 벽 선의 위치·방향 연속성을 우선하며 짧은 선으로의 갑작스러운 방향 교체를 차단한다.
양쪽 벽이 각각 최소 선 길이·점 수의 3배 이상으로 관측되며 폭·평행성이 맞으면 실제 차량 회전에 따른 더 큰 방향 변화도 허용한다.
첫 벽은 서로 다른 3 scan에서 확인하며 이때 `corridor_initializing`·속도 명령 0이 나올 수 있다. 같은 scan에서 재계산한 횟수는 중복으로 세지 않는다.
`/exp/state`의 `guardrail_local_view`로 적용 여부, `wall_initializing`으로 초기 확인 중 여부를 확인한다.
좌우 선의 방향 추정 차이를 최대 5° 허용하고 초기에는 양쪽에서 충분히 지지되는 선을 우선한다. 실측 폭과의 허용 차이는 기존 0.10 m다.
이 모드에서 정지 중 횡 적분은 동결한다. 실제 W는 줄자로 잰 값으로 지정한다.

제공 bag에는 `/scan`, `/tf_static`이 0개여서 원본 점군에 기존 장착 보정을 적용해 scan을 재구성했다.
다음 기록은 QoS override 파일을 사용해 두 토픽도 함께 받는다. Foxy의 공식 rosbag2 CLI와 YAML 정책 지원을 확인했다.
[Foxy record 구현](https://github.com/ros2/rosbag2/blob/foxy/ros2bag/ros2bag/verb/record.py), [QoS YAML 해석](https://github.com/ros2/rosbag2/blob/foxy/ros2bag/ros2bag/api/__init__.py)

```bash
mkdir -p ~/runs
cd ~/runs
ros2 bag record \
  --qos-profile-overrides-path "$(ros2 pkg prefix my_algo)/share/my_algo/config/perception_record_qos.yaml" \
  -o debug_perception_$(date +%m%d_%H%M%S) \
  /livox/lidar /livox/imu /scan /exp/state /sensors/core /reactive/lidar_command /tf_static
```

Ctrl+C로 끝낸 뒤 `ros2 bag info 생성된폴더명`에서 `/scan`과 `/tf_static`의 Count가 0보다 큰지 확인한다.
원본 점군 재구성 재생과 실차 주행은 구분한다. 이 수정의 새 실차 주행·재기록 결과는 **실차 미확인**이다.

### 검정이 (앞차): 시나리오별 명령

환경 설정 (터미널 5를 열 때마다):

```bash
export ROS_DOMAIN_ID=37
source /opt/ros/jazzy/setup.bash
source ~/f1_tenth_ws/install/setup.bash
source ~/ros2_ws/install/setup.bash
mkdir -p ~/runs
```

**시나리오 Y: 비켜 준다**

```bash
ros2 launch my_algo exp_overtake.launch.py role:=leader intent:=Y run_id:=dY1 W:=1.0 servo_center:=0.50 2>&1 | tee ~/runs/$(date +%m%d_%H%M)_dY1_black.log
```

- 동작: 0.9 m/s로 통로 중앙을 달린다. 뒤차가 `d_y`(0.6~1.2 m, `run_id`로 정해짐) 안으로 오면 오른쪽 차선(오른쪽 벽에서 약 0.24 m)으로 비키고 그대로 달린다.
- 로그:

```text
[EXP leader] 시작 role=leader intent=Y d_y=0.87 m brake_delay=1.32 s W=1.0 v_B=0.90 m/s speed_source=sensors_core
[EXP leader] 주행 (중앙)
[EXP leader] 출발 감지 (t0), 후행차 1.62 m
[EXP leader] 양보 시작: 오른쪽 차선 y_ref=-0.27, 후행차 0.85 m
```

**시나리오 NY: 안 비키고 급정지한다**

```bash
ros2 launch my_algo exp_overtake.launch.py role:=leader intent:=NY run_id:=dNY1 W:=1.0 servo_center:=0.50 2>&1 | tee ~/runs/$(date +%m%d_%H%M)_dNY1_black.log
```

- 동작: 0.9 m/s로 중앙을 달린다. 비키지 않는다. 뒤차가 1.0 m 안으로 오면 "무장"하고, `brake_delay`(0.5~2.0 s, `run_id`로 정해짐) 뒤 급정지해서 그대로 서 있는다.
- 로그:

```text
[EXP leader] 시작 role=leader intent=NY d_y=0.87 m brake_delay=1.32 s W=1.0 v_B=0.90 m/s speed_source=sensors_core
[EXP leader] 출발 감지 (t0), 후행차 1.62 m
[EXP leader] 급정지 무장: 1.32 s 뒤 정지
[EXP leader] 급정지 (NY)
```

| 인자 | 값 | 뜻 |
|---|---|---|
| `intent` | `Y` / `NY` | 위 두 시나리오 |
| `run_id` | 회차 이름 (예: `dY1`, `bNY2`) | `d_y`와 급정지 지연이 이 이름으로 정해진다. 같은 이름이면 같은 값. 시작 줄의 값을 회차표에 적는다 |
| `W` | 실측 통로 폭 [m] | 중앙·비키는 차선 위치를 오른쪽 벽 기준으로 잡는 데 쓴다. **실제와 다르면 차가 한쪽으로 쏠린다** |
| `d_y` | m (선택) | 주면 `run_id` 대신 이 반응 거리로 고정 |
| `speed_source` | `sensors_core`(기본) / `command` | 출발 감지에 쓰는 자기 속도 |

주의:
- 앞차는 **뒤쪽 1.2 m 안의 차 크기 물체를 뒤차로 본다.** 출발할 때 뒤에 사람·상자·벽 모서리가 있으면 바로 비킨다. 뒤가 빈 곳에서 출발한다.
- 오른쪽 벽이 끊기거나 울퉁불퉁하면 횡위치가 흔들린다 (양쪽 벽이 이어진 직선 통로 전제).
- 차가 선 이유가 시나리오 급정지면 `[EXP leader] 급정지 (NY)`, AEB면 `[Drive] mode=AEB ...` 줄이 뜬다.

검정이만 먼저 확인할 때: 바퀴를 띄우고 시작 줄 확인 → 내려서 달리게 → 뒤에서 차 크기 상자를 들고 다가가 양보·급정지 줄이 뜨는지 본다.

### 파랑이 (추월하는 차): 추월 알고리즘 켜기

환경 설정 (터미널 5를 열 때마다):

```bash
export ROS_DOMAIN_ID=38
source /opt/ros/foxy/setup.bash
source ~/vesc_ws/install/setup.bash          # 없으면 실제 속도를 못 읽는다
source ~/f1tenth_ws/install/setup.bash
mkdir -p ~/runs
```

```bash
ros2 launch my_algo exp_overtake.launch.py role:=ego method:=d W:=1.0 servo_center:=0.50 2>&1 | tee ~/runs/$(date +%m%d_%H%M)_dY1_blue.log
```

- 동작: 출발 → 앞차에 다가감(APPROACH) → 안전 거리가 허용하는 곳까지 붙어 관찰(PROBE) → 앞차가 비켰다고 확정하면 왼쪽으로 추월(PASS) → 앞질러서 중앙으로 복귀(MERGE) → 완료(DONE). 안 비킨다고 확정하면 안전 거리로 따라간다(FOLLOW). 추월 중 앞차가 다시 들어오면 중단하고 물러난다(RETREAT → FOLLOW).
- 로그 예시 (Y 회차. 형식은 시뮬 scan을 노드에 넣어 본 출력과 같고, 거리·시각은 회차마다 다르다):

```text
[EXP ego] 시작 role=ego method=d W=1.0 t_d=0.25 s (+scan 0.10 s) v_pass=1.50 m/s eta=0.90 speed_source=sensors_core
[EXP ego] - → WAIT (d), 선행차 1.48 m
[EXP ego] WAIT → APPROACH (d), 선행차 1.48 m
[EXP ego] APPROACH → PROBE (d), 선행차 0.45 m
[EXP ego] belief 확정 Y (b=0.99)
[EXP ego] PROBE → PASS (pass_start), 선행차 0.51 m
[EXP ego] PASS → MERGE (merging), 선행차 -1.22 m
[EXP ego] MERGE → DONE (done), 선행차 -0.97 m
```

NY 회차는 `belief 확정 NY` → `→ FOLLOW (belief_NY)`로 끝난다. 추월 중단은 `추월 중단 (...)` 줄이 뜬다.

| 인자 | 값 | 뜻 |
|---|---|---|
| `method` | `d`(제안, 기본) / `a` / `b` / `c` | a 고정 규칙(0.8 m에서 3 s 기다린 뒤 추월), b 소극적(1.5 m에서 관찰), c 공격적(0.4 m까지 붙음, 안전 거리 무시), d 안전 거리가 허용하는 최근접까지 |
| `W` | 실측 통로 폭 [m] | 추월 차선 위치와 "추월할 공간이 있나" 판단에 쓴다. 안 주면 양쪽 벽으로 추정 |
| `t_d` | s (기본 0.25) | 제동 지연. 안전 거리 계산에 쓴다. 보정 실험으로 잰 값으로 바꾼다 (scan 주기 0.1 s는 따로 더해진다) |
| `speed_source` | `sensors_core`(기본) / `command` | 안전 거리에 쓰는 자기 속도. `vesc_msgs`가 없으면 명령 속도로 대신하고 경고를 낸다 (안전 거리가 낙관적이 된다) |

**방법 c는 안전 거리를 무시하는 비교용이다.** NY 회차에서 추돌할 수 있으니 처음에는 d로만 하고, c는 AEB와 사람 대기 상태를 확인한 뒤 한다.

### 한 회차 순서 (두 차 같이)

1. 두 차 모두 터미널 1~3을 띄우고 토픽을 확인한다 (위 각 차 절).
2. 검정이를 앞, 파랑이를 뒤에 통로 중앙으로 놓는다. **간격(파랑이 앞범퍼 → 검정이 뒷면) 1.5 m 이상.** 각 차 앞 0.2 m에 출발 게이트.
3. 두 차 터미널 4(기록)를 시작한다. 파일 이름의 `test1`을 회차 이름으로.
4. 검정이 터미널 5 (`intent`는 회차표대로) → 파랑이 터미널 5 (`method`는 회차표대로). 둘 다 `시작` 줄이 뜨는지 본다.
5. 검정이 게이트를 치우고 1 s 뒤 파랑이 게이트를 치운다.
6. 끝나면(파랑이 DONE, 또는 따라가다 통로 끝): 두 차 터미널 3에서 정지 명령 → 터미널 5 Ctrl+C → 터미널 4 Ctrl+C.
7. 회차표에 적는다: 검정이 시작 줄의 `d_y`·`brake_delay`, 파랑이 마지막 상태, 충돌·벽 접촉·사람 개입.

### 끝나고 볼 것

```bash
grep -E "EXP|mode=AEB" ~/runs/*_dY1_*.log      # 그 회차의 상태 전이·양보·AEB
```

| 보이는 것 | 뜻 |
|---|---|
| 파랑이 `PASS`가 검정이 `양보 시작`보다 먼저 | 비정상. 회차와 로그를 보관하고 알려 준다 |
| 파랑이가 NY 회차에서 `PASS` | 비정상 (NY에 추월 시작은 0이어야 한다) |
| 파랑이 `[Drive] mode=AEB`가 자주 | AEB 정지 거리가 지금 0.25 m 고정 (속도 입력 0). 횟수를 회차표에 적는다 |
| 파랑이가 왼쪽 벽 앞에서 `mode=AEB`로 섬 | 차선 변경 중 벽이 검사 띠에 들어옴. 통로 폭이 1.0 m보다 넓으면 잘 생긴다 (시뮬에서도 확인된 문제) |
| `/scan`이 안 나옴 (2026-11-17 이후) | 라이다 기울기 보정(`level_scan:=true`, 기본)이 정적 TF `livox_level → livox_frame`을 쓴다. `ros2 run tf2_ros tf2_echo livox_level livox_frame`으로 TF 확인. 다른 곳에서 `livox_frame`의 부모 TF를 띄우면(예전 tf_static) 충돌한다. 급하면 `level_scan:=false` |
| 추월 중 차체-벽 5 cm 근처에서 AEB로 섬 | 2026-11-17에 AEB 띠를 실측 차 반폭(±0.135 m, 여유 0)으로, 정지 거리를 0.20 m로 좁혔다. 벽 5 cm에서 벽 쪽으로 6°까지 틀어져도 안 선다. 시작 줄 `effective parameters`에 `aeb_vehicle_half_width=0.135, aeb_path_margin=0.0, aeb_bumper_emergency_distance=0.2`인지 확인 |
| 차가 옆으로 비키거나 차선을 바꾸다 벽에 여유가 있는데 섬 (AEB `closest=0.1x m`) | AEB 거리는 라이다가 아니라 **앞범퍼 기준, 조향 명령대로 휘어 갈 길을 따라 잰 거리**다. 조향이 크게 꺾이면(최대 0.78 rad) 옆에 나란한 벽이 그 길 위에 놓여 섰다. 2026-11-17에 휜 검사 띠의 조향 상한 0.3 rad + 정면 띠와 겹치는 점만 보게 바꿨다 (`aeb_curve_max_steering`, `aeb_band_intersect`). 그래도 서면 `git pull`·빌드 했는지, 시작 줄 `effective parameters`에 `aeb_band_intersect=True`가 있는지, 서보 중심을 보정했는지 확인. 조향이 왜 크게 꺾이는지는 `/exp/state`의 `heading`·`ego_lateral`로 본다 (차를 통로 중앙에 똑바로 세웠을 때 둘 다 0 근처여야 함) |
| 파랑이 `MERGE`에서 검정이 AEB가 걸림 | 2026-11-16에 MERGE를 앞차 앞 0.35 m에서 1.5 m/s로 시작하게 바꿨다 (시뮬 검정이 AEB 0). 그래도 걸리면 `git pull` 했는지, 검정이 서보 중심을 보정했는지 확인 |
| NY에서 파랑이가 선 뒤 `mode=AEB`가 켜졌다 꺼졌다 | 앞차 뒤 0.2~0.3 m에 서 있으면 라이다 잡음으로 AEB 경계(0.25 m)를 오간다. 차는 서 있다 (시뮬에서 확인). AEB 횟수는 움직이는 동안의 것만 센다 |
