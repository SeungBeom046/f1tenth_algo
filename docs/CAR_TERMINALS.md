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

`<run_id>`를 회차 이름으로 바꾼다 (예: `calib1`, `dY2`).

```bash
export ROS_DOMAIN_ID=37
source /opt/ros/jazzy/setup.bash
mkdir -p ~/runs && cd ~/runs
ros2 bag record -o $(date +%m%d_%H%M)_<run_id>_black \
  /scan /sensors/core /commands/motor/speed /commands/servo/position \
  /reactive/lidar_command /aeb/active /aeb/status /autonomous_mode
```

### 터미널 5: 주행 알고리즘 (실행하면 바로 출발)

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
ros2 bag record -o $(date +%m%d_%H%M)_<run_id>_blue \
  /scan /sensors/core /commands/motor/speed /commands/servo/position \
  /reactive/lidar_command /aeb/active /aeb/status /autonomous_mode
```

### 터미널 5: 주행 알고리즘 (실행하면 바로 출발)

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
| `/scan`, `/aeb/status`가 없다 (터미널 5 전) | 정상. 둘 다 터미널 5(`lidar_drive.launch.py`)가 만든다 |
| `ros2 topic list`에 방금 띄운 토픽이 없다 | ros2 daemon이 늦게 갱신한다. `ros2 daemon stop` 후 `ros2 topic list --no-daemon --spin-time 5` |
| `No module named 'vesc_msgs'` | 그 터미널에서 VESC 워크스페이스를 source하지 않았다 (파랑이 `source ~/vesc_ws/install/setup.bash`) |
| RViz에는 점군이 보이는데 `Unknown topic '/livox/lidar'` | 터미널끼리 `ROS_DOMAIN_ID`가 다르다. 각 터미널에서 `echo $ROS_DOMAIN_ID`로 비교한다 (빈칸 = 0) |
| `/scan`이 안 나온다 | 터미널 1이 꺼졌거나 `/livox/lidar`가 PointCloud2가 아니다 (터미널 3 확인) |
| 바퀴가 안 돈다 | 터미널 2 VESC 오류, 또는 차 앞 0.25 m 안에 무언가가 있어 AEB가 걸렸다 (`/aeb/status`) |
| 다른 차가 같이 움직인다 / 토픽이 섞인다 | 두 차 `ROS_DOMAIN_ID`가 같다. 37·38로 나눈다 |

## 추월 실험 때 바뀌는 것 (아직 코드 없음)

터미널 1~4는 그대로이고 **터미널 5 명령만 바뀐다.** 이 노드와 launch는 아직 없다 ([CAR_CODE.md](CAR_CODE.md) 4절).

파랑이 터미널 5 (추월하는 차):

```bash
export ROS_DOMAIN_ID=38
source /opt/ros/foxy/setup.bash
source ~/vesc_ws/install/setup.bash
source ~/f1tenth_ws/install/setup.bash
ros2 launch my_algo exp_overtake.launch.py role:=ego method:=d          # method: a b c d (회차표대로)
```

검정이 터미널 5 (앞차):

```bash
export ROS_DOMAIN_ID=37
source /opt/ros/jazzy/setup.bash
source ~/f1_tenth_ws/install/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch my_algo exp_overtake.launch.py role:=leader intent:=Y       # 또는 intent:=NY (회차표대로)
```

출발 순서: 검정이 터미널 5 → 1 s 뒤 파랑이 터미널 5 (또는 출발 게이트를 그 순서로 치운다).

기록(터미널 4)에는 `/exp/state`, `/aeb/events`, `/estop/heartbeat`가 추가된다.
