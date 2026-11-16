# Jetson에서 exp/aur-overtake 받아서 빌드·실행하기

대상: 차량 Jetson (Orin Nano, Ubuntu 20.04, ROS 2 Foxy, Python 3.8).

> 이 문서의 명령은 Mac에서 작성했고 **Jetson에서 아직 실행해 보지 않았다.** 처음 따라 할 때 각 단계의 "확인" 항목을 보고, 다르면 이 문서를 고친다.

## 0. 먼저 알아둘 것

- 저장소 하나에 브랜치가 여럿이다. Jetson에서는 폴더를 골라 옮기지 않고 **`exp/aur-overtake` 브랜치로 바꾸면** 된다. 브랜치 전체(`occlusion_archive/`, `sim/` 포함)가 내려오지만 colcon은 `my_algo` 패키지(`my_algo/baseline`, `my_algo/aur`, `launch/`)만 빌드·설치한다. `occlusion_archive/`, `archive/`는 `COLCON_IGNORE`로 빠지고 `sim/`은 Mac 전용이다.
- **지금 이 브랜치로 차에서 돌릴 수 있는 것은 기본 주행(`lidar_drive.launch.py`)과 추월 실험(`exp_overtake.launch.py role:=leader` / `role:=ego`)이다.** 추월 실험은 차에서 아직 확인 전이다. 차별 터미널 명령은 [CAR_TERMINALS.md](CAR_TERMINALS.md).
- 실행 파일·launch 이름은 main 때와 같다 (`ros2 launch my_algo lidar_drive.launch.py`). 내부 모듈 경로만 `my_algo.baseline.*`로 바뀌어서 **다시 빌드해야 한다.**
- 이전 연구 브랜치(`exp/occlusion-speed`)에 있던 `scripts/env_ego.sh`, `launch/livox_mid360.launch.py`, `launch/vesc_odom.launch.py`는 이 브랜치에 없다 (`occlusion_archive/algo/`에 보관만).

## 1. 브랜치 받기

저장소 위치는 `~/f1tenth_ws/src/f1tenth_algo`로 가정한다 (다르면 바꿔서).

```bash
cd ~/f1tenth_ws/src/f1tenth_algo
git status                        # 지금 브랜치와 로컬 수정 확인
```

로컬 수정이 있으면 먼저 보관한다. 예전에 `config/imu.yaml`을 차에서 고쳐 둔 적이 있다. 이 브랜치에는 `config/imu.yaml`이 없어서, 수정된 채로는 브랜치 전환이 막힐 수 있다.

```bash
git stash push -m "jetson-local-$(date +%Y%m%d)"     # 나중에 되돌리기: git stash list → git stash apply <번호>
```

브랜치로 바꾼다.

```bash
git fetch origin
git switch exp/aur-overtake 2>/dev/null || git switch -c exp/aur-overtake origin/exp/aur-overtake
git pull --ff-only                # 이미 이 브랜치였으면 최신으로
git log --oneline -1              # 확인: origin/exp/aur-overtake와 같은 커밋
```

## 2. 빌드

모듈 경로가 바뀌었으므로 예전 빌드 결과를 지우고 다시 빌드한다. `scripts/__pycache__`가 있으면 colcon이 실패한 적이 있어 같이 지운다.

```bash
source /opt/ros/foxy/setup.bash
[ -f ~/vesc_ws/install/setup.bash ] && source ~/vesc_ws/install/setup.bash     # VESC 드라이버 워크스페이스
cd ~/f1tenth_ws
rm -rf src/f1tenth_algo/scripts/__pycache__ build/my_algo install/my_algo
colcon build --packages-select my_algo
source install/setup.bash
```

확인:

```bash
ros2 pkg executables my_algo
# 기대: my_algo autonomous_drive / autonomous_metrics_logger / camera_reactive_drive / joy_controller / lidar_reactive_drive
python3 -c "import my_algo.baseline.lidar_reactive_drive, my_algo.aur.ego_fsm; print('import ok')"
```

## 3. 터미널마다 환경 설정

이 브랜치에는 `env_ego.sh`가 없다. 새 터미널마다 직접 source한다 (추월 실험 단계에서 `scripts/env_ego.sh`, `env_leader.sh`를 새로 만들 예정).

```bash
source /opt/ros/foxy/setup.bash
[ -f ~/vesc_ws/install/setup.bash ] && source ~/vesc_ws/install/setup.bash
source ~/f1tenth_ws/install/setup.bash
export ROS_DOMAIN_ID=37           # 예전 env_ego.sh 기본값. 노트북·다른 차와 맞춘다
```

## 4. 기본 주행 실행

드라이버 두 개를 먼저 띄운다. 아래 명령은 이전 연구 런북(`occlusion_archive/docs/RUNBOOK.md`) 기준이고, 이 브랜치로 실행해 보지는 않았다.

```bash
# 터미널 1: Livox Mid-360 (/livox/lidar). livox_ros_driver2 패키지의 launch, RViz가 같이 뜬다
ros2 launch livox_ros_driver2 rviz_MID360_launch.py

# 터미널 2: VESC 드라이버 (/sensors/core, /commands/* 구독)
ros2 launch vesc_driver vesc_driver_node.launch.py
```

**바퀴를 띄운 상태에서** 주행을 시작한다. `lidar_drive.launch.py`는 실행하면 조이스틱 없이 **바로 AUTO로 출발한다.**

```bash
# 터미널 3
ros2 launch my_algo lidar_drive.launch.py                                          # 카메라·trial 로거 포함
ros2 launch my_algo lidar_drive.launch.py use_camera:=false use_metrics_logger:=false   # LiDAR 주행만
```

확인:

```bash
ros2 topic hz /scan                          # 약 10 Hz
ros2 topic echo /reactive/lidar_command      # planner 명령 (JSON)
ros2 topic echo /commands/motor/speed        # ERPM
ros2 topic echo /aeb/status
```

정지:

```bash
ros2 topic pub --once /autonomous_mode std_msgs/msg/Bool "{data: false}"   # AUTO 해제 → rpm 0, 조향 중앙
```

그 밖의 정지 수단(planner 정지선, AEB, 명령 타임아웃)은 [DRIVE_BASELINE.md](DRIVE_BASELINE.md)의 "조이스틱 없이 바로 자율주행을 시작한다". 조이스틱 B 버튼은 joy 노드가 꺼져 있어 쓸 수 없다. 손으로 차를 잡을 수 있는 상태에서 시험한다.

## 5. 알려진 제한

- **속도 입력이 0이다.** `autonomous_drive`와 `lidar_reactive_drive`는 `/vesc/odom`을 구독하지만 VESC launch가 `vesc_to_odom`을 띄우지 않아 속도가 0으로 계산된다. 그래서 AEB 한계가 0.25 m 고정이다 ([OPEN_ITEMS.md](OPEN_ITEMS.md)). `/sensors/core` 속도 소스는 추월 실험 4단계에서 추가할 예정이다.
- 예전에 `vesc_to_odom`을 따로 띄웠을 때는 서보 명령을 한 번 받기 전까지 아무것도 발행하지 않았다 (`ros2 topic pub --once /commands/servo/position std_msgs/msg/Float64 "{data: 0.5}"`로 깨움).
- 이 브랜치의 실차 빌드·실행은 아직 확인되지 않았다. 처음 해 본 결과(빌드 성공 여부, 실행 파일 목록, 토픽)를 [PROGRESS.md](PROGRESS.md)에 적는다.

## 6. 예전 브랜치로 돌아가기

```bash
cd ~/f1tenth_ws/src/f1tenth_algo
git switch exp/occlusion-speed                 # 또는 main
git stash list                                 # 1단계에서 보관한 로컬 수정
git stash apply <stash@{번호}>
cd ~/f1tenth_ws && rm -rf build/my_algo install/my_algo && colcon build --packages-select my_algo
```
