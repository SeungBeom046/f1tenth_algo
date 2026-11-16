# 하드웨어 실험 런북 (차량 2대: 검정이, 파랑이)

실험하는 사람이 차 옆에서 보는 문서. 각 차의 터미널 순서, 실험 환경 만들기, 기록할 것, 기록에서 볼 값을 정리했다.
코드 흐름은 [PIPELINE.md](PIPELINE.md), 설치·빌드 상세는 [JETSON.md](JETSON.md), 기본 주행 정지 수단은 [DRIVE_BASELINE.md](DRIVE_BASELINE.md).

## 0. 지금 할 수 있는 것과 아직 없는 것

| 구분 | 상태 |
|---|---|
| 기본 주행 (`lidar_drive.launch.py`): 두 차 모두 | **가능** |
| 보정 실험 (제동 지연·감속도, ERPM 환산, 정지 선행차 측정 잡음, 벽 헤딩) | **가능** (기본 주행 + 수동 배치 + bag 기록) |
| 추월 실험 (ego가 belief로 판단, 선행차 양보/비양보) | **아직 불가.** 실차 노드 `exp_planner`, 실험 launch `exp_overtake.launch.py`, 상태 토픽 `/exp/state`, 비상정지 heartbeat, `/sensors/core` 속도 소스가 없다 (지시서 3·4·6·7단계). 아래 5~6절의 추월 회차 절차는 그 코드가 생긴 뒤를 위한 계획이다 |
| 선행차 양보 동작 측정 (`t_react`) | 선행차 횡 이동 코드가 필요해서 **아직 불가** |

이 문서의 명령은 Mac에서 작성했고 차에서 실행해 확인하지 않았다. 다르면 고친다.

## 1. 차량 정리

| 항목 | 검정이 | 파랑이 |
|---|---|---|
| ROS | jazzy를 source한다 (아래 터미널 기록 기준). **humble이라고 들었는데 기록은 `/opt/ros/jazzy`라 확인 필요** | Foxy (Python 3.8) |
| 이 저장소 | `~/ros2_ws/src/f1tenth_algo` (빌드: `~/ros2_ws`) | `~/f1tenth_ws/src/f1tenth_algo` (빌드: `~/f1tenth_ws`) |
| Livox 드라이버 워크스페이스 | `~/f1_tenth_ws` | `~/f1tenth_ws` |
| VESC 드라이버 | (기록에 source 없음, 기본 환경에 있는 것으로 보임) | `~/vesc_ws` |
| Livox launch | `msg_MID360_launch.py` | `rviz_MID360_launch.py` |
| `ROS_DOMAIN_ID` (제안) | 37 | 38 |
| 역할 (회차마다 기록) | ego 또는 leader | ego 또는 leader |

- **두 차는 서로 다른 `ROS_DOMAIN_ID`를 쓴다.** 차량 간 통신이 없다는 실험 전제와 맞고, 같은 토픽 이름(`/scan`, `/commands/*`)이 섞이지 않는다. 그 차를 보는 노트북·Foxglove도 같은 ID로 맞춘다. 배포판이 달라도(jazzy/Foxy) 서로 통신하지 않으므로 상관없다.
- **확인 필요 (검정이)**: livox_ros_driver2의 `msg_MID360_launch.py`는 기본으로 Livox 전용 메시지(`CustomMsg`)를 낸다고 알고 있다 (미확인). `pointcloud_to_laserscan`은 `sensor_msgs/msg/PointCloud2`가 필요하다. 처음 한 번 `ros2 topic info /livox/lidar`로 타입을 확인하고, CustomMsg면 `rviz_MID360_launch.py`를 쓰거나 설정의 `xfer_format`을 0으로 바꾼다.

## 2. 코드 받기·빌드 (실험 날 처음 한 번)

처음 이 브랜치로 바꿀 때만 `git switch`가 필요하다. 그다음부터는 `git pull`만.

**검정이**
```bash
cd ~/ros2_ws/src/f1tenth_algo
git status                                    # 로컬 수정이 있으면: git stash push -m jetson-local
git fetch origin && git switch exp/aur-overtake    # 처음 한 번 (이미 이 브랜치면 생략)
git pull --ff-only
cd ~/ros2_ws
rm -rf src/f1tenth_algo/scripts/__pycache__ build/my_algo install/my_algo   # 브랜치를 바꾼 직후 한 번
source /opt/ros/jazzy/setup.bash              # humble이면 humble
colcon build --packages-select my_algo
source install/setup.bash
ros2 pkg executables my_algo                  # 확인: lidar_reactive_drive, autonomous_drive 등 5개
```

**파랑이**
```bash
cd ~/f1tenth_ws/src/f1tenth_algo
git status
git fetch origin && git switch exp/aur-overtake    # 처음 한 번
git pull --ff-only
cd ~/f1tenth_ws
rm -rf src/f1tenth_algo/scripts/__pycache__ build/my_algo install/my_algo   # 브랜치를 바꾼 직후 한 번
source /opt/ros/foxy/setup.bash
colcon build --packages-select my_algo
source install/setup.bash
ros2 pkg executables my_algo
```

## 3. 터미널 실행

> 터미널마다 복사해서 붙일 명령은 [CAR_TERMINALS.md](CAR_TERMINALS.md)에 정리돼 있다. 이 절은 요약이다.

**모든 터미널에서 맨 먼저 `export ROS_DOMAIN_ID=<그 차의 값>`.** 순서: A(라이다) → B(VESC) → 확인 → C(기록) → D(알고리즘).
`lidar_drive.launch.py`는 실행하는 순간 **조이스틱 없이 바로 AUTO로 출발한다.** 출발 전에 차 앞 0.2 m 안에 장애물(출발 게이트)을 두면 AEB로 멈춰 있고, 장애물을 치우면 출발한다.

### 검정이

```bash
# 터미널 A: Livox
export ROS_DOMAIN_ID=37
source ~/ros2_ws/install/setup.bash
source ~/f1_tenth_ws/install/setup.bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py          # 1절의 메시지 타입 확인

# 터미널 B: VESC
export ROS_DOMAIN_ID=37
source /opt/ros/jazzy/setup.bash
ros2 launch vesc_driver vesc_driver_node.launch.py

# 터미널 C: 기록 (7절)
export ROS_DOMAIN_ID=37
source /opt/ros/jazzy/setup.bash
mkdir -p ~/runs && cd ~/runs
ros2 bag record -o $(date +%m%d)_<run_id>_black /scan /sensors/core /commands/motor/speed /commands/servo/position /reactive/lidar_command /aeb/active /aeb/status /autonomous_mode

# 터미널 D: 알고리즘
export ROS_DOMAIN_ID=37
source /opt/ros/jazzy/setup.bash
source ~/f1_tenth_ws/install/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch my_algo lidar_drive.launch.py use_camera:=false
```

(예전 기록의 터미널 D) `ros2 topic pub /autonomous_mode std_msgs/msg/Bool "data: true" -r 10`는 **계속 true를 보내므로, 정지하려고 false를 보내도 바로 덮어써진다.** `start_autonomous`가 이미 true라서 이 명령 없이도 출발한다. 꼭 써야 하면 정지할 때 이 터미널을 먼저 Ctrl+C 한다.

Foxglove: 검정이에서 bridge가 떠 있으면 노트북 Foxglove로 `ws://<검정이 IP>:8765` (노트북 `ROS_DOMAIN_ID`와 무관, bridge가 차 쪽 ID로 떠 있어야 한다).

### 파랑이

```bash
# 터미널 A: Livox + PointCloud2
export ROS_DOMAIN_ID=38
source /opt/ros/foxy/setup.bash
source ~/f1tenth_ws/install/setup.bash
ros2 launch livox_ros_driver2 rviz_MID360_launch.py

# 터미널 B: VESC
export ROS_DOMAIN_ID=38
source /opt/ros/foxy/setup.bash
source ~/vesc_ws/install/setup.bash
ros2 launch vesc_driver vesc_driver_node.launch.py

# 터미널 C: 기록 (7절)
export ROS_DOMAIN_ID=38
source /opt/ros/foxy/setup.bash
mkdir -p ~/runs && cd ~/runs
ros2 bag record -o $(date +%m%d)_<run_id>_blue /scan /sensors/core /commands/motor/speed /commands/servo/position /reactive/lidar_command /aeb/active /aeb/status /autonomous_mode

# 터미널 D: 알고리즘 (빌드는 2절에서 했으면 생략)
export ROS_DOMAIN_ID=38
cd ~/f1tenth_ws
source /opt/ros/foxy/setup.bash
source install/setup.bash
ros2 launch my_algo lidar_drive.launch.py use_camera:=false
```

### 실행 직후 확인 (두 차 모두, 바퀴를 띄운 상태에서)

```bash
ros2 topic info /livox/lidar          # sensor_msgs/msg/PointCloud2
ros2 topic hz /scan                   # 약 10 Hz
ros2 topic hz /sensors/core           # VESC 상태가 나오는지 (속도 소스로 쓸 예정)
ros2 topic echo /aeb/status --once    # 'clear' 또는 closest 거리
ros2 topic echo /reactive/lidar_command --once
```

### 정지

| 수단 | 명령 |
|---|---|
| 소프트웨어 정지 | `ros2 topic pub --once /autonomous_mode std_msgs/msg/Bool "{data: false}"` (같은 `ROS_DOMAIN_ID` 터미널에서) |
| 알고리즘 종료 | 터미널 D에서 Ctrl+C. 종료 시 정지 명령을 보내려고 하지만 실제로 나가는지 확인되지 않았다 |
| 물리 | 차를 손으로 잡는다 / 전원. **추월 실험 전에 비상정지 heartbeat(6단계)를 만든다** |

## 4. 실험 환경 만들기

### 통로

| 항목 | 값 | 이유 |
|---|---|---|
| 형태 | 곧은 통로, 양쪽 벽 연속 | ego 횡위치·헤딩을 양쪽 벽 거리로 구한다. 벽이 끊기면 횡 제어와 차량 검출(벽 점 제거)이 흔들린다 |
| 폭 W | 1.0 m (기본), 비교용 0.8 / 1.3 m | 양보해야만 추월 가능한 폭: 2w + 3c = 0.84 ~ 3w + 4c = 1.21 m (w = 0.27, c = 0.10). W는 줄자로 재서 기록한다 (선행차에 실측 W를 준다) |
| 길이 | 15 m 이상 직선 + 끝에 정지 여유 3 m | 시뮬에서 출발 → 추월 완료 약 12 s, ego 이동 약 13 m |
| 벽 높이 | 라이다 높이(바닥에서 약 0.12~0.2 m, 미측정) 위아래로 덮도록, 0.3 m 이상 | `pointcloud_to_laserscan`이 라이다 기준 −0.1 ~ +0.5 m 점만 쓴다 |
| 벽 재질 | 평평하고 반사되는 것 (박스·판재). 유리·검은 천은 피한다 | 벽 직선 맞춤으로 헤딩을 구한다 |
| 바닥 | 평평, 미끄럽지 않게 | 제동 거리·조향 응답 |
| 표시 | 바닥 테이프로 출발선, 1 m 간격 눈금, 통로 중심선 | 영상·손 기록과 대조 |

### 출발 배치 (추월 실험, 계획)

- 선행차를 앞, ego를 뒤에. **ego 앞범퍼 → 선행차 뒷면 1.5 m 이상** (가까우면 선행차가 출발하자마자 양보 거리 안이라 회차가 무효. d_y는 0.6~1.2 m).
- 두 차 모두 통로 중심. 각 차 앞에 출발 게이트 장애물 (AEB로 정지 대기).
- 출발: 선행차 게이트를 먼저 치우고 1 s 뒤 ego 게이트를 치운다 (출발 시각 차이를 기록).

### 안전

- 처음 실행·새 코드는 반드시 **바퀴를 띄운 상태**에서 토픽부터 확인.
- 통로 끝에 받아 줄 사람과 부드러운 정지벽.
- 차 옆에 사람 한 명씩 (손으로 잡을 수 있게). 노트북에서 정지 명령을 미리 쳐 둔다.
- 추월 실험은 비상정지 heartbeat(`require_estop_heartbeat`)가 생긴 뒤에 한다.

## 5. 세션 순서 (추월 실험, 계획)

1. 두 차 `git pull` → 빌드 → `ros2 pkg executables my_algo` 확인 (2절)
2. 각 차 `ROS_DOMAIN_ID` 확인, 두 차 시계 확인 (`date`. 선행차·ego bag을 맞춰 볼 때 필요. chrony 동기 가정)
3. 통로 W·출발 간격을 재서 회차표에 기록
4. 터미널 A·B 실행 → 바퀴 띄우고 확인 → 내려놓기
5. 출발 게이트 배치 → 터미널 C(기록) 시작 → 터미널 D(알고리즘) 실행
6. 선행차 게이트 제거 → ego 게이트 제거
7. 회차 끝 (ego가 추월 완료하거나, 따라가다 통로 끝): 정지 → D 종료 → C 종료 (Ctrl+C)
8. 회차표에 결과·특이사항 손으로 기록 (충돌, 사람 개입, 벽 접촉)

회차표 (지시서 10단계): 방법 4(a·b·c·d) × 선행차 의도 2(Y·NY) × 3회 = 24회 + 예비 4회. run_id 규칙 `<방법><의도><번호>` (예: `dY2`, `bNY1`). 순서는 섞는다.

| run_id | 날짜·시각 | ego 차 | leader 차 | W [m] | 출발 간격 [m] | 선행차 d_y (로그) | 결과 (DONE/FOLLOW/중단/충돌) | 사람 개입 | 메모 |
|---|---|---|---|---|---|---|---|---|---|

## 6. 보정 실험 (지금 가능)

시뮬과 판단 로직의 가정값을 실차 값으로 바꾸기 위한 측정이다. 결과는 `config/exp_common.yaml`의 해당 값(TUNE 태그)에 반영하고 근거를 "측정"으로 바꾼다 ([TUNING.md](TUNING.md)).

| 무엇 | 방법 | 기록 | 구하는 값 | 지금 가정값 |
|---|---|---|---|---|
| 제동 지연 t_d, 최대 감속 a | 한 차씩 기본 주행으로 직선을 달리다 앞에 장애물을 두어 AEB로 세운다 (차마다 3회). 또는 `/autonomous_mode` false | bag (`/commands/motor/speed`, `/sensors/core`, `/aeb/active`) + 옆에서 영상 | 0 명령 시각 → 속도가 줄기 시작한 시각 = t_d, 감속 구간 기울기 = a | t_d 0.15 s, a 3.0 m/s² |
| ERPM → m/s 환산 | 테이프로 5 m 구간 표시, 일정 속도로 통과하는 시간을 잰다 | bag (`/sensors/core` 속도) + 스톱워치/영상 | ERPM_GAIN = 평균 ERPM / 실제 m/s | 4614 |
| 정지 선행차 측정 잡음 | 선행차를 통로 중심에 세우고, ego를 2 m, 1 m, 0.5 m 뒤에 세워 각 10 s 기록 (ego는 알고리즘 끄고 드라이버만) | bag (`/scan`) | 선행차 거리 d와 횡위치의 표준편차 (σ), 줄자 값과의 차이 | σ 0.02 m |
| 벽 헤딩 정확도 | ego를 통로 안에 0°, ±5°, ±10° 틀어 세우고 각 10 s | bag (`/scan`) + 각도기 | 벽 직선 맞춤 헤딩 오차 (시뮬에서는 0.2° 수준, 실차 미확인) | – |
| 라이다 높이·위치 | 줄자 | 손 기록 | 라이다 높이, 뒤축 → 라이다 0.13 m 재확인 | – |

`/sensors/core`의 속도 값이 ERPM인지도 이 측정에서 확인한다 (지시서 4단계, 미확인).

## 7. 기록 (ros2 bag)

- 차마다 따로 기록한다 (`ROS_DOMAIN_ID`가 달라 한 곳에서 둘 다 받을 수 없다). 파일 이름: `<MMDD>_<run_id>_<black|blue>`.
- 지금 기록할 토픽 (기본 주행·보정): `/scan`, `/sensors/core`, `/commands/motor/speed`, `/commands/servo/position`, `/reactive/lidar_command`, `/aeb/active`, `/aeb/status`, `/autonomous_mode`.
- 추월 실험에서 추가할 토픽 (코드 생긴 뒤): `/exp/state`, `/aeb/events`, `/estop/heartbeat`.
- 원본 점군 `/livox/lidar`는 용량이 커서 기본으로 빼고, 문제를 다시 보고 싶은 회차만 넣는다.
- Foxy는 sqlite3(`.db3`), jazzy는 기본이 mcap이다. 분석 스크립트(지시서 8단계)는 둘 다 읽게 만들 예정.

## 8. 기록에서 볼 값

### 지금 있는 토픽

| 토픽 | 내용 | 볼 것 |
|---|---|---|
| `/reactive/lidar_command` | planner 명령 JSON: `mode`(WALL_FOLLOW/OBSTACLE_AVOIDANCE/OPEN_SPACE/STOP), `rpm`, `steering_rad`, `reason` | 모드가 자주 바뀌면 벽 인식 문제. 추월 실험에서는 `mode`에 상태 기계 상태가 들어갈 예정 |
| `/commands/motor/speed` | 최종 ERPM (0 또는 ±4000~12000) | 0으로 떨어진 시각 = 정지 명령 (AEB 또는 planner) |
| `/commands/servo/position` | 서보 0.03~0.97 (0.5 중앙) | 조향 떨림 |
| `/aeb/active` | AEB 작동 여부 | **false → true로 바뀐 횟수 = N_AEB** (정상 추종에서는 0이어야 한다) |
| `/aeb/status` | `closest`(앞범퍼 → 가장 가까운 점), `aeb_limit`(그때 한계), `ttc` | AEB가 걸린 이유. closest가 벽이면 검사 띠 문제 |
| `/sensors/core` | VESC 상태 (속도, 전압, 전류) | 실제 속도. 명령 대비 지연 (t_d) |
| `/scan` | 360° 2D scan | 오프라인으로 tracker·corridor를 다시 돌려 볼 수 있다 |

참고: 기본 주행은 아직 `/vesc/odom`을 속도로 쓰는데 VESC launch가 그것을 띄우지 않아 속도가 0으로 계산된다. 그래서 AEB 한계가 항상 0.25 m다 ([OPEN_ITEMS.md](OPEN_ITEMS.md)).

### 추월 실험 지표 (계획: `/exp/state`에서 계산, 시뮬과 같은 이름)

모든 지표는 출발 시각 t₀(속도가 0.3 m/s를 처음 넘은 시각) 이후만 쓴다. 시뮬 결과는 [PROGRESS.md](PROGRESS.md)와 비교한다.

| 지표 | 뜻 | 좋은 값 / 시뮬 참고 |
|---|---|---|
| `success` | Y 회차에서 DONE (추월 완료) 도달 | Y에서 높을수록 |
| `pass_entered` (NY) | 양보 안 하는 선행차에 추월 시작 | **0이어야 한다** |
| `decided`, `T_id` | belief 확정(Y/NY)과 t₀부터 걸린 시간 | 시뮬 방법 d: Y 약 3.0 s, NY 약 3.3 s |
| `T_pass` | t₀ → DONE | 시뮬 방법 d 약 10~12 s |
| `h_min` | 급정지 전 최소 안전 여유 (식 7) | ≥ 0 (시뮬 방법 d −0.11 m, 열린 문제) |
| `stop_gap` | 선행차 급정지 후 최소 간격 | **≥ m = 0.15 m** (시뮬 방법 d 0.19 m) |
| `N_AEB` | ego AEB 작동 횟수 | 0이 목표. 걸렸으면 `/aeb/status`로 이유 |
| `d_probe_err` | PROBE 중 평균 거리 − d* | 0에 가까울수록 (d*(0.9) = 0.425 m) |
| `hold_count` | 최소 속도 규칙(0 < v < 0.867 m/s) 분기 횟수 | 많으면 선행차가 너무 느림 |
| `n_abort` | 추월 중단 횟수 | Y에서 0이어야 (오판 중단) |
| `d0` | t₀ 시점 거리 | ≥ 1.4 m (d_y_max + 0.2) 아니면 회차 무효 |
| 선행차 `d_y`, `yield_trigger_time`, `brake_time` | 선행차 bag의 양보 반응 거리, 양보·급정지 시각 | 두 bag 시각을 맞춰 판별 지연 계산 |

손으로 기록할 것: 충돌·벽 접촉, 사람 개입, 출발 간격·W 실측, 특이 현상 (추적 끊김으로 보이는 흔들림 등).

## 9. 미확인 목록

- 검정이 ROS 배포판 (jazzy인지 humble인지), VESC 드라이버 위치
- 검정이 `msg_MID360_launch.py`의 메시지 타입
- 두 차 라이다 높이, 실제 차폭·차 길이 (코드 가정 0.27 m, 0.50 m)
- `/sensors/core` 속도 단위 (ERPM 가정)
- 두 차 시계 동기 (chrony)
- 이 브랜치의 실차 빌드·실행 자체 (Mac에서만 확인)
