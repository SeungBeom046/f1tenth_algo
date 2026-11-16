# 차량별로 올리는 코드

이 문서는 검정이·파랑이에 각각 무엇을 받아 빌드하고 무엇을 실행하는지 정리한다. 터미널 순서와 실험 절차는 [HARDWARE_EXPERIMENT.md](HARDWARE_EXPERIMENT.md), 설치 상세는 [JETSON.md](JETSON.md)에 있다.

## 1. 결론: 두 차에 같은 코드를 올린다

- 두 차 모두 같은 저장소의 같은 브랜치 **`exp/aur-overtake`** 를 받는다. ego 코드와 leader 코드를 차마다 나눠 올리지 않는다.
- **역할: 파랑이(Foxy) = 추월하는 차 ego, 검정이(jazzy) = 앞차 leader.** 역할은 코드가 아니라 실행 명령(`role:=`)으로 정하므로 코드는 같다.
- 빌드 대상은 `my_algo` 패키지 하나다.

| 폴더 | 차에서 쓰나 | 설명 |
|---|---|---|
| `my_algo/aur/` | 쓴다 | 추월 판단 로직 (ego 상태 기계, 선행차 양보 로직, 인지·추적). 실차 노드가 import한다 |
| `my_algo/baseline/` | 쓴다 | 기본 주행 노드, AEB(`aeb.py`), ERPM·서보 변환(`vehicle_utils.py`), VESC로 명령을 보내는 `autonomous_drive` |
| `launch/` | 쓴다 | `lidar_drive.launch.py`(기본 주행), `exp_overtake.launch.py`(추월 실험) |
| `config/exp_common.yaml` | 쓴다 (추월 실험) | 추월 실험 파라미터. 빌드하면 `share/my_algo/config/`에 설치되고 `exp_planner`가 읽는다 |
| `my_algo/exp_planner.py` | 쓴다 (추월 실험) | 추월 실험 ROS 노드 (`role:=leader` 앞차 / `role:=ego` 추월하는 차) |
| `sim/` | 안 쓴다 | Mac 전용 MuJoCo 시뮬 |
| `occlusion_archive/`, `archive/` | 안 쓴다 | 보관용 (`COLCON_IGNORE`로 빌드 제외) |
| `test/`, `scripts/` | 안 쓴다 | Mac 단위 테스트와 분석 도구 |

필요 없는 폴더도 같이 내려오지만 colcon이 빌드하지 않는다. 그래서 차에서 폴더를 골라 지울 필요는 없다.

## 2. 저장소 밖에서 필요한 것 (차마다 설치돼 있어야 함)

| 패키지 | 용도 | 검정이 | 파랑이 |
|---|---|---|---|
| `livox_ros_driver2` | 라이다 점군 `/livox/lidar` | `~/f1_tenth_ws` | `~/f1tenth_ws` |
| `vesc_driver` | `/commands/*` → 모터·서보, `/sensors/core` | 기본 환경 (확인 필요) | `~/vesc_ws` |
| `pointcloud_to_laserscan` | 점군 → `/scan` (launch가 띄운다) | 확인 필요 | 확인 필요 |
| `realsense2_camera` | 카메라 | 확인 필요 | 확인 필요 |

- 추월 실험 launch(`exp_overtake.launch.py`)는 카메라를 찾지 않는다. `vesc_msgs`(VESC 워크스페이스)가 source돼 있으면 두 차 모두 자기 속도를 `/sensors/core`에서 읽는다 (ego는 안전 거리 계산에 쓴다).
- **`lidar_drive.launch.py`를 쓸 때 `realsense2_camera`는 카메라를 쓰지 않아도 설치돼 있어야 한다.** `lidar_drive.launch.py`는 `use_camera:=false`일 때도 시작하면서 이 패키지 경로부터 찾는다(13~17행). 없으면 launch가 바로 실패한다.
- 위 4개는 이 저장소에서 확인할 수 없어서, 차에서 `ros2 pkg prefix <패키지>`로 확인한다.

## 3. 기본 주행 (보정 실험용, 추월 코드 아님)

두 차 모두 같은 것을 실행한다. 역할 구분이 없다. gap follow 값은 main 브랜치(`b054758`)와 같다 (`scripts/verify_refactor_offline.py`로 출력 일치 확인).

| | 검정이 | 파랑이 |
|---|---|---|
| 실행 | `ros2 launch my_algo lidar_drive.launch.py use_camera:=false` | 같음 |
| 뜨는 노드 | `pointcloud_to_laserscan`, `lidar_reactive_drive` (gap follow), `autonomous_drive` (AEB → VESC), `autonomous_metrics_logger` | 같음 |
| 추월 판단 코드 | **돌지 않음** | **돌지 않음** |

이 launch로 차가 옆으로 피하는 동작은 모두 gap follow(`lidar_reactive_drive`)에서 나온다. 추월 판단 코드는 돌지 않는다.

## 4. 추월 실험: 역할별로 뜨는 노드

`launch/exp_overtake.launch.py`가 `role`에 따라 노드를 띄운다. 두 역할 모두 구현됐다 (차에서는 아직 실행 확인 전). 실행 명령은 [CAR_TERMINALS.md](CAR_TERMINALS.md) "추월 실험" 절에 있다.

| | ego = **파랑이** (뒤차, 추월하는 쪽) | leader = **검정이** (앞차, 실험 조건) |
|---|---|---|
| 실행 | `ros2 launch my_algo exp_overtake.launch.py role:=ego method:=d W:=1.0` | `ros2 launch my_algo exp_overtake.launch.py role:=leader intent:=Y run_id:=dY1 W:=1.0` |
| 판단 | `exp_planner` → `EgoRunner`(`my_algo/aur/ego_runner.py`) → `EgoFSM`(`ego_fsm.py`). 접근 → belief로 Y/NY 판단 → 추월(PASS → MERGE → DONE) 또는 추종(FOLLOW) | `exp_planner`(`my_algo/exp_planner.py`) → `LeaderRunner`(`my_algo/aur/leader_runner.py`) → `Leader`(`leader_logic.py`). Y면 뒤차가 d_y 안으로 오면 오른쪽으로 비키고, NY면 비키지 않고 급정지 |
| 횡 제어 | `my_algo/aur/lateral.py` | 같음 |
| AEB·VESC 출력 | `autonomous_drive` (`aeb_model:=geometric`, 휜 띠 ±0.16 m, TTC 0.35 s: 시뮬과 같은 값) | 같음 |
| gap follow (`lidar_reactive_drive`) | **띄우지 않는다** | **띄우지 않는다** |
| 터미널 로그 | `[EXP ego]` 줄: 시작(method, t_d), 상태 전이와 이유, belief 확정, 추월 중단. AEB는 `[Drive] mode=AEB` | `[EXP leader]` 줄: 시작(d_y, brake_delay), 출발 감지, 양보 시작, 급정지 무장·발동. AEB는 `[Drive] mode=AEB` |
| 상태 토픽 | `/exp/state` (JSON: 상태, 선행차 d·y_B·v_B, 안전 여유 h, belief b, 확정 시각) | `/exp/state` (JSON: 모드, 뒤차 거리, d_y, 양보·급정지 시각) |

- **추월 실험에서는 두 차 모두 gap follow를 끈다.** 그래야 앞차가 비킨 것이 양보 로직 때문이고 뒤차가 옆으로 간 것이 추월 판단 때문이라고 확실히 말할 수 있다. gap follow는 지금처럼 기본 주행·보정 실험에서만 쓴다.
- 앞차는 조이스틱 없이 자동 주행한다. 비키는 거리 d_y는 `run_id`로 정해지는 0.6~1.2 m 값이고 시작할 때 터미널과 `/exp/state`에 남는다.
- 차 사이 통신은 쓰지 않는다. 두 차가 모두 라이다로만 상대 차를 본다.
- `LeaderRunner`·`EgoRunner`가 시뮬과 같은 명령을 내는지는 `sim/aur_overtake/test_sim.py`의 `test_leader_runner_matches_sim_leader`, `test_ego_runner_matches_sim_ego`가 매 scan 비교한다 (Y·NY).

### 한계 (두 역할 공통)

| 무엇 | 영향 |
|---|---|
| 차에서 실행해 보지 않았다 (Mac에서 단위·시뮬 테스트만) | 처음에는 바퀴를 띄우고 시작 줄과 토픽부터 본다 |
| `/sensors/core` 속도를 ERPM으로 가정 (미확인) | leader: 출발 감지가 어긋난다. **ego: 안전 거리 h 계산이 틀린다.** 보정 실험(ERPM 환산)으로 먼저 확인 |
| ego `t_d` 0.25 s는 시뮬 값 | 실차 제동 지연을 재서 `t_d:=`로 준다 |
| 서보 중심 오차(트림) | 차가 한쪽으로 흐르고, 큰 트림은 앞차 양보 반응을 늦춰 판단을 틀리게 한다. 차마다 `role:=straight`로 `servo_center`를 보정한다 |
| `autonomous_drive`의 AEB 속도는 여전히 0 (`/vesc/odom` 없음) | geometric AEB 한계가 고정 0.25 m. 시뮬은 실제 속도 × 0.35 s |
| 앞차 속도 0.9 m/s, 추월 속도 1.5 m/s, 차선 오프셋은 `config/exp_common.yaml` 값 | 바꾸려면 그 파일을 고치고 다시 빌드 |

### 남은 코드

| 필요한 코드 | 단계 |
|---|---|
| `/sensors/core` 속도를 `autonomous_drive` AEB에도 | 4 |
| MERGE 느림·끼어들기, 급정지 전 h < 0 수정 (열린 문제) | - |
| 비상정지 heartbeat | 6 |
| 차별 환경 스크립트 | 7 |
| bag 분석 스크립트 | 8 |

## 5. 받는 명령 (두 차 공통, 경로만 다름)

```bash
cd ~/f1tenth_ws/src/f1tenth_algo      # 파랑이. 검정이는 ~/ros2_ws/src/f1tenth_algo
git fetch origin && git switch exp/aur-overtake   # 처음 한 번
git pull --ff-only
cd ~/f1tenth_ws                       # 파랑이. 검정이는 ~/ros2_ws
colcon build --packages-select my_algo
source install/setup.bash
```

두 차가 같은 커밋인지는 `git log --oneline -1`로 확인하고, 그 커밋을 회차표에 적는다.
