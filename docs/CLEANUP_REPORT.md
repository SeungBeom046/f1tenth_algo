# main 기준 정리 보고서

조사일 2026-11-11. 대상 `github.com/SeungBeom046/f1tenth_algo` (`my_algo`). 코드 수정 없음.
로컬 사본: Mac `~/f1tenth_ws/f1tenth_gym_ros/my_algo`. Jetson 사본(`~/f1tenth_ws/src/f1tenth_algo`)은 **미확인**.

## 1. 브랜치 구조

| 브랜치 | 커밋 | 비고 |
|---|---|---|
| `main` | `b054758` imu속도 측정 및 min rpm 4000으로 수정 (2026-10-27) | local = origin |
| `exp/occlusion-speed` | `47121b9` 세션 1 측정값 반영 | local = origin |
| `sim/mujoco` | `47121b9` (**exp/occlusion-speed와 같은 커밋**) | local = origin |

- 히스토리는 일직선이다. main(`b054758`)이 두 브랜치의 merge-base이고, main 뒤로 44개 커밋이 쌓였다 (`rev-list --left-right`: main 쪽 0, 브랜치 쪽 44).
- `git diff --stat main...<branch>`: 두 브랜치 모두 152 files, +15038 / −576 (같은 커밋이라 동일).
- sim/mujoco 전용 커밋은 없다. MuJoCo 커밋(`c6b587b`, `cec6651` 이후 `sim:` 접두 커밋)은 exp/occlusion-speed에도 들어 있다.
- 원격 default 브랜치: `git ls-remote --symref origin HEAD` → `refs/heads/exp/occlusion-speed` (확인). 로컬 `origin/HEAD`는 main을 가리키지만 오래된 캐시다. `gh` CLI가 이 Mac에 없다.
- 태그: `baseline-pre-refactor` (annotated, → `9fff141`, 원격에도 있음). 그 밖의 태그 없음.

## 2. main에 실험 코드가 섞였는가

**섞이지 않았다.** `git grep -i` (phantom, occlu, mujoco, sim/, B0/B1/B2, exp_mode, trigger, car2, v_cap, gap-aware, popout) on `origin/main` 결과:
`joy_controller.py`의 `get_left_trigger_scale` (조이스틱 LT 트리거 버튼) 3줄뿐. 실험 기능 아님.

main 파일 목록 (22개): `.DS_Store`, `.gitignore`, `README.md`, `launch/{autonomous_drive,camera_drive,imu_motion,lidar_drive}.launch.py`,
`my_algo/{aeb,autonomous_drive,autonomous_metrics_logger,camera_reactive_drive,imu_integration,imu_motion_logger,joy_controller,lidar_reactive_drive,vehicle_utils}.py`,
`package.xml`, `resource/my_algo`, `setup.cfg`, `setup.py`, `test/test_imu_integration.py`.

참고: main에 남은 것 중 주행과 무관한 것
- IMU 단독 기록 도구 (`imu_motion_logger.py`, `imu_integration.py`, `imu_motion.launch.py`, 테스트). 별도 launch로만 실행되고 주행 경로에는 없다. exp에서는 `ece4f2c`로 `archive/`에 옮겼다.
- `.DS_Store` 추적 중.
- README가 코드와 다르다: README `MIN_DRIVING_RPM = 2000`, `CRUISE_RPM = 8200`, `safety_margin 0.18`, `aeb_path_margin 0.10`, `ros-humble-*` apt 안내 ↔ 코드 4000 / 5740 / launch 0.25 / 0.15, 차량은 Foxy.

## 3. main 실행 경로 (`ros2 launch my_algo lidar_drive.launch.py`)

```
/livox/lidar (PointCloud2, Livox 드라이버 — 이 launch 밖)
  → pointcloud_to_laserscan  (min_height −0.1, max_height 0.5, target_frame livox_frame, 0.00436 rad, range 0.05–20)
  → /scan
  → lidar_reactive_drive      (my_algo/lidar_reactive_drive.py)  /scan, /vesc/odom, /autonomous_mode, /joy_active 구독
       → /reactive/lidar_command (String JSON ReactiveDriveCommand)
  → autonomous_drive          (my_algo/autonomous_drive.py + aeb.py)  50 Hz 타이머
       → /commands/motor/speed (ERPM), /commands/servo/position
       → /autonomous_mode heartbeat 10 Hz, /aeb/active, /aeb/status
realsense2_camera rs_launch.py + camera_reactive_drive   (제어 토픽 발행 없음, /aeb/* 만 구독)
autonomous_metrics_logger                                (발행 없음, CSV)
joy_node, joy_controller                                 (launch에서 주석 처리)
```

- 모듈 의존: `lidar_reactive_drive`, `autonomous_drive`, `aeb`, `joy_controller` → `vehicle_utils`. `autonomous_drive` → `aeb.AEBSafetyModel`.
- IMU·TF: 주행 경로 코드에 IMU 구독, tf2 사용 없음 (확인). 좌표 변환은 `x = r·cos θ − lidar_to_bumper_dist(0.30)`, yaw 보정 없음.
- 파라미터 위치: **config YAML 없음.** 전부 launch 파일 안 dict + 노드 `declare_parameter` 기본값 + 코드 안 숫자 상수.
  - launch dict: `lidar_reactive_drive` 20개 (vehicle_width 0.30, safety_margin 0.25, lookahead 2.6 …), `autonomous_drive` 10개 (aeb_ttc 0.55, half_width 0.15, path_margin 0.15 …).
  - 코드 안 상수: `lidar_reactive_drive.py` 약 38개 (FOV 120°, corridor 섹터 45–115°, 점수 가중치, mode_hold 0.45 s, rate limit 0.08/0.22 …), `aeb.py` comfortable_decel 2.8, `vehicle_utils.py` RPM·서보 상수.
- `setup.py` entry_points 6개: `imu_motion_logger, lidar_reactive_drive, camera_reactive_drive, autonomous_drive, joy_controller, autonomous_metrics_logger`. data_files: `package.xml`, `launch/*.launch.py` (config 디렉터리 없음).
- `autonomous_drive.launch.py`: 같은 노드 + rosbridge_websocket. 파라미터 일부 다름 (`max_steering`을 autonomous_drive에 안 넘김 → 노드 기본 0.78로 같음, 로거 파라미터 추가).
- Python 3.8: main 전체 `.py`를 `ast.parse(feature_version=(3,8))`로 파싱 통과. `list[int]`, `X | None`, `match` 없음 (grep).

### 조이스틱 없이 바로 자율주행하는 위치
- `lidar_reactive_drive.py:62`, `autonomous_drive.py:60` `declare_parameter('start_autonomous', True)` + launch에서도 `True`.
- launch에서 joy 노드가 주석이다. 주석을 풀면 `joy_controller`가 시작 시 `autonomous_mode=False`를 20 Hz로 발행하므로 자율주행이 바로 시작되지 않는다 (코드 확인, 차량 미확인).

### 이 상태에서 쓸 수 있는 정지 수단 (코드 기준, 차량 미확인)
| 수단 | 위치 | 동작 |
|---|---|---|
| planner 정지선 | `lidar_reactive_drive.scan_callback` | 진행 띠 안 최근접 ≤ `0.35 + v·0.45 + 0.20` (상한 1.5 m) → rpm 0 |
| gap 없음 정지 | `plan_gap_follow` | gap 없고 전방 ≤ 0.75 m → rpm 0 |
| AEB | `aeb.py`, `autonomous_drive.control_timer_callback` | 띠 ±0.30 m 안 최근접 ≤ max(0.25, v²/5.6+0.20, v·0.55) → rpm 0. `/scan` 미수신이어도 정지 |
| planner 명령 만료 | `select_reactive_command` | 명령이 1.0 s 넘게 안 오면 rpm 0, 조향 중앙 |
| 소프트웨어 정지 | `/autonomous_mode` 구독 | `ros2 topic pub --once /autonomous_mode std_msgs/msg/Bool "{data: false}"` → 두 노드가 AUTO 해제, autonomous_drive heartbeat가 이후 False를 계속 발행 |
| Ctrl+C | 각 노드 `main()` finally | 정지 명령 발행 시도. Foxy에서 SIGINT 뒤 publish가 실제로 나가는지 **미확인** |
| 하드웨어 | — | 조이스틱 B 버튼은 노드가 꺼져 있어 **없음**. VESC 드라이버 명령 타임아웃 동작 **미확인** |

## 4. 실험 브랜치에만 있는 것 중 범용 개선 후보

검증: exp HEAD의 주행 파일(`vehicle_utils`, `aeb`, `lidar_reactive_drive`, `autonomous_drive`, `lidar_drive.launch.py`)을 main 기준으로 `scripts/verify_refactor_offline.py --baseline origin/main --seeds 6` 실행 → **28,521개 메시지 전부 일치** (OPEN_SPACE / WALL_FOLLOW / OBSTACLE_AVOIDANCE / STOP, AEB 작동 포함). 합성 입력 기준이며 녹화 스캔으로는 미확인.

| # | 항목 | 커밋 | 파일 | main 영향 | 위험 |
|---|---|---|---|---|---|
| 1 | AEB·planner 하드코딩 상수 → 파라미터 (값 불변) | `fce47dd` | `aeb.py`, `autonomous_drive.py`, `lidar_reactive_drive.py` | 출력 동일 (위 검증). 3단계 "상수→파라미터"를 이미 해 둔 것 | 낮음. 파라미터가 38개 늘어 launch에서 잘못 덮어쓸 여지 |
| 2 | 오프라인 회귀 비교 도구 + rosbag 재생 검증 + launch 인자 | `9fff141`, `b98707b` | `scripts/verify_refactor_offline.py`, `verify_refactor.py`, `replay_drive.sh`, `lidar_drive.launch.py` (`use_camera`, `use_sim_time`, `use_metrics_logger`, 기본값 = 현재 동작) | 인자 기본값이면 동작 동일. 3단계 회귀 기준으로 바로 사용 가능 | 낮음. `replay_drive.sh` 원본은 Humble `--clock` 사용 → Foxy용 수정은 #3에 섞여 있음 |
| 3 | Foxy용 bag 재생기 (`/clock` 발행) | `714321f` (일부) | `scripts/bag_player.py`, `test/test_bag_player.py`, `replay_drive.sh` 수정분 | 주행 코드 무관 | 중간. 커밋에 car2·livox·tf·pc2·occupancy가 섞여 있어 파일 단위로 골라야 함 |
| 4 | 읽기 전용 차량 점검 스크립트 | `c266f90`, `12bc05f`, `714321f` | `scripts/check_vehicle.sh`, `scripts/echo_once.py`, `scripts/env_ego.sh` | 주행 코드 무관 | 중간. 9곳이 IMU/TF/exp 항목을 점검 → main용으로 줄여야 함 |
| 5 | IMU 단독 기록 도구 archive, `.DS_Store` 추적 해제 | `ece4f2c` | `archive/…`, `setup.py`(entry_point 1개 삭제), `setup.cfg`, `.gitignore` | 주행 경로 무관. `imu_motion_logger` 실행 파일이 없어짐 | 낮음. 파일 이동이라 승인 필요 |
| 6 | 순수 로직 테스트용 가짜 rclpy | `deb451f` 외 | `test/_rosstub.py` | 테스트 전용 | 낮음. exp 메시지 타입까지 포함 → 필요한 부분만 |
| 7 | 노드 docstring 형식 정리 | `cb56f14` | drive 노드 6개 | 코드 변경 없음 | 내용에 exp 모드·common.yaml 언급 → 그대로 이식 불가, 다시 써야 함 |
| 8 | 차량 실측 치수·정적 TF | `096a238`, `15371fe`, `47121b9` | `config/vehicle_geometry.yaml`, `launch/tf_static.launch.py` | main은 TF 미사용이라 영향 없음 | 낮음. main 기준(TF 미사용)과 목적이 달라 기본은 제외 권장 |

후보가 아닌 것:
- **조이스틱 데드맨·비상 정지**: exp의 개선은 `exp_drive_mux.py`/`mux_logic.py`(B 버튼이 속도 계층·AEB보다 우선) 안에 있다. exp 파이프라인 전용이라 main에 바로 이식할 범용 코드는 없다. main `joy_controller.py`는 exp에서 docstring만 바뀌었다.
- **발행자 점검(preflight / publisher_guard)**: `/commands/*`에 다른 발행자가 있는지 보는 기능은 범용이지만 `exp_drive_mux`의 `/exp/preflight_ok`에 묶여 있다. 쓰려면 main용으로 새로 연결해야 한다.
- IMU republisher, deskew, occupancy, phantom, speed layer, trigger, car2, analysis/, sim/ — 실험 전용.

## 5. 작업 트리·stash·최소 속도

- `git status` (Mac, `sim/mujoco` 체크아웃): 커밋 안 된 변경 2개
  - `.gitignore`: `reports/` 추가 (랩미팅 자료 비공개)
  - `config/imu.yaml`: `gyro_bias` 세션 1 측정값으로 교체 (`[-0.00605, 0.00343, -0.00071]`)
  - 둘 다 exp 파일이고 main에는 없다. 건드리지 않았다.
- `git stash list` (Mac): **비어 있음.** Jetson의 stash는 **미확인**.
- 바깥 레포 `f1tenth_gym_ros`(f1tenth 원본)에 대량 삭제 상태가 있으나 이 작업 대상 아님.

최소 속도 정의 (모두 `my_algo/vehicle_utils.py`, main과 exp 같음):

```python
ERPM_GAIN = 4614.0
MIN_DRIVING_RPM = 4000.0          # = 0.867 m/s
MIN_DRIVE_ERPM = MIN_DRIVING_RPM
MIN_DRIVE_SPEED_MS = MIN_DRIVING_RPM / ERPM_GAIN
DRIVE_SPEED_SCALE = 0.70
CRUISE_RPM = 5740.0
MAX_RPM = 12000.0
```

- 적용: `sanitize_rpm()` — 0이 아니고 |rpm| < 4000이면 ±4000으로 올림, ±12000 클램프. `lidar_reactive_drive`(명령 발행 전), `autonomous_drive`(발행 직전 두 번)에서 호출. gap이 없을 때 fallback 속도도 `MIN_DRIVING_RPM`.
- 조이스틱: `apply_min_drive_speed()`로 같은 규칙. 최대 수동 속도 `2.0 × DRIVE_SPEED_SCALE` m/s.
- exp: `speed_policy.V_MIN = MIN_DRIVING_RPM / ERPM_GAIN`으로 같은 상수를 참조.
- git 이력 (`git log --all -G`): `093c5c8` MIN 2000 / CRUISE 8200 / SCALE 0.80 → `dcc0210` CRUISE 5740 / SCALE 0.70 → `b054758` MIN 4000.
- **4500 RPM, CRUISE 3000은 어떤 브랜치·커밋에도 없다** (`git log --all -S`, `git grep`). "예전 stash"는 Jetson 로컬에만 있었던 것으로 보이며 **미확인**. 주석 한 곳(`sanitize_rpm` 안 "최소 2000 RPM")이 값과 맞지 않는다.

## 6. 결정 (2026-11-11)

1. 태그 하나: `archive/occlusion-mujoco-20261110` → `47121b9`. 두 브랜치 유지.
2. IMU 기록 도구: 파일 이동 없음. setup.py의 entry_point와 `imu_motion.launch.py` 설치만 해제.
3. 후보 #1(`fce47dd`), #2(`9fff141`, `b98707b`) 이식. `refactor/main-baseline`에서, main 직접 push 없음.
4. README를 코드 기준으로 다시 씀.
5. 이 보고서는 `refactor/main-baseline`의 `docs/CLEANUP_REPORT.md`.
6. bag 재생 검증 보류. 방법은 `docs/OPEN_ITEMS.md`.
7. Mac의 미커밋 `.gitignore`, `config/imu.yaml`은 그대로 (작업은 별도 git worktree에서 함).

## 7. 결과

- `fce47dd`는 exp 전용 모듈·YAML에 의존하지 않는다 (추가 import 없음, 바뀐 파일 3개: `aeb.py`, `autonomous_drive.py`, `lidar_reactive_drive.py`). 충돌 없이 cherry-pick됨.
- 파라미터 기본값 = 기존 main 상수. `MIN_DRIVING_RPM` 4000, `CRUISE_RPM` 5740, `MAX_RPM` 12000은 ROS 파라미터로 바꾸지 않고 `vehicle_utils.py` 상수로 둔다 (planner·AEB·조이스틱이 같은 값을 쓰도록 한 곳에 유지).
- 회귀: `verify_refactor_offline.py --baseline b054758 --seeds 12` → 57,042개 메시지 일치 (이식 직후, 파라미터 출력 추가 후 각각).
- 회귀 도구 한계: 합성 입력이 0 < rpm < 4000 구간을 지나지 않아 `MIN_DRIVING_RPM` 4000→4100 변경을 잡지 못했다 (`CRUISE_RPM` 변경은 잡음). `test_min_speed_rule_unchanged`로 값과 `sanitize_rpm` 규칙을 직접 고정했다.
- Foxy 파라미터 우선순위: 두 노드가 시작 시 `effective parameters: ...`를 출력한다. 실행 중 확인은 `ros2 param dump <node> --print`.
