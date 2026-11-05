# 세션별 실행 문서 (차 앞에서 순서대로)

[RUNBOOK.md](RUNBOOK.md)가 "어떻게 돌리는가"라면, 이 문서는 "오늘 차 앞에서 무엇을 순서대로 치는가"다.
차량은 **ROS2 Foxy (Ubuntu 20.04, Python 3.8)**. "TBD(세션 1)"는 세션 1 결과로 확정할 값 ([OPEN_ITEMS.md](OPEN_ITEMS.md) 4절).

## 공통

### 터미널 이름 (이 문서 전체에서 같은 뜻)

| 이름 | 기기 | 하는 일 |
|---|---|---|
| **A** | 에고 | Livox 드라이버 |
| **B** | 에고 | VESC 드라이버 |
| **C** | 에고 | 알고리즘 (drive: `lidar_drive.launch.py` / exp: `bringup.launch.py`) |
| **D** | 에고 | 기록·점검 스크립트 |
| **E** | 에고 | 정적 TF, odom(`vesc_odom.launch.py`, 필요할 때) |
| **F** | 에고 | 모니터링 |
| **A2** | #2 | `car2.launch.py` (이것 하나만) |
| **L** | 노트북 | RViz, 로그 백업, 분석 |

### 모든 에고 터미널 첫 줄

```bash
source ~/f1tenth_ws/src/my_algo/scripts/env_ego.sh     # TBD(세션 1): my_algo 소스 위치가 다르면 그 경로
```
정상: `[env_ego] ROS_DISTRO=foxy ROS_DOMAIN_ID=37 MY_ALGO_REPO=... S=...`.
`env_ego.sh`가 Foxy → `~/vesc_ws` → `~/f1tenth_ws` 순서로 source하고, `ROS_DOMAIN_ID`(기본 37), `$S`(스크립트 경로),
`$MY_ALGO_REPO`를 정한다. #2는 `env_car2.sh` (같은 `ROS_DOMAIN_ID`).

### 기존 실행 방식과의 관계

| 기존 (사용자 방식) | 이 문서 | 차이 |
|---|---|---|
| A: `ros2 launch livox_ros_driver2 rviz_MID360_launch.py` | `ros2 launch my_algo livox_mid360.launch.py` | 같은 드라이버 설정(PointCloud2, xfer_format 0), **Jetson RViz 없음** (RViz는 노트북 L) |
| B: `ros2 launch vesc_driver vesc_driver_node.launch.py` | 그대로 | 이 launch는 vesc_driver만 띄운다. odom은 E의 `vesc_odom.launch.py` (세션 1 결과로 결정) |
| C: `colcon build --symlink-install ...; ros2 launch my_algo lidar_drive.launch.py` | drive는 그대로, exp는 `bringup.launch.py mode:=exp ...` | |

### 안전 기본 규칙 (모든 주행 단계)
- 주행 전 트랙 비우기, 폼 방호벽 위치, **물리 전원 차단 담당자 1명** (위치: TBD(세션 1)).
- exp 모드는 조이스틱 B가 최우선 정지. **drive 모드(`lidar_drive.launch.py`)에는 조이스틱이 없다** (주석 처리 상태 그대로) →
  drive 주행 중 정지 수단은 C 터미널 Ctrl+C와 물리 전원 차단뿐이다. 짧게, 낮은 위험 구간에서만.
- 리플레이·정지 측정 단계는 **VESC 전원 OFF 또는 바퀴를 띄운 상태**.

---

## 세션 1: 정비 검증 (약 80분)

목적: 토픽·노드 이름 확정, drive 모드 리팩터 불변 검증(rosbag), 장착값 측정. 결과물은 `~/session1/`.

> **⚠ 모든 터미널(A, B, C, D, E, F, #2의 A2)은 반드시 env 스크립트로 시작한다.**
> 에고: `source ~/f1tenth_ws/src/my_algo/scripts/env_ego.sh` / #2: `source ~/f1tenth_ws/src/my_algo/scripts/env_car2.sh`
> 한 터미널이라도 빠지면 그 터미널만 다른 `ROS_DOMAIN_ID`(기본 0)로 떠서 서로 안 보인다.
>
> | 증상 | 원인 |
> |---|---|
> | `ros2 topic list`에 드라이버 토픽이 안 보임, `echo_once.py`가 `토픽 없음`·시간 초과 | 그 터미널의 도메인이 다르다 |
> | 노드는 떠 있는데 다른 노드가 데이터를 못 받음 (예: bringup은 떴는데 `/livox/lidar` 수신 없음) | 드라이버 터미널과 알고리즘 터미널의 도메인이 다르다 |
> | `ros2 node list`에 #2(`/car2/...`)가 안 보이거나, 반대로 다른 사람 장비 노드가 보임 | 두 차 도메인 불일치, 또는 근처 장비와 도메인이 겹침 |
>
> 확인: 각 터미널에서 `echo $ROS_DOMAIN_ID $ROS_DISTRO` → 모두 `37 foxy`(기본값)여야 한다.
> 이미 띄운 노드는 도메인을 바꿀 수 없으니, 틀린 터미널은 Ctrl+C → env 다시 source → 다시 실행.

순서는 터미널 상태가 같은 것끼리 묶었다: **(a) → (a2) → (b) → (b1) → (b2) → (c) → (e) → (f) → (g) → (d)**.
라이다를 켜 둔 상태에서 할 일(b, b2, c, e, f)을 먼저 끝내고, 드라이버를 모두 끄는 재생 검증(d)을 마지막에 한다.
(b)부터 라이다가 켜져 있어 (e) 시점에는 예열 5분이 지나 있다.

### (a) 코드 받기, 빌드 — 에고 D (5분)

| 켜져 있어야 함 | 꺼야 함 |
|---|---|
| 없음 | 없음 |

```bash
source ~/f1tenth_ws/src/my_algo/scripts/env_ego.sh
mkdir -p ~/session1 && cd $MY_ALGO_REPO
git fetch origin --tags && git checkout exp/occlusion-speed && git pull
git log --oneline -1 | tee ~/session1/a_git.txt
cd ~/f1tenth_ws && colcon build --symlink-install --packages-select my_algo 2>&1 | tail -n 3 | tee -a ~/session1/a_git.txt
source ~/f1tenth_ws/src/my_algo/scripts/env_ego.sh
# (d)에서 기준선 태그를 checkout해도 쓸 수 있게 현재 브랜치 스크립트 사본을 만든다
mkdir -p ~/session1/tools && cp $MY_ALGO_REPO/scripts/*.py $MY_ALGO_REPO/scripts/*.sh ~/session1/tools/
```
- 정상: `Finished <<< my_algo`, `[env_ego] ... S=...`
- 저장: `~/session1/a_git.txt`
- 보낼 것: `a_git.txt`

### (a2) numpy 1.17.4 호환 확인 — 에고 D (2분)

Jetson 기본 numpy(Ubuntu 20.04 apt 1.17.4)로 순수 모듈(격자, phantom, 지표, 바닥 평면, deskew, PointCloud2, IMU, 제동 분석)
단위 테스트를 돌린다. ROS 노드 테스트는 실제 rclpy가 있으면 자동으로 건너뛴다.
```bash
cd $MY_ALGO_REPO && python3 -c "import sys, numpy; print(sys.version.split()[0], 'numpy', numpy.__version__)" | tee ~/session1/a2_numpy.txt && PYTHONPATH=.:test python3 -m unittest discover -s test 2>&1 | tail -n 4 | tee -a ~/session1/a2_numpy.txt
```
- 정상: `3.8.x numpy 1.17.4`, 마지막 줄 `OK` (`skipped=N`은 정상 — 가짜 rclpy·rosbags 테스트).
- `FAILED`나 `AttributeError`가 나오면 (b)로 넘어가도 되지만 `a2_numpy.txt`를 꼭 보낸다 (해당 노드가 차량에서 죽을 수 있음).
- 보낼 것: `a2_numpy.txt`

### (b) 차량 점검 — 에고 A, B, D (10분)

| 켜져 있어야 함 | 꺼야 함 |
|---|---|
| A Livox, B VESC (차는 정지, 바퀴를 띄우면 더 안전) | C 알고리즘 |

```bash
# A
ros2 launch my_algo livox_mid360.launch.py
# B
ros2 launch vesc_driver vesc_driver_node.launch.py
# D
cd ~/session1 && bash $S/check_vehicle.sh
ros2 node list | tee b_nodes.txt
python3 $S/echo_once.py /livox/lidar --field fields | tee b_cloud_fields.txt     # 점별 timestamp 필드 (deskew 판단)
ros2 topic info -v /vesc/odom | tee b_odom_info.txt                              # vesc_to_odom이 도는지 (발행자 0이면 안 돔)
```
- 정상: `check_vehicle_<host>_<시각>.log` 생성. 로그 "IMU 정지 통계"에서 `|a|` ≈ 1.0(g 단위) 또는 ≈ 9.8.
  `b_cloud_fields.txt`에 `x y z intensity tag line timestamp` 같은 필드 목록.
- 저장: `check_vehicle_*.log`, `b_nodes.txt`, `b_cloud_fields.txt`, `b_odom_info.txt`
- 보낼 것: 네 파일 전부

### (b1) `/vesc/odom` 존재 여부 판정 — 에고 D (1분, 이후 단계가 갈린다)

```bash
cd ~/session1
N=$(ros2 topic info /vesc/odom 2>/dev/null | sed -n 's/^Publisher count: //p'); echo "vesc_odom_publishers=${N:-0}" | tee b1_odom_verdict.txt
ros2 topic list | grep -i odom | tee -a b1_odom_verdict.txt      # 다른 이름(/odom 등)으로 나오는지
```

| 결과 | 판정 | 이후 |
|---|---|---|
| `vesc_odom_publishers=1` | 기존 주행에 odom 있음 | (c) 그대로. 세션 2의 E에서 `vesc_odom.launch.py`는 **띄우지 않는다** (발행자 중복) |
| `vesc_odom_publishers=0`이고 다른 odom 토픽도 없음 | **기존 drive 모드는 odom 없이 동작해 왔다** — drive AEB가 속도와 무관하게 앞 끝 0.25 m 고정, planner 정지선·lookahead도 속도 0 기준 | (c)는 `vesc_odom.launch.py` **없이** 지금 상태 그대로 녹화 (drive 모드 불변 유지). 아래 (c) 안전 주의를 지킨다. 세션 2부터 exp 세션에만 E에서 `vesc_odom.launch.py` |
| 다른 이름(예: `/odom`)으로 있음 | odom은 있지만 이름이 다름 | (c) 그대로 녹화하되 그 토픽도 기록에 추가 (`ODOM_TOPIC=<이름>`). 이름 확정은 보고 후 |

- 보낼 것: `b1_odom_verdict.txt`
- odom이 없던 것으로 확인되면 OPEN_ITEMS 7절에 기록하고, drive 모드는 고치지 않는다 (별도 과제).

### (b2) 두 차 같은 도메인 발행자 확인 + 발행자 점검(warn) + /joy 주기 — 에고 D, #2 A2 (10분)

| 켜져 있어야 함 | 꺼야 함 |
|---|---|
| A, B (에고), A2 (#2) | C, #2의 Livox·TF·그 밖의 모든 것 |

#2 Jetson (먼저 (a)와 같이 pull·빌드, `~/car2_vesc.yaml`에 #2 VESC 포트 등. 바퀴를 띄운다):
```bash
# A2
source ~/f1tenth_ws/src/my_algo/scripts/env_car2.sh
ros2 launch my_algo car2.launch.py vesc_config:=$CAR2_VESC_CONFIG
```
정상 (A2): `#2 대기: v=0.8 m/s (3691 ERPM) ... motor_topic=/car2/commands/motor/speed`.
VESC 패키지 이름이 다르면 `vesc_driver_pkg:=` 등으로 바꾼다 (TBD(세션 1)).

에고:
```bash
# D
cd ~/session1
for t in /livox/lidar /livox/imu /commands/motor/speed /commands/servo/position /tf /tf_static \
         /vesc/odom /car2/commands/motor/speed /car2/odom /trigger; do
  echo "===== $t"; ros2 topic info -v "$t"
done 2>&1 | tee b2_domain_check.txt
ros2 node list | tee -a b2_domain_check.txt

# 발행자 점검을 warn 모드로 (실제 노드 이름 확인용, 회차를 막지 않음)
P=$(ros2 pkg prefix my_algo)/share/my_algo/config
ros2 run my_algo publisher_guard_node --ros-args --params-file $P/preflight.yaml -p mode:=warn &
sleep 6; python3 $S/echo_once.py /exp/preflight --transient-local | tee b2_preflight.txt; kill %1

# 조이스틱을 켜고 손대지 않은 상태에서 /joy 주기
ros2 run joy joy_node --ros-args -p autorepeat_rate:=20.0 &
sleep 2; timeout -s INT 5 ros2 topic hz /joy 2>&1 | tail -n 3 | tee b2_joy_hz.txt; kill %1
```
- 정상: `/livox/*`, `/commands/motor/speed`, `/tf_static`, `/tf`의 Publisher에 `/car2/...` 노드가 **없다**.
  `/car2/commands/motor/speed`의 Subscriber는 #2 VESC 드라이버뿐. `b2_joy_hz.txt` 약 20 Hz.
- `b2_preflight.txt`의 `problems`에 나온 노드 이름이 실제 이름이다 (예상 이름과 다르면 다음 커밋에서 `preflight.yaml` 확정 → enforce 전환).
- 이상하면 (c)로 넘어가지 말고 보낸다. 끝나면 **A2 Ctrl+C**.
- 보낼 것: `b2_domain_check.txt`, `b2_preflight.txt`, `b2_joy_hz.txt`

### (c) 기존 drive 모드 주행 bag 기록 — 에고 A, B, C, D (10분) ⚠ 주행

| 켜져 있어야 함 | 꺼야 함 |
|---|---|
| A, B, C(drive), D | #2 |

```bash
# D (주행 시작 전에 먼저)
cd ~/session1
ros2 bag record -s sqlite3 -o drive_bag \
  /livox/lidar /livox/imu /vesc/odom /autonomous_mode /sensors/core \
  /scan /reactive/lidar_command /commands/motor/speed /commands/servo/position /aeb/active
# C (기존 방식 그대로)
ros2 launch my_algo lidar_drive.launch.py
```
30초 이상 정상 주행 → **C Ctrl+C**(차 정지) → D Ctrl+C →
```bash
ros2 bag info drive_bag | tee c_bag_info.txt
```
- 안전: drive 모드는 조이스틱이 없다. 정지 = C Ctrl+C 또는 물리 전원. 트랙을 비우고 짧게.
- 안전 (b1에서 odom 없음으로 판정된 경우): **drive AEB가 속도와 무관하게 앞 끝 0.25 m에서만 걸릴 수 있다.**
  2 m/s면 0.25 m 안에서 멈추지 못한다. **저속 구간 위주로, 장애물 없는 넓은 곳에서**, 사람·물건과 충분히 떨어져 주행한다.
  벽·박스로 직진하는 경로를 만들지 않는다. 녹화 목적은 drive 명령 기록이지 AEB 시험이 아니다.
- 정상: `c_bag_info.txt`에 `/livox/lidar` 약 10 Hz × 시간. `/vesc/odom`이 0개여도 그대로 둔다 (그게 현재 drive 상태).
- 토픽 이름이 (b)와 다르면 실제 이름으로 바꿔 기록한다.
- 보낼 것: `c_bag_info.txt` (bag 자체는 보관만)

### (e) IMU 정지 캘리브레이션 — 에고 A, D (10분)

| 켜져 있어야 함 | 꺼야 함 |
|---|---|
| A (켠 지 5분 이상) | C. B는 꺼도 됨 |

평평한 바닥, 차 완전 정지:
```bash
# D
cd ~/session1
python3 $S/imu_static_calib.py 10 --topic /livox/imu | tee e_imu_calib_0deg.txt
# 차를 제자리에서 180° 돌린다 (같은 바닥 위치)
python3 $S/imu_static_calib.py 10 --topic /livox/imu | tee e_imu_calib_180deg.txt
```
- 정상: `중력 크기 검산 ≈ 1.00 g`, `실제 주기 ≈ 200 Hz`, 흔들림 경고 없음.
- 보낼 것: 두 파일

### (f) 라이다 원점 높이 — 에고 A, D (5분)

같은 자리, 차 주변 0.4–4 m 바닥을 비운다.
```bash
python3 $S/measure_lidar_height.py --topic /livox/lidar --frames 10 | tee f_lidar_height.txt
```
- 정상: `보정 후 바닥 z 평균`이 `-height_m`과 비슷, std ≤ 0.01 m, inlier 경고 없음.
- 보낼 것: `f_lidar_height.txt`

### (g) 로고 방향, 바퀴 지름 — 수동 (5분)

`~/session1/g_manual.txt`에 적는다:
```text
Mid-360 로고 방향: 전방 / 후방 / 좌측 / 우측
바퀴 지름 [m]:            (자로 재거나, 타이어 표시 후 5바퀴 굴린 거리 / (5 × 3.1416))
물리 전원 차단 위치:
배터리 셀 수 / 완충 전압 / 교체 기준 전압:
```

### (d) 리팩터 불변 검증: rosbag 재생 — 에고 D (15분) ⚠ 드라이버 OFF

| 켜져 있어야 함 | 꺼야 함 |
|---|---|
| D | **A Livox, B VESC (VESC 전원 OFF)**, C |

`replay_drive.sh`는 VESC 노드가 보이면 스스로 멈춘다. (a)에서 만든 **`~/session1/tools/` 사본**으로 실행한다
(기준선 태그의 스크립트는 Humble의 `bag play --clock`을 써서 Foxy에서 안 돈다).
```bash
# D
T=~/session1/tools
# 1) 기준선 (리팩터링 전 코드)
cd $MY_ALGO_REPO && git checkout baseline-pre-refactor
cd ~/f1tenth_ws && colcon build --symlink-install --packages-select my_algo && source install/setup.bash
cd ~/session1 && bash $T/replay_drive.sh drive_bag baseline_cmd.csv 0.5
# 2) 현재 브랜치
cd $MY_ALGO_REPO && git checkout exp/occlusion-speed
cd ~/f1tenth_ws && colcon build --symlink-install --packages-select my_algo && source install/setup.bash
cd ~/session1 && bash $T/replay_drive.sh drive_bag after_cmd.csv 0.5
# 3) 비교
python3 $T/verify_refactor.py compare baseline_cmd.csv after_cmd.csv | tee d_verify.txt
```
- 정상: 두 재생 모두 `재생 완료: N개 메시지`, `d_verify.txt` 마지막 줄 `결과: 일치`.
- `미매칭 비율` 경고 → 배속 0.25로 1)–3) 다시. 차이가 나오면 멈추고 `d_verify.txt`, `*_launch.log`, `*_params_*.yaml` 전부 보낸다.
- 보낼 것: `d_verify.txt`

### 세션 1 보낼 것 한 번에

```bash
cd ~ && tar czf session1.tgz --exclude='session1/drive_bag' --exclude='session1/tools' session1/
```

| 단계 | 파일 | 확정되는 것 |
|---|---|---|
| a | `a_git.txt` | 커밋, 빌드 |
| a2 | `a2_numpy.txt` | numpy 1.17.4 호환 |
| b | `check_vehicle_*.log`, `b_nodes.txt`, `b_cloud_fields.txt`, `b_odom_info.txt` | 토픽·노드 이름, IMU 단위, 점별 시각 필드(deskew), odom 유무, speed_to_erpm_gain |
| b1 | `b1_odom_verdict.txt` | 기존 주행 odom 유무 (세션 2의 E 구성, drive AEB 동작 해석) |
| b2 | `b2_domain_check.txt`, `b2_preflight.txt`, `b2_joy_hz.txt` | #2 격리, 발행자 점검 허용 이름, /joy 주기 |
| c | `c_bag_info.txt` | drive bag 내용 |
| d | `d_verify.txt` | drive 모드 불변 (차이 나면 3단계 중단) |
| e | `e_imu_calib_0deg.txt`, `e_imu_calib_180deg.txt` | roll, pitch, 자이로 바이어스, 단위 |
| f | `f_lidar_height.txt` | ORIGIN_H |
| g | `g_manual.txt` | LIDAR_YAW, 바퀴 지름, 전원 차단 위치, 배터리 기준 |

세션 1 뒤: 결과로 `tf_static.launch.py`, `imu.yaml`, `preflight.yaml`(이름 확정, `mode: enforce`), 토픽 기본값,
`vehicle_geometry.yaml`(바퀴 지름)을 고친 커밋을 받아 pull·빌드한 뒤 세션 2를 시작한다.

---

## 세션 2: Exp0 플랫폼 특성 (약 3시간)

목적: [EXP0-MEASURE] 전부 해소. 명령·판정 규칙은 [EXPERIMENT_PLAN.md](EXPERIMENT_PLAN.md) Exp0.
주행 단계는 exp 모드라 조이스틱 B 정지와 데드맨이 켜져 있다.

### 2-1. 준비와 TF 확인 — 에고 A, B, E, F, 노트북 L (15분)

| 켜져 있어야 함 | 꺼야 함 |
|---|---|
| A, B, E | C (아직) |

```bash
# A
ros2 launch my_algo livox_mid360.launch.py
# B
ros2 launch vesc_driver vesc_driver_node.launch.py
# E (정적 TF + odom. 세션 1 b1 판정이 'odom 없음'일 때만 vesc_odom)
ros2 launch my_algo tf.launch.py &
ros2 launch my_algo vesc_odom.launch.py
# F
ros2 topic info -v /tf          # Publisher가 vesc_to_odom_node 하나 (odom→base_link 발행자 하나)
cd ~ && ros2 run tf2_tools view_frames.py && ls frames*.pdf    # Foxy는 view_frames.py
ros2 run tf2_ros tf2_echo base_link livox_imu                   # 0.141, 0.018, 0.196 근처 (가정값 기준)
```
노트북 L (같은 `ROS_DOMAIN_ID`, my_algo 레포 사본):
```bash
export ROS_DOMAIN_ID=37 && source /opt/ros/foxy/setup.bash      # 노트북 ROS 배포판에 맞게
rviz2 -d <my_algo 레포>/config/exp.rviz
```
- 정상: `frames.pdf`에 `odom → base_link → livox_frame → livox_imu` 한 줄. RViz 체크리스트 ([RUNBOOK 부록 C](RUNBOOK.md#부록-c-tf장착-검증-rviz-fixed-frame--base_link)) 통과.
- 보낼 것: `frames.pdf`, RViz 체크 결과 (사진 1장)

### 2-2. IMU 확인 — 에고 D (5분)

```bash
ros2 launch my_algo imu.launch.py input_topic:=/livox/imu        # 차 정지 상태에서 시작
python3 $S/echo_once.py /imu/status                                # calibrated: true, g_ref_norm ≈ 9.81, tf_ok: true
python3 $S/echo_once.py /imu --field linear_acceleration           # 정지 시 z ≈ 9.8
```
축 방향: 차를 손으로 앞으로 밀었다 멈추며 `ros2 topic echo /imu/linear_accel_base` → x가 먼저 양수 ([RUNBOOK 부록 B](RUNBOOK.md#부록-b-imu-확인)).
끝나면 Ctrl+C (bringup이 다시 띄운다).

### 2-3. 회차 공통 루프 (2-4 ~ 2-6, 세션 4·5도 같음)

| 켜져 있어야 함 | 꺼야 함 |
|---|---|
| A, B, E (2-1과 같음), 조이스틱 | 이전 회차의 C |

```bash
# C (회차마다 새로)
ros2 launch my_algo bringup.launch.py mode:=exp method:=<M> scenario:=<S> run_id:=<n> [d_e:=...] [test_v_ref:=...]
# 정상 로그를 확인한 뒤 (발행자 점검 통과, IMU 정지 보정 완료)
# D
[TEST_V_REF=<값>] bash $S/record_run.sh exp <M> <S> <d_e|na> <n>
# F (회차 중)
ros2 topic echo /exp/stop_reason
```
1. 에고 시작선 정지 → 조이스틱 켬(MANUAL) → **LB**로 AUTO → 주행
2. 끝나면 **B**(또는 LB)로 MANUAL → **D Ctrl+C** (`discard=... aeb=...` 확인) → **C Ctrl+C**
3. 다음 회차 위치로. #2를 쓰면 출발 표시에 두고 `ros2 topic pub --once /popout/reset std_msgs/msg/Bool "{data: true}"`
- 저장: `~/exp_logs/<날짜>/<scenario>_<method>_de<..>_run<NN>/`
- 안전: 회차마다 트랙 비우기, 방호벽, 전원 차단 담당자.

### 2-4. odom 스케일 (b) — 30분
빈 트랙에 5 m 직선 표시. E0 정지선이 5 m 표시 뒤가 되게.
```bash
# C/D: 0.9 m/s 3회 (n = 1..3)
ros2 launch my_algo bringup.launch.py mode:=exp method:=B0 scenario:=E0 run_id:=<n> test_v_ref:=0.9
TEST_V_REF=0.9 bash $S/record_run.sh exp B0 E0 na <n>
# 2.0 m/s 3회 (n = 11..13)
ros2 launch my_algo bringup.launch.py mode:=exp method:=B0 scenario:=E0 run_id:=<n>
bash $S/record_run.sh exp B0 E0 na <n>
```
회차마다 정지 위치를 줄자로 잰다 (시작선 → 앞 끝). 노트북에서:
```bash
python3 analysis/odom_travel.py ~/exp_logs/<날짜>/E0_B0_dena_run0{1,2,3} --tape <줄자1> <줄자2> <줄자3>
```
결과(비율)는 보내기만 한다 — **ERPM_GAIN 변경은 drive 속도도 바꾸므로 승인 후 별도 커밋** ([OPEN_ITEMS 2절](OPEN_ITEMS.md)).

### 2-5. 급제동 a_b (c) + 추종 지연 t_r (i) — 45분
```bash
ros2 launch my_algo bringup.launch.py mode:=exp method:=B0 scenario:=E0 run_id:=<n>    # c: n = 21..30
bash $S/record_run.sh exp B0 E0 na <n>
```
i (같은 빈 트랙): 방법과 무관한 속도 계단. mux가 주행 거리 2.5 m에서 명령을 2.0 → 1.2 m/s로 내리고, 실측 속도가 얼마나 늦게 따라오는지 잰다.
```bash
ros2 launch my_algo bringup.launch.py mode:=exp method:=B0 scenario:=E0 run_id:=<n> test_step_v:=1.2 test_step_x:=2.5   # i: n = 31..35
TEST_STEP_V=1.2 bash $S/record_run.sh exp B0 E0 na <n>
```
노트북: `python3 analysis/exp0_braking.py ~/exp_logs/<날짜> --tracking E0_B0_dena_run31 E0_B0_dena_run32 ...`
→ `exp0_braking.json` 권장값 (a_b = 급제동 CI 하한, **t_r = 인지 지연 + 추종 지연 CI 상한**). i 회차는 meta.json `test_step_v`로
구분되어 c의 급제동 분석에서 빠진다 (기록할 때 `TEST_STEP_V=1.2`를 꼭 붙일 것). 비교 실험에는 넣지 않는다.

### 2-6. AEB 단독 정지 (g) — 15분 ⚠ 정지 박스
빈 트랙, 차선 중앙 x ≈ 5 m에 박스 하나(폼). S2 정지선(7.5 m)보다 앞.
```bash
ros2 launch my_algo bringup.launch.py mode:=exp method:=B0 scenario:=S2 run_id:=<n>    # n = 91..93
bash $S/record_run.sh exp B0 S2 na <n>
```
- 정상: D 종료 출력 `aeb=1`, 충돌 없음. 이 3회는 실험 회차가 아니므로 `run_meta.py discard <run_dir> exp0_aeb_test`로 표시.

### 2-7. 트리거 지연 (d)와 #2 단독 시험 (e) — #2 A2, 에고 D (40분)

| 켜져 있어야 함 | 꺼야 함 |
|---|---|
| A2(#2), 에고 D (에고는 정지, C 꺼도 됨) | |

```bash
# A2 (#2): 0.8 m/s 5회, 끝나면 v_popout:=0.9로 다시 띄워 5회
ros2 launch my_algo car2.launch.py vesc_config:=$CAR2_VESC_CONFIG v_popout:=0.8
# D (에고): 회차마다 — 트리거 (현재 시각 stamp) → #2 정지 확인 → #2를 출발 표시로 → reset
python3 -c "import rclpy,time;from std_msgs.msg import Header;rclpy.init();n=rclpy.create_node('t');p=n.create_publisher(Header,'/trigger',10);time.sleep(1);h=Header();h.stamp=n.get_clock().now().to_msg();h.frame_id='exp0';p.publish(h);time.sleep(0.5)"
python3 $S/echo_once.py /popout/status --timeout 10
ros2 topic pub --once /popout/reset std_msgs/msg/Bool "{data: true}"
```
- 정상: `/popout/status`에 `latency_s`(수 ms, 음수면 chrony 확인), `reach_lane_s`, `speed_mean`, `speed_std`.
- 저장: #2의 `~/exp_logs/popout/popout_events.jsonl`
- 판정: [EXPERIMENT_PLAN Exp0 e](EXPERIMENT_PLAN.md) (0.8 불안정 → 0.9)
- 보낼 것: `popout_events.jsonl`

### 2-8. 측정값 반영 위치

| 측정 | 파일 · 키 |
|---|---|
| ORIGIN_H, LIDAR_ROLL/PITCH/YAW | `launch/tf_static.launch.py` |
| 자이로 바이어스, accel_unit | `config/imu.yaml` |
| a_b, t_r(구동 + 인지 지연) | `config/common.yaml` |
| trigger_latency, popout_enter_time | `config/common.yaml` |
| v_popout, erpm_gain(#2), servo_center(#2), lane_boundary_distance | `config/popout.yaml` |
| odom 스케일 | 승인 후 `vehicle_utils.ERPM_GAIN` + VESC `speed_to_erpm_gain` (OPEN_ITEMS 2절) |
| 트랙 좌표, 좌측 벽 목표 거리 | `config/scenario_*.yaml`, `config/common.yaml` `target_wall_distance` |

값은 보내 주면 커밋으로 반영하고, 반영 커밋을 pull·빌드한 뒤 세션 3.

---

## 세션 3: Exp1 경계 안정성 (약 2시간)

목적: 누적 프레임 N, deskew 결정 (RQ4). 트랙: S2 배치(틈 0.45), 이후 S3 배치(틈 0.15).
켜 둘 것: A, B, E (세션 2와 같음), 조이스틱.

### 3-1. 정지 (AUTO 없이 30초 기록)
에고를 P1 앞 1 m / 2 m / 3 m에 세우고, 위치마다 N ∈ {1, 2, 3, 5, 8}:
```bash
ros2 launch my_algo bringup.launch.py mode:=exp method:=P scenario:=S2 run_id:=<n> n_frames:=<N> known_on:=<min(2,N)> known_off:=1
bash $S/record_run.sh exp P S2 na <n>      # 30초 뒤 Ctrl+C (AUTO 진입하지 않음)
```
run_id는 위치·N 조합마다 다르게 (예: 101–115). 메모: `run_id → (위치, N)`를 `~/exp_logs/<날짜>/exp1_notes.txt`에.

### 3-2. 0.9 m/s, 2.0 m/s 통과 (N ∈ {1, 3, 5}, 각 3회)
```bash
ros2 launch my_algo bringup.launch.py mode:=exp method:=P scenario:=S2 run_id:=<n> n_frames:=<N> known_on:=<min(2,N)> test_v_ref:=0.9
TEST_V_REF=0.9 bash $S/record_run.sh exp P S2 na <n>
ros2 launch my_algo bringup.launch.py mode:=exp method:=P scenario:=S2 run_id:=<n> n_frames:=<N> known_on:=<min(2,N)>
bash $S/record_run.sh exp P S2 na <n>
```

### 3-3. 틈 폭: 정지 대 2.0 m/s, deskew off/on (S2·S3 배치, 각 3회)
`deskew:=true`는 세션 1에서 점별 시각 필드가 확인됐을 때만:
```bash
ros2 launch my_algo bringup.launch.py mode:=exp method:=P scenario:=S3 run_id:=<n> deskew:=false
ros2 launch my_algo bringup.launch.py mode:=exp method:=P scenario:=S3 run_id:=<n> deskew:=true
```
노트북: `python3 analysis/extract_run.py --all ~/exp_logs/<날짜>` → `python3 analysis/plots.py ~/exp_logs/<날짜>` (RQ4 그림).
결정 규칙은 EXPERIMENT_PLAN Exp1. 결정값(N, known_on, deskew)은 `common.yaml`에 커밋 후 고정.

---

## 세션 4: Exp2 효율 (S2–S4, 120회, 약 1.5일)

켜 둘 것: A, B, E, 조이스틱. #2 없음.

1. 순서: `python3 $S/randomize_runs.py --seed <날짜숫자>` (S2 → S3 → S4, 블록마다 방법 셔플). 출력의 `bringup:`·`record:` 인자를 그대로 쓴다.
2. 트랙 배치 체크 (시나리오가 바뀔 때마다):
   - [ ] S2: 박스 P1–P3, 틈 0.45 m, 좌측 벽, 박스 차선 쪽 면 위치 (마킹)
   - [ ] S3: 틈 0.15 m
   - [ ] S4: 우측도 연속 벽
   - [ ] 시작선, 가속 구간 끝, 정지선, 방호벽, 바닥 눈금
3. 회차 루프: 2-3과 같다. 회차 전 체크 ([RUNBOOK 2절](RUNBOOK.md#2-회차-진행-체크리스트)): 배터리 전압, 박스 위치, IMU 재보정
   (`ros2 topic pub --once /imu/recalibrate std_msgs/msg/Bool "{data: true}"`), 발행자 점검 통과.
4. 폐기·AEB 오작동: D 종료 출력이 `discard=True` 또는 `malfunction=True`면 순서표에 표시하고 같은 조건을 블록 끝에 다시.
   수동 폐기: `python3 $S/run_meta.py discard <run_dir> <사유>`.
5. 비상 정지: 조이스틱 B → (끊김 0.5 s 자동 정지) → 정지선 → 물리 전원.
6. 하루가 끝나면 부록 B로 백업.

## 세션 5: Exp3 안전 (S1, 115회, 약 1.5일) ⚠ 충돌 가능

켜 둘 것: 에고 A, B, E, 조이스틱 + #2 A2. 폼 범퍼(#2 측면), 방호벽, 천장 카메라 60 fps.

1. 순서: `randomize_runs.py` 출력의 S1 부분 (B0·d_e 1.0은 5회).
2. 트랙 체크: 틈 0.45 m에 #2를 숨김(출발 표시), 충돌 지점 마킹, 천장 카메라 화각에 두 차 ArUco 보임.
3. #2 (A2): `ros2 launch my_algo car2.launch.py vesc_config:=$CAR2_VESC_CONFIG` (세션 2 판정 속도).
4. 회차 루프: 2-3과 같고, 회차마다 #2를 출발 표시에 두고 `/popout/reset`. 천장 카메라 파일은 `<run_dir>/camera/ceiling.mp4`로 복사.
5. 시간 동기: 세션 시작 때 #2에서 `chronyc tracking` (System time < 1 ms), 2-7의 트리거로 `latency_s` 양수·수 ms 확인.
6. 충돌 회차: 차량 점검 후 계속. 충돌은 폐기가 아니라 결과다.

---

## 부록 A. 자주 나는 문제

| 증상 | 확인 | 조치 |
|---|---|---|
| `Package 'my_algo' not found` | `echo $ROS_DISTRO` | `source .../env_ego.sh` |
| 토픽이 안 보임 (다른 기기) | `echo $ROS_DOMAIN_ID` (세 기기 같아야) | `EXP_ROS_DOMAIN_ID` 맞추고 env 다시 source |
| TF 끊김 | `ros2 run tf2_ros tf2_echo odom base_link`, `ros2 topic info -v /tf` | E 터미널, odom→base_link 발행자 하나 |
| `발행자 점검 위반` | `python3 $S/echo_once.py /exp/preflight --transient-local` | 위반 노드 끄기 (#2의 불필요 노드 등) |
| AUTO 직후 `joy_timeout` | `ros2 topic hz /joy` | 조이스틱 연결, autorepeat 20 Hz |
| IMU `calibrated: false` | `python3 $S/echo_once.py /imu/status` | 차 정지, `accel_unit` |
| 트리거 지연 음수 | #2 `chronyc tracking` | RUNBOOK 0.3 chrony |
| `stale_perception` | `ros2 topic echo /exp/grid_debug` (ms) | 부하 확인 |
| Foxy 명령 차이 | — | `view_frames.py`, `topic echo`에 `--once`/`--field` 없음 → `echo_once.py`, `param dump --print` |

## 부록 B. 세션 종료 백업 (노트북에서)

```bash
rsync -av --progress <user>@<에고 IP>:~/exp_logs/ ~/exp_logs/
rsync -av <user>@<#2 IP>:~/exp_logs/popout/ ~/exp_logs/popout/
du -sh ~/exp_logs/<날짜>        # 회차당 약 0.6 GB 추정 (LOGGING 6절)
```
분석 환경(노트북): `python3 -m venv ~/venv_exp && source ~/venv_exp/bin/activate && pip install -r analysis/requirements.txt`
(rosbags 0.9.23 고정, Python 3.8–3.11). 레포 루트에서 `PYTHONPATH=. python3 analysis/...`.
