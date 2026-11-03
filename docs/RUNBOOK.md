# 실행 절차 (RUNBOOK)

실험일에 이 순서대로 한다. 각 명령 아래 **정상**은 그 명령이 잘 됐을 때 보여야 하는 것이다.
차량은 **ROS2 Foxy (Ubuntu 20.04, Python 3.8)**. 세션별로 차 앞에서 그대로 칠 명령은 [SESSIONS.md](SESSIONS.md).
"TBD(세션 1)"는 차량 세션 1 결과로 확정할 값이다 ([OPEN_ITEMS.md](OPEN_ITEMS.md) 1·4절).

공통 변수 (모든 터미널):

```bash
source ~/f1tenth_ws/src/my_algo/scripts/env_ego.sh   # #2는 env_car2.sh. TBD(세션 1): my_algo 소스 경로
```
`env_ego.sh`: Foxy → `~/vesc_ws` → `~/f1tenth_ws` source, `ROS_DOMAIN_ID`(기본 37, 세 기기 같게), `$S`(스크립트 경로),
`$MY_ALGO_REPO`(meta.json git hash 기록용).

---

## 0. 실험일 사전 준비 (하루 1회, 약 20분)

### 0.1 빌드와 커밋 확인
```bash
cd $MY_ALGO_REPO && git checkout exp/occlusion-speed && git pull && git status --short
cd ~/f1tenth_ws && colcon build --symlink-install --packages-select my_algo && source install/setup.bash
```
정상: `git status --short` 출력 없음 (dirty면 meta.json에 `git_dirty: true`가 남는다), 빌드 `Finished`.

### 0.2 배터리
- 에고·#2 배터리 전압 기록. 기준 이하(TBD(세션 1): 셀 수·기준 전압)면 교체. 회차 시작 전압은 `meta.json`에 자동 기록.

### 0.3 시간 동기 (chrony, 두 Jetson)
에고를 시간 기준으로 쓴다 (인터넷이 없어도 동작).

에고 `/etc/chrony/chrony.conf`에 추가:
```
allow <서브넷, 예: 192.168.0.0/24>
local stratum 8
```
#2 `/etc/chrony/chrony.conf`에 추가 (다른 `pool`/`server` 줄은 주석 처리):
```
server <에고 IP> iburst prefer
makestep 0.1 3
```
양쪽 `sudo systemctl restart chrony` 후 **#2에서**:
```bash
chronyc sources -v     # 정상: 에고 IP 줄 맨 앞이 '^*'
chronyc tracking       # 정상: "System time"이 1 ms 미만 (예: 0.000213 seconds fast)
```
동기 오차 확인 (ROS 수준): 3절 #2를 띄운 상태에서 에고가 정지 상태로 `/trigger`를 보내면 #2 로그의
`트리거 수신: 지연 +0.00xx s`가 네트워크 지연만큼(수 ms) 나와야 한다. **음수**면 시계가 어긋난 것이다.
```bash
# 에고에서: 현재 시각을 stamp로 넣은 /trigger 1회 발행
python3 -c "import rclpy;from std_msgs.msg import Header;rclpy.init();n=rclpy.create_node('t');p=n.create_publisher(Header,'/trigger',10);import time;time.sleep(1);h=Header();h.stamp=n.get_clock().now().to_msg();h.frame_id='sync_check';p.publish(h);time.sleep(0.5)"
ros2 topic pub --once /popout/reset std_msgs/msg/Bool "{data: true}"   # 확인 후 #2를 대기 상태로
```
결과(`chronyc tracking`의 System time, 트리거 지연)를 그날 `session.json`에 적는다 (`time_sync` 키, 수동).

### 0.4 라이다 예열과 IMU 캘리브레이션
1. Livox 드라이버를 켜고 **5분** 기다린다.
2. 평평한 바닥에서 (실험일마다 자이로 바이어스):
   ```bash
   python3 $S/imu_static_calib.py 10 --topic /livox/imu      # TBD(세션 1) 토픽
   # 차를 제자리에서 180° 돌려 한 번 더
   ```
   정상: `중력 크기 검산 ≈ 1.00 g`, `실제 주기 ≈ 200 Hz`, 흔들림 경고 없음.
3. `gyro bias` 두 회 평균을 `config/imu.yaml` `gyro_bias`에 넣고 다시 빌드. 결과 출력은 `session.json`의 `imu_calib`에 붙인다.
   roll/pitch가 `tf_static.launch.py` 값과 0.3° 이상 다르면 장착이 바뀐 것이므로 실험 전에 보고.

### 0.5 회차 순서
```bash
python3 $S/randomize_runs.py --seed <날짜 숫자>        # 정상: 회차 목록과 "총 N회", session.json 기록
```

---

## 1. 에고 Jetson

### T1 센서 드라이버 + 정적 TF + odom (SESSIONS의 A, B, E 터미널)
```bash
ros2 launch my_algo livox_mid360.launch.py            # A: rviz_MID360_launch.py와 같은 설정(PointCloud2), RViz 없음
ros2 launch vesc_driver vesc_driver_node.launch.py    # B: vesc_driver만 (vesc_to_odom 없음)
ros2 launch my_algo tf.launch.py                      # E: 정적 TF
ros2 launch my_algo vesc_odom.launch.py               # E: odom + odom→base_link TF (세션 1에서 기존 odom 발행자가 없을 때만)
```
정상: `ros2 run tf2_tools view_frames.py`(Foxy는 `.py`)의 `frames.pdf`가 `odom → base_link → livox_frame → livox_imu` 한 줄,
`ros2 topic info -v /tf`의 Publisher가 하나 (vesc_to_odom 또는 EKF).

### T2 bringup (회차마다 새로 띄운다)
```bash
ros2 launch my_algo bringup.launch.py mode:=exp method:=P scenario:=S1 d_e:=1.5 run_id:=3
ros2 launch my_algo bringup.launch.py mode:=exp method:=B2 scenario:=S3 run_id:=1          # 돌출 없는 시나리오
ros2 launch my_algo bringup.launch.py mode:=exp method:=P scenario:=S3 run_id:=1 w_min:=0.30  # 민감도: [EXP-TUNE]만
```
정상 로그 (순서대로):
- `[bringup] exp method=P scenario=S1 d_e=1.5 run_id=3 deskew=False run_dir=...`
- `phantom 전략 P, 앞 끝 0.41 m`, `method=P v_ref=2.0 ... policy=hold V_MIN=0.867` (경고 없음)
- `exp AEB: lidar_to_front=0.28 half_width=0.135 ttc=0.55`
- `exp 조향: WALL_FOLLOW 고정(좌측 벽), lookahead 4.4 m`
- 약 3초 뒤 `발행자 점검 통과` (위반이 나오면 문제 해결 5번)
- `IMU 정지 보정 완료: |g_ref|=9.8x` (차가 정지해 있어야 한다)

**회차 중 `ros2 param set`으로 method를 바꾸지 않는다.** 방법·회차가 바뀌면 T2를 Ctrl+C 후 다시 띄운다 (격자 누적·히스테리시스 상태를 넘기지 않기 위해).

### T3 기록
```bash
bash $S/record_run.sh exp P S1 1.5 3
bash $S/record_run.sh exp B2 S3 na 1
```
정상: `회차 폴더: ~/exp_logs/<날짜>/S1_P_de1.5_run03`, `기록 중 (Ctrl+C로 종료)`.
거부되면: `회차 시작 거부: 발행자 점검 실패` → 문제 해결 5번. `이미 기록된 회차` → run_id 확인.
토픽 이름이 기본값과 다르면 `CLOUD_TOPIC= ODOM_TOPIC= RAW_IMU_TOPIC= VESC_STATE_TOPIC= CAMERA_TOPIC=`를 앞에 붙인다 (TBD(세션 1)).

### T4 모니터링
```bash
ros2 topic hz /livox/lidar          # 정상: 약 10 Hz (기록 전과 같아야 함)
ros2 topic hz /exp/grid             # 약 10 Hz
ros2 topic echo /exp/grid_debug     # ms가 100 미만 (넘으면 프레임을 놓친다)
ros2 topic echo /exp/stop_reason    # 대기 중 '' 또는 'preflight_failed'면 안 됨, 주행 중 ''
ros2 topic echo /exp/min_speed_flag
```

---

## 2. 회차 진행 (체크리스트)

시작 전:
- [ ] 배터리 전압 기준 이상 (에고, #2)
- [ ] 트랙 마킹 확인: 시작선, 가속 구간 끝, 박스 P1–P3 위치·틈 폭(시나리오), #2 출발 표시, 바닥 0.5 m 눈금
- [ ] 폼 범퍼(#2 측면), 트랙 끝 폼 방호벽 위치
- [ ] **물리 전원 차단 위치**(TBD(세션 1): 에고 스위치 위치)를 담당자가 손 닿는 곳에서 확인
- [ ] 천장 카메라 녹화 시작 (60 fps), 두 차 ArUco 마커 부착 상태
- [ ] T2 bringup 정상 로그, `발행자 점검 통과`
- [ ] 에고를 시작선에 정지, `ros2 topic pub --once /imu/recalibrate std_msgs/msg/Bool "{data: true}"` → `보정 완료`
- [ ] (S1) #2를 출발 표시에 두고 `ros2 topic pub --once /popout/reset std_msgs/msg/Bool "{data: true}"`
- [ ] T3 기록 시작

주행:
1. 조이스틱을 켠다 → 시작은 항상 **MANUAL**이다 (joy_controller가 `/autonomous_mode=false`를 계속 보냄).
2. **LB**로 AUTO 진입 → 차가 출발한다.
3. 회차가 끝나면 (정지선, AEB, 또는 수동) **B** 또는 **LB**로 MANUAL.

종료·리셋:
- [ ] T3 Ctrl+C → `discard=... aeb=... malfunction=...` 출력 확인. 폐기나 오작동이면 회차 순서표에 표시하고 같은 조건을 다시 한다.
- [ ] 수동 폐기(배터리, 트랙 어긋남 등): `python3 $S/run_meta.py discard <run_dir> <사유>`
- [ ] T2 Ctrl+C (다음 회차는 새로 띄움)
- [ ] 천장 카메라 파일을 `<run_dir>/camera/`로 복사 (이름: `ceiling.mp4`)
- [ ] 차량·#2를 시작 위치로

## 3. 돌출 차량 #2 Jetson

**#2에서는 이 launch 하나만 띄운다.** Livox 드라이버, 정적 TF, joy, bringup은 띄우지 않는다.

### T1
```bash
ros2 launch my_algo car2.launch.py vesc_config:=<#2 VESC YAML>   # TBD(세션 1): VESC 패키지 이름이 다르면 vesc_driver_pkg:= 등
```
정상: `#2 대기: v=0.8 m/s (... ERPM) ... motor_topic=/car2/commands/motor/speed`

### T2 확인
```bash
ros2 topic echo /popout/status     # 회차마다 trigger 수신 지연, reach_lane_s, speed_mean/std, stop_reason
```
에고 T4에서 `ros2 topic info -v /commands/motor/speed`의 Publisher에 `/car2/...`가 없어야 한다.

## 4. 노트북

```bash
rviz2 -d $(ros2 pkg prefix my_algo)/share/my_algo/config/exp.rviz
```
노트북은 RViz와 모니터링만 한다 (다른 노드를 띄우면 발행자 점검에 걸릴 수 있다).
처음 한 번 TF 검증 (부록 C)을 한다.

## 5. 비상 정지 (우선순위 순)

1. **조이스틱 B 버튼** — 즉시 0 명령 + MANUAL. 속도 계층·AEB보다 우선. 기본 수단.
2. **조이스틱 끊김** — `/joy`가 0.5 s 끊기면 자동 정지, 그 회차는 폐기(`joy_timeout`).
3. **소프트웨어 정지선** — AUTO 후 주행 거리 > `stop_distance`(시나리오)면 0 명령.
4. **물리 전원 차단** — 위 셋이 안 될 때.
5. (보조) 노트북: `ros2 topic pub --once /autonomous_mode std_msgs/msg/Bool "{data: false}"` — mux가 명령을 멈춘다.

## 6. 문제 해결

| 증상 | 확인 명령 | 조치 |
|---|---|---|
| TF 끊김 (`TF 없음` 경고, 격자 안 나옴) | `ros2 run tf2_ros tf2_echo odom base_link`, `ros2 topic info -v /tf` | T1 `tf.launch.py`, vesc_to_odom `publish_tf: true`. odom→base_link 발행자가 둘이면 EKF·vesc_to_odom 중 하나 끄기 |
| 토픽 없음 | `ros2 topic list -t` | 드라이버 실행 여부, bringup 인자 `cloud_topic:=` 등 (TBD(세션 1) 이름) |
| IMU 단위 (`calibrated` 계속 false, \|g_ref\| 경고) | `ros2 topic echo /imu/status`, `check_vehicle.sh` IMU 통계 | `config/imu.yaml` `accel_unit` (정지 \|a\| ≈ 1 → g) |
| 시간 동기 (트리거 지연 음수·큼) | `chronyc tracking` (#2) | 0.3절 다시. 동기 전 S1 회차 금지 |
| 발행자 점검 실패 | `ros2 topic echo /exp/preflight`, `ros2 topic info -v <토픽>` | 위반 노드를 끈다. 허용 이름이 실제와 다르면 `config/preflight.yaml` 수정(세션 1 후 확정) |
| AUTO 직후 정지 `joy_timeout` | `ros2 topic hz /joy` | joy_node `autorepeat_rate` 20 Hz, 패드 연결 |
| `stale_speed` / `stale_perception` | `ros2 topic hz /exp/phantom_d`, `/exp/grid_debug`의 ms | 격자 처리 지연 → 다른 부하 확인, `max_range` 검토(방법 간 동일 유지) |
| `preflight_failed`로 출발 안 함 | 위 발행자 점검 | |
| Foxy 명령 차이 | — | `view_frames.py`, `ros2 topic echo`에 `--once`·`--field` 없음 → `python3 $S/echo_once.py <토픽> [--field a.b]`, `ros2 param dump <노드> --print`, `ros2 bag play`에 `--clock` 없음 → `python3 $S/bag_player.py` |
| bag 저장소 | `ros2 bag info <bag>` | Foxy 기본 sqlite3 (`*.db3`). 분석은 rosbags 0.9.23 |
| 기록 중 점군 주기 저하 | T4 `ros2 topic hz /livox/lidar` 기록 전후 비교 | 카메라 해상도·fps 낮추기, 기록 위치 디스크 확인 |

---

## 부록 A. 노드 개별 실행 (디버깅)

bringup과 같은 파라미터로 하나씩 띄운다 (`P=$(ros2 pkg prefix my_algo)/share/my_algo/config`):

```bash
ros2 run my_algo occupancy_grid_node --ros-args --params-file $P/common.yaml
ros2 run my_algo phantom_node --ros-args --params-file $P/common.yaml --params-file $P/method_P.yaml \
  -p vehicle_geometry_file:=$P/vehicle_geometry.yaml
ros2 run my_algo speed_layer_node --ros-args --params-file $P/common.yaml --params-file $P/method_P.yaml
ros2 run my_algo exp_aeb_node --ros-args --params-file $P/common.yaml -p vehicle_geometry_file:=$P/vehicle_geometry.yaml
ros2 run my_algo exp_steering_node --ros-args --params-file $P/common.yaml \
  -r /reactive/lidar_command:=/exp/steering_command -p vehicle_geometry_file:=$P/vehicle_geometry.yaml
ros2 run my_algo publisher_guard_node --ros-args --params-file $P/preflight.yaml -p mode:=warn
ros2 launch my_algo imu.launch.py input_topic:=/livox/imu
```
명령 노드(`exp_drive_mux`)는 개별로 띄울 때 바퀴를 띄워 둔다.

## 부록 B. IMU 확인

```bash
ros2 launch my_algo imu.launch.py input_topic:=/livox/imu accel_unit:=g   # 차 정지 상태에서 시작
python3 $S/echo_once.py /imu --field linear_acceleration   # 정상: 정지 시 크기 ≈ 9.8 (z ≈ 9.8)
python3 $S/echo_once.py /imu/status                         # calibrated: true, g_ref_norm ≈ 9.81, tf_ok: true
ros2 topic echo /imu/linear_accel_base                      # (Foxy echo에는 --field 없음) vector 값을 본다
```
축 방향: 손으로 차를 **앞으로** 짧게 밀었다 멈추면 x가 먼저 양수, 멈출 때 음수. **왼쪽**으로 밀면 y 양수.
정지 시 x, y, z 모두 ±0.05 m/s² 안. 부호가 반대면 TF가 잘못된 것이다 (노드에서 부호를 뒤집지 말 것).

## 부록 C. TF·장착 검증 (RViz, fixed frame = base_link)

- [ ] `ros2 run tf2_tools view_frames.py`: `odom → base_link → livox_frame → livox_imu` 한 줄, odom→base_link 발행자 하나
- [ ] 바닥 점이 z ≈ 0에서 평평 (옆에서 보아 기울지 않음)
- [ ] 직선 벽이 x축과 평행 (yaw 확인)
- [ ] 앞 끝에서 1 m, 2 m 앞에 둔 박스가 base_link x = 1.41, 2.41 m에 표시
- [ ] 카메라 장착부가 Mid-360 시야(수직 −7°~52°)를 가리지 않음 (점군에 차체 쪽 빈 부채꼴이 없는지)

장착값 측정: `python3 $S/measure_lidar_height.py --topic /livox/lidar --frames 10` (원점 높이),
`imu_static_calib.py` 0°/180° (roll, pitch). 값 교체 위치는 [OPEN_ITEMS.md](OPEN_ITEMS.md).
