# 차량 세션 1 체크리스트

목적: 토픽 이름 확정, drive 모드 리팩터링 불변 검증(rosbag), 장착값 측정.
예상 시간 약 70분. 결과물은 모두 `~/session1/`에 모은다. 마지막 표의 파일을 그대로 보내면 된다.

시작 전에 경로 두 개를 자기 환경에 맞게 정한다:

```bash
export WS=~/f1tenth_ws                       # colcon 워크스페이스
export REPO=<my_algo 레포 경로>               # 예: $WS/src/my_algo 또는 $WS/f1tenth_gym_ros/my_algo
mkdir -p ~/session1
```

---

## (a) 코드 받기, 빌드 (5분)

```bash
cd $REPO
git fetch origin --tags
git checkout exp/occlusion-speed && git pull
git log --oneline -1 | tee ~/session1/a_git.txt
cd $WS && colcon build --packages-select my_algo 2>&1 | tail -n 5 | tee -a ~/session1/a_git.txt
source install/setup.bash
```

정상: 마지막 줄에 `Finished <<< my_algo`. 실패하면 그 출력 전체를 보낸다.

## (b) 차량 점검 (5분)

평소 주행처럼 Livox, VESC(+vesc_to_odom), 카메라 드라이버를 띄우고 **차는 정지**.
자율주행 launch는 띄우지 않는다.

```bash
cd ~/session1 && bash $(ros2 pkg prefix my_algo)/share/my_algo/scripts/check_vehicle.sh
```

저장 파일: `~/session1/check_vehicle_<host>_<시각>.log` (자동 생성).
이 로그로 토픽 이름, IMU 단위, 점군 필드(deskew 가능 여부), TF, `speed_to_erpm_gain`을 확정한다.

## (b2) 두 차 같은 도메인 발행자 확인 (10분, 안전)

#2가 에고 토픽에 섞이지 않는지 본다. 두 Jetson의 `ROS_DOMAIN_ID`를 같게 하고:

- **#2 Jetson**: `ros2 launch my_algo car2.launch.py vesc_config:=<#2 VESC YAML>` 만 띄운다
  (Livox 드라이버, 정적 TF, joy 등 다른 것은 띄우지 않는다). #2는 바퀴를 띄워 둔다.
- **에고 Jetson**: (b)와 같이 드라이버를 띄운 상태에서

```bash
cd ~/session1
for t in /livox/lidar /livox/imu /commands/motor/speed /commands/servo/position /tf /tf_static \
         /car2/commands/motor/speed /car2/odom /trigger; do
  echo "===== $t"; ros2 topic info -v "$t"
done 2>&1 | tee b2_domain_check.txt
ros2 node list | tee -a b2_domain_check.txt
# 조이스틱을 켜고 손대지 않은 상태에서 (데드맨이 쓰는 /joy 재발행 주기)
timeout -s INT 6 ros2 run joy joy_node --ros-args -p autorepeat_rate:=20.0 &
sleep 2; timeout -s INT 4 ros2 topic hz /joy 2>&1 | tail -n 3 | tee b2_joy_hz.txt
```

- 정상: `/livox/*`, `/commands/motor/speed`, `/tf_static`의 Publisher 목록에 `/car2/...` 노드가 **없다**.
  `/car2/commands/motor/speed`의 Subscriber에 #2 VESC 드라이버만 있다. `/tf`의 Publisher에 #2 노드가 없다.
- `b2_joy_hz.txt`: 약 20 Hz. 0이거나 안 나오면 데드맨이 AUTO 직후 정지시키므로 보고.
- 다른 점이 보이면 (d) 이후로 넘어가지 말고 `b2_domain_check.txt`를 보낸다.
- 확인 후 #2 launch는 끈다 (나머지 단계는 에고만).

## (c) drive 모드 주행 bag 기록 (10분)

트랙에서 **기존 주행 모드** 그대로 30초 이상 정상 주행한다 (`ros2 launch my_algo lidar_drive.launch.py`).
다른 터미널에서 주행 시작 **전에** 기록을 켠다:

```bash
cd ~/session1
ros2 bag record -o drive_bag \
  /livox/lidar /vesc/odom /autonomous_mode \
  /scan /reactive/lidar_command /commands/motor/speed /commands/servo/position /aeb/active
# 30초 이상 주행 후 Ctrl+C
ros2 bag info drive_bag | tee c_bag_info.txt
```

- 토픽 이름이 (b) 로그와 다르면 실제 이름으로 바꿔서 기록한다 (예: 점군이 다른 이름이면 그 이름).
- 정상: `c_bag_info.txt`에서 `/livox/lidar` 약 10 Hz × 기록 시간, `/vesc/odom` 메시지 수가 0이 아님.
- 주행 중 AEB가 한 번쯤 걸리는 구간이 있으면 검증 범위가 넓어져 좋다 (안전한 범위에서).

## (d) 리팩터링 불변 검증: rosbag 재생 (15분)

**VESC 드라이버를 반드시 끈다** (재생 명령이 모터로 가지 않게). Livox 드라이버도 끈다 (bag의 점군과 섞이지 않게).
`replay_drive.sh`는 VESC 노드가 떠 있으면 스스로 멈춘다.

```bash
# 점군/odom 토픽 이름이 기본값(/livox/lidar, /vesc/odom)과 다르면:
# export CLOUD_TOPIC=<실제 점군 토픽> ODOM_TOPIC=<실제 odom 토픽>

# 1) 기준선 (리팩터링 전 코드)
cd $REPO && git checkout baseline-pre-refactor
cd $WS && colcon build --packages-select my_algo && source install/setup.bash
cd ~/session1 && bash $(ros2 pkg prefix my_algo)/share/my_algo/scripts/replay_drive.sh drive_bag baseline_cmd.csv 0.5

# 2) 현재 브랜치
cd $REPO && git checkout exp/occlusion-speed
cd $WS && colcon build --packages-select my_algo && source install/setup.bash
cd ~/session1 && bash $(ros2 pkg prefix my_algo)/share/my_algo/scripts/replay_drive.sh drive_bag after_cmd.csv 0.5

# 3) 비교
python3 $(ros2 pkg prefix my_algo)/share/my_algo/scripts/verify_refactor.py compare \
  baseline_cmd.csv after_cmd.csv | tee d_verify.txt
```

- 정상: `d_verify.txt` 마지막 줄 `결과: 일치`.
- `미매칭 비율` 경고가 나오면 재생 배속을 0.25로 낮춰 1)~3)을 다시 한다.
- 차이가 나오면 여기서 멈추고 `d_verify.txt`, `*_launch.log`, `*_params_*.yaml`을 전부 보낸다.

## (e) IMU 정지 캘리브레이션 (10분, 예열 포함)

Livox 드라이버를 다시 켠다. **라이다 켜고 5분 기다린다** (예열). 평평한 바닥, 차 완전 정지.

```bash
S=$(ros2 pkg prefix my_algo)/share/my_algo/scripts
cd ~/session1
python3 $S/imu_static_calib.py 10 --topic /livox/imu | tee e_imu_calib_0deg.txt
# 차를 제자리에서 180° 돌린다 (같은 바닥 위치)
python3 $S/imu_static_calib.py 10 --topic /livox/imu | tee e_imu_calib_180deg.txt
```

- IMU 토픽이 다르면 `--topic`을 (b) 로그의 이름으로.
- 정상: `중력 크기 검산 ≈ 1.00 g`, `실제 주기 ≈ 200 Hz`, 자이로 흔들림 경고 없음.

## (f) 라이다 원점 높이 (5분)

같은 자리, 차 주변 0.4–4 m 바닥을 비운다.

```bash
python3 $S/measure_lidar_height.py --topic /livox/lidar --frames 10 | tee f_lidar_height.txt
```

- 정상: `보정 후 바닥 z 평균`이 `-height_m`과 비슷하고 std ≤ 0.01 m, inlier 경고 없음.

## (g) 로고 방향, 바퀴 지름 (5분)

`g_manual.txt`에 직접 적는다:

```text
Mid-360 로고(케이블 반대쪽 아님, 상단 로고) 방향: 전방 / 후방 / 좌측 / 우측
바퀴 지름 [m]: (자로 직접 재거나, 타이어에 표시하고 5바퀴 굴린 거리 / (5 × 3.1416))
측정 방법:
```

---

## 보낼 것

| 단계 | 파일 | 확인할 내용 |
|---|---|---|
| a | `a_git.txt` | 커밋 해시, 빌드 성공 |
| b | `check_vehicle_*.log` | 토픽, IMU 단위, 점군 필드, TF, speed_to_erpm_gain |
| b2 | `b2_domain_check.txt`, `b2_joy_hz.txt` | #2 격리, 발행자 점검 허용 노드 이름, /joy 주기 |
| c | `c_bag_info.txt` | 기록 토픽과 메시지 수 (bag 자체는 보내지 않아도 됨) |
| d | `d_verify.txt` (+ 차이 있으면 `*_launch.log`, `*_params_*.yaml`) | drive 모드 불변 |
| e | `e_imu_calib_0deg.txt`, `e_imu_calib_180deg.txt` | roll, pitch, 자이로 바이어스, 단위 |
| f | `f_lidar_height.txt` | ORIGIN_H |
| g | `g_manual.txt` | LIDAR_YAW, wheel_diameter |

한 번에 묶기: `cd ~ && tar czf session1.tgz --exclude='session1/drive_bag' session1/`
