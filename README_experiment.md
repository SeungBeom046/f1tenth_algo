# 관측 공백 속도 계획 실험 (exp/occlusion-speed)

기존 주행 스택(`lidar_drive.launch.py`)은 바꾸지 않는다. 이 문서는 실험용으로 추가한 것만 다룬다.
진행 상태: **2단계(정리·리팩터링·TF·IMU 통합) 완료**. 인지, phantom, v_cap 노드, 로깅, 분석은 3단계.

## 0. 차량 점검 (가장 먼저)

```bash
source /opt/ros/$ROS_DISTRO/setup.bash && source ~/f1tenth_ws/install/setup.bash
# Livox / VESC / (카메라) 드라이버를 평소처럼 띄우고 차는 정지
bash $(ros2 pkg prefix my_algo)/share/my_algo/scripts/check_vehicle.sh
```

생성된 `check_vehicle_<host>_<시각>.log`를 통째로 전달한다. 확인 항목:
토픽 이름과 타입, 주기, IMU 단위(정지 |a| 평균), 점군 필드(점별 타임스탬프 여부),
`/tf` 발행 노드, vesc_to_odom 파라미터, VESC 상태 토픽의 전압 필드, chrony 상태.
읽기 전용이며 아무 명령도 발행하지 않는다.

## 1. 장착 측정 [EXP0-MEASURE]

`launch/tf_static.launch.py`의 가정값(ORIGIN_H, LIDAR_ROLL/PITCH/YAW)을 측정값으로 바꾸는 절차.
모두 평평한 바닥, 라이다 켜고 **5분 이상 예열** 후.

### C1. IMU 정지 캘리브레이션 (roll, pitch, 자이로 바이어스, 단위)

```bash
S=$(ros2 pkg prefix my_algo)/share/my_algo/scripts
python3 $S/imu_static_calib.py 10 --topic <IMU 토픽>   # 차 전방 방향
# 차를 제자리에서 180° 돌려 놓고 한 번 더
python3 $S/imu_static_calib.py 10 --topic <IMU 토픽>
```

- 두 회 roll/pitch 평균 → `LIDAR_ROLL`, `LIDAR_PITCH` (바닥 기울기는 180° 회전으로 상쇄)
- `gyro bias` 평균 → `config/imu.yaml`의 `gyro_bias` (실험일마다)
- `가속도 단위` → `config/imu.yaml`의 `accel_unit` (g면 `g`)

### C2. 라이다 원점 높이 (교차 확인 포함)

```bash
python3 $S/measure_lidar_height.py --topic <점군 토픽> --frames 10
```

- `height_m` = 바닥에서 점군 원점까지 높이 → `ORIGIN_H = height_m − MOUNT_Z(0.21)`
- 여기서 나온 roll/pitch는 바닥 평면 기준이라 C1과 같은 규약(Rz·Ry·Rx)이다.
  C1과 0.3° 이상 다르면 바닥이 기울었거나 IMU-라이다 축이 어긋난 것이므로 보고.
- yaw는 바닥으로 알 수 없다. 로고 방향(B5)과 RViz 벽 평행 검사(3절)로 확인한다.

## 2. IMU (/imu)

```bash
# 차를 정지시킨 상태로 시작 (정지 보정 400 샘플, 200 Hz 기준 2 s)
ros2 launch my_algo imu.launch.py input_topic:=<IMU 토픽> accel_unit:=g
```

| 토픽 | 타입 | 내용 |
|---|---|---|
| `/imu` | sensor_msgs/Imu | m/s², rad/s, 자이로 바이어스 차감, frame_id=livox_imu (축 회전 없음) |
| `/imu/linear_accel_base` | geometry_msgs/Vector3Stamped | 중력(roll/pitch 포함) 제거 후 base_link 축 선형 가속도. **x = 종방향** |
| `/imu/status` | std_msgs/String JSON | 수신율, 보정 여부, \|g_ref\|, TF 확보 여부 |
| `/imu/recalibrate` | std_msgs/Bool | true 발행 시 정지 재보정 (회차 시작 전 정지 상태에서) |

중력 제거 방식: 정지 상태 비력 벡터 g_ref(장착 기울기와 바닥 경사 포함)를 측정하고,
이후 자세 변화는 자이로 적분으로 추적한다. 제동 중 pitch 변화 1°는 0.17 m/s²(a_b 약 6%)
오차가 되므로 이 추적이 필요하다. 회차마다 정지 상태에서 재보정한다. 제어 루프에는 쓰지 않는다.

### 단위 확인
```bash
ros2 topic echo /imu --field linear_acceleration   # 정지 시 |a| ≈ 9.8
ros2 topic echo /imu/status                         # g_ref_norm ≈ 9.81
```
|g_ref|가 9.81에서 0.3 이상 벗어나면 경고가 뜬다. 원본이 g 단위인데 `accel_unit:=mps2`로 두면
정지 보정 자체가 완료되지 않는다 (의도된 동작).

### 축 방향 확인 (base_link 기준 +x 전진)
1. 차를 정지시키고 `imu.launch.py` 실행, `보정 완료` 로그 확인
2. `ros2 topic echo /imu/linear_accel_base --field vector`
3. 손으로 차를 **앞으로** 짧게 밀었다 멈춘다 → x가 먼저 **양수**, 멈출 때 음수
4. 차 **왼쪽**으로 밀면 y 양수 (REP-103: x 전방, y 좌측, z 위)
5. 정지 시 x, y, z 모두 ±0.05 m/s² 안쪽
6. 부호가 반대면 TF(livox_frame/livox_imu 회전)가 잘못된 것이다. 노드에서 부호를 뒤집지 말 것.

## 3. TF

```bash
ros2 launch my_algo tf.launch.py                 # 정적 TF, odom->base_link는 vesc_to_odom
ros2 launch my_algo tf.launch.py use_ekf:=true   # 선택: EKF가 odom->base_link 발행
```

| TF | 발행 | 값 |
|---|---|---|
| base_link → livox_frame | tf_static.launch.py | x 0.13, y −0.005, z 0.24 (=MOUNT_Z 0.21 + ORIGIN_H 0.03 임시), rpy 0 |
| livox_frame → livox_imu | tf_static.launch.py | (0.011, 0.02329, −0.04412), 회전 없음 (Mid-360 매뉴얼) |
| odom → base_link | vesc_to_odom (기본) 또는 EKF | 동적 |

base_link→livox_frame을 발행하는 것은 `tf_static.launch.py`(실측 원본) 하나뿐이다.
측정값이 나오면 그 파일 위쪽 상수(ORIGIN_H, LIDAR_ROLL/PITCH/YAW)를 교체하고 다시 빌드한다.
차량 치수는 `config/vehicle_geometry.yaml` 하나에서 모든 노드가 읽는다.

### odom → base_link (vesc_to_odom)
vesc_to_odom은 이 패키지가 아니라 VESC 쪽 bringup이 띄운다. 그 설정 YAML의
`vesc_to_odom` 항목에서 다음을 맞춘다 (값은 check_vehicle.sh의 param dump로 확인):

```yaml
publish_tf: true
odom_frame: odom
base_frame: base_link
```

EKF(`use_ekf:=true`)를 쓸 때는 반대로 `publish_tf: false`로 둔다. 두 노드가 동시에
odom→base_link를 내면 TF가 튄다. EKF는 휠 odom의 vx, vy와 `/imu` yaw rate만 쓰고 가속도는 쓰지 않는다
(`sudo apt install ros-$ROS_DISTRO-robot-localization` 필요).

### TF 검증

- `ros2 run tf2_tools view_frames` → `odom → base_link → livox_frame → livox_imu` 한 줄로 연결
- `/tf`에 odom→base_link를 내는 노드가 하나뿐인지 `ros2 topic info -v /tf`로 확인
- RViz (Fixed Frame = base_link)
  - [ ] 바닥 점이 z ≈ 0에서 평평 (옆에서 보아 기울지 않음)
  - [ ] 직선 벽이 x축과 평행 (yaw 확인)
  - [ ] 앞범퍼 앞 1 m, 2 m에 둔 박스가 base_link x = 0.41+1, 0.41+2 m에 표시

## 4. 실험 태그 목록

`grep -rn "\[EXP-\|\[EXP0-" .` 로 전체를 본다. 현재 등록된 태그:

| 태그 | 위치 | 항목 |
|---|---|---|
| [EXP0-MEASURE] | config/imu.yaml `gyro_bias` | 실험일마다 재측정 |
| [EXP0-MEASURE] | launch/tf_static.launch.py | ORIGIN_H (임시 0.03), LIDAR_YAW, LIDAR_ROLL, LIDAR_PITCH |
| [EXP0-MEASURE] | config/vehicle_geometry.yaml | wheel_diameter (미측정) |
| [EXP-FIXED] | launch/tf_static.launch.py, config/vehicle_geometry.yaml | 장착 위치·차량 치수 전부 |
