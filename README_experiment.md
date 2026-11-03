# 관측 공백 속도 계획 실험 (exp/occlusion-speed)

기존 주행 스택(`lidar_drive.launch.py`)은 바꾸지 않는다. 이 문서는 실험용으로 추가한 것만 다룬다.
진행 상태: **TF/IMU/점검 단계**. 인지, phantom, v_cap, 로깅, 분석은 차량 측정값을 받은 뒤 진행한다.

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

## 1. 라이다 높이와 roll/pitch 측정 [EXP0-MEASURE]

평평한 바닥, 차 주변 0.4–4 m에 물체가 없는 곳에서:

```bash
python3 $(ros2 pkg prefix my_algo)/share/my_algo/scripts/measure_lidar_height.py --topic <점군 토픽> --frames 10
```

- `height_m` = base_link(바닥 z=0) 기준 livox_frame z
- `roll`, `pitch` = livox_frame 장착 각도 (tf2 static_transform_publisher 규약, Rz·Ry·Rx)
- `보정 후 바닥 z 평균`이 `-height_m`과 같고 std가 1 cm 안쪽이어야 정상
- yaw는 바닥으로 알 수 없다. RViz 벽 평행 검사(아래 3절)로 확인한다.

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

## 3. TF 검증 (첨부 tf_static.launch.py 반영 후)

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
| [EXP0-MEASURE] | scripts/measure_lidar_height.py | livox_frame z, roll, pitch |
