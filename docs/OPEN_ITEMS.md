# 미해결 항목

차량 세션 결과가 들어오면 갱신한다. 태그 전체 목록은 [EXP_TAGS.md](EXP_TAGS.md).

## 1. [EXP0-MEASURE] 가정값

| 항목 | 현재 가정값 | 측정 방법 | 바꿀 파일 · 키 |
|---|---|---|---|
| 라이다 원점 높이 ORIGIN_H | 0.03 (z = 0.24) | `measure_lidar_height.py` (세션 1 f) | `launch/tf_static.launch.py` `ORIGIN_H` |
| 라이다 roll, pitch | 0, 0 | `imu_static_calib.py` 0°/180° 평균 (세션 1 e), 바닥 평면과 교차 확인 | `launch/tf_static.launch.py` `LIDAR_ROLL`, `LIDAR_PITCH` |
| 라이다 yaw | 0 | 로고 방향(세션 1 g) + RViz 벽 평행 검사 | `launch/tf_static.launch.py` `LIDAR_YAW` |
| 자이로 바이어스 | 0, 0, 0 | `imu_static_calib.py` (실험일마다) | `config/imu.yaml` `gyro_bias` |
| IMU 가속도 단위 | g | `check_vehicle.sh` IMU 정지 통계 | `config/imu.yaml` `accel_unit` |
| 바퀴 지름 | 미측정 | 자 또는 5바퀴 굴림 (세션 1 g) | `config/vehicle_geometry.yaml` `wheel_diameter` |
| a_b | 3.0 m/s² | Exp0: 2.0 m/s 급제동 10회, `/imu/linear_accel_base` x | `config/common.yaml` `a_b` |
| t_r | 0.15 s | Exp0: 명령 0 발행 시각 → 감속 시작(IMU) 지연 | `config/common.yaml` `t_r` |
| 트리거 지연 | 0.05 s | Exp0: `/trigger` stamp vs #2 `latency_s`, LED+카메라 | `config/common.yaml` `trigger_latency` |
| #2 진입 시간 | 0.2 s | Exp0: #2 출발 → 앞 끝 차선 경계 (천장 카메라) | `config/common.yaml` `popout_enter_time` |
| #2 ERPM 계수, 직진 서보 | 4614, 0.5 | #2에서 odom 스케일 보정과 같은 방법 | `config/popout.yaml` `erpm_gain`, `servo_center` |
| 트랙 좌표 | collision_x 5.0, stop_distance 7.5, accel_end_x 2.5, test_x 2.5–7.0, row_x 4.0–6.3 | 트랙 마킹 후 줄자 | `config/scenario_S*.yaml` |
| 좌측 벽 목표 거리 | 0.355 | 좌측 벽 ~ 차선 왼쪽 경계 거리 실측 → 식은 `common.yaml` 주석 | `config/common.yaml` `target_wall_distance` |
| 점별 시각 필드 (deskew) | `timestamp`, 1e-9, absolute | `check_vehicle.sh` 점군 필드 목록 | `config/common.yaml` `time_field`, `time_scale`, `time_mode` |

## 2. 속도 환산 일관성 (확인 대기)

- 명령용 `ERPM_GAIN` = **4614** (`my_algo/vehicle_utils.py`, drive·exp 공통 유일 출처)
- odom 계산용 `speed_to_erpm_gain` = **미확인** (VESC 드라이버 설정, 레포 밖)
  → 세션 1 `check_vehicle.sh` 로그의 "속도 환산 계수" 절에서 확인한다.
- 두 값이 다르면: 명령 2.0 m/s에 odom은 다른 속도를 보고한다. 실측 기준 손실, v_ref 도달 판정,
  트리거 거리(odom 누적), AEB의 에고 속도가 모두 영향을 받는다.
- **지금은 고치지 않는다.** 두 값과 `V_MIN` 환산값은 회차마다 `meta.json`에 기록된다
  (`erpm_gain`, `vesc_speed_to_erpm_gain`, `v_min`). `run_meta.py`는 값이 다르면 경고를 낸다.

### Exp0 odom 스케일 보정 절차 (계획)
1. 바닥에 5 m 직선 표시. 저속(0.9 m/s)과 2.0 m/s로 각 3회 직진.
2. 줄자 거리 / odom 주행 거리 비율로 실제 속도-ERPM 계수를 구한다.
3. 두 값을 같은 보정값으로 맞춘다 (VESC `speed_to_erpm_gain`, `vehicle_utils.ERPM_GAIN`).

> **경고:** `ERPM_GAIN`을 바꾸면 drive 모드의 속도도 바뀐다 (planner RPM은 그대로인데 m/s 해석이 달라지고,
> `MIN_DRIVING_RPM` 환산 V_MIN도 바뀐다). 승인 후 별도 커밋으로 하고 verify_refactor 기준선을 다시 잡는다.

## 3. 코드 분석 중 발견한 확인 필요 사항

1. **같은 ROS_DOMAIN_ID에서 토픽 충돌.** 에고와 #2가 `/commands/motor/speed`, `/vesc/odom`, `/tf`를 같은 이름으로
   쓰면 #2 명령이 에고 VESC로 들어갈 수 있다. #2는 namespace(`car2`)로 드라이버를 띄우고 `publish_tf: false`.
   `popout_vehicle_node`는 상대 토픽이라 namespace를 따른다. 차량에서 `ros2 topic info -v /commands/motor/speed`로
   발행자가 에고 노드뿐인지 확인할 것.
2. **#2의 0.8 m/s는 에고 최소 속도(0.867 m/s, 4000 ERPM)보다 낮다.** #2 노드는 최소 RPM 규칙을 적용하지 않는다.
   #2가 0.8 m/s에서 떨림 없이 출발·유지하는지 Exp0에서 확인.
3. **realsense 토픽 이름** (`/camera/color/...` vs `/camera/camera/...`): drive launch의 camera_reactive_drive 구독
   이름이 실제와 다르면 YOLO에 영상이 안 들어가고 있을 수 있다. 녹화 토픽(`CAMERA_TOPIC`)도 확인 후 확정.
4. **joy_node autorepeat**: 입력이 없어도 `/joy`가 약 20 Hz로 오는지 `ros2 topic hz /joy`로 확인
   (안 오면 데드맨이 AUTO 직후 정지시킨다). 무선 패드가 범위를 벗어날 때 `/joy`가 끊기는지도 확인.
5. **scan 높이 필터**: `pointcloud_to_laserscan`이 livox_frame z −0.1~0.5를 쓴다. 라이다 높이 0.24 m면
   바닥에서 0.14 m 아래 물체는 AEB·조향에 안 보인다 (drive와 exp 동일, 바꾸지 않음).
6. **점유격자 free 근거**: 높이 대역(0.05–0.30 m) 안 점만 쓰므로 바닥 점은 free를 만들지 않는다.
   빈 바닥은 벽·박스로 가는 광선이 지나갈 때만 free가 된다. 트랙이 벽으로 둘러싸여 있어야 한다.
7. **exp 조향은 좌측 벽만 본다.** 좌측 벽이 일정 구간 안 보이면 직전 조향을 유지한다.
8. **lidar_to_front_edge(0.28)**는 `vehicle_geometry.yaml`의 파생값이다. `tf_static.launch.py`의 `LIDAR_X`를
   바꾸면 이 값도 함께 바꿔야 한다.
9. **AEB는 장애물 상대속도를 쓰지 않는다** (drive와 같은 정의). 분석의 최소 TTC는 상대속도 기반과
   에고 속도 기반(`min_ego_ttc`)을 둘 다 낸다.
