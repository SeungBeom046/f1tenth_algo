# archive/

빌드·실행 대상이 아닌 옛 코드 보관소. 원래 경로 구조를 유지한다 (`archive/<원래 경로>`).
setup.py와 launch 탐색 경로(`launch/*.launch.py`)에 포함되지 않으므로 colcon 빌드에 들어가지 않는다.

| 파일 | 원래 위치 | 이동일 | 이동 사유 |
|---|---|---|---|
| my_algo/imu_motion_logger.py | my_algo/ | 2026-11-03 | IMU 단독 적분 속도 기록. 주행 launch에서 쓰이지 않고 실험 IMU 경로(`imu_republisher`)와 겹침. 기본 `acceleration_scale=1.0`이라 드라이버가 g 단위면 정지 보정이 끝나지 않음 |
| my_algo/imu_integration.py | my_algo/ | 2026-11-03 | 위 노드 전용 계산 모듈 |
| launch/imu_motion.launch.py | launch/ | 2026-11-03 | 위 노드 전용 launch |
| test/test_imu_integration.py | test/ | 2026-11-03 | 위 모듈 테스트 |

되살리려면 원래 위치로 `git mv` 하고 setup.py에
`'imu_motion_logger = my_algo.imu_motion_logger:main'`을 다시 추가한다.

## 옛 사용법 (원래 README.md 내용)

### 별도 LiDAR IMU 속도·거리 기록 (실험용)

기존 주행/AEB/`/vesc/odom` 및 기존 metrics logger는 변경하지 않는다.
`imu_motion_logger`는 `/livox/imu`와 `/autonomous_mode`를 구독하고,
`/imu_motion/status`만 발행한다. 센서 모델과 실제 토픽은 차량에서 확인해야 한다.

```bash
colcon build --packages-select my_algo
source install/setup.bash
# 드라이버를 먼저 켜고 차를 지면에 완전히 정지시킨 상태에서 실행
ros2 launch my_algo imu_motion.launch.py imu_topic:=/livox/imu logs_dir:=/tmp/imu_logs
# 'IMU calibration ready' 메시지 확인 후, 다른 터미널에서 기존 주행 launch 실행
ros2 launch my_algo lidar_drive.launch.py
```

처음 정지 상태의 유효한 200 sample로 중력 벡터와 자이로 bias를 보정한다.
가속도 단위는 m/s², 각속도는 rad/s여야 한다. 정지 가속도 크기는 약 9.81이어야
한다. 드라이버가 g 단위라면 별도 실행 시 `-p acceleration_scale:=9.80665`를
지정한다. 드라이버가 이미 SI 단위를 내보내면 기본값 1.0을 유지한다.
차체의 기울기는 시작 중력 벡터로 처리하며, 자세 출력이 없는 IMU도 자이로를
적분해 회전을 보정한다. 각 trial 시작 시 실제 차량이 정지해 있어야 한다.
등속 이동과 정지는 IMU만으로 구별할 수 없으므로 자동 영속도 판정은 하지 않는다.

AUTO 시작~해제를 한 trial로 기록한다. 노드 Ctrl+C, 또는 AUTO heartbeat가
1초 이상 끊겨도 종료 기록을 저장한다. AEB 정지나 RPM 0만으로 trial을 끝내지는
않는다(AUTO 상태가 유지되기 때문). SIGKILL/전원 차단은 종료 저장을 보장하지 않는다.

- `imu_<session>_<trial>.csv`: 센서 timestamp, 경과시간, 순간 속도, 누적 이동거리.
  실행 중 0.2초마다 flush한다.
- `imu_trials.csv`: 주행시간, 총 이동거리, 평균/최대 속도, sample 수,
  누락·불량 sample 수, 상태와 종료 사유. 주행 종료 시 자동 추가한다.
- `incomplete`: 보정 전 출발, IMU 미수신/끊김, timestamp 이상 등으로 불완전한 기록.
  유효 측정이 없으면 속도·거리 칸을 비워 실제 정지 기록과 구별한다.
- `imu_only_drifting`: IMU 단독 적분 추정값. 정확도 보증을 뜻하지 않는다.

정지 보정 재실행은 AUTO 해제 후 차를 완전히 멈추고 다음을 발행한다.

```bash
ros2 topic pub --once /imu_motion/recalibrate std_msgs/msg/Bool '{data: true}'
```

IMU 가속도 적분은 작은 bias와 자세 오차가 속도·거리 오차로 계속 누적된다.
따라서 장시간 실제 이동거리 계측이나 제어 피드백에는 LiDAR scan matching /
LiDAR-inertial odometry 같은 외부 위치 보정이 필요하다. 이 노드는 IMU 단독
결과를 기존 VESC 기반 로그와 비교하기 위한 용도다. 현재 환경에서는 실차 ROS2
실행을 검증하지 못했으므로 IMU 토픽·단위·정지 보정 및 실제 거리 비교가 필요하다.
