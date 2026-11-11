# 남은 일

## 1. rosbag 재생 회귀 검증 (보류, 2026-11-11)

현재 회귀 검증은 합성 입력(`scripts/verify_refactor_offline.py`)뿐이다. 실제 점군으로 리팩터링 전후를 비교하려면 bag 재생이 필요하다.

**문제**
- Jetson `~/session1/` 녹화에는 `/scan`이 0개다. `/livox/lidar`(PointCloud2)와 `/vesc/odom`만 쓸 수 있다.
- `scripts/replay_drive.sh`는 Humble용이다. Foxy `ros2 bag play`에는 `--clock`이 없고, `ros2 param dump`는 `--print`가 있어야 표준출력으로 나온다.

**방법**
1. `/scan`은 따로 만들 필요 없다. `replay_drive.sh`는 `lidar_drive.launch.py use_camera:=false use_sim_time:=true use_metrics_logger:=false`를 띄우고, 이 launch에 `pointcloud_to_laserscan`(min_height −0.1, max_height 0.5, target_frame `livox_frame`, angle_increment 0.00436)이 들어 있다. bag에서 `/livox/lidar`와 `/vesc/odom`만 재생하면 `/scan`이 같은 설정으로 다시 만들어진다.
2. Foxy용으로 `exp/occlusion-speed`의 두 가지를 가져온다 (`714321f`에서 파일 단위로):
   - `scripts/bag_player.py` (+ `test/test_bag_player.py`): sqlite3 bag을 읽어 재생하면서 bag 시각으로 `/clock`을 발행. my_algo에 의존하지 않음.
   - `scripts/replay_drive.sh`의 Foxy 수정분: `ros2 bag play --clock` → `python3 bag_player.py`, `ros2 param dump ... --print`.
3. 리팩터링 전(`b054758`)과 후(`refactor/main-baseline`)를 각각 빌드해 같은 bag으로 한 번씩 실행하고 `python3 scripts/verify_refactor.py compare before.csv after.csv`.
   스크립트들은 두 빌드 모두에서 같은 사본(예: `~/session1/tools/`)으로 실행한다.
4. VESC 드라이버는 끈다 (재생 중 명령이 `/commands/*`로 나간다. 스크립트가 VESC 노드를 감지하면 멈춘다).

**확인할 것**
- bag의 `/livox/lidar` frame_id가 `livox_frame`인지 (pointcloud_to_laserscan target_frame과 같아야 TF 없이 변환된다).
- `use_sim_time` 재생에서 planner 모드 유지 시간(0.45 s)·명령 타임아웃(1.0 s)이 bag 시각 기준으로 동작하는지.
- 실시간이 아닌 재생(rate 0.5)에서 메시지 유실로 생기는 차이와 실제 동작 차이를 구별하는 기준.

## 2. 차량에서 확인 안 된 것

- Foxy에서 Ctrl+C 뒤 각 노드의 정지 명령 publish가 실제로 나가는지.
- VESC 드라이버의 명령 타임아웃 (명령이 끊기면 모터가 멈추는지).
- 시작 시 `effective parameters` 로그가 Foxy rclpy에서 값이 채워져 나오는지 (rclpy 내부 `_parameters` 사용. 비어 있으면 `ros2 param dump --print`로 대체).
