# my_algo/baseline — 기본 LiDAR 주행 + 공통 유틸

main(`b054758`)에서 온 실차 주행. 동작은 바꾸지 않는다: `python3 scripts/verify_refactor_offline.py --baseline b054758`가 합성 입력에서 발행 메시지를 하나씩 비교한다.
상세(정지 수단, 파라미터, 알고리즘): [docs/DRIVE_BASELINE.md](../../docs/DRIVE_BASELINE.md).

| 파일 | 실행 파일 (`ros2 run my_algo …`) | 역할 |
|---|---|---|
| `lidar_reactive_drive.py` | `lidar_reactive_drive` | scan → 주행 모드(벽 추종, 장애물 회피, gap follow) → 조향·RPM 계획 → `/reactive/lidar_command` |
| `autonomous_drive.py` | `autonomous_drive` | planner 명령 선택, AEB 검사, VESC 명령 발행 (50 Hz). 추월 실험에서도 이 노드를 그대로 쓴다 (예정) |
| `aeb.py` | (모듈) | AEB 판정. `legacy`(기본 주행, 기본값) / `geometric`(추월 실험: 좁은 띠, 속도 TTC, `curved_band`) |
| `vehicle_utils.py` | (모듈) | ERPM·서보 변환, 최소 속도 규칙(`sanitize_rpm`), 명령 JSON. 추월 실험·시뮬도 쓴다 |
| `camera_reactive_drive.py` | `camera_reactive_drive` | D455 표시용 인지 (제어 명령 없음) |
| `joy_controller.py` | `joy_controller` | 수동 주행·AUTO 토글 (launch에서 꺼져 있음) |
| `autonomous_metrics_logger.py` | `autonomous_metrics_logger` | AUTO 구간 trial CSV 기록 |

## 실차 실행 (Jetson, ROS 2 Foxy)

```bash
cd ~/f1tenth_ws && colcon build --packages-select my_algo && source install/setup.bash
# Livox 드라이버와 VESC 드라이버를 먼저 띄운 뒤
ros2 launch my_algo lidar_drive.launch.py            # 바로 AUTO로 출발한다 (docs/DRIVE_BASELINE.md의 정지 수단 확인)
ros2 launch my_algo lidar_drive.launch.py use_camera:=false use_metrics_logger:=false
```
