# 정리 보고서 (1단계, 승인 대기)

기준 커밋: `15371fe` (exp/occlusion-speed). 대상: `my_algo` 레포.
바깥 `f1tenth_gym_ros` 레포(시뮬레이터 원본)는 대상이 아니다. 그 레포 작업 트리에서 시뮬 파일이
대량 삭제된 상태지만, 이 작업에서는 건드리지 않는다.

판정 근거는 세 가지만 쓴다: launch 파일의 `executable=`, 파이썬 `import`, 토픽 연결.
"기존 주행 모드" = `lidar_drive.launch.py` (README의 기본 실행 명령).

## 1. 노드와 모듈

| 파일/노드 | 사용 여부 근거 | 제안 | 이유 |
|---|---|---|---|
| `my_algo/lidar_reactive_drive.py` | lidar_drive·autonomous_drive launch에서 실행. `/reactive/lidar_command` → autonomous_drive | **유지** | 기존 주행 모드의 planner. exp 모드에서도 조향을 그대로 씀 |
| `my_algo/autonomous_drive.py` | 두 주행 launch에서 실행. 최종 `/commands/motor/speed`, `/commands/servo/position` 발행 | **유지** | 기존 주행 모드의 최종 명령 노드 |
| `my_algo/aeb.py` | `autonomous_drive.py`가 import (`AEBSafetyModel`) | **유지** | 기존 AEB 로직. 3단계에서 별도 AEB 노드로 분리할 때 로직 재사용 |
| `my_algo/vehicle_utils.py` | lidar_reactive_drive, autonomous_drive, aeb, joy_controller가 import | **유지** | `MIN_DRIVING_RPM`, ERPM·서보 변환. 값 변경 금지 대상 |
| `my_algo/autonomous_metrics_logger.py` | 두 주행 launch에서 실행. odom·명령·scan 구독, `/metrics/*` 구독 | **유지** | 기존 주행 모드에서 쓰임. git hash 함수는 실험 메타데이터에 재사용 가능 |
| `my_algo/camera_reactive_drive.py` | lidar_drive·autonomous_drive·camera_drive launch에서 실행. `/aeb/active` 구독 | **유지** | 기존 주행 모드 launch에 포함됨. 2단계에서 `use_yolo:=false`(기본)로 노드 자체를 안 띄우게 함 |
| `my_algo/joy_controller.py` | setup.py 엔트리만 있음. 두 주행 launch에서 **주석 처리**됨. 토픽 `/joy_active`는 autonomous_drive가 구독 | **유지** | 현재 launch에서 안 쓰이지만, B 버튼 긴급 정지와 수동 개입 경로가 살아 있음. 실험 비상 정지 수단 후보 (RUNBOOK에서 결정) |
| `my_algo/imu_motion_logger.py` | `launch/imu_motion.launch.py`에서만 실행. 주행 launch에는 없음 | **archive (선택)** | IMU 단독 적분 속도 기록용. 실험 IMU 경로(`imu_republisher` + 분석 스크립트)와 겹침. 기본 `acceleration_scale=1.0`이라 드라이버가 g 단위면 정지 보정이 끝나지 않음. 유지를 원하면 그대로 둠 |
| `my_algo/imu_integration.py` | `imu_motion_logger.py`와 `test/test_imu_integration.py`만 import | imu_motion_logger와 같이 | 위 노드의 계산 모듈 |
| `launch/imu_motion.launch.py` | imu_motion_logger 전용 | imu_motion_logger와 같이 | |
| `test/test_imu_integration.py` | imu_integration 테스트 | imu_motion_logger와 같이 | |
| `my_algo/imu_normalize.py`, `imu_republisher.py`, `floor_plane.py` | 실험 브랜치에서 추가. imu.launch.py, measure_lidar_height.py, 테스트가 사용 | **유지** | 실험 TF·IMU 경로 |

## 2. launch / config / scripts

| 파일 | 사용 여부 근거 | 제안 | 이유 |
|---|---|---|---|
| `launch/lidar_drive.launch.py` | README 기본 실행 명령 | **유지** | 기존 주행 모드. 2단계에서 `use_yolo` 인자만 추가 (기본 false) |
| `launch/autonomous_drive.launch.py` | README "통합 launch". rosbridge 포함 | **유지** | README에 실행 경로로 문서화됨 |
| `launch/camera_drive.launch.py` | README "카메라 perception 확인용" | **유지** | 카메라 장착 시야 확인(2.1절)에 쓸 수 있음 |
| `launch/tf_static.launch.py` (초안, 15371fe) | `launch/tf.launch.py`가 include | **교체** | `incoming/tf_static.launch.py` 원본으로 덮어씀. base_link→livox_frame 발행 노드를 하나만 남김 |
| `launch/tf.launch.py`, `config/ekf.yaml` | 실험 TF 진입점, EKF 기본 off | **유지** | |
| `launch/imu.launch.py`, `config/imu.yaml` | imu_republisher 실행 | **유지** | |
| `scripts/check_vehicle.sh`, `scripts/measure_lidar_height.py` | 차량 점검, 원점 높이 측정 (2.2절) | **유지** | |
| `README_experiment.md` | 실험 절차 초안 | **유지 → 4단계에서 docs/로 흡수** | RUNBOOK/OPEN_ITEMS로 옮긴 뒤 링크만 남김 |

## 3. incoming/ 통합 위치 (2단계)

| 원본 | 옮길 위치 | 메모 |
|---|---|---|
| `incoming/tf_static.launch.py` | `launch/tf_static.launch.py` (초안 대체) | 값 변경 없음 |
| `incoming/vehicle_geometry.yaml` | `config/vehicle_geometry.yaml` | 모든 노드가 공유. setup.py data_files에 이미 `config/*.yaml` 등록됨 |
| `incoming/imu_static_calib.py` | `scripts/imu_static_calib.py` | 값 변경 없음. 단, 토픽 `/livox/imu`가 코드에 고정되어 있음 (0.3절 위반 소지) → 인자로 받게 할지 승인 필요 |

## 4. 삭제 제안 (빌드 산출물·OS 파일만)

| 대상 | 근거 | 제안 |
|---|---|---|
| `.DS_Store` (git 추적 중) | macOS Finder가 만드는 파일. 093c5c8에서 실수로 커밋됨. 지금도 Finder가 계속 수정함 | **추적 해제 + `.gitignore`에 추가** |
| `my_algo/launch/` (`__pycache__/real_launch.cpython-313.pyc`만 남음) | 093c5c8에서 `real_launch.py`가 삭제된 뒤 남은 캐시. git 추적 안 됨 | **삭제** |
| 각 폴더 `__pycache__/` | 빌드 산출물, 이미 .gitignore 대상 | **삭제** (로컬만) |

`*_old.py`, `*_backup` 같은 중복 백업 파일과 빈 소스 파일은 없다. `resource/my_algo`는 비어 있지만
ament_index 마커라 **유지**.

## 5. 2단계 진입 전 결정이 필요한 것

1. `imu_motion_logger` 묶음 archive 여부
2. `imu_static_calib.py`의 토픽을 인자로 바꿔도 되는지 (기본값은 그대로 `/livox/imu`)
3. `.DS_Store` 추적 해제
