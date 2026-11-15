# archive

현재 연구(AUR 추월)와 기본 주행 경로에서 쓰지 않는 파일을 보관한다. 빌드(`setup.py`), launch, 테스트 수집(`setup.cfg`의 `testpaths = test`)에서 모두 빠진다.
코드는 옮기기만 하고 고치지 않았다. 다시 쓰려면 원래 경로로 되돌린다 (import 경로가 원래 위치 기준이다).

## 이전 연구: 관측 공백(occlusion) 속도 계획

이 브랜치에는 해당 코드가 없다. 전부 아래에 보존되어 있다.

- 태그 `archive/occlusion-mujoco-20261110` (→ `47121b9`): phantom, occupancy, speed layer, MuJoCo 시뮬(`sim/`), 분석(`analysis/`), 문서
- 브랜치 `exp/occlusion-speed`, `sim/mujoco` (같은 커밋)

## imu_tools/ — IMU 단독 속도 기록 도구

| 파일 | 원래 경로 | 용도 |
|---|---|---|
| `imu_integration.py` | `my_algo/imu_integration.py` | IMU 가속도 적분 (속도·이동 거리 추정) |
| `imu_motion_logger.py` | `my_algo/imu_motion_logger.py` | 위 적분을 쓰는 ROS 노드 (`/imu_motion/status`) |
| `imu_motion.launch.py` | `launch/imu_motion.launch.py` | 위 노드 실행 |
| `test_imu_integration.py` | `test/test_imu_integration.py` | `imu_integration` 단위 테스트 |

미사용 근거 (2026-11-15, 커밋 510afa1 기준): `setup.py` entry_points에 `imu_motion_logger`가 없고, launch 설치 목록에서 `imu_motion.launch.py`를 명시적으로 뺐다. 네 파일은 서로끼리만 import한다. 주행 노드(`lidar_reactive_drive`, `autonomous_drive` 등)와 `my_algo/aur`는 IMU를 쓰지 않는다 (지시서: IMU 사용 안 함).
