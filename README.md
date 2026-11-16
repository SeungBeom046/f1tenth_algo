# my_algo — exp/aur-overtake: 차량 2대 안전 추월

의도를 모르는 선행차를 후행차(ego)가 안전하게 추월하는 연구다 (브랜치 이름의 aur는 작업 이름).
1/10 F1TENTH 2대, LiDAR(Livox Mid-360 → `/scan`)만 쓰고 지도·TF·차량 간 통신·IMU는 쓰지 않는다.
ego는 가까이 붙어 선행차가 비키는지(Y/NY)를 belief로 판단하고, 비키면 왼쪽으로 추월, 아니면 안전 거리로 따라간다.

- **현재 상태**: 판단 로직 순수 모듈(`my_algo/aur/`)과 MuJoCo 시뮬(`sim/aur_overtake/`)까지. 실차 노드(`exp_planner`), 실험 launch, 속도 소스(`/sensors/core`), 비상정지 heartbeat는 아직 없다. 열린 문제는 [docs/PROGRESS.md](docs/PROGRESS.md).
- 기본 주행(main의 LiDAR 주행)은 `my_algo/baseline/`에 그대로 있고 동작을 바꾸지 않는다 (`scripts/verify_refactor_offline.py`가 main `b054758`과 비교).
- 이전 연구(틈 인지 관측 공백 속도 계획, phantom)는 [occlusion_archive/](occlusion_archive/README.md) (보관용, 실행 대상 아님).

## 폴더

```text
my_algo/                    ROS 2 패키지 (Foxy, Python 3.8). setup.py가 설치하는 것은 이 폴더뿐
  aur/                      현재 연구: 추월 판단 로직, rclpy 없음 (실차 노드·시뮬이 같은 코드를 import)
  baseline/                 기본 LiDAR 주행 노드 + 공통 유틸 (vehicle_utils, aeb)
config/exp_common.yaml      추월 실험 파라미터 (TUNE 태그 → docs/TUNING.md)
launch/                     기본 주행 launch (lidar_drive, autonomous_drive, camera_drive)
sim/aur_overtake/           현재 연구 MuJoCo 시뮬 (Mac, ROS 없음): 뷰어, 일괄 실행, 동작 기준선, 분석
test/                       단위 테스트 (ROS 없이) + 기본 주행 회귀 테스트
scripts/                    기본 주행 회귀 비교, TUNING.md 생성, bag 재생 도구
docs/                       파이프라인, 진행·결정 기록, 조정 파라미터, 기본 주행 설명
occlusion_archive/          이전 연구 보관 (빌드·설치·테스트 제외)
archive/imu_tools/          쓰지 않는 IMU 단독 기록 도구 보관 (main 시절, 빌드·테스트 제외)
```

## 현재 연구에 필요한 파일 (읽는 순서)

1. [docs/PIPELINE.md](docs/PIPELINE.md): 전체 흐름, 상태 기계, 식
2. [my_algo/aur/README.md](my_algo/aur/README.md): 판단 로직 파일 (파이프라인 순서)
   - `params.py` → `tracker.py`(scan_to_points) → `corridor.py` → `tracker.py`(VehicleTracker) → `belief.py` → `safety.py` → `ego_fsm.py` / `leader_logic.py` → `lateral.py`, `speed.py`
3. `my_algo/baseline/` 중 추월 연구가 쓰는 것: `aeb.py`(AEB, `geometric` 모드), `vehicle_utils.py`(ERPM·서보 변환, 최소 속도 규칙). 실차에서는 `autonomous_drive.py`가 AEB를 적용해 VESC로 보낸다 ([my_algo/baseline/README.md](my_algo/baseline/README.md))
4. `config/exp_common.yaml`과 [docs/TUNING.md](docs/TUNING.md): 파라미터
5. [sim/aur_overtake/README.md](sim/aur_overtake/README.md): 시뮬 (`sim.py`가 실차 노드 순서를 흉내 낸다)
6. [docs/PROGRESS.md](docs/PROGRESS.md): 기획안 변경, 결과, 열린 문제

## 문서

| 문서 | 내용 |
|---|---|
| [docs/PIPELINE.md](docs/PIPELINE.md) | 노드·토픽, 모듈 파이프라인, 상태 기계 전이 조건, belief 식, 안전 거리, AEB 2층 구조, 담당 파일·함수 |
| [docs/PROGRESS.md](docs/PROGRESS.md) | 현재 상태, 열린 문제, 기획안 변경 이력, 실험 결과 |
| [docs/TUNING.md](docs/TUNING.md) | 조정 파라미터 표 (`scripts/gen_tuning_doc.py`가 생성) |
| [sim/aur_overtake/README.md](sim/aur_overtake/README.md) | 시뮬 환경 세팅, 뷰어, 일괄 실행, 옵션, 결과 읽는 법, 키 |
| [my_algo/aur/README.md](my_algo/aur/README.md), [my_algo/baseline/README.md](my_algo/baseline/README.md) | 패키지 파일별 역할 |
| [docs/JETSON.md](docs/JETSON.md) | Jetson에서 이 브랜치 받기·빌드·기본 주행 실행 |
| [docs/CAR_CODE.md](docs/CAR_CODE.md) | 차량별로 올리는 코드 (두 차 같은 브랜치, 역할은 실행 명령으로), 지금과 추월 실험 때 뜨는 노드 |
| [docs/HARDWARE_EXPERIMENT.md](docs/HARDWARE_EXPERIMENT.md) | 차량 2대(검정이·파랑이) 터미널 순서, 실험 환경, 보정 실험, 기록·볼 값 |
| [docs/DRIVE_BASELINE.md](docs/DRIVE_BASELINE.md) | 기본 주행 상세 (정지 수단, 파라미터, 알고리즘) |
| [docs/OPEN_ITEMS.md](docs/OPEN_ITEMS.md), [docs/CLEANUP_REPORT.md](docs/CLEANUP_REPORT.md) | 기본 주행 리팩터링(refactor/main-baseline) 당시 기록 (경로는 그때 기준) |

## 빠른 확인

```bash
python3 -m pytest -q                                              # 단위 테스트 (Python 3.8, ROS 없이)
python3 scripts/verify_refactor_offline.py --baseline b054758     # 기본 주행이 main(b054758)과 같은지
python3 scripts/gen_tuning_doc.py --check                         # TUNING.md가 최신인지
```

시뮬 테스트와 실행은 [sim/aur_overtake/README.md](sim/aur_overtake/README.md).
