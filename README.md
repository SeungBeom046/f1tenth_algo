# my_algo — exp/aur-overtake

의도를 모르는 선행차를 안전하게 추월하는 실험 브랜치다 (브랜치 이름의 aur는 작업 이름).
1/10 F1TENTH 2대, LiDAR만, 지도·TF·차량 간 통신·IMU 없음. 후행차(ego)가 가까이 붙어 선행차가 비키는지(Y/NY)를 belief로 판단하고, 비키면 추월, 아니면 따라간다.

- 기준: `refactor/main-baseline` (main의 기본 주행 + 회귀 테스트). 기본 주행 동작은 바꾸지 않는다 (`scripts/verify_refactor_offline.py`로 확인).
- 판단 로직은 rclpy 없는 순수 파이썬(`my_algo/aur/`)이고, MuJoCo 시뮬(`sim/aur_overtake/`)이 같은 코드를 돌린다.
- **현재 상태**: 순수 모듈과 시뮬까지. 실차 노드(`exp_planner`), 실험 launch, 속도 소스(`/sensors/core`), 비상정지 heartbeat는 아직 없다. 열린 문제는 `docs/PROGRESS.md`.
- 이전 연구(관측 공백 속도 계획)는 이 브랜치에 없다 → `archive/README.md`.

## 폴더

| 경로 | 내용 |
|---|---|
| `my_algo/aur/` | 추월 실험 판단 로직: 인지(corridor) → 추적(tracker) → belief → 상태 기계(ego_fsm, leader_logic, safety) → 횡·종 제어(lateral, speed) |
| `my_algo/baseline/aeb.py` | AEB. `legacy`(기본 주행) / `geometric`(추월 실험 1층) |
| `my_algo/*.py` 나머지, `launch/` | 기본 주행 노드와 launch (main에서 온 것, 그대로) |
| `config/exp_common.yaml` | 추월 실험 파라미터 (TUNE 태그) |
| `sim/aur_overtake/` | MuJoCo 2대 시뮬: 뷰어, 헤드리스 일괄 실행, 동작 기준선 |
| `test/` | 단위 테스트 (ROS 없이), 기본 주행 회귀 테스트 |
| `scripts/` | `verify_refactor_offline.py`(기본 주행 회귀), `gen_tuning_doc.py`(TUNING.md 생성), bag 재생 도구 |
| `archive/` | 쓰지 않는 파일 보관 (IMU 단독 기록 도구) |
| `docs/` | 아래 문서 |

## 문서

| 문서 | 내용 |
|---|---|
| [docs/PIPELINE.md](docs/PIPELINE.md) | 모듈 파이프라인, 토픽 흐름, 상태 기계 전이 조건, belief 식, AEB 2층 구조, 담당 파일·함수 |
| [sim/aur_overtake/README.md](sim/aur_overtake/README.md) | 시뮬 환경 세팅, 뷰어, 일괄 실행, 옵션, 결과 읽는 법, 키 |
| [docs/PROGRESS.md](docs/PROGRESS.md) | 기획안 변경 이력, 결과, 열린 문제 |
| [docs/TUNING.md](docs/TUNING.md) | 조정 파라미터 표 (`scripts/gen_tuning_doc.py`가 생성) |
| [docs/DRIVE_BASELINE.md](docs/DRIVE_BASELINE.md) | 기본 주행(lidar_drive.launch.py) 실행, 정지 수단, 노드, 알고리즘 |
| [docs/OPEN_ITEMS.md](docs/OPEN_ITEMS.md), [docs/CLEANUP_REPORT.md](docs/CLEANUP_REPORT.md) | 기본 주행 리팩터링(refactor/main-baseline) 기록 |

## 빠른 확인

```bash
python3 -m pytest -q                                              # 단위 테스트 (Python 3.8, ROS 없이)
python3 scripts/verify_refactor_offline.py --baseline b054758     # 기본 주행이 main(b054758)과 같은지
python3 scripts/gen_tuning_doc.py --check                         # TUNING.md가 최신인지
```

시뮬은 [sim/aur_overtake/README.md](sim/aur_overtake/README.md).
