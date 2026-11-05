# my_algo

F1TENTH 실차(Livox Mid-360, VESC, **ROS2 Foxy**) 주행 스택과, 관측 공백(가림) 기반 속도 계획 비교 실험(B0/B1/B2/P) 코드.

## 문서

| 문서 | 내용 |
|---|---|
| [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) | 파이프라인, 노드 표, TF 트리, drive·exp 모드 차이 |
| [docs/SESSIONS.md](docs/SESSIONS.md) | **세션별로 차 앞에서 순서대로 칠 명령** (세션 1 정비 검증 ~ 세션 5 Exp3) |
| [sim/README.md](sim/README.md) | Mac MuJoCo 시뮬 빠른 시작: 시나리오·방법, 처음 돌려볼 조합, 뷰어 키, 기본 명령 |
| [docs/SIMULATION.md](docs/SIMULATION.md) | 시뮬 세부: 구조, 근거, 검증, 설치, 인자 전체, 출력, 가정값, 시뮬 전용 차이 |
| [docs/RUNBOOK.md](docs/RUNBOOK.md) | 터미널별 실행 절차, 회차 체크리스트, 비상 정지, 문제 해결 |
| [docs/EXPERIMENT_PLAN.md](docs/EXPERIMENT_PLAN.md) | RQ·가설, Exp0–Exp3 명령과 회차, 지표, 계획서 이후 변경 사항 |
| [docs/LOGGING.md](docs/LOGGING.md) | 로그 경로·meta.json·기록 토픽·용량 |
| [docs/OPEN_ITEMS.md](docs/OPEN_ITEMS.md) | 실측 대기 값, TBD(세션 1), 확인 필요 사항 |
| [docs/EXP_TAGS.md](docs/EXP_TAGS.md) | [EXP-*] 태그 목록 (자동 생성) |
| [docs/DRIVE_STACK.md](docs/DRIVE_STACK.md) | 기존 drive 모드 설명 (토픽, 알고리즘 공식) |
| [analysis/README.md](analysis/README.md) | 분석 스크립트 순서 |

## 빠른 시작

```bash
source scripts/env_ego.sh && cd ~/f1tenth_ws && colcon build --symlink-install --packages-select my_algo && source install/setup.bash
ros2 launch my_algo bringup.launch.py                                                    # 기존 주행 (drive)
ros2 launch my_algo bringup.launch.py mode:=exp method:=P scenario:=S1 d_e:=1.5 run_id:=1 # 실험 회차
bash $(ros2 pkg prefix my_algo)/share/my_algo/scripts/record_run.sh exp P S1 1.5 1        # 기록
```
