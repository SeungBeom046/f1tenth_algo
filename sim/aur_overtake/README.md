# MuJoCo 추월 시나리오 시뮬 (Mac, ROS 없음)

`my_algo.aur`(corridor / tracker / belief / 상태 기계 / 횡 제어)과 `my_algo.aeb`를 그대로 돌린다. 직선 통로에 ego와 선행차 1대.
scan마다 MuJoCo 레이캐스트 → `pointcloud_to_laserscan` 규칙의 scan → 판단 → ERPM·서보 → AEB(50 Hz) → 운동학 차량 모델. 흐름은 `docs/PIPELINE.md`.

## 환경 세팅

Python 3.12 (Homebrew, arm64 Mac에서 확인). 모든 명령은 저장소 루트에서.

```bash
python3.12 -m venv sim/aur_overtake/.venv          # .venv는 .gitignore에 있다
source sim/aur_overtake/.venv/bin/activate
pip install -r sim/aur_overtake/requirements.txt    # mujoco 3.12.0, numpy 2.5.4, PyYAML 6.0.3, pytest 9.2.0
python -m pytest sim/aur_overtake/test_sim.py -q    # 7개, 약 5초
```

예전 시뮬의 venv(`sim/mujoco` 브랜치 체크아웃의 `sim/.venv`)가 같은 버전이면 그것을 써도 된다 (pytest만 없다).

## 1회 실행과 뷰어

```bash
python sim/aur_overtake/run.py --method d --leader Y                                  # 창 없음, 결과 JSON 한 줄
mjpython sim/aur_overtake/run.py --method d --leader Y --viewer --start-paused        # 뷰어 (macOS는 mjpython), Enter로 시작
python sim/aur_overtake/run.py --method c --leader NY --aeb off --set start.gap0=2.0 eta=0.95
```

| 옵션 | 값 | 설명 |
|---|---|---|
| `--method` | `a` `b` `c` `d` | a 고정 규칙, b 수동 belief, c 공격적 탐색, d 제안 (`docs/PIPELINE.md` 3절) |
| `--leader` | `Y` `NY` `YR` | 선행차 행동 (아래 표). 기본은 `scenario.yaml` |
| `--d-y` | m | 선행차 양보 반응 거리. 없으면 seed로 U[0.6, 1.2] |
| `--yield-yaw` | deg | 양보 중 선행차 yaw 고정 (tracker 강건성 테스트) |
| `--W` | m | 통로 폭 |
| `--aeb` | `off` `geometric` `geometric_ttc055` `geometric_unknown` `legacy_zero` `legacy_zero_wide` `legacy_true` | AEB 조건 (`batch.py`의 `AEB` 표). 기본 = `scenario.yaml` (`geometric`) |
| `--belief` | `original` `survival` | belief 모델 |
| `--seed` | 정수 | 라이다 시드, d_y·급정지 지연 시드 |
| `--set` | `K=V ...` | 덮어쓰기. 점이 있으면 `scenario.yaml` (`start.gap0=2.0`), 없으면 `AurParams` (`eta=0.95`) |

## 헤드리스 일괄 실행

```bash
python sim/aur_overtake/batch.py belief --n 20          # belief original·survival × 방법 c·d, Y (기본 200회, 약 6분)
python sim/aur_overtake/batch.py yaw                    # 선행차 양보 yaw 0/10/20/30° × d_y 0.8/1.0/1.2, 추적 유지
python sim/aur_overtake/batch.py matrix --aeb geometric # 방법 a·b·c·d × 선행차 Y/NY, 5회씩
python sim/aur_overtake/batch.py aeb                    # W 1.0/1.1/1.2 × AEB 조건 × c·d × Y/NY (교착, N_AEB)
python sim/aur_overtake/batch.py yrw --aeb geometric    # 양보 후 복귀 × W × 선행차 W 고정 on/off
python sim/aur_overtake/batch.py treact                 # t_react 0~1.6 s별 Y 성공률 (약 15분)
python sim/aur_overtake/batch.py treact_ny              # t_react별 NY 쪽 비용 (확정 시간, 간격, 급정지)
python sim/aur_overtake/baseline.py save out.csv        # 동작 기준선 (252회, 약 2.5분). compare a.csv b.csv로 비교
```

공통 옵션: `--n`(회차 수), `--methods cd`, `--W`, `--aeb`(기본 `off`: 판단 로직만 볼 때), 모드별 `--yaws`, `--d-ys`, `--Ws`, `--conds`, `--returns`.
`batch.py`는 AEB를 기본으로 끈다. 켜면 아래 "열린 문제"가 결과를 덮기 때문이다.

### 결과 파일

- 표는 표준출력(markdown), 회차별 결과는 `sim_logs/aur_overtake/<모드>.csv` (git에 올리지 않는다, 같은 모드를 다시 돌리면 덮어쓴다).
- CSV 한 줄 = 한 회차 (`Sim.result()`). 주요 열:

| 열 | 뜻 |
|---|---|
| `success`, `final_state`, `cause` | DONE 도달 여부, 마지막 상태, 실패 원인 (`belief_NY`, `T_max`, `T_wait_a`, `track_lost`, `collision_cars`, `collision_ego_wall`, `collision_leader_wall`, `other`) |
| `decided`, `T_id` | belief 확정(Y/NY)과 t₀부터 걸린 시간 [s] |
| `T_pass` | t₀ → DONE [s] |
| `h_min` | 급정지 전 최소 안전 여유 (식 7) [m] |
| `stop_gap`, `stop_gap_ok` | 급정지 후 최소 간격, 그 간격 ≥ m인가 |
| `gap_min` | x 방향 최소 간격 (옆으로 나란하면 음수가 된다. 충돌은 `collision`으로 판정) |
| `N_AEB`, `N_AEB_leader` | AEB 상승 에지 수 (ego, 선행차) |
| `deadlock` | 회차 끝까지 ego가 APPROACH~MERGE에서 정지 |
| `n_abort`, `return_after_overlap` | 추월 중단 횟수, (YR) 후행차가 옆에 온 뒤 복귀했는가 (시뮬 전용 시나리오) |
| `track_lost`, `track_lost_longest_s`, `width_max` | 추적이 끊긴 scan 수, 최장 끊김, 클러스터 폭 최대 |
| `hold_count` | 최소 속도 규칙 분기 진입 횟수 |

## 선행차 행동 (`scenario.yaml`의 `leader`)

| 값 | 동작 |
|---|---|
| `Y` | 후행차가 `d_y` 안으로 오면 오른쪽 차선(−δ)으로 비킨다 |
| `NY` | 비키지 않는다. 후행차가 1.0 m 안으로 온 뒤 0.5~2.0 s 후 급정지 |
| `YR` | 비켰다가 `return_after`초 뒤 중앙으로 돌아온다. 후행차가 이미 옆에 온 뒤의 복귀는 시뮬 전용 시나리오 |
| `fix_W` | 선행차에 실측 W(= `track.W`)를 준다 (기본 켜짐). 옆에 온 후행차를 벽으로 보지 않는다 |
| `yield_yaw_deg` | `null`: 선행차 자신의 횡 제어기. 숫자: 횡위치를 `yield_lat_speed`로 옮기면서 yaw 고정 (선행차-벽 충돌 판정은 뺀다) |

## 뷰어 키 (예전 시뮬과 같은 양식)

| 키 | 동작 | 예전 시뮬에서 |
|---|---|---|
| Enter | 일시정지 / 재개 | 같음 |
| `.` | 일시정지 중 scan 1프레임(0.1 s) 진행 | 같음 |
| 9 | 배속 1× ↔ 0.25× | 같음 |
| 6 | 라이다 점 (초록) | 같음 |
| 7 | 추적 중인 선행차 표시 (주황) | 가림 경계 표시였음 |
| 8 | 차선 ±δ (보라)·y_ref (노랑) | phantom 표시였음 |
| End (fn + →) | 글자 패널 | 같음 |
| Home (fn + ←) | 재시작: 같은 설정·시드로 처음부터 | 같음 |
| 마우스 | 끌기 회전, 오른쪽 끌기 이동, 스크롤 확대 | 같음 |

PgDn(미관측 바닥)은 쓰지 않는다. MuJoCo 기본 Reset/Run/Pause(Backspace, Space, 왼쪽 패널)는 이 시뮬에서 동작하지 않는다.

## 파일

| 파일 | 내용 |
|---|---|
| `scenario.yaml` | 시뮬 전용 값 (통로, 출발 간격, 선행차 행동, AEB, 차량 응답, 라이다, `params`로 AurParams 덮어쓰기). TUNE 태그 → `docs/TUNING.md` |
| `sim.py` | `Sim`(회차), `Car`(차 한 대), 충돌 판정, 회차 지표 |
| `run.py`, `batch.py`, `baseline.py` | 1회 실행·뷰어 / 일괄 실행 / 동작 기준선 저장·비교 |
| `viewer_ui.py`, `world.py` | 뷰어 오버레이·키, MJCF 생성 (벽 2개 + 차 2대 mocap box) |
| `lidar.py`, `vehicle.py` | 예전 시뮬(`sim/mujoco` 브랜치)에서 그대로 가져온 라이다·운동학 모델 |
| `test_sim.py` | 시뮬 점검 테스트 |

## 실차와 다른 점

- 판단에 쓰는 속도는 차량 모델의 참값이다 (실차는 `/sensors/core` 예정).
- 차량은 운동학 자전거 모델이다 (미끄러짐 없음). 응답 값(`t_r` 0.15, `tau` 0.10, `a_b` 3.0 등)은 가정값이다.
- `params.t_d` = 0.25 (구동 지연 + 1차 지연), 여기에 `scan_period` 0.1이 더해진다. 실차 값은 보정 실험으로 정한다.
- 라이다는 수평 한 줄, 기본 무잡음이다 (`lidar.range_noise_std`). 그래서 같은 조건의 회차끼리 결과가 거의 같다.
- AEB는 두 차 모두 `geometric`(띠 ±0.16 m, TTC 0.35 s, 실제 속도)이 기본이다. 실차 `autonomous_drive`에는 아직 이 모드를 고르는 파라미터가 없다.
- 열린 문제 (`docs/PROGRESS.md`): AEB를 켜면 ego가 차선 변경 중 벽에서 서서 교착된다. 측면 추적 y_B 편향(약 0.1 m)으로 추월을 잘못 중단한다.
