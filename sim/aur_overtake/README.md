# sim/aur_overtake — 차량 2대 추월 MuJoCo 시뮬 (Mac, ROS 없음)

`my_algo.aur`(판단 로직)와 `my_algo.baseline`(AEB, 속도·서보 변환)을 그대로 돌린다. 직선 통로에 ego와 선행차 1대.
scan마다 MuJoCo 레이캐스트 → `pointcloud_to_laserscan` 규칙의 scan → 인지·추적·belief·상태 기계·횡 제어 → ERPM·서보 → AEB(50 Hz) → 운동학 차량 모델.
실차 노드 순서와의 대응은 [docs/PIPELINE.md](../../docs/PIPELINE.md) 1절.

## 환경 세팅

Python 3.12 (Homebrew, arm64 Mac에서 확인). 명령은 모두 저장소 루트에서.

```bash
python3.12 -m venv sim/aur_overtake/.venv            # .venv는 .gitignore에 있다
source sim/aur_overtake/.venv/bin/activate
pip install -r sim/aur_overtake/requirements.txt      # mujoco 3.12.0, numpy 2.5.4, PyYAML 6.0.3, pytest 9.2.0
python -m pytest sim/aur_overtake/test_sim.py -q      # 14 통과, 약 20초
```

## 1회 실행과 뷰어

```bash
python sim/aur_overtake/run.py --method d --leader Y                                 # 창 없음, 회차 결과 JSON 한 줄
mjpython sim/aur_overtake/run.py --method d --leader Y --viewer --start-paused       # 뷰어 (macOS는 mjpython), Enter로 시작
mjpython sim/aur_overtake/run.py --method d --leader Y --viewer --speed 5          # 5배속으로 시작 (끝나면 실제 재생 배속을 터미널에 찍는다)
python sim/aur_overtake/run.py --method c --leader NY --aeb off --set start.gap0=2.0 eta=0.95
```

실행 중 터미널(stderr)에 바뀐 것만 한 줄씩 찍힌다. 끄려면 `--quiet`. 마지막 결과 JSON은 stdout이라 `2>/dev/null`로 따로 받을 수 있다.

```text
[  1.50 s] leader 양보 시작: 오른쪽 차선 y_ref=-0.27 (후행차 거리 ≤ d_y=1.00)
[  3.10 s] ego    belief 확정 Y (b=0.99)
[  3.20 s] ego    PROBE → PASS (pass_start)
[ 10.90 s] leader AEB 작동 (ttc=0.31s closest=0.27m aeb_limit=0.31m model=geometric)
```

ego 상태 전이(괄호는 FSM reason), belief 확정, 추월 중단, 선행차 양보 시작·끝·급정지(NY), 두 차의 AEB 작동(상승 에지 = `N_AEB`).

| 옵션 | 값 | 설명 |
|---|---|---|
| `--method` | `a` `b` `c` `d` | a 고정 규칙, b 수동 belief, c 공격적 탐색, d 제안 (PIPELINE 3절) |
| `--leader` | `Y` `NY` `YR` | 선행차 행동 (아래 표). 기본은 `scenario.yaml` |
| `--d-y` | m | 선행차 양보 반응 거리. 없으면 seed로 U[0.6, 1.2] |
| `--yield-yaw` | deg | 양보 중 선행차 yaw 고정 (추적 강건성 테스트) |
| `--W` | m | 통로 폭 |
| `--aeb` | `off` `geometric` `geometric_straight` `geometric_ttc055` `geometric_unknown` `legacy_zero` `legacy_zero_wide` `legacy_true` | AEB 조건 (`batch.py`의 `AEB` 표). 기본 = `scenario.yaml` (`geometric`, 휜 띠 ±0.16 m, TTC 0.35 s, 실제 속도) |
| `--belief` | `original` `survival` | belief 모델 |
| `--seed` | 정수 | 라이다·d_y·급정지 지연 시드 |
| `--set` | `K=V ...` | 덮어쓰기. 점이 있으면 `scenario.yaml` (`start.gap0=2.0`), 없으면 `AurParams` (`eta=0.95`, `heading_comp=false`) |

## 헤드리스 일괄 실행 (`batch.py`)

```bash
python sim/aur_overtake/batch.py belief --n 20          # belief original·survival × 방법 c·d, 선행차 Y (기본 200회, 약 6분)
python sim/aur_overtake/batch.py yaw                    # 선행차 양보 yaw 0/10/20/30° × d_y 0.8/1.0/1.2, 추적 유지
python sim/aur_overtake/batch.py matrix --aeb geometric # 방법 a·b·c·d × 선행차 Y/NY, 5회씩
python sim/aur_overtake/batch.py aeb                    # W 1.0/1.1/1.2 × AEB 조건 × c·d × Y/NY (교착, N_AEB)
python sim/aur_overtake/batch.py yrw --aeb geometric    # 양보 후 복귀 × W × 선행차 W 고정 on/off
python sim/aur_overtake/batch.py treact                 # t_react 0~1.6 s별 Y 성공률 (약 15분)
python sim/aur_overtake/batch.py treact_ny              # t_react별 NY 쪽 비용
```

공통 옵션: `--n`(회차 수), `--methods cd`, `--W`, `--aeb`(**기본 `off`**: 판단 로직만 볼 때). 모드별: `--yaws`, `--d-ys`, `--Ws`, `--conds`, `--returns`.

## 동작 기준선과 분석

```bash
python sim/aur_overtake/baseline.py save before.csv            # 고정 시드 252회 + 스텝 로그 해시 (약 2.5분)
python sim/aur_overtake/baseline.py compare before.csv after.csv   # 바뀐 회차를 표로, 다르면 종료 코드 1
python sim/aur_overtake/analysis/yb_bias.py collect runs.json --no-abort   # 선행차 횡위치 추정 오차 측정 (320회, 약 4분)
python sim/aur_overtake/analysis/yb_bias.py report runs.json
```

코드를 바꿀 때 바꾸기 전후로 `baseline.py save`를 하고 `compare`로 의도한 회차만 바뀌었는지 본다.

## 결과 읽는 법

- 표는 표준출력(markdown). 회차별 결과는 `sim_logs/aur_overtake/<모드>.csv` (git에 올리지 않는다, 같은 모드를 다시 돌리면 덮어쓴다).
- CSV 한 줄 = 한 회차 (`Sim.result()`). 주요 열:

| 열 | 뜻 |
|---|---|
| `success`, `final_state`, `cause` | DONE 도달 여부, 마지막 상태, 실패 원인 (`belief_NY`, `T_max`, `T_wait_a`, `track_lost`, `collision_cars`, `collision_ego_wall`, `collision_leader_wall`, `other`) |
| `decided`, `T_id` | belief 확정(Y/NY)과 t₀부터 걸린 시간 [s] |
| `T_pass` | t₀ → DONE [s] |
| `h_min` | 급정지 전 최소 안전 여유 (식 7) [m] |
| `stop_gap`, `stop_gap_ok` | 급정지 후 최소 간격, 그 간격 ≥ m인가 |
| `gap_min` | x 방향 최소 간격 (옆으로 나란하면 음수. 충돌은 `collision`으로 판정) |
| `N_AEB`, `N_AEB_leader` | AEB 상승 에지 수 (ego, 선행차) |
| `N_AEB_moving`, `N_AEB_leader_moving` | 그중 차 속도 > 0.1 m/s일 때 걸린 것 (서 있는 동안 잡음 깜빡임 제외). 지표는 이것을 본다 |
| `deadlock` | 회차 끝까지 ego가 APPROACH~MERGE에서 정지 |
| `n_abort`, `return_after_overlap` | 추월 중단 횟수, (YR) 후행차가 옆에 온 뒤 복귀했는가 (시뮬 전용 시나리오) |
| `track_lost`, `track_lost_longest_s`, `width_max` | 정면 추적이 끊긴 scan 수, 최장 끊김, 클러스터 폭 최대 |
| `hold_count` | 최소 속도 규칙 분기 진입 횟수 |

## 선행차 행동 (`scenario.yaml`의 `leader`)

| 값 | 동작 |
|---|---|
| `Y` | 후행차가 `d_y` 안으로 오면 오른쪽 차선(−δ)으로 비킨다 |
| `NY` | 비키지 않는다. 후행차가 1.0 m 안으로 온 뒤 0.5~2.0 s 후 급정지 |
| `YR` | 비켰다가 `return_after`초 뒤 중앙으로 돌아온다. 후행차가 이미 옆에 온 뒤의 복귀는 시뮬 전용 시나리오 |
| `fix_W` | 선행차에 실측 W(= `track.W`)를 준다 (기본 켜짐) |
| `yield_yaw_deg` | `null`: 선행차 자신의 횡 제어기. 숫자: 횡위치를 `yield_lat_speed`로 옮기면서 yaw 고정 (선행차-벽 충돌 판정은 뺀다) |

## 발표·기록 자료 (`media/make_media.py`)

```bash
pip install -r sim/aur_overtake/media/requirements.txt
python sim/aur_overtake/media/make_media.py --out research_record/2026-11-16_sim   # 영상 3개, 장면 캡처, 그래프 6개 (약 3분)
```

1920×1080 오프스크린 렌더 + 한국어 화면 글자(macOS Apple SD Gothic Neo) + 미니맵. 결과물 설명은 [research_record/2026-11-16_sim/README.md](../../research_record/2026-11-16_sim/README.md).

## 뷰어 키

| 키 | 동작 |
|---|---|
| Enter | 일시정지 / 재개 |
| `.` | 일시정지 중 scan 1프레임(0.1 s) 진행 |
| 9 / 0 | 배속 1× → 2× → 5× → 0.5× 순서로 바꿈 / 1×로. 화면은 실제 시간 기준 약 50~60 fps로, 글자 패널은 초당 4번 갱신한다 (글자 갱신이 느려 매번 하면 재생이 밀린다). 글자 패널 playback 줄에 실제 배속이 나온다. 라이다 점은 실차처럼 scan 주기(시뮬 시간 0.1 s)마다 바뀐다 |
| 5 | 라이다가 맞히는 판정용 모양 (벽 판, 차 상자) 보기. 화면의 차 모양·가드레일·차선은 보기용이라 scan에 안 들어간다 |
| 6 | 라이다 점 (초록) |
| 7 | ego가 인식한 선행차 영역 (주황 반투명, `d` 표시) + ego AEB 검사 영역 (빨강, 작동하면 진해짐) |
| 8 | 차선 ±δ (보라)·y_ref (노랑) |
| End (fn + →) | 글자 패널 |
| Home (fn + ←) | 재시작: 같은 설정·시드로 처음부터 |
| 마우스 | 끌기 회전, 오른쪽 끌기 이동, 스크롤 확대 |

MuJoCo 기본 Reset/Run/Pause(Backspace, Space, 왼쪽 패널)는 이 시뮬에서 동작하지 않는다.

## 파일

| 파일 | 내용 |
|---|---|
| `scenario.yaml` | 시뮬 전용 값 (통로, 출발 간격, 선행차 행동, AEB, 차량 응답, 라이다, `params`로 AurParams 덮어쓰기). TUNE 태그 → docs/TUNING.md |
| `sim.py` | `Sim`(회차), `Car`(차 한 대: 인지·추적·횡 제어·AEB), 충돌 판정, 회차 지표 |
| `run.py`, `batch.py`, `baseline.py` | 1회 실행·뷰어 / 일괄 실행 / 동작 기준선 |
| `analysis/yb_bias.py` | 선행차 횡위치 추정 오차, 가짜 복귀 상승, 헤딩 오차 측정 |
| `viewer_ui.py`, `world.py` | 뷰어 오버레이·키, MJCF 생성 (벽 2개 + 차 2대 mocap box) |
| `lidar.py`, `vehicle.py` | 라이다 근사·운동학 차량 모델 (이전 연구 시뮬에서 가져온 것, 내용 그대로) |
| `test_sim.py` | 시뮬 점검 테스트 |

## 실차와 다른 점

- 판단에 쓰는 속도는 차량 모델의 참값이다 (실차는 `/sensors/core` 예정).
- 차량은 운동학 자전거 모델이다 (미끄러짐 없음). 응답 값(`t_r` 0.15, `tau` 0.10, `a_b` 3.0 등)은 가정값이다.
- `params.t_d` = 0.25 (구동 지연 + 1차 지연), 여기에 `scan_period` 0.1이 더해진다. 실차 값은 보정 실험으로 정한다.
- 라이다는 수평 한 줄, 기본 무잡음이다 (`lidar.range_noise_std`). 같은 조건의 회차끼리 결과가 거의 같다.
- AEB는 두 차 모두 `geometric`이 기본이다. 실차 `autonomous_drive`에는 아직 이 모드를 고르는 파라미터가 없다.
- 뷰어 창은 Claude 세션에서 띄워 확인하지 못했다 (오버레이 그리기와 키 처리는 창 없이 테스트).
