# MuJoCo 시뮬 (Mac, ROS 없음)

실차 exp 코드(B0/B1/B2/P)를 그대로 돌려 S1–S4를 Mac에서 재현한다. 모든 명령은 **레포 루트(my_algo)** 에서, 창은 **`mjpython`** 으로만 뜬다.

```bash
bash sim/setup_env.sh                       # 처음 한 번 (sim/.venv 설치·점검)
source sim/.venv/bin/activate               # 터미널마다
mjpython sim/runner.py --method P --scenario S2 --viewer --start-paused     # 창이 뜨면 Enter로 시작
```

세부(설치·출력 파일·가정값·결정성·시뮬 전용 차이·알 수 있는 것/없는 것)는 [docs/SIMULATION.md](../docs/SIMULATION.md).

## 시나리오

| 코드 | 트랙 | 돌출 | 무엇을 보나 | 가설 |
|---|---|---|---|---|
| S1 | 주차 박스 3개, 틈 0.45 m (통과 가능) | 틈에 숨은 #2가 d_e(1.0/1.5/2.0 m) 시점에 튀어나옴. d_e가 작을수록 늦게 나와 어려움 | 안전: 충돌 여부, 충돌 속도 | H1 |
| S2 | S1과 같은 배치 | 없음 | 위험이 있을 수 있는 곳의 감속 비용 (정당한 감속) | H2 기준 |
| S3 | 박스 3개, 틈 0.15 m (차가 못 나오는 폭) | 없음 | 불필요한 감속. B2는 감속, P는 거의 안 해야 함 | H2 |
| S4 | 우측 전체가 연속 벽 (뒤가 넓게 안 보임) | 없음 | 면적 기반 B1의 과잉 감속 | H3 |
| E0 | 빈 직선 | 없음 | Exp0 플랫폼 측정: 급제동(a_b), 속도 계단(`--test-step-v`, 추종 지연), odom 스케일 | – |

## 방법

| 코드 | 한 줄 설명 | 뷰어에서 보이는 것 |
|---|---|---|
| B0 | AEB만. 안 보이는 곳은 무시하고 v_ref 유지 | phantom 없음. 가까운 장애물에서만 급정지 |
| B1 | 전방 ROI(앞 3 m × 반폭 약 1 m)의 미관측 칸 비율 ρ만큼 감속, v = v_ref·(1 − k·ρ) | phantom 없음. 안 보이는 면적이 크면 넓게 감속 |
| B2 | 자유공간과 맞닿은 **모든** 가림 경계에 phantom → 가장 가까운 phantom까지 거리로 v_cap | 초록 phantom이 경계 곳곳에. 보수적 |
| P (제안) | 앞쪽 주차열의 **통과 가능한 틈(w_min 0.35 ≤ 폭 ≤ w_max 0.50)의 입구와 열 양 끝에만** phantom → v_cap | 주황 phantom이 틈 입구·열 끝에만. 좁은 틈엔 없음 |

공통: 점유격자, v_cap 식, AEB, 조향, phantom 유지(지나갈 때까지)가 모두 같고 phantom 배치 규칙만 다르다.

## 처음 돌려볼 조합

| 명령 | 보여야 하는 것 |
|---|---|
| `mjpython sim/runner.py --method P --scenario S2 --viewer` | 틈 입구에 주황 phantom, 그 앞에서 살짝 감속 후 통과 |
| `mjpython sim/runner.py --method B2 --scenario S3 --viewer` | 좁은 틈마다 초록 phantom, 크게 감속 |
| `mjpython sim/runner.py --method P --scenario S3 --viewer` | 좁은 틈엔 phantom 없음, 거의 감속 안 함 |
| `mjpython sim/runner.py --method B1 --scenario S4 --viewer` | 벽 뒤 미관측 때문에 전 구간 감속 |
| `mjpython sim/runner.py --method B0 --scenario S1 --d-e 1.0 --viewer` | #2와 충돌 (AEB만으로는 늦음) |
| `mjpython sim/runner.py --method P --scenario S1 --d-e 1.0 --viewer` (B2도) | 방법 간 안전 비교의 핵심 장면 |

## 뷰어 조작

| 키 | 동작 | | 키 | 동작 |
|---|---|---|---|---|
| Enter | 일시정지 / 재개 | | 6 | 라이다 점 켜기/끄기 |
| `.` | 일시정지 중 1프레임(0.1 s) 진행 | | 7 | 가림 경계: 누적 → 이번 프레임 → 끔 |
| 9 | 배속 1× ↔ 0.25× | | 8 | phantom 켜기/끄기 |
| End (fn + →) | 글자 패널 켜기/끄기 | | Home (fn + ←) | 재시작: 같은 설정·시드로 처음부터 |
| PgDn (fn + ↓) | 미관측 영역 바닥 칠하기 켜기/끄기 | | 마우스 | 끌기 회전, 오른쪽 끌기 이동, 스크롤 확대 |

색: 라이다 점 **초록** = 높이 대역 안(격자에 쓰임), **회색** = 대역 밖 · 가림 경계 **보라** 선(기본 N 프레임 누적, `--boundary-pillars`면 얇은 기둥) ·
미관측 바닥 **반투명 남색** · phantom **P 주황 / B2 초록**.

- 회차가 시뮬 5–11초라 금방 끝난다. `--start-paused`를 붙이면 멈춘 채로 뜬다 (창을 앞으로 가져온 뒤 Enter로 시작). 보는 중엔 `9`(0.25배속).
- 다시 보려면 **Home**: 창은 그대로, 카메라·토글 유지, 같은 시드로 새 회차. 직전 회차는 저장되고 새 회차는 `_r1`, `_r2` 폴더.
- MuJoCo 기본 Reset/Run/Pause 버튼(Backspace, Space, 왼쪽 패널)은 이 시뮬에서 동작하지 않는다 (화면 키 목록 아래에도 표시).

## 배치·일괄 실행

```bash
python sim/runner.py --method P --scenario S1 --d-e 1.5 --seed 0       # 1회, 창 없음 (약 5초)
python sim/sweep.py --seeds 0 1 2                                      # 전체 72회 + 요약 표·그림 5·6 (약 1분)
python sim/sweep.py --analyze sim_logs/<날짜>/sweep_main               # 표·그림만 다시
python sim/runner.py --method P --scenario S1 --d-e 1.0 --video        # 위에서 본 mp4
python sim/sweep.py --methods P --w-min 0.25 0.35 0.45 --name wmin     # [EXP-TUNE] 덮어쓰기 (값마다 별도 회차)
```

결과: `sim_logs/<날짜>/<회차>/` (sweep은 `sim_logs/<날짜>/sweep_<name>/`, 표·그림은 그 아래 `analysis/`).
회차마다 **`report.html`** (브라우저로 열기: 속도·d·ρ·이벤트·궤적 그래프 + 판정 체크리스트 통과/주의/실패 + 같은 조건 다른 방법 비교),
sweep 폴더에는 회차 목록과 판정 요약 **`index.html`**. 기존 회차는 `python sim/report.py <회차>` / `--index <sweep 폴더>`.

## 로그 정리

`sim_logs/`는 git에 올라가지 않으므로 지워도 코드에 영향이 없다. 기본은 **미리보기**, `--yes`일 때만 휴지통으로 옮긴다 (`--permanent`면 영구 삭제).
폴더에 `KEEP` 파일이 있으면 절대 지우지 않는다.

```bash
python sim/clean_logs.py --du                                   # 전체·폴더별 용량
python sim/clean_logs.py --restarts --incomplete --yes          # 뷰어 재시작 중 끊긴 회차를 휴지통으로
python sim/clean_logs.py --before 20261101 --keep-latest 5      # 미리보기: 11/1 전 회차 중 최근 5개 빼고
python sim/clean_logs.py --keep sim_logs/20261105/sweep_main_v3 # 보호 표시
```

조건(`--before --tag --restarts --incomplete --keep-latest --videos-only`)은 함께 쓰면 모두 만족하는 회차만 고른다.

## 자주 나는 문제

- `can't open file ... sim/runner.py: No such file` → 레포 루트(my_algo)에서 실행
- ``RuntimeError: `launch_passive` requires ... `mjpython` `` → `python`이 아니라 `mjpython`으로
- 창이 안 보임 → 다른 창 뒤. Dock의 Python(로켓) 아이콘이나 Cmd+Tab
- `ModuleNotFoundError: mujoco` → venv 미활성. `source sim/.venv/bin/activate` (프롬프트에 `(.venv)`)
- Reset 버튼으로 다시 안 됨 → Home(fn + ←)으로 재시작
