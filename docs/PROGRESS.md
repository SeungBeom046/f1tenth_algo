# exp/aur-overtake 진행 상황

다음 세션은 `CLAUDE_TASK.md`(커밋하지 않음, 사용자가 직접 저장)와 이 파일만 읽고 시작한다.

## 끝낸 단계
- 세션 1 (2026-11-14): 0단계(점검), 1단계(`my_algo/aur/` 순수 모듈 + 테스트).
- 검토 반영 (2026-11-14): belief survival 모델, 폐루프 재실험, hold 히스테리시스, TUNE 태그·`config/exp_common.yaml`·`docs/TUNING.md`.
- 커밋은 `git log --oneline` 참고. push는 아직 안 했다.

## 남은 일
- 지시서 세션 2: 4단계(속도 소스), 5단계(AEB relative). 이후 지시서 표대로.
- `setup.py`에 `config/*.yaml` 설치 추가는 7단계에서 한다 (지금은 소스 경로로만 읽는다).
- 승인 대기 (제안만 한 것): PASS 중단 조건 (아래 "안전 확인").

## 기획안 변경

### belief 우도 (식 2~5) → survival 모델. `belief_model: original | survival` (기본 survival)
- **변경 전** (original, 기획안 식): scan마다 독립으로 곱한다.
  λ_k = λ_{k−1} + ln[ π(d_k)·N(z_k; −v_lat·Δt, σ²) + (1−π(d_k))·N(z_k; 0, σ²) ] − ln N(z_k; 0, σ²)
- **변경 후** (survival): 관측을 "누적 횡변위 s = y_B − y_B(t₀)가 −y_thr 아래로 내려갔는가" 하나로 본다.
  - 안 내려감: scan별 우도 P(변위 없음 | Y, 이전 모두 없음) = (1−π(d_k)) / (1−π(d_{k−1})). 곱하면 망원급수로 Λ = 1−π(d), 즉 λ = ln(1−π(d_lag)).
    d_lag는 τ = y_thr / v_lat 전까지의 최소 d (양보를 시작해도 임계값을 넘는 데 τ가 걸리므로, 지금 변위가 없다는 것은 τ 전까지 시작하지 않았다는 증거일 뿐이다).
  - 내려감: Λ = π(d_min) / p_fa, 흡수 상태 (이후 z = 0이어도 내려가지 않는다. d가 줄면 π가 커져 λ는 오르기만 한다).
  - π(d) = 0인 거리(d ≥ d_y_max)에서의 변위는 무시한다 (λ 불변, 기획안 핵심 관찰 유지).
- **이론적 근거**: Y 가설에서 "아직 양보 안 함"은 d_y < d라는 하나의 사건이고 확률은 1−π(d)다. original은 이것을 scan 수만큼 곱해 (독립성 가정 위반) 증거를 과다 계수하고 결과가 scan rate에 따라 달라진다. 조건부 생존확률로 쓰면 누적 우도가 정확히 1−π(d)가 되어 scan rate와 무관하다 (10 Hz와 40 Hz에서 b 차이 < 1e-9, 테스트로 고정).
- **양보 후 z = 0 문제**: "흡수 상태"를 골랐다. 이유: (1) 관측이 누적 변위라 scan rate와 무관하고 추적이 잠깐 끊겨도 변위를 잃지 않는다, (2) z를 재정의해 scan별 우도를 유지하면 차분 잡음의 상관을 따로 모델링해야 한다. 대가: 선행차가 복귀해도 belief는 내려가지 않는다 → PASS 중단 조건이 따로 필요하다 (제안, 미구현).
- **d_y 분포 가정**: d_y ~ U[d_y_min, d_y_max] = U[0.6, 1.2] m, 따라서 π(d) = P(d_y ≥ d)는 선형이다. 근거: 실험의 선행차(`leader_logic`)가 회차마다 바로 이 분포에서 d_y를 뽑으므로 모델과 생성 과정이 일치한다. 범위 0.6·1.2 m 자체는 기획안 가정값이며 사람 운전자나 다른 차량에 대한 근거는 없다 (미확인).
- **한계**: b ≤ 1−η 판정은 d_lag ≤ d_y_min + (1−η)(d_y_max−d_y_min) = 0.667 m에서 일어난다. d_y가 그보다 작은 Y 선행차(전체의 약 1−η)는 설계상 NY로 판정된다.

## 재실험 결과 (노이즈 없는 폐루프, 참값 관측, 10 Hz, Y 200회, 같은 d_y 표본)
`python test/aur_closed_loop_report.py`

| belief | 방법 | 성공 | belief 오판 | T_wait_a | T_max | 추적 끊김 | 기타 | d_y 0.6~0.7 | 0.7~0.8 | 0.8~0.9 | 0.9~1.0 | 1.0~1.1 | 1.1~1.2 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| original | c | 115/200 | 85 | 0 | 0 | 0 | 0 | 0/24 | 0/34 | 8/35 | 40/40 | 29/29 | 38/38 |
| original | d | 115/200 | 85 | 0 | 0 | 0 | 0 | 0/24 | 0/34 | 8/35 | 40/40 | 29/29 | 38/38 |
| survival | c | 188/200 | 12 | 0 | 0 | 0 | 0 | 12/24 | 34/34 | 35/35 | 40/40 | 29/29 | 38/38 |
| survival | d | 188/200 | 12 | 0 | 0 | 0 | 0 | 12/24 | 34/34 | 35/35 | 40/40 | 29/29 | 38/38 |

c와 d가 같은 것은 이 테스트가 참값 관측·무잡음이라 접근 속도만 다르고 판단 경로가 같기 때문이다 (차이는 h·AEB에서 난다). 추적 끊김 0은 tracker를 거치지 않기 때문이다.

## 안전 확인 (보고만, 코드 수정 없음)
- **PASS 중단 조건: 없다.** Y로 고정된 뒤 선행차가 복귀해도 PASS·MERGE를 계속한다. 제안: PASS에서 앞뒤로 겹치기 전(d > 0)에 `lateral_pass_ok`가 거짓이 되거나 선행차 y_B가 양보 감지 시점보다 y_thr 넘게 왼쪽으로 돌아오면 ABORT → y_ref = 0, 방법별 추종 속도, FOLLOW로 종료. 이미 겹친 뒤(d ≤ 0)에는 감속하면 더 오래 나란히 있게 되므로 차선 유지 + v_pass로 빠져나간다.
- **급정지 때 h < 0: 이산화 때문이다.** 방법 d·NY, 같은 시나리오의 급정지 후 최소 h: 10 Hz −0.063, 20 Hz −0.010, 50 Hz +0.019, 200 Hz +0.039 (ε = 0.05로 수렴). 최종 간격은 모두 0.24~0.25 m > m. 실차는 scan 10 Hz이므로 보정 실험의 t_d에 scan 주기가 포함되게 재야 한다.
- **min_point_x −1.0, 곡선**: 이 브랜치에는 곡선 시나리오가 없어 합성 원호 통로(W = 1.0, R = 1.0/1.5/3.0 m)로 확인했다. 중앙 주행 시 ego 횡위치 오차가 −0.05에서 −0.065/−0.047/−0.022 m, −1.0에서 −0.011/−0.008/−0.004 m로 −1.0이 덜 치우친다. 다만 −0.05의 치우침은 곡선 안쪽으로 미리 꺾게 하는 방향이라, 곡선 주행에는 −1.0이 선회 시작을 늦출 수 있다 (실험 통로는 직선).

## 다음 세션이 알아야 할 결정사항
1. 작업 위치는 worktree `f1tenth_gym_ros/my_algo_aur`. 원래 체크아웃 `my_algo`(sim/mujoco, 미커밋 변경 있음)는 건드리지 않는다. 테스트는 Python 3.8 환경으로 `python -m pytest test/ -q` (Mac 기본 python에는 pytest 없음).
2. 함수는 파라미터 객체 `p`(AurParams)를 마지막 인자로 받는다. 조정은 `config/exp_common.yaml`(TUNE 태그)에서 하고 `params.py` 기본값과 같아야 한다 (테스트). 값·태그를 바꾸면 `python3 scripts/gen_tuning_doc.py`로 `docs/TUNING.md`를 다시 만든다 (테스트가 최신 여부를 확인).
3. 판단은 APPROACH·PROBE 모두에서 하고 처음 넘은 쪽으로 고정(`fsm.decided`). PROBE 진입은 `d ≤ d_ref + probe_tol`. T_wait_a·T_max는 PROBE 진입부터 센다.
4. 최소 속도 hold는 히스테리시스(켜짐 v_E > 0.35, 꺼짐 < 0.25, 임시값)로 달리는 중에만 한다. `hold_count`는 분기 진입 횟수.
5. PASS에서 선행차와 중심 간격이 w + c가 되기 전에는 방법별 추종 속도를 쓴다.
6. tracker는 헤딩이 통로와 나란하다고 가정한다. 측면 추적의 y_B는 보이는 면 중앙이라 최대 w/2 치우친다.
7. `exp_planner`가 매 scan에 부를 순서: `tracker.update(points, t, v_E, corridor.ego_lateral, W)` → `corridor.update(points, track.idx)` → `fsm.step(Obs(...))`. PASS·MERGE에서는 `update_side`. 2단계의 "차량이 아닌 정지 물체 → STOP"은 아직 없다.
8. 기획안 PDF와 지시서는 커밋하지 않는다 (`.gitignore`: `CLAUDE_TASK.md`, `docs/*.pdf`, `docs/CLAUDE_TASK*.md`).
9. exp/occlusion-speed 자산(`scripts/bag_player.py`, `analysis/bag_io.py`, `scripts/env_ego.sh`)은 쓰는 단계에서만 `git checkout origin/exp/occlusion-speed -- <path>`로 가져온다. 아직 안 가져왔다.
