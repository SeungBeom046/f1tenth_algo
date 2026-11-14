# exp/aur-overtake 진행 상황

다음 세션은 `CLAUDE_TASK.md`(커밋하지 않음, 사용자가 직접 저장)와 이 파일만 읽고 시작한다.

## 끝낸 단계
- 세션 1 (2026-11-14): 0단계(점검), 1단계(`my_algo/aur/` 순수 모듈 + 테스트).
- 검토 반영 (2026-11-14): belief survival 모델, 폐루프 재실험, hold 히스테리시스, TUNE 태그·`config/exp_common.yaml`·`docs/TUNING.md`.
- MuJoCo 추월 시뮬 `sim/mujoco_overtake/` (2026-11-14): 헤드리스 일괄 실행, 뷰어, 선행차 Y/NY/YR, yaw 강건성 테스트. 횡 제어 순수 모듈 `my_algo/aur/lateral.py`(지시서 2단계의 순수 부분)를 앞당겨 만들었다.
- 커밋은 `git log --oneline` 참고. push는 아직 안 했다.

## 남은 일
- 지시서 세션 2: 4단계(속도 소스), 5단계(AEB relative). 이후 지시서 표대로.
- `setup.py`에 `config/*.yaml` 설치 추가는 7단계에서 한다 (지금은 소스 경로로만 읽는다).
- 승인 대기 (제안만 한 것, 미구현): PASS 중단 조건 (아래 "안전 확인"), yaw에 강한 선행차 위치 추정 (아래 "강건성 테스트").
- 지시서 9단계(`sim/aur/`, F1TENTH gym)는 `sim/mujoco_overtake/`로 대신할지 사용자 확인 필요.

## 기획안 변경

### belief 우도 (식 2~5) → survival 모델. `belief_model: original | survival` (기본 survival)
- **변경 전** (original, 기획안 식): scan마다 독립으로 곱한다.
  λ_k = λ_{k−1} + ln[ π(d_k)·N(z_k; −v_lat·Δt, σ²) + (1−π(d_k))·N(z_k; 0, σ²) ] − ln N(z_k; 0, σ²)
- **변경 후** (survival): 관측을 "누적 횡변위 s = y_B − y_B(t₀)가 −y_thr 아래로 내려갔는가" 하나로 본다.
  - 안 내려감: scan별 우도 P(변위 없음 | Y, 이전 모두 없음) = (1−π(d_k)) / (1−π(d_{k−1})). 곱하면 망원급수로 Λ = 1−π(d), 즉 λ = ln(1−π(d_lag)).
    d_lag는 τ = y_thr / v_lat 전까지의 최소 d (양보를 시작해도 임계값을 넘는 데 τ가 걸리므로, 지금 변위가 없다는 것은 τ 전까지 시작하지 않았다는 증거일 뿐이다).
    (추가) τ = t_react + y_thr / v_lat. t_react는 선행차가 양보를 시작한 뒤 횡변위가 보이기까지의 지연이다. MuJoCo 시뮬의 선행차는 약 1.4 s가 걸려서 t_react = 0이면 전부 NY로 오판한다. 임시값 1.3 s, 실차 측정으로 대체.
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

위 표는 t_react = 0일 때다. t_react = 1.3 s(현재 기본값)로는 survival c·d 모두 200/200 (이 테스트의 선행차는 지연 없이 0.4 m/s로 비키므로 기다리면 다 잡는다).

MuJoCo 시뮬 (`python sim/mujoco_overtake/batch.py belief`, W = 1.0, 같은 d_y 표본, t_react = 1.3 s):

| belief | 방법 | 성공 | belief 오판 | T_wait_a | T_max | 추적 끊김 | 충돌 | 기타 | d_y 0.6~0.7 | 0.7~0.8 | 0.8~0.9 | 0.9~1.0 | 1.0~1.1 | 1.1~1.2 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| original | c | 0/200 | 200 | 0 | 0 | 0 | 0 | 0 | 0/24 | 0/34 | 0/35 | 0/40 | 0/29 | 0/38 |
| original | d | 0/200 | 200 | 0 | 0 | 0 | 0 | 0 | 0/24 | 0/34 | 0/35 | 0/40 | 0/29 | 0/38 |
| survival | c | 164/200 | 36 | 0 | 0 | 0 | 0 | 0 | 0/24 | 22/34 | 35/35 | 40/40 | 29/29 | 38/38 |
| survival | d | 176/200 | 24 | 0 | 0 | 0 | 0 | 0 | 0/24 | 34/34 | 35/35 | 40/40 | 29/29 | 38/38 |

MuJoCo에서 original이 0/200인 것은 선행차 횡 응답이 느려(양보 시작 → 변위 0.08 m까지 약 1.4 s) 그 사이 z = 0이 NY 증거로 쌓이기 때문이다.

c와 d가 같은 것은 이 테스트가 참값 관측·무잡음이라 접근 속도만 다르고 판단 경로가 같기 때문이다 (차이는 h·AEB에서 난다). 추적 끊김 0은 tracker를 거치지 않기 때문이다.

## 안전 확인 (보고만, 코드 수정 없음)
- **PASS 중단 조건: 없다.** Y로 고정된 뒤 선행차가 복귀해도 PASS·MERGE를 계속한다. 제안: PASS에서 앞뒤로 겹치기 전(d > 0)에 `lateral_pass_ok`가 거짓이 되거나 선행차 y_B가 양보 감지 시점보다 y_thr 넘게 왼쪽으로 돌아오면 ABORT → y_ref = 0, 방법별 추종 속도, FOLLOW로 종료. 이미 겹친 뒤(d ≤ 0)에는 감속하면 더 오래 나란히 있게 되므로 차선 유지 + v_pass로 빠져나간다.
- **급정지 때 h < 0: 이산화 때문이다.** 방법 d·NY, 같은 시나리오의 급정지 후 최소 h: 10 Hz −0.063, 20 Hz −0.010, 50 Hz +0.019, 200 Hz +0.039 (ε = 0.05로 수렴). 최종 간격은 모두 0.24~0.25 m > m. 실차는 scan 10 Hz이므로 보정 실험의 t_d에 scan 주기가 포함되게 재야 한다.
- **min_point_x −1.0, 곡선**: 이 브랜치에는 곡선 시나리오가 없어 합성 원호 통로(W = 1.0, R = 1.0/1.5/3.0 m)로 확인했다. 중앙 주행 시 ego 횡위치 오차가 −0.05에서 −0.065/−0.047/−0.022 m, −1.0에서 −0.011/−0.008/−0.004 m로 −1.0이 덜 치우친다. 다만 −0.05의 치우침은 곡선 안쪽으로 미리 꺾게 하는 방향이라, 곡선 주행에는 −1.0이 선회 시작을 늦출 수 있다 (실험 통로는 직선).

## MuJoCo 시뮬에서 찾아 고친 것 (지시서와 다른 점)
- **횡 제어에 헤딩 감쇠 추가** (`heading_gain` 0.7, 0이면 지시서 방식). 횡위치 P 제어만으로는 선행차가 양보하다 벽에 닿았다. 헤딩은 `corridor`가 벽 점 직선 맞춤으로 구한다.
- **`steering_deadband` 0.055 → 0.02**. 0.055 rad는 횡오차 0.09 m에 해당해 벽 여유 c = 0.10 m와 맞지 않는다 (선행차 중심이 −0.349까지 넘어감, 0.02에서는 −0.293).
- **v_B 추정 필터의 상태를 ḋ에서 v_B로 바꿈**. ḋ를 상태로 두면 ego가 가감속할 때 v_B = v_E + ḋ가 튀어 (0.9 → 1.45 m/s) 급정지 때 추돌했다.
- **시뮬에서는 t_d = 0.35 s** (구동 지연 0.15 + 1차 지연 0.10 + scan 주기 0.10, `scenario.yaml`의 `params`). 기획안 값 0.15로는 방법 d도 NY 급정지에서 추돌한다. 실차 t_d도 scan 주기와 구동 응답을 포함해 재야 한다.
- 방법 × 선행차 5회씩 (`batch.py matrix`): d는 Y 4/5 성공(실패 1회는 d_y < 0.667), NY 충돌 0, 최소 간격 0.19 m. c는 NY 5회 중 3회 추돌 (AEB 없는 시뮬). a는 Y 3/5, b는 0/5 (T_max).

## 강건성 테스트: 선행차 양보 yaw (`batch.py yaw`, 방법 d, yaw 고정·횡속도 0.4 m/s)
| yaw | 끊긴 scan / 필요 scan | 최장 끊김 | 클러스터 폭 최대 | 결과 |
|---|---|---|---|---|
| 0° | 1 / 32~41 | 0.1 s | 0.27 m | DONE 3/3 |
| 10° | 1 / 32~44 | 0.1 s | 0.34 m | DONE 3/3 |
| 20° | 1 / 32~44 | 0.1 s | 0.42 m | DONE 2/3, 차량 충돌 1 |
| 25° | 1~3 / 32~43 | 0.2 s | 0.45 m (게이트) | 차량 충돌 3/3 |
| 30° | 6~7 / 35~45 | 0.3 s | 0.45 m (게이트) | 차량 충돌 3/3 |
| 40° | 8 / 35~45 | 0.3 s | 0.45 m (게이트) | 차량 충돌 3/3 |

- 추적 자체는 25°부터 폭 게이트(0.45 m)에 걸려 끊기지만 0.3 s 이하라 `track_hold`(0.5 s)가 메운다.
- 더 큰 문제는 **y_B 편향**이다. 선행차가 틀면 뒷면 + 옆면이 같이 보여 범위 중앙이 실제 뒷면 중심보다 오른쪽으로 간다. ego가 선행차가 실제보다 많이 비켰다고 보고 일찍 가속해 선행차 뒤 왼쪽 모서리에 닿는다.
- 선행차가 자기 횡 제어기로 비킬 때의 yaw는 약 10°여서 지금 설정에서는 문제가 없다 (실차 yaw는 미확인).
- 제안 (미구현): (1) 횡 여유 판정(`lateral_pass_ok`, PASS 가속 조건)에 중심 대신 클러스터의 가장 왼쪽 점을 선행차 왼쪽 끝으로 쓴다 (작은 변경, 보수적). (2) L자/OBB 피팅으로 뒷면 중심과 yaw를 구하고 폭 게이트를 맞춘 사각형 크기로 판정한다.

## 예전 시뮬레이션 코드 (조사만, 옮기지 않음)
이 브랜치(refactor/main-baseline 기반)에는 시뮬 코드가 없다. 전부 `sim/mujoco` 브랜치에 있고 그 브랜치는 수정 금지라 `git mv`하지 않았다. 새 환경과는 브랜치로 이미 분리되어 있다.
- `sim/runner.py`(540줄): 관측 공백 실험(B0/B1/B2/P, S1~S4) 회차 실행, ROS 노드를 `rosstub`으로 돌림
- `sim/world.py`, `sim/vehicle.py`, `sim/lidar.py`: MJCF 생성, 운동학 차량, Mid-360 근사 라이다
- `sim/viewer_ui.py`: 뷰어 오버레이·키 / `sim/video.py`: mp4 / `sim/report.py`, `sim/sweep.py`, `sim/clean_logs.py`: 리포트·일괄 실행·로그 정리
- `sim/simconfig.py`, `sim/config/sim.yaml`, `sim/config/report_criteria.yaml`, `sim/popout_timing.py`, `sim/rosstub.py`, `sim/check_env.py`, `sim/setup_env.sh`, `sim/requirements.txt`, `sim/tests/*`
- 시뮬 밖: `analysis/sim_reader.py`, `my_algo/track_layout.py`, `test/_track.py`, `test/test_track_layout.py`, `docs/SIMULATION.md`
- 모델 xml 파일과 시뮬용 launch는 없다 (MJCF는 `world.py`가 문자열로 만든다).
- 새 환경으로 가져온 것: `lidar.py`, `vehicle.py` (내용 그대로, 파일 머리에 출처 표시). 키 양식은 `viewer_ui.py`에서 가져왔다.
- 원래 체크아웃 `my_algo`의 미커밋 변경(`.gitignore`, `config/imu.yaml`)은 시뮬 파일이 아니며 건드리지 않았다.

## 다음 세션이 알아야 할 결정사항
1. 작업 위치는 worktree `f1tenth_gym_ros/my_algo_aur`. 원래 체크아웃 `my_algo`(sim/mujoco, 미커밋 변경 있음)는 건드리지 않는다. 테스트는 Python 3.8 환경으로 `python -m pytest test/ -q` (Mac 기본 python에는 pytest 없음). 시뮬은 Python 3.12 + mujoco 3.12.0 (`../my_algo/sim/.venv`를 읽기 전용으로 썼고 거기에는 pytest가 없다).
2. 함수는 파라미터 객체 `p`(AurParams)를 마지막 인자로 받는다. 조정은 `config/exp_common.yaml`(TUNE 태그)에서 하고 `params.py` 기본값과 같아야 한다 (테스트). 값·태그를 바꾸면 `python3 scripts/gen_tuning_doc.py`로 `docs/TUNING.md`를 다시 만든다 (테스트가 최신 여부를 확인).
3. 판단은 APPROACH·PROBE 모두에서 하고 처음 넘은 쪽으로 고정(`fsm.decided`). PROBE 진입은 `d ≤ d_ref + probe_tol`. T_wait_a·T_max는 PROBE 진입부터 센다.
4. 최소 속도 hold는 히스테리시스(켜짐 v_E > 0.35, 꺼짐 < 0.25, 임시값)로 달리는 중에만 한다. `hold_count`는 분기 진입 횟수.
5. PASS에서 선행차와 중심 간격이 w + c가 되기 전에는 방법별 추종 속도를 쓴다.
6. tracker는 헤딩이 통로와 나란하다고 가정한다. 측면 추적의 y_B는 보이는 면 중앙이라 최대 w/2 치우친다.
7. `exp_planner`가 매 scan에 부를 순서는 `sim/mujoco_overtake/sim.py`의 `Car.perceive` / `Sim.frame`과 같다: `tracker.update(points, t, v_E, corridor.ego_lateral, W)` → `corridor.update(points, track.idx)` → `fsm.step(Obs(...))` → `lateral.step(ego_lateral, y_ref, dt, corridor.heading)`. PASS·MERGE에서는 `update_side`. 선행차는 후방 추적이 끊기면 `update_side`로 옆에 온 후행차 점을 벽 계산에서 뺀다. 2단계의 "차량이 아닌 정지 물체 → STOP"은 아직 없다.
8. 기획안 PDF와 지시서는 커밋하지 않는다 (`.gitignore`: `CLAUDE_TASK.md`, `docs/*.pdf`, `docs/CLAUDE_TASK*.md`).
9. exp/occlusion-speed 자산(`scripts/bag_player.py`, `analysis/bag_io.py`, `scripts/env_ego.sh`)은 쓰는 단계에서만 `git checkout origin/exp/occlusion-speed -- <path>`로 가져온다. 아직 안 가져왔다.
