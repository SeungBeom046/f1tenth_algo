# ForzaETH race_stack 추월 코드 검토와 반영

[ForzaETH/race_stack](https://github.com/ForzaETH/race_stack) (2026-11-18 main, `88da6c2` 기준)의 추월 관련 코드를 읽고 이 연구(exp/aur-overtake)에 옮길 수 있는 것을 정리했다 (2026-11-19).
`leader_pred_time`은 기본 off, `go_hold_s`는 시뮬 결과를 보고 기본 0.3 s로 켰다 (2026-11-19). 둘 다 실차 미확인.

## 1. race_stack 추월 구조

| 노드 | 파일 | 하는 일 |
|---|---|---|
| perception | `perception/perception/detect.py`, `tracking.py` | 2D LaserScan 클러스터링 → 상대 차 칼만 추적 → Frenet (s, d, vs, vd) |
| spline planner | `planner/local_planners/spline_planner/spline_planner.py` | 20 Hz. 레이싱 라인에서 \|d\| < 0.3 m, 앞 10 m 안 상대 차만 본다. 상대 차를 `fixed_pred_time` 0.15 s 앞으로 예측 (`_predict_obs_movement`, 기본 constant). 트랙 경계 여유로 좌/우 결정 (`_more_space`, 회피 거리 0.65 m, 경계 여유 0.2 m). apex 앞뒤 7점 3차 스플라인 (속도에 따라 1~1.5배, 바깥쪽이면 1.75배). 안쪽 추월은 속도 0.9배. 경로가 경계에 가깝거나 라인 밖에서 반대쪽으로 바꿔야 하면 경로 폐기 |
| state machine | `state_machine/state_machine/state_machine.py`, `transitions.py` | GB_TRACK / TRAILING / OVERTAKE / FTGONLY. OVERTAKE 조건 = 추월 구간 안 + 유효 스플라인 + 방향 전환 뒤 `splini_hyst_timer_sec` 경과 + 추월 경로 위 장애물 없음(`_check_ofree`) |
| controller | `controller/base_controllers/controller/pp.py` `trailing_controller` | TRAILING에서 상대 차와 간격을 PID로 유지 |

전제: 전역 레이싱 라인, Frenet 변환, 지도 기반 위치 추정, 여러 바퀴 주행.

## 2. 이 연구와의 대응

| race_stack | 이 연구 | 판단 |
|---|---|---|
| GB_TRACK / TRAILING / OVERTAKE | APPROACH·PROBE / FOLLOW / PASS·MERGE | 이미 있다 |
| 상대 차 예측 (`_predict_obs_movement`) | 횡 여유·중단 판정은 현재 y_B만 쓴다 | **옮김**: `leader_pred_time` (아래 3.1) |
| 스플라인 유효 유지 시간 (`splini_hyst_timer_sec`) | go 조건 한 scan으로 PASS | **옮김**: `go_hold_s` (아래 3.2) |
| `_check_ofree` (추월 경로 위 장애물) | `lateral_pass_ok`, PASS 중단 판정 | 이미 있다 (선행차 1대, 통로 직선) |
| 경계 여유로 경로 폐기 (`spline_bound_mindist`) | δ = W/2 − w/2 − c가 벽 여유 c를 보장 | 이미 있다 |
| 좌/우 선택 (`_more_space`) | 왼쪽으로만 추월 (연구 가정) | 안 옮김 |
| apex 스플라인 경로 | 뒤에서 차선을 먼저 바꾸고 옆으로 어긋난 뒤 가속 (AEB·벽 때문에 정한 설계) | 안 옮김: apex 경로는 감속 없이 지나가는 레이싱용이라 설계와 충돌 |
| 안쪽 추월 0.9배 속도 | 직선 통로 | 해당 없음 |
| TRAILING PID 간격 제어 | 식 8 v_lim (급정지에도 m 확보) | 안 옮김: 이 연구의 안전 거리 식이 더 엄격하다 |
| FTGONLY (오래 막히면 gap follow) | gap follow 안 씀 | 안 옮김 |
| Frenet·레이싱 라인 | 지도 없이 벽 직선으로 통로 좌표 | 안 옮김 (연구 가정: 지도·TF 없음) |

## 3. 옮긴 기능

### 3.1 선행차 횡위치 예측 `leader_pred_time` (기본 0 = 끔)

- v_lat = 최근 `leader_vlat_window`(5)개 유효 y_B의 최소제곱 기울기 (`EgoFSM._update_leader_vlat`).
- 횡 여유(go, PASS 중단)와 PASS 복귀 판정에 y_B + max(0, v_lat)·t_pred (y_L도 같은 이동)를 쓴다 (`EgoFSM._lat_shift`).
- race_stack은 s·d 양쪽을 예측하지만 여기서는 **왼쪽(추월 차선 쪽) 움직임만** 더한다. 비키는 움직임으로 여유를 앞당겨 쓰지 않으므로 추월을 늦추거나 중단을 앞당길 뿐이다.
- PASS 진입 때 기울기 이력을 비운다 (정면 → 측면 추적으로 y_B 기준점이 튈 수 있다).
- `/exp/state`에 `leader_vlat` 기록.

### 3.2 추월 조건 유지 시간 `go_hold_s` (기본 0.3 s, 0 = 끔)

- go 조건(belief Y 확정 + 횡 여유, method a는 대기 규칙)이 이 시간 동안 끊김 없이 맞아야 PASS. 한 scan이라도 깨지면 다시 센다.
- 노린 것: 횡 여유 경계 근처에서 y_B 잡음으로 PASS에 들어갔다 바로 중단하는 경우.

## 4. 시뮬 결과

[PROGRESS.md](PROGRESS.md)의 2026-11-19 절. 요약: `go_hold_s` 0.3 s는 잡음 조건 Y에서 가짜 추월 중단을 절반으로 줄였고(45회 16 → 8) 안전 지표는 그대로다. `leader_pred_time` 0.2 s는 잡음에서 가짜 중단을 늘려 권하지 않는다.

## 5. 검토 (2026-11-19)

- 두 기능의 코드는 설명대로 동작한다. 결함은 찾지 못했다.
- 실차 조건(20 cm 가드레일, 라이다 거리 잡음 0.02 m)의 시뮬에서는 이식 전후 모두 추월이 거의 되지 않는다. 원인은 이식 기능이 아니라 벽 선 헤딩 잡음과 추월 중단 기준의 잡음 민감도다. 숫자와 시도한 것은 [PROGRESS.md](PROGRESS.md)의 같은 날짜 절.
- race_stack은 지도 위치 추정(헤딩 잡음이 작다)을 전제로 하므로, 이 연구에서는 헤딩 추정을 먼저 잡음에 강하게 만들어야 옮긴 기능의 효과를 실차에서 볼 수 있다.
