# 실험 계획 (실행판)

**가려진 곳에서 튀어나오는 보행자에 대비한 '틈 인지' 관측 공백 속도 계획**
(Gap-Aware Phantom Placement for Occlusion-Aware Speed Planning — 1/10 Scale Real-Vehicle Validation)

원 계획서를 실제 코드와 명령으로 옮긴 문서다. 원 계획서와 달라진 점은 [8절 변경 사항](#8-계획서-이후-변경-사항)에 이유와 함께 모았다.
실행 절차는 [RUNBOOK.md](RUNBOOK.md), 로그 구조는 [LOGGING.md](LOGGING.md), 미확정 값은 [OPEN_ITEMS.md](OPEN_ITEMS.md).
"TBD(세션 1)"는 차량 세션 1 결과로 확정할 값이다.

## 1. 연구 질문, 가설, 판정

| RQ | 가설 | 판정 지표 | 실험 · 그림 |
|---|---|---|---|
| RQ1 공백 대응이 AEB만보다 돌출 상황에서 얼마나 안전한가 | H1: B1·B2·P 모두 B0보다 무충돌 비율과 최소 TTC가 높다. 차이는 d_e = 1.0 m에서 가장 크다 | S1 무충돌 비율, 최소 TTC, 최소 거리, 충돌 시 속도 (d_e별) | Exp3, 그림 6 y축 |
| RQ2 어떤 공백을 위험으로 보는가가 효율을 얼마나 바꾸나 | H2: P는 B2와 비슷한 안전성을 보이면서 S3(통과 불가 틈)에서 불필요 감속이 더 적다 | S3 명령 기준 ΔT (P < B2), S1 무충돌 비율 (P ≈ B2) | Exp2·Exp3, 그림 5, 그림 6 |
| RQ3 공백 면적 기반 감속(B1)이 위험과 맞아떨어지나 | H3: B1은 S4(위험 없는 긴 벽)에서 과잉 감속, S1(면적 작은 틈)에서 감속이 부족할 수 있다 | S4 ΔT (B1 > 다른 방법), S1 최소 TTC (B1 < B2·P) | Exp2·Exp3 |
| RQ4 실제 LiDAR 가림 경계는 이상적인 가림과 얼마나 다른가 | H4: 비반복 스캔·10 Hz 때문에 경계가 프레임마다 흔들리고 N 프레임 누적이 필요하다 | 프레임별 경계 위치 표준편차, phantom 생성·소멸 횟수 (N별, 속도별) | Exp1, RQ4 그림 |

H2 성립 조건: 그림 6에서 P가 B2와 같은 높이(무충돌 비율 95% CI 겹침)에서 왼쪽(S3+S4 ΔT 작음)에 있다.

## 2. 방법

| 코드 | 규칙 | 구현 |
|---|---|---|
| B0 | 공백 무시, v_ref 유지. AEB만 | `phantom.B0` |
| B1 | 전방 ROI(앞 끝부터 3 m, 반폭 = 차선 반폭 + v_popout·(t_r + v_ref/a_b), 기본 약 1.0 m) 미관측 칸 비율 ρ로 v = v_ref(1 − kρ) | `phantom.B1` + `speed_policy.b1_speed` |
| B2 | ROI 안 모든 가림 경계 칸에 phantom, 최근접 d로 v_cap(d) | `phantom.B2` |
| P | 측면 주차열(차체 앞 끝보다 앞쪽, 차선 쪽 면 0.10 m 띠의 occupied 군집) 사이 틈 폭 w_min–w_max 입구와 열 양 끝에만 phantom. w_max보다 넓은 빈 공간은 열 끝 | `phantom.P` |

공통 phantom 유지 (B2·P 같은 규칙, `phantom.PhantomHold`): 한 번 생긴 phantom은 odom 좌표에 고정해 차체 앞 끝이 그 x + 0.10 m를
지날 때까지 유지한다 (`phantom_hold`, `phantom_hold_margin` [EXP-FIXED]). 방법별 차이는 배치 규칙에만 있다.

공통: v_cap(d) = a_b(−t_r + √(t_r² + 2(d − s_m)/a_b + v_imp²/a_b²)), d ≤ s_m이면 v_cap(s_m).
t_r은 **관측 → 실제 감속 시작까지 전체 지연**(인지 + 통신 + 제어 + 구동, 속도 제어 1차 지연은 등가 지연으로 포함)이다 (Exp0 c·i). 명령 = min(v_ref, v_cap) →
최소 속도 정책(hold) → AEB override → VESC. d는 **차체 앞 끝**에서 phantom의 경로상 진입 지점(phantom의 base_link x)까지
종방향 거리이고, 앞 끝보다 뒤의 phantom은 무시한다.

## 3. 시나리오와 변인

트랙: 직선 약 9.0 × 2.5 m. 가속 구간(0 → 2.0 m/s) → 시험 구간 → 정지·여유 구간 → 폼 방호벽. 좌측 연속 벽 패널,
우측 주차 박스 P1–P3 (0.55 × 0.30 × 0.35(H) m). 차선 폭 0.7 m. 바닥 0.5 m 눈금, 천장 카메라 60 fps.
트랙 좌표(앞 끝 시작점 = 0)는 `config/scenario_*.yaml` — 트랙 마킹 후 실측해 교체 (TBD(세션 1) 이후).

| 코드 | 배치 | 돌출 | 보려는 것 | 회차 |
|---|---|---|---|---|
| S1 | 틈 0.45 m | #2, d_e = 1.0 / 1.5 / 2.0 m | 안전 (RQ1) | 4방법 × 3시점 × 10 − 5 (B0·d_e 1.0은 5회) = 115 |
| S2 | 틈 0.45 m | 없음 | 위험 구간의 정당한 감속 비용 | 4 × 10 = 40 |
| S3 | 틈 0.15 m | 없음 | B2 대비 P의 불필요 감속 (RQ2) | 40 |
| S4 | 우측도 연속 벽 | 없음 | B1 과잉 감속 (RQ3) | 40 |
| E0 | 빈 트랙 | 없음 | Exp0 플랫폼 시험 (급제동) | Exp0 |

d_e 정의: #2 앞부분이 차선 경계에 닿는 순간, 에고 차체 앞 끝과 충돌 지점 사이 거리 (원 계획서에 정확한 정의가 없어 이렇게 정함).
트리거 거리 = 충돌 지점 x − d_e − v_ref × (트리거 지연 + #2 진입 시간). 실제 d_e는 천장 카메라로 사후 계산해 그 값으로 분석한다.

| 변인 | 내용 | 설정 위치 |
|---|---|---|
| 독립 [EXP-VAR] | 방법, 시나리오, d_e (S1) | bringup `method:= scenario:= d_e:=` |
| 통제 [EXP-FIXED] | v_ref 2.0, t_r, a_b, s_m, AEB 임계값, 높이 대역, 격자, 조향 게인·좌측 벽 목표, 최소 속도 정책(hold), deskew, #2 속도, 데드맨·정지선, 박스·바닥·타이어·조명, 시작 배터리 전압 기준 | `config/common.yaml`, `popout.yaml`, `vehicle_geometry.yaml` |
| 민감도 [EXP-TUNE] | v_imp, w_min, N·히스테리시스, k, B1 ROI, row_face_tol | bringup 개별 인자 (7절) |
| 종속 | 안전(충돌, 충돌 시 속도, 최소 거리, 최소 TTC), 효율(4절), 승차감(jerk RMS), RQ4(경계 표준편차, 깜빡임), AEB 오작동 수 | `analysis/` |

## 4. 지표 정의 (`my_algo/metrics.py`)

- **절대 손실 ΔT = ∫(1/v − 1/v_ref) ds** [s], 시험 구간(`test_x_start`–`test_x_end`)에서. 주 지표.
  - **명령 기준**: v = `/exp/v_raw` = min(v_ref, v_cap) (최소 속도 정책 전). 배치 규칙의 효과이므로 **논문 주장의 핵심 근거**.
  - **실측 기준**: v = odom 속도. 정책·플랫폼·제어 응답이 섞인 실제 비용.
  - 고정 창 손실 % (주차열 ±1 m)는 보조로만. 구간 길이에 희석되기 때문.
- **시나리오별로 따로 보고한다. S2/S3/S4 평균은 내지 않는다** (S2 = 정당한 감속, S3·S4 = 과잉 보수성).
- 그림 6 x축 = S3 + S4 명령 기준 ΔT (불필요 감속). S2는 옆 패널에 따로.
- 감속 구간 길이 = v < 0.95 v_ref인 구간 총 길이. 최저 속도 = 시험 구간 min v.
- jerk RMS = `/imu/linear_accel_base` x → **2차 Butterworth 5 Hz 저역통과(앞뒤 두 번, 위상 지연 없음)** → 미분 → RMS. 방법 간 상대 비교 전용.
- 충돌 속도: 충돌 회차의 실측 속도. 요약 표에 최대·평균과 **v_imp 이하 충돌 / 초과 충돌** 수를 따로 낸다 (`metrics.impact_class`).
  v_imp 이하 충돌은 설계가 허용한 결과, 초과 충돌은 v_cap 보장이 깨진 결과다.
- 최소 거리·충돌 = 차체 footprint 다각형 기준. 1순위 천장 카메라(두 차 ArUco, 바닥 호모그래피), 보조 `/scan` 차선 안 점.
- 최소 TTC = 거리 / 접근 속도(상대). 에고 속도 기준 TTC(AEB와 같은 정의)도 함께 낸다.
- 통과시각 = 천장 카메라 1순위. 2.0 m/s·60 fps면 프레임당 약 3.3 cm라 선 전후 두 프레임 선형 보간으로 서브프레임 시각을 낸다.
- **폐기** (`metrics.discard_check`, 실차·시뮬 같은 함수, 모든 방법 공통): 가속 구간 끝(`accel_end_x`)까지 보낸 명령
  (`/exp/final_cmd`)이 계속 v_ref였는데 그 지점 실측 속도 < 0.95 v_ref (플랫폼 문제). 가속 구간 안에서 명령이 v_ref 아래로
  내려간 회차(알고리즘이 스스로 감속, 최소 속도 정책, AEB)는 폐기하지 않고 지표에 그대로 넣는다. 그 밖에 조이스틱 끊김,
  예상 외 발행자, 수동 사유. 폐기 수와 사유를 표에 함께 낸다.
- **AEB 오작동** (`metrics.aeb_malfunction`, 실차·시뮬 같은 함수): S2–S4에서 **정지선(`stop_distance`) 전에** AEB가 처음 발동한 회차.
  정지선 뒤 감속 중 방호벽에 가까워져 걸린 경우는 세지 않는다. 효율 통계에서 빼고, 뺀 수를 표에 내고, 방법별 오작동 수를 따로 집계한다.
- 통계: 조건별 평균과 95% CI (t 분포).

## 5. 실험 단계

명령의 `$S` = `$(ros2 pkg prefix my_algo)/share/my_algo/scripts`. bringup은 T2, 기록은 T3 (RUNBOOK 1절).

### Exp0 플랫폼 특성 — [EXP0-MEASURE] 전부 해소

목적: 가정값을 실측값으로 바꾼다. 완료 기준: OPEN_ITEMS 1절 표가 모두 실측값.

| 항목 | 트랙 · 방법 | 명령 | 회차 | 산출 → 바꿀 곳 |
|---|---|---|---|---|
| a. 장착값 | 평평한 바닥, 정지 | RUNBOOK 0.4, 부록 C (`imu_static_calib.py` 0°/180°, `measure_lidar_height.py`, 로고 방향, RViz) | 각 2 | ORIGIN_H, LIDAR_ROLL/PITCH/YAW → `tf_static.launch.py`. 자이로 바이어스 → `imu.yaml` |
| b. odom 스케일 | 빈 트랙 5 m 직선 표시 | `bringup ... method:=B0 scenario:=E0 run_id:=<n> test_v_ref:=0.9` (2.0은 인자 없이). 기록 `TEST_V_REF=0.9 bash $S/record_run.sh exp B0 E0 na <n>`. 정지 후 줄자 | 0.9 m/s × 3, 2.0 m/s × 3 | `analysis/odom_travel.py <runs> --tape ...` → 비율. **ERPM_GAIN 변경은 drive 속도도 바꾼다 (OPEN_ITEMS 2절)** |
| c. a_b, 정지거리 | 빈 트랙, 정지선 `stop_distance` 4.5 m (E0) | `bringup ... method:=B0 scenario:=E0 run_id:=<n>` + 기록 | 2.0 m/s × 10 | `analysis/exp0_braking.py ~/exp_logs/<날짜>` → a_b(CI 하한) → `common.yaml`. 계단 구동 지연 `t_r_step`·정지거리 등가 지연 `t_r_equiv`는 참고값 (계단 명령은 감속이 a_b로 바로 포화돼 1차 지연이 드러나지 않음) |
| i. 추종 지연 (t_r) | 빈 트랙 (E0) | `bringup ... method:=B0 scenario:=E0 run_id:=<n> test_step_v:=1.2 test_step_x:=2.5` + 기록 (`TEST_STEP_V=1.2 bash $S/record_run.sh exp B0 E0 na <n>`). 방법과 무관한 속도 계단 2.0 → 1.2 m/s (mux가 주행 거리 2.5 m에서 명령을 내림) | 5 | `analysis/exp0_braking.py ~/exp_logs/<날짜> --tracking <회차 폴더들>` → 계단 응답을 순수 지연 + 1차 지연 + 감속 한계 a_b 모델로 맞춘 등가 지연(`braking.step_lag`) CI 상한 + 인지 지연(라이다 주기 평균 50 ms + 격자 처리 ms) = **t_r** → `common.yaml`. 비교 실험 회차가 아니다 |
| d. 트리거 지연 | S1 배치, #2 대기(틈 안), 에고 정지 상태에서 트리거만 | RUNBOOK 0.3 동기 확인 명령을 20회 (`/popout/reset` 사이사이) | 20 | `popout_events.jsonl` latency_s 분포 → `trigger_latency` (중앙값), LED+천장 카메라로 교차 확인 |
| e. #2 단독 주행 | #2만, 틈 출발 표시 → 차선 경계 | #2: `ros2 launch my_algo car2.launch.py vesc_config:=... v_popout:=0.8`, 에고/노트북: `ros2 topic pub --once /trigger std_msgs/msg/Header "{frame_id: exp0}"` 후 `/popout/reset`. 0.9도 같은 방법 | 0.8 × 5, 0.9 × 5 | `reach_lane_s`(출발 → 차선 경계 도달), `speed_mean`, `speed_std`. 판정 아래 |
| f. #2 진입 시간 | e와 같음 | e의 `reach_lane_s` 평균 | (e) | `popout_enter_time` |
| g. AEB 단독 정지 (exp 모드) | 빈 트랙, 차선 중앙 x ≈ 5 m에 정지 박스 | `bringup ... method:=B0 scenario:=S2 run_id:=<n>` (S2 정지선이 박스 뒤가 되게 박스 위치 조정) | 3 | `meta.json` aeb.triggered = true, 충돌 없음, `first_cause_point`가 박스. **완료 기준 항목** |
| h. 기록 부하 | 정지 | T4 `ros2 topic hz /livox/lidar`, odom 을 기록 전·중 비교 | 1 | 주기 저하 없음 (수치 기록) |

**#2 속도 판정 (e)**: 0.8 m/s에서 5회 중 하나라도 출발 실패(정지·떨림), `speed_std > 0.05 m/s`, 또는 `reach_lane_s` 변동계수 > 10%면
불안정으로 보고 **0.9 m/s로 바꾼다** (`config/popout.yaml` `v_popout`, [EXP-FIXED], 모든 시나리오 공통) → 8절 변경 사항에 기록.
0.9도 불안정하면 실험 전 보고.

### Exp1 경계 안정성 (RQ4) — N 결정, deskew 결정

목적: 경계 흔들림을 재고 누적 프레임 수 N과 deskew 사용 여부를 정한다.
트랙: S2 배치(틈 0.45)와 S3 배치(틈 0.15).

| 조건 | 명령 | 회차 |
|---|---|---|
| 정지 (에고를 P1 앞 1, 2, 3 m에 세움) | `bringup ... method:=P scenario:=S2 run_id:=<n> n_frames:=<N> known_on:=<on> known_off:=1` 후 AUTO 없이 30 s 기록 | N ∈ {1, 2, 3, 5, 8} × 위치 3 |
| 0.9 m/s 통과 | 위 + `test_v_ref:=0.9` | N ∈ {1, 3, 5} × 3 |
| 2.0 m/s 통과 | 위 (v_ref 기본) | N ∈ {1, 3, 5} × 3 |
| 틈 폭 측정: 정지 대 2.0 m/s, deskew off/on | S2·S3 배치, `deskew:=false` / `deskew:=true` (점별 시각 필드 확인 후) | 각 3 |

(원 계획서의 저속 0.5 m/s는 플랫폼 최소 안정 속도 0.867 m/s 아래라 0.9 m/s로 바꿨다. 8절)
`known_on`은 N 이하로 (N = 1이면 1). 지표: `boundary_y_std_mean`, `phantom_on_off_toggles`(`extract_run.py`),
틈 폭은 `/exp/phantom_debug` gaps. **N 결정 규칙**: 2.0 m/s에서 경계 표준편차가 격자 2칸(0.10 m) 이하이고
phantom 깜빡임이 회차당 2회 이하인 가장 작은 N. **deskew 결정**: 2.0 m/s 틈 폭 오차(정지 대비)가 deskew로 절반 이상
줄면 on. 정한 값은 `common.yaml`에 넣고 이후 고정.
산출: RQ4 그림 (`plots.py`), 표.

### Exp2 효율 (S2–S4)

목적: RQ2, RQ3. 돌출 없이 4방법 × 10회 × 3시나리오 = 120회.
순서: `randomize_runs.py`의 S2 → S3 → S4 (블록마다 방법 셔플). 명령은 그 출력 그대로:
```bash
ros2 launch my_algo bringup.launch.py mode:=exp method:=<M> scenario:=<S> run_id:=<블록>
bash $S/record_run.sh exp <M> <S> na <블록>
```
지표: 명령·실측 ΔT, 최저 속도, 감속 구간, jerk, AEB 오작동 수. 산출: 그림 5 (S2, S3 위치별 속도), 요약 표.
완료 기준: 조건별 사용 회차 ≥ 8 (폐기·오작동 제외).

### Exp3 안전 (S1)

목적: RQ1. 4방법 × 3시점 × 10회, B0의 d_e = 1.0은 5회 (충돌 예상) = 115회. 폼 범퍼·방호벽 필수.
```bash
ros2 launch my_algo bringup.launch.py mode:=exp method:=<M> scenario:=S1 d_e:=<d_e> run_id:=<블록>
bash $S/record_run.sh exp <M> S1 <d_e> <블록>
```
#2는 회차마다 출발 표시에 두고 `/popout/reset`. 천장 카메라로 실제 d_e 계산 (`ceiling_camera.py`).
지표: 무충돌 비율, 충돌 시 속도, 최소 거리, 최소 TTC (d_e별). 산출: 그림 6 (x: Exp2의 S3+S4 ΔT, y: S1 무충돌 비율).
완료 기준: 조건별 사용 회차 ≥ 8 (B0·1.0은 ≥ 4).

### 분석

```bash
python3 analysis/extract_run.py --all ~/exp_logs/<날짜>        # 날짜마다
python3 analysis/ceiling_camera.py --video ... --calib ... --run-dir ...   # S1 회차
python3 analysis/summarize.py ~/exp_logs/<날짜1> ~/exp_logs/<날짜2> ...
python3 analysis/plots.py ~/exp_logs/<날짜1> ...
```

## 6. 회차 순서와 체크리스트

- `python3 $S/randomize_runs.py --seed <정수>`: 시나리오 순서는 트랙 재배치를 줄이려 S1 → S2 → S3 → S4로 고정하고,
  블록(= 한 조건에서 4방법 1회씩, S1은 3시점 × 4방법) 안에서 순서를 섞는다. 결과는 `session.json`.
- 회차 체크리스트: [RUNBOOK.md 2절](RUNBOOK.md#2-회차-진행-체크리스트) (배터리 전압, 트랙 마킹, 박스 위치, 폼 범퍼, 물리 전원 차단 위치,
  IMU 재보정, 발행자 점검, LB로 AUTO 진입).
- 폐기·AEB 오작동 회차는 같은 조건을 블록 끝에 다시 한다.

## 7. 민감도 분석

본 실험 값(기본)은 고정하고, 한 번에 하나만 바꾼다. 먼저 Exp2·Exp3 bag으로 **오프라인 재생**(`offline_replay.sh`, 개루프)으로
명령 차이를 보고, 차이가 큰 값만 실차 5회씩 추가한다.

| 파라미터 | 기본 | 탐색 | 대상 방법 · 시나리오 | 인자 |
|---|---|---|---|---|
| v_imp | 0.867 (= V_MIN, 하한) | 0.867, 1.0, 1.2 | B2, P · S1(d_e 1.5), S3 | `v_imp:=` |
| w_min | 0.35 | 0.25, 0.35, 0.45 | P · S1(d_e 1.5), S3 | `w_min:=` |
| w_max | 0.50 (차 한 대 길이) | 0.40 (2 s_m), 0.50, 1.17 (v_ref 정지거리) | P · S1–S4 | `w_max:=` |
| N (+ known_on) | Exp1 결과 | 결정값 ± 2 | 전 방법 · S3 | `n_frames:= known_on:=` |
| row_face_tol | 0.10 | 0.05, 0.10, 0.15 | P · S1(d_e 1.5), S3 | `row_face_tol:=` |
| k (B1) | 1.0 | 0.5, 1.0, 1.5 | B1 · S2, S4 | `b1_k:=` |
| B1 ROI | 3.0 × 식(약 1.0) | 길이 2–4, 반폭 0.6 / 0.8 / 1.0 | B1 · S2, S4 | `b1_roi_length:= b1_roi_half_width:=` |

w_min을 너무 크게 잡으면 P가 실제 틈을 놓쳐 위험해진다 (원 계획서 리스크). 결과는 한계에 명시한다.

## 8. 계획서 이후 변경 사항

| 항목 | 원 계획서 | 변경 | 이유 |
|---|---|---|---|
| TF | 없음 | base_link(뒤축 중심, z = 0 바닥) → livox_frame → livox_imu 정적 TF, odom → base_link는 vesc_to_odom (EKF 선택) | 점유격자를 odom 프레임에 쌓고, 높이 대역을 바닥 기준으로 자르기 위해 |
| IMU | 언급 없음 | Mid-360 IMU를 g → m/s² 변환, 바이어스 차감, 장착 roll/pitch 보정 후 중력 제거한 종방향 가속도 | 드라이버가 g 단위로 낸다는 보고. pitch 1° = 0.17 m/s² (a_b 3 m/s² 대비 6%) 오차 |
| 높이 대역 레이캐스팅 | 높이 필터 | base_link z 0.05–0.30 m 안 점만 occupied·free 모두에 사용 | 위쪽 빔이 박스 위를 넘어 뒤를 free로 지우면 가림이 사라진다 |
| deskew | 없음 | 점별 시각으로 odom 보간 (on/off는 Exp1에서 결정, 모든 방법 동일) | 2.0 m/s에서 프레임당 0.2 m 이동 → 틈 폭(0.45/0.15 m) 측정 오염 |
| v_cap 불연속 | d ≤ s_m이면 v_imp | d ≤ s_m이면 v_cap(s_m) | v_imp로 두면 s_m 안쪽에서 속도가 오히려 올라간다 |
| d, d_e 정의 | 모호 | d: 앞 끝 → phantom 경로 진입 지점 종방향 거리. d_e: #2 앞부분이 차선 경계에 닿을 때 앞 끝 ↔ 충돌 지점 | 기준점을 차체 앞 끝으로 통일 (AEB·v_cap·분석) |
| 최소 속도 정책 | 없음 | **v_imp = V_MIN(0.867 m/s, 4000 ERPM)**, v_cap < V_MIN이면 V_MIN 유지 + 플래그. B1도 같은 정책. 히스테리시스 정지는 비교용(기본 off) | 플랫폼 최소 안정 속도 아래 명령은 낼 수 없다. v_imp ≥ V_MIN이면 V_MIN 주행 시에도 충돌 지점 도달 속도 ≤ v_imp가 유지되고, 정지 후 phantom이 그대로면 재출발하지 못하는 교착이 없다 |
| v_imp 기본값 | 0.5 m/s | 0.867 m/s (= V_MIN) | 위와 같음. **플랫폼 최소 안정 속도가 v_imp 하한을 정한다** (한계) |
| Exp1 속도 조건 | 정지, 0.5 m/s | 정지, 0.9 m/s, 2.0 m/s | 0.5 m/s는 V_MIN(0.867) 아래라 낼 수 없다 |
| exp 조향 | 기존 조향 공유 | 기존 planner 상속, 좌측 벽 WALL_FOLLOW 고정, 회피 모드 끔, lookahead 4.4 m 고정, 정지 시 조향 유지 | 돌출 차량을 조향으로 피하면 안전 결과를 속도 계획 효과로 해석할 수 없다. 실측 속도에 따라 lookahead가 바뀌면 방법별로 조향이 달라진다 |
| exp AEB 기준점 | — | 같은 클래스·임계값, 기준점만 앞 끝(라이다 + 0.28 m)·반폭 0.135 (drive는 0.30·0.15 유지). 회차 끝까지 래치 | 치수 기준 통일. drive 모드 불변 |
| AEB 오작동 | — | S2–S4에서 **정지선 전** AEB 발동 회차는 효율 통계에서 빼고 방법별로 따로 집계 (2026-11-05: 정지선 전만 세도록 실차·시뮬 통일, `metrics.aeb_malfunction`) | 돌출이 없는데 정지한 회차의 손실은 배치 규칙의 효과가 아니다. 정지선 뒤 방호벽 근접 발동은 시험 구간 밖이다 |
| 폐기 규칙 (2026-11-05) | — | 가속 구간 끝 실측 < 0.95 v_ref → **가속 구간 끝까지 명령이 v_ref였는데 실측 < 0.95 v_ref일 때만** (`metrics.discard_check`, 모든 방법·실차·시뮬 공통) | 시뮬에서 B1이 스스로 감속해 전 회차 폐기됨 (OPEN_ITEMS 9절). 알고리즘이 낸 감속은 그 방법의 비용이라 지표에 넣어야 한다. "그 시점 명령 = v_ref"만 보면 가속 구간 중 스스로 감속했다가 v_ref로 돌아온 회차가 폐기되므로 가속 구간 전체 명령을 본다 |
| B1 ROI 반폭 (2026-11-05, 같은 날 재정의) | 1 m | **lane_width/2 + b1_object_speed·(t_r + v_ref/a_b)** = 0.35 + 0.8·(0.15 + 2/3) ≈ 1.003 m (실차 기본값, `phantom.b1_roi_half_width_auto`가 `common.yaml` 값으로 계산, 손 반올림 없음. 시뮬은 t_r 0.25로 ≈ 1.083). 0.6 / 0.8 / 1.0은 민감도 | 에고가 v_ref에서 반응해 멈출 때까지 차선 밖 물체가 차선에 도달할 수 있는 범위. 처음 정한 0.35(차선 + 갓길)는 B1이 구조물 뒤 미관측 칸(물체가 숨을 수 있는 곳)을 세지 못하게 만들어 기준 자체가 잘못됐다 |
| phantom 유지 (2026-11-05) | 프레임마다 새로 배치 | 생긴 phantom을 odom에 고정해 앞 끝이 x + 0.10 m를 지날 때까지 유지 (B2·P 공통, `phantom_hold`·`phantom_hold_margin` [EXP-FIXED]) | 시뮬에서 #2가 틈에서 나오는 순간 P가 #2를 열로 묶어 틈 phantom이 사라지고 v_cap이 풀려 v_imp 초과 충돌 (OPEN_ITEMS 10절). 가장 위험한 순간에 phantom이 사라지면 안 된다. 방법 차이는 배치 규칙에만 둔다. 테스트 `test_hold_keeps_phantom_when_car_fills_gap` |
| P 주차열 범위 (2026-11-05) | 모든 occupied 칸 | 차체 앞 끝보다 앞쪽 칸만. 첫 칸이 앞 끝 바로 앞(row_link_dist 안)이면 열이 차 옆으로 이어지는 것으로 보고 시작 끝 phantom을 두지 않는다 | 출발점 뒤 벽이 열로 잡혀 뒤 벽 ~ P1 사이를 틈으로 보고 가속 구간에 phantom을 뒀다 (OPEN_ITEMS 10절). 시작 끝 예외는 예전에 앞 끝보다 뒤에 있어 d에서 빠지던 경우와 같은 결과를 내기 위함 |
| P w_max (2026-11-05) | 없음 | 0.50 m. 넘는 빈 공간은 틈이 아니라 열 끝(앞 열 끝·뒤 열 시작 phantom) | 차 한 대 길이 이상 빈 공간은 주차 칸이 비어 열이 끊긴 것. 결과 보기 전 근거로 정함 (승인). 민감도 0.40 / 0.50 / 1.17 |
| Exp0 i 추종 지연 측정 (2026-11-05, 같은 날 변경) | B2 주행 | 방법과 무관한 속도 계단 2.0 → 1.2 m/s (`test_step_v`), 응답을 순수 + 1차 지연 + a_b 포화 모델로 맞춤 | 측정이 비교 대상 방법(B2)에 의존하지 않게 |
| t_r 정의 (2026-11-05) | 반응시간 | 관측 → 실제 감속 시작 전체 지연 (인지 + 통신 + 제어 + 구동, 1차 지연은 등가 지연). Exp0 i 추종 지연 시험 추가, c의 계단 구동 지연은 참고값 | v_cap 식은 '순수 지연 t_r 뒤 a_b 감속'을 가정한다. 1차 지연 τ인 속도 제어는 v_cap처럼 완만히 내려가는 명령을 τ 늦게 따라가므로 τ가 t_r에 들어가야 v_imp 보장이 맞는다 (시뮬에서 틈 입구 속도 1.24 > v_imp 0.867, OPEN_ITEMS 9절). 계단 급제동은 감속이 바로 포화돼 τ가 거의 안 보인다 |
| 충돌 속도 보고 (2026-11-05) | 충돌 여부 | 충돌 회차의 충돌 속도 최대·평균, v_imp 이하 / 초과 충돌 수 | v_imp 이하 충돌은 설계가 허용한 결과라 초과 충돌과 구분해야 한다 |
| **P 주차열 군집** | occupied 군집 사이 틈 | **차선 쪽 면에서 row_face_tol(0.10 m) 띠 안의 칸만 열로 묶는다** ([EXP-TUNE] 0.05/0.10/0.15) | 틈 안에 숨은 #2의 앞부분이 occupied로 잡히면 두 박스가 한 열로 이어져 P가 바로 그 틈을 놓친다. 테스트: `test_occupancy_phantom.test_p_hidden_car_does_not_bridge_gap` (면에서 0.15 m 들어간 숨은 차가 있어도 틈 입구 phantom 유지). 가정: 숨은 물체는 면보다 0.10 m 이상 안쪽 |
| #2 격리 | 언급 없음 | #2는 `/car2` 네임스페이스로 VESC 드라이버(publish_tf false)와 #2 노드만 (`car2.launch.py`). 에고는 발행자 점검(`publisher_guard_node`) 통과 전 출발·기록 거부 | 같은 ROS_DOMAIN_ID에서 토픽 이름이 겹치면 #2 명령이 에고 VESC로 들어갈 수 있다 |
| #2 속도 | 0.8 m/s | 0.8 m/s, Exp0 e에서 불안정하면 0.9 m/s (결과 여기에 기록: TBD) | 0.8 m/s(3691 ERPM)는 에고 최소 RPM(4000)보다 낮아 안정성 확인 필요 |
| YOLO, 카메라 | 카메라 인지 | YOLO 끔(exp), 전방 카메라 녹화 전용(압축, depth 끔), AEB 표시는 오프라인 오버레이 | 연산 부하로 t_r이 흔들리는 것 방지, 디스크 I/O로 LiDAR 누락 방지 |
| 실험 전환 | — | 설정 파일(common/method/scenario)과 bringup 인자로만. 회차마다 노드 재시작 | 주석 처리 전환 금지, 상태 이월 방지 |
| 효율 지표 | 손실 % | 절대 손실 ΔT 주 지표, 명령·실측 분리, 시나리오 분리, 폐기 수·사유 보고 | % 손실은 구간 길이에 희석. S2와 S3·S4는 성격이 다름. 주장 근거는 명령 기준 |
| 안전 장치 | 폼 범퍼 | + 조이스틱 B 최우선, 조이스틱 끊김 0.5 s 정지(폐기), 소프트웨어 정지선 | 실차 회차 수가 많아 사람 반응만으로는 부족 |

## 9. 한계 (원고에 명시)

- 플랫폼 최소 안정 속도(0.867 m/s)가 v_imp 하한을 정한다. v_imp를 더 낮춘 설계는 이 플랫폼에서 검증할 수 없다.
- AEB는 장애물 상대속도를 쓰지 않는다 (에고 속도 기반, drive 모드와 같은 정의).
- exp 조향은 좌측 벽만 보고 회피하지 않는다. 실제 차량의 회피 거동과 결합한 효과는 다루지 않는다.
- 높이 대역 아래(0.05 m 미만)·위(0.30 m 초과) 물체는 격자에 없다. scan(AEB·조향)은 바닥에서 약 0.14 m 아래가 안 보인다.
- 오프라인 재생 비교는 개루프다 (녹화된 궤적 그대로).
- P는 "숨은 물체가 열의 면보다 row_face_tol 이상 안쪽"이라는 가정에 기대며, w_min 선택이 틀리면 위험해질 수 있다.
- 선행 연구에 대해 "실차 검증 사례가 없다"고 쓰지 않는다 (전수 조사가 아님).

## 10. 미결정 사항 (팀 논의)

- 돌출 객체: F1TENTH #2 vs 줄로 당기는 폼 더미. **현재는 #2 기준으로 구현.**
- 9 m 직선 공간 확보 여부. 안 되면 x 방향을 비율 유지하며 축소 (가속·정지 구간은 유지) → `scenario_*.yaml` 좌표 교체.
- 역할 분담: 인지 노드 / phantom·v_cap 노드 / 트랙·돌출 차량·트리거 / 측정·분석.
- 기존 Unknown-aware Gap Planning 코드 재사용: **레포·이력에 없어 재사용 불가**, 새로 구현함 (ARCHITECTURE 6절).

## 11. 일정 영향

원안: 4주 중 구현 약 1.5주. TF·IMU·deskew·안전 장치(발행자 점검, 데드맨, AEB 기록)·Exp0 항목 추가로 차량 확인 작업이 늘었다.
코드는 대부분 작성됐으나 **실차 실행 확인 전**이다. 수정안 (팀 확정 필요):

| 주 | 원안 | 수정안 |
|---|---|---|
| 1 | Exp0, 구현 | 세션 1(토픽 확정·리팩터 검증·장착값) → 토픽 이름 반영 → Exp0 a–h (약 2일) |
| 2 | 구현, Exp1 | 실차 통합 확인(rosbag 재생 4방법, AEB 단독 정지) → Exp1 (N·deskew 결정) |
| 3 | Exp2, Exp3 | Exp2 120회 + Exp3 115회 (회당 약 1.5분 + 리셋 → 실기록 약 6시간, 3일 배정) |
| 4 | 분석·원고 | 분석·민감도(오프라인 재생 위주) · 원고. 실차 민감도는 여유가 있을 때만 |

원안 대비 약 3–5일 밀릴 수 있다. 여유가 부족하면 실차 민감도 분석을 오프라인 재생으로 대체하고 한계에 적는다.
