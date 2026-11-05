# MuJoCo 시뮬레이션 — 구조, 설계 근거, 검증

실차 전에 파이프라인 버그를 잡고, 민감도(w_min, v_imp, N, B1 ROI)를 미리 훑고, RQ4 기준선(노이즈 없는 가림)을 얻는 용도다.
브랜치 `sim/mujoco`. 실차 exp 노드 클래스를 가짜 rclpy(`sim/rosstub.py`) 위에서 **그대로** 실행하므로
시뮬 결과는 실차 코드의 결과다 (알고리즘 재구현 없음).

**설치·실행·뷰어 키·출력 파일·파라미터 표·알 수 있는 것/없는 것·시뮬 전용 차이·문제 해결은 [sim/README.md](../sim/README.md)** 에 있다.
이 문서는 내부 구조와 시뮬 전용 값의 근거, 검증만 다룬다. 시뮬에서 드러난 설계 문제는 [OPEN_ITEMS 9–10절](OPEN_ITEMS.md).

## 1. 구조

```
라이다(mj_multiRay, 10 Hz) ─/livox/lidar─> occupancy_grid_node ─/exp/grid─> phantom_node ─/exp/phantom_d,/exp/rho─> speed_layer_node ─/exp/v_cmd─┐
           └─(pointcloud_to_laserscan 규칙)─/scan─> exp_aeb_node ─/aeb/active─────────────────────────────────────────────────────────┤
                                                 └> exp_steering_node ─/exp/steering_command────────────────────────────────────────────┤
차량 모델 ─/vesc/odom─> (위 노드들, trigger_node) ──/trigger──> popout_vehicle_node(#2) ─commands/motor/speed─> #2 차량 모델           │
                                                                                                          exp_drive_mux <─────────┘
                                                              exp_drive_mux ─/commands/motor/speed, servo─> to_vesc 역변환 ─> 에고 차량 모델
```

| 파일 | 역할 |
|---|---|
| `sim/rosstub.py` | 가짜 rclpy·메시지. **시계 = 시뮬 시간** (모드 전환 0.45 s, 명령 만료 1 s 등 노드 내부 시간이 시뮬 스텝에 묶임 → 배치·뷰어 결과 동일). 타이머는 시뮬이 시뮬 시간 기준으로 호출 |
| `sim/simconfig.py` | 실차와 같은 config 파일을 bringup과 같은 우선순위로 읽어 노드별 파라미터 생성. 장착값은 `tf_static.launch.py` 실행으로 읽음 |
| `sim/world.py` | MJCF: 박스 위치는 scenario yaml + `track_layout`, 치수는 `sim/config/sim.yaml`. box 도형만 |
| `sim/vehicle.py` | 뒤축 기준 운동학 자전거 모델. 명령은 `mux_logic.to_vesc`(최소 4000 ERPM) → ERPM·서보 → m/s·조향각. t_r 지연 + 1차 지연 τ + 가속 a_max·감속 a_b 제한 |
| `sim/lidar.py` | Mid-360 근사, 2D scan 생성 |
| `sim/runner.py` | 노드 생성, 토픽 라우팅, 고정 스텝 루프, 저장 |
| `sim/popout_timing.py` | 시뮬 #2 모델의 진입 시간 (Exp0 f와 같은 측정) → 시뮬 trigger_node 값 |
| `sim/viewer_ui.py` | 대화형 뷰어 오버레이·재생 제어. 스텝 뒤 상태를 읽기만 한다 |
| `sim/sweep.py`, `sim/video.py` | 일괄 실행·그림, 영상 |
| `analysis/sim_reader.py` | 시뮬 로그 → `my_algo.metrics`로 실차와 같은 키의 `metrics.json`·`speed_vs_s.csv` (`summarize.py`·`plots.py`가 그대로 읽음) |

한 스텝(기본 dt 0.01 s): 차량 갱신 → /vesc/odom → (0.1 s마다) 라이다 점군·scan → 노드 타이머(50 Hz 등) → mux 명령 → 차량
→ 기록 → (뷰어면) 그리기·대기. 뷰어는 마지막 단계에서 상태를 읽기만 하므로 배치와 결과가 같다 ([sim/README.md 3절](../sim/README.md#3-대화형-뷰어)).

## 2. 시뮬 전용 시간 값의 근거 (`sim.yaml` `timing_overrides`)

**t_r_eff = t_r + τ의 근거.** t_r은 '관측 → 실제 감속 시작 전체 지연'이다 (EXPERIMENT_PLAN 2절). 시뮬 차량은 명령을 순수 지연
t_r(0.15 s) 뒤에 받아 1차 지연 τ(0.10 s)로 따라간다. 1차 지연 계는 기울기 a로 내려가는 명령에 대해 정상 상태에서 v(t) = u(t − τ)이므로
(오차 a·τ = 시간 지연 τ), v_cap처럼 완만히 내려가는 상한을 따라갈 때 순수 지연 τ와 같다. 그래서 v_cap 식에는 t_r + τ를 넣는다.
확인: 시뮬 S2 회차에 실차 Exp0 i와 같은 함수(`braking.tracking_lag`)를 쓰면 0.268 s (순수 0.146 + 1차 0.122)가 나온다
(`metrics.json` `tracking_lag`). 계단 급제동은 감속이 a_b로 바로 포화돼 τ가 거의 안 보인다 (`test_braking.test_step_braking_hides_tau`).
시뮬의 인지 지연(라이다 10 Hz 주기, 평균 0.05 s)은 t_r_eff에 넣지 않았다 (결정: t_r + τ). 시뮬의 실제 총 지연은 그만큼 t_r_eff보다 길다.

**popout_enter_time.** 실차 Exp0 f와 같은 방법(실차 popout_vehicle_node의 `reach_lane_s`)을 시뮬 #2 모델(같은 Kinematic, sim.yaml car2·vehicle)로
재서 쓴다. 트리거 시각과 노드 타이머(50 Hz) 위상 5개 평균 0.556 s (0.55–0.56). 시뮬 /trigger 전달 지연은 0이라 trigger_latency = 0.
실차 `common.yaml` 값(0.05, 0.2)은 Exp0까지 [EXP0-MEASURE]로 둔다.

## 3. 검증

| 무엇 | 어떻게 | 결과 (2026-11-05) |
|---|---|---|
| 라이다 거리, scan 규칙, 박스 위 빔, 정지거리, 최소 ERPM | `sim/tests/test_sim.py` | 통과 |
| 뷰어·배치 같은 결과 | `sim/tests/test_viewer_determinism.py`: 같은 시드, 뷰어 경로(그리기·0.25배속·일시정지·프레임 진행·토글) 대 배치. 로그 전 항목·프레임별 점군 지문·격자·이벤트 비트 단위 비교 | 통과. 변이(그리기 중 라이다 난수 1개 소비, 차량 속도 1e-12 변경) 둘 다 검출 |
| exp 노드 순수 모듈 분리 전후 | `scripts/verify_exp_refactor.py --baseline 2187633` (B1 ROI 반폭은 기준 커밋 값 1.0으로 고정해 비교) | 101,075개 메시지 일치 |
| 추종 지연 = t_r + τ | 시뮬 S2·S3 B2·P에 `braking.tracking_lag` | 0.252–0.268 s (모델 0.25) |
| #2 진입 시간 | `sim/popout_timing.py` | 0.556 s, 재실행 시 실제 d_e가 명목과 맞음 (B0 1.00/1.48/2.00) |
