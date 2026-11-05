# MuJoCo 시뮬레이션 (Mac, ROS 없음)

실차 전에 파이프라인 버그를 잡고, 민감도(w_min, v_imp, N)를 미리 훑고, RQ4 기준선(노이즈 없는 가림)을 얻는 용도다.
브랜치 `sim/mujoco`. 실차 exp 노드 클래스를 가짜 rclpy(`sim/rosstub.py`) 위에서 **그대로** 실행하므로
시뮬 결과는 실차 코드의 결과다 (알고리즘 재구현 없음).

## 1. 설치

```bash
cd <my_algo 레포>
bash sim/setup_env.sh        # sim/.venv 생성(Homebrew python3.12, arm64) + 고정 버전 설치 + 점검. 다시 실행해도 안전
```
- 고정 버전 (`sim/requirements.txt`): mujoco 3.12.0, numpy 2.5.4, PyYAML 6.0.3, matplotlib 3.11.2, imageio 2.38.1 (+ffmpeg 0.8.1).
- 시스템 Python·Homebrew·conda는 건드리지 않는다. conda Python이 PATH 앞에 있어도 항상 `sim/.venv/bin/python`, `sim/.venv/bin/mjpython`을 경로로 부른다.
- 점검만: `sim/.venv/bin/python sim/check_env.py` (mujoco 버전, mjpython, 순수 모듈 import, 오프스크린 렌더).

## 2. 실행

| 터미널 | 명령 | 결과 |
|---|---|---|
| 배치 1회 | `sim/.venv/bin/python sim/runner.py --method P --scenario S1 --d-e 1.5 --seed 0` | 실시간보다 빠름 (시뮬 5 s ≈ 벽시계 4 s) |
| 대화형 뷰어 | `sim/.venv/bin/mjpython sim/runner.py --method P --scenario S3 --viewer` | **macOS는 반드시 mjpython**. 실시간 속도 |
| 위에서 본 영상 | `... runner.py ... --video` 또는 `sim/.venv/bin/python sim/video.py <run_dir>` | `<run_dir>/topdown.mp4` (격자·경계·phantom·차량·v_cap) |
| 민감도 1회 | `... runner.py --method P --scenario S3 --w-min 0.30` | [EXP-TUNE] 값만 바꿀 수 있다 (`--v-imp --n-frames --known-on --known-off --b1-k --b1-roi-length --b1-roi-half-width --row-face-tol`) |
| 일괄 | `sim/.venv/bin/python sim/sweep.py --seeds 0 1 2` | S1(1.0/1.5/2.0) + S2–S4 × 4방법 × 시드, 병렬. 요약 표·그림 5·6 |
| 민감도 일괄 | `sim/.venv/bin/python sim/sweep.py --scenarios S1 S3 --methods P --w-min 0.25 0.35 0.45 --d-e 1.5 --name wmin` | 값마다 별도 회차 |
| 조향 비교 | `... --steering simple` | 시뮬 전용 단순 조향 (기본은 실차 exp_steering_node) |
| 시뮬 테스트 | `sim/.venv/bin/python -m unittest discover -s sim/tests` | 라이다 해석해, scan 규칙, 박스 위 빔, 정지거리, 최소 ERPM |

노드 상태 출력은 기본으로 숨긴다 (`--verbose`로 보기).

## 3. 구조

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
| `sim/sweep.py`, `sim/video.py` | 일괄 실행·그림, 영상 |
| `analysis/sim_reader.py` | 시뮬 로그 → `my_algo.metrics`로 실차와 같은 키의 `metrics.json`·`speed_vs_s.csv` (`summarize.py`·`plots.py`가 그대로 읽음) |

한 스텝(기본 dt 0.01 s): 차량 갱신 → /vesc/odom → (0.1 s마다) 라이다 점군·scan → 노드 타이머(50 Hz 등) → mux 명령 → 차량.

## 4. 로그

```
sim_logs/YYYYMMDD/<scenario>_<method>_de<d_e>_seed<NN>[_<tag>]/          (sweep은 sim_logs/YYYYMMDD/sweep_<name>/ 아래)
  log.npz        스텝별: t, x, y, yaw, v(실제), steer, s(AUTO 후 주행 거리), final_cmd(보낸 속도), v_cmd(정책 후),
                 v_raw(min(v_ref, v_cap), 명령 기준 손실용), v_cap, d, rho, aeb, stop_reason, flag(최소 속도 정책),
                 car2_x/y/v, dist_car2(footprint 다각형 거리), dist_static
                 프레임별: frame_t, phantoms_xy/phantoms_frame, boundary_xy/boundary_frame (RQ4 기준선)
  grids.npz      프레임별 발행 격자 (g0000...) + origin
  meta.json      설정 전체(common/method/scenario/popout/sim.yaml/장착값), git hash·dirty, mujoco 버전, 시드,
                 이벤트(/trigger, /exp/aeb_event, /exp/abort, /popout/status), 월드 박스, 시뮬 전용 차이 목록
  analysis/metrics.json, speed_vs_s.csv   실차와 같은 지표 키 (+ d_e_actual, car2_enter_after_trigger_s, static_contact)
  topdown.mp4    (--video)
```
`sim_logs/`와 `sim/.venv/`는 git에 넣지 않는다.

## 5. 시뮬 파라미터 (`sim/config/sim.yaml`)

| 키 | 기본 | 의미 |
|---|---|---|
| sim.dt / max_time / auto_start_time | 0.01 / 15 / 0.3 s | 고정 스텝, 회차 최대 시간, AUTO 진입 시각 |
| track.start_x / end_x | −0.6 / 9.0 m | 뒤 벽, 방호벽 (가정) |
| track.lane_width, left_wall_offset | 0.7, 0.0 | 차선, 좌측 벽 위치 (common.yaml target_wall_distance 0.355와 같은 가정) |
| track.box_depth / box_height / box_offset_from_lane | 0.30 / 0.35 / 0.0 | 박스 치수 (길이는 scenario yaml `box_len`) |
| track.left_wall_height, right_wall_height, track_width | 0.5, 0.35, 2.5 | 가정 |
| car2.length / width / rear_overhang / height | 0.50 / 0.30 / 0.09 / 0.25 | #2 치수 (길이·높이 가정) |
| vehicle.tau / a_max / steer_rate | 0.10 s / 2.5 / 6 rad/s | 종방향 1차 지연, 최대 가속, 조향 변화율 (가정). t_r·a_b는 common.yaml |
| lidar.rays_per_frame / pattern / range_noise_std | 20000 / nonrepetitive / 0.0 | 광선 수, 패턴(시드 고정 무작위 / 격자), 거리 노이즈(0 = RQ4 기준선) |
| mux_overrides | require_joy false, require_preflight false | 시뮬에서만 끔 |

실차 값(v_ref, t_r, a_b, s_m, v_imp, AEB 임계값, 격자, 조향 게인 ...)은 `config/*.yaml`을 그대로 쓴다. 시뮬이 바꾸는 것은 위 표뿐이다.

## 6. 시뮬로 알 수 있는 것 / 없는 것

**알 수 있는 것**
- 파이프라인 연결 버그 (노드 간 토픽, 파라미터 우선순위, 노드 내부 시간 로직, mux 우선순위, 트리거 계산)
- 같은 관측 조건에서 방법별 phantom 배치·v_cap·명령 속도의 상대 차이 (그림 5·6 시뮬판)
- 민감도의 방향 (w_min, v_imp, N, row_face_tol, k)
- RQ4 기준선: 노이즈 0, 차량 정지 시 경계의 흔들림 = 스캔 패턴만의 효과
- 설계 가정 사이의 불일치 (아래 OPEN_ITEMS 9절처럼)

**알 수 없는 것 (실차 필요)**
- Mid-360 비반복 스캔의 실제 패턴과 거리 노이즈, 반사율, 다중 반사
- 실제 a_b·t_r·바닥 슬립, VESC 속도 제어 응답
- 최소 속도(4000 ERPM) 근처의 떨림·출발 토크 문제
- DDS 지연, 메시지 유실, CPU 부하에 따른 주기 흔들림, 시간 동기
- odom 오차 (시뮬 odom은 참값), IMU (시뮬에 없음)
- 실제 #2 진입 거동 (출발 가속, 직진성)

## 7. 시뮬 전용 차이

| 항목 | 실차 | 시뮬 |
|---|---|---|
| 조향 | exp_steering_node | 같음 (기본). `--steering simple`은 시뮬 전용 좌측 벽 P 제어 — 방법 간 동일 |
| 조이스틱 데드맨·발행자 점검 | 켬 | 끔 (sim.yaml `mux_overrides`). 나머지 mux 우선순위·래치는 같음 |
| AUTO 진입 | 조이스틱 LB | `auto_start_time`에 `/autonomous_mode=true` |
| odom | VESC 휠 odom | 참값 |
| 2D scan | pointcloud_to_laserscan 노드 | 같은 규칙을 numpy로 (`lidar.cloud_to_scan`, 테스트로 확인) |
| 차량 | 실제 | 운동학 모델. t_r을 구동 지연으로 넣어 인지 지연(라이다 주기)과 일부 겹칠 수 있다 (보수적) |
| 충돌 | 천장 카메라·scan | footprint 다각형 참값 (`geometry.polygon_distance`, 천장 카메라 분석과 같은 함수) |
| AEB 오작동 판정 | S2–S4에서 발동 | 정지선(stop_distance) 전에 발동한 경우만 (정지선 뒤 방호벽 근접 제외). OPEN_ITEMS 9절 |
