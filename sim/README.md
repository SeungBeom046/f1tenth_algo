# MuJoCo 시뮬 실행 안내 (Mac)

실차 exp 노드(B0/B1/B2/P, 속도 계층, AEB, 조향, mux, 트리거, #2)를 ROS 없이 Mac에서 그대로 돌려 S1–S4를 재현한다.
알고리즘은 다시 구현하지 않았다. 실차 코드가 가짜 rclpy 위에서 돈다.
구조·설계 근거·검증은 [docs/SIMULATION.md](../docs/SIMULATION.md), 시뮬에서 드러난 문제는 [OPEN_ITEMS 9–10절](../docs/OPEN_ITEMS.md)에 있다.

아래 명령은 모두 **레포 루트(my_algo)** 에서 실행한다.

## 1. 사전 준비 (처음 한 번)

```bash
uname -m                      # arm64 여야 한다. x86_64면 Rosetta 터미널 → 아래 10절
bash sim/setup_env.sh         # sim/.venv 생성 + 고정 버전 설치 + 점검. 다시 실행해도 안전
```

터미널을 열 때마다:

```bash
source sim/.venv/bin/activate          # 프롬프트 앞에 (.venv)
python sim/check_env.py                # 점검만 다시: mujoco 버전, mjpython, 실차 모듈 import, 오프스크린 렌더 → "점검 결과: 통과"
```

- 설치 위치: `sim/.venv` (git 제외). 기반은 Homebrew `python3.12` (arm64). 시스템 Python·Homebrew·conda 패키지는 건드리지 않는다.
- 고정 버전 (`sim/requirements.txt`): mujoco 3.12.0, numpy 2.5.4, PyYAML 6.0.3, matplotlib 3.11.2, imageio 2.38.1, imageio-ffmpeg 0.8.1.
- venv를 활성화하지 않았으면 `python` 대신 `sim/.venv/bin/python`, `mjpython` 대신 `sim/.venv/bin/mjpython`을 쓴다.

## 2. 실행

### 대화형 뷰어 1회

```bash
mjpython sim/runner.py --method P --scenario S1 --d-e 1.0 --seed 0 --viewer
```

macOS에서 MuJoCo 창은 메인 스레드에서 돌아야 해서 **`python`이 아니라 `mjpython`** 으로 실행한다 (`python`이면 창이 안 뜨고 오류).
실시간으로 돈다. 키와 화면 설명은 3절. 회차가 끝나도 창은 남아 있고, 창을 닫으면 저장된다.

### 배치 1회 (창 없음, 실시간보다 빠름)

```bash
python sim/runner.py --method P --scenario S1 --d-e 1.5 --seed 0
python sim/runner.py --method B2 --scenario S3 --seed 1
```

끝나면 한 줄 결과가 나온다: `S1_P_de1.5_seed00: sim 5.3s (wall 4.1s) dT_cmd=... collision=... → <회차 폴더>`.

### 일괄 실행 (sweep)

```bash
python sim/sweep.py --seeds 0 1 2                       # 72회: S1(d_e 1.0/1.5/2.0) + S2–S4 × 4방법 × 시드 3. 병렬, 약 1분
python sim/sweep.py --scenarios S2 S3 --methods B2 P --seeds 0 1 --name quick
```

### 그림 5·6, 요약 표 다시 만들기

sweep이 끝나면 자동으로 만든다. 회차를 다시 돌리지 않고 표·그림만 다시 만들 때:

```bash
python sim/sweep.py --analyze sim_logs/20261105/sweep_main
```

### 위에서 본 영상 (mp4)

```bash
python sim/runner.py --method P --scenario S1 --d-e 1.0 --video        # 회차 실행 + 영상
python sim/video.py sim_logs/20261105/S1_P_de1.0_seed00                 # 이미 있는 회차
```

`<회차 폴더>/topdown.mp4`: 점유격자, 가림 경계, phantom, 두 차량, 속도·v_cap.

### 자주 쓰는 인자

| 인자 | 값 | 뜻 |
|---|---|---|
| `--method` | B0 B1 B2 P | 방법 |
| `--scenario` | S1 S2 S3 S4 E0 | 시나리오 (`config/scenario_*.yaml`) |
| `--d-e` | 1.0 1.5 2.0 | S1 돌출 시점 [m] (S1에서만) |
| `--seed` | 0, 1, ... | 라이다 비반복 패턴 난수. 같은 시드 = 같은 결과 |
| `--steering` | node(기본) / simple | 실차 exp_steering_node / 시뮬 전용 좌측 벽 P 제어 |
| `--tag` | 문자열 | 회차 폴더 이름 꼬리표 |
| `--out-dir` | 경로 | 회차 폴더를 직접 지정 |
| `--verbose` | | 노드 상태 출력 보이기 (기본 숨김) |
| `--viewer-max-points` | 4000 | 뷰어에 그릴 라이다 점 수 (표시용) |

[EXP-TUNE] 값만 덮어쓸 수 있다 (실차 bringup 인자와 같은 이름). 다른 값을 바꾸려 하면 오류가 난다.

```bash
python sim/runner.py --method P --scenario S3 --w-min 0.30
python sim/sweep.py --scenarios S1 S3 --methods P --w-min 0.25 0.35 0.45 --d-e 1.5 --name wmin      # 값마다 별도 회차
python sim/sweep.py --methods B1 --b1-roi-half-width 0.6 0.8 1.0 --seeds 0 1 2 --name b1roi
```

덮어쓸 수 있는 것: `--v-imp --w-min --n-frames --known-on --known-off --b1-k --b1-roi-length --b1-roi-half-width --row-face-tol`.

## 3. 대화형 뷰어

### 키

MuJoCo 뷰어 기본 단축키와 겹치지 않는 키만 쓴다. 기본 단축키는 A–Z(표시 토글), 0–5(그룹), Space, 화살표, Backspace(리셋), Tab, `[` `]`, `+` `-`, F1–F5다.
같은 목록이 화면 왼쪽 아래에도 나온다.

| 키 | 동작 |
|---|---|
| Enter | 일시정지 / 재개 |
| `.` (마침표) | 일시정지 중 라이다 1프레임(시뮬 0.1 s) 진행 |
| 9 | 재생 배속 1× ↔ 0.25× |
| 6 | 라이다 점 켜기/끄기 |
| 7 | 가림 경계 칸 켜기/끄기 |
| 8 | phantom 켜기/끄기 |
| End (Mac: fn + →) | 글자 패널 켜기/끄기 |

MuJoCo 자체의 Space(일시정지)는 이 시뮬에서는 쓰지 않는다. 시뮬 진행은 위 Enter로 멈춘다.
마우스: 왼쪽 끌기 = 회전, 오른쪽 끌기 = 이동, 스크롤 = 확대 (MuJoCo 기본).

### 오버레이 색

| 표시 | 모양 · 색 | 뜻 |
|---|---|---|
| 라이다 점 (높이 대역 안) | 작은 초록 상자 | base_link z 0.05–0.30 m. 점유격자에 쓰이는 점 |
| 라이다 점 (대역 밖) | 작은 회색 상자 | 바닥, 박스 위로 넘어간 점. 격자에 쓰이지 않음 |
| 가림 경계 칸 | 납작한 파란 사각형 | 이번 프레임 경계 (`/exp/boundary_frame`) |
| phantom | 세운 원기둥. P 주황, B2 초록 | 방법의 가상 장애물. B0·B1은 phantom이 없다 |

화면 왼쪽 위 글자: method / scenario, 실제 속도, 명령 속도(`/exp/final_cmd`), v_cap, d(최근접 phantom), AEB 상태, 시뮬 시간,
재생 상태(RUN / PAUSED / STEP / ENDED와 배속), phantom 색, 그린 라이다 점 수. MuJoCo 내장 글꼴이 ASCII만 그려서 화면 글자는 영어다.

라이다 점은 프레임당 최대 2만 개(광선 수, sim.yaml `rays_per_frame`)다. 뷰어의 추가 도형 한도(`user_scn.maxgeom`)는 100,000개라 넘지 않지만,
그리기 부담 때문에 표시용으로만 4000개로 솎아낸다 (고정 간격, 난수 안 씀). `--viewer-max-points 20000`이면 전부 그린다. 알고리즘 입력(점군)은 그대로다.

### 배치와 같은 결과

오버레이·배속·일시정지는 화면 표시와 벽시계 대기만 바꾼다. 시뮬은 같은 고정 스텝(dt 0.01 s)으로 돌고, 노드 시계(가짜 rclpy)도
시뮬 시간을 따른다. 0.25×는 스텝마다 벽시계로 더 기다릴 뿐 dt는 그대로다.
`sim/tests/test_viewer_determinism.py`가 같은 시드에서 배치와 뷰어 경로(그리기, 배속, 일시정지, 프레임 진행, 토글을 모두 실행)의
궤적·명령·로그·프레임별 점군 지문·이벤트가 비트 단위로 같은지 확인한다 (창 없이 가짜 화면으로).
화면 조작(마우스 끌기 등)이 MuJoCo 데이터를 바꿔도 다음 스텝에서 차량 위치를 시뮬 모델 값으로 다시 놓은 뒤 라이다를 쏘므로 결과에 들어가지 않는다.

### 직접 확인할 것 (체크리스트)

- [ ] 2절 뷰어 명령으로 창이 뜬다 (Dock의 Python 아이콘 창이 다른 창 뒤에 있을 수 있음)
- [ ] 왼쪽 위 글자 패널에 method, scenario, 속도, v_cap, d, AEB, sim time, playback이 보이고 시간이 흐른다
- [ ] 초록·회색 라이다 점이 박스·벽 위에 보인다. 박스 윗면 위 점은 회색이다
- [ ] 박스 사이 틈 근처에 파란 경계 칸이 보인다
- [ ] P에서 주황 phantom이 틈 입구에 보인다 (`--method B2`면 초록, 경계 칸 위)
- [ ] Enter: 멈춘다 (playback이 PAUSED). 다시 Enter: 이어서 간다
- [ ] 멈춘 상태에서 `.`: 시뮬 시간이 0.1 s씩 늘고 점·경계가 한 프레임씩 바뀐다
- [ ] 9: playback이 0.25x가 되고 느려진다. 다시 9: 1x
- [ ] 6 / 7 / 8: 점 / 경계 / phantom이 사라졌다 다시 나온다
- [ ] End (fn + →): 글자 패널이 사라졌다 다시 나온다
- [ ] 위 키를 눌러도 MuJoCo 기본 동작(표시 토글 등)이 같이 바뀌지 않는다
- [ ] 회차가 끝나면 playback이 ENDED가 되고 창이 남는다. 창을 닫으면 터미널에 결과 한 줄이 나온다 (중간에 닫으면 거기까지 잘린 회차)
- [ ] 같은 명령을 `--viewer` 없이 돌린 결과와 `analysis/metrics.json`의 dT_cmd·collision이 같다 (끝까지 둔 경우)

## 4. 출력

```
sim_logs/                                  (git 제외)
  <YYYYMMDD>/
    <S>_<M>_de<d_e|na>_seed<NN>[_<tag>]/   runner.py 1회
    sweep_<name>/                          sweep.py
      <S>_<M>_de.._seed..[_<EXP-TUNE 값>]/ 회차마다 같은 구조
      analysis/                            summary.md, summary.csv, fig5_speed_profile.png/pdf, fig6_safety_efficiency.png/pdf
```

회차 폴더:

| 파일 | 내용 |
|---|---|
| `log.npz` | 스텝(0.01 s)별: t, x, y, yaw, v(실제), steer, s(AUTO 후 주행 거리), final_cmd(보낸 속도), v_cmd(정책 후), v_raw(min(v_ref, v_cap)), v_cap, d, rho, aeb, stop_reason, flag, car2_x/y/v, dist_car2, dist_static. 프레임(0.1 s)별: frame_t, phantoms_xy/phantoms_frame, boundary_xy/boundary_frame |
| `grids.npz` | 프레임별 발행 격자 (g0000, g0001, ...) + origin |
| `meta.json` | 설정 전체(common/method/scenario/popout/sim.yaml/장착값/시뮬 timing), git hash·dirty, mujoco 버전, 시드, 이벤트(/trigger, AEB, abort, #2 상태), 시뮬 전용 차이 목록 |
| `analysis/metrics.json` | 실차 `extract_run.py`와 같은 키 (dT_cmd, dT_meas, min_speed, decel_length, jerk_rms, collision, impact_speed, impact_class, min_ttc, discard, aeb_malfunction, RQ4 ...) + 시뮬 전용(d_e_actual, car2_enter_after_trigger_s, static_contact, tracking_lag) |
| `analysis/speed_vs_s.csv` | 위치(0.01 m)별 v_raw, v_final, v_meas (그림 5 원자료) |
| `_popout/` | #2 노드 기록 (popout_events.jsonl) |
| `topdown.mp4` | `--video`일 때 |

### 요약 표 (`summary.md`) 보는 법

- 평균 ± 95% CI (t 분포). 폐기 회차는 빼고, 폐기 수·사유를 같은 줄에 낸다. 시나리오마다 따로 본다 (S2–S4 평균은 내지 않는다).
- **ΔT cmd** = 명령 기준 손실 [s] (min(v_ref, v_cap), 정책 전) — 방법 비교의 주 지표. **ΔT meas** = 실제 속도 기준.
- S1 표: 실제 d_e(시뮬 참값), 충돌 수 / 사용 회차, 충돌 속도 최대 / 평균, **≤v_imp / >v_imp** 충돌 수(v_imp 이하 충돌은 설계가 허용한 결과),
  최소 TTC, 최소 거리. S1의 ΔT는 회차가 AEB로 멈춰 시험 구간을 다 못 가면 `–`.
- S2–S4 표: AEB 오작동(정지선 전 발동, 효율 통계에서 제외) 수, 최저 속도, 감속 구간 길이, jerk.
- 폐기 규칙: 가속 구간 끝까지 명령이 v_ref였는데 실측 < 0.95 v_ref일 때만 (알고리즘이 스스로 감속한 회차는 포함).
- sweep의 [EXP-TUNE] 값이 여러 개면 표는 값을 합쳐서 낸다. 값별 비교는 회차 폴더 이름 꼬리표로 나눠 본다.

## 5. 시뮬 파라미터와 가정값 — 실측 후 바꿀 곳

실차 값(v_ref, t_r, a_b, s_m, v_imp, AEB 임계값, 격자, 조향 게인, 트랙 좌표)은 `config/*.yaml`을 그대로 읽는다.
시뮬에만 있는 값은 `sim/config/sim.yaml` 하나에 있다.

| 값 | 지금 | 근거 | 실측 후 바꿀 곳 |
|---|---|---|---|
| #2 길이 / 폭 / 뒤 돌출 / 높이 | 0.50 / 0.30 / 0.09 / 0.25 m | 폭만 계획서, 나머지 가정 | `sim.yaml` `car2` |
| #2 출발 위치 → 차선 경계 | 0.30 m | popout.yaml (가정) | `config/popout.yaml` `lane_boundary_distance` (실차와 공유) |
| 박스 폭 / 높이 | 0.30 / 0.35 m | 계획서 | `sim.yaml` `track.box_depth`, `box_height` (길이는 scenario `box_len`) |
| 좌측 벽 높이, S4 우측 벽 높이 | 0.5, 0.35 m | 가정 | `sim.yaml` `track.left_wall_height`, `right_wall_height` |
| 갓길 (차선 경계 ~ 좌측 벽 / 박스 면) | 0, 0 m | 가정 (계획서상 붙어 있음) | `sim.yaml` `track.left_wall_offset`, `box_offset_from_lane` (+ 실차 `target_wall_distance`, `b1_roi_half_width`) |
| 뒤 벽, 방호벽 위치 | x −0.6, 9.0 m | 가정 | `sim.yaml` `track.start_x`, `end_x` |
| 트랙 폭 | 2.5 m | 계획서 | `sim.yaml` `track.track_width` |
| 속도 1차 지연 τ | 0.10 s | 가정 | `sim.yaml` `vehicle.tau` (Exp0 i 추종 지연 − 순수 지연) |
| 최대 가속 a_max | 2.5 m/s² | 가정 | `sim.yaml` `vehicle.a_max` (E0 가속 구간 odom) |
| 조향 변화율 | 6 rad/s | 가정 | `sim.yaml` `vehicle.steer_rate` |
| 순수 구동 지연, 최대 감속 | t_r 0.15 s, a_b 3.0 | `common.yaml` [EXP0-MEASURE] | Exp0 c·i → `common.yaml` (실차와 공유) |
| v_cap에 넣는 t_r | t_r + τ = 0.25 s | 1차 지연의 등가 지연 ([SIMULATION.md](../docs/SIMULATION.md)) | `sim.yaml` `timing_overrides.t_r_eff` |
| #2 진입 시간, 트리거 지연 | 0.556 s(#2 모델에서 계산), 0 | 시뮬 #2 모델 | 자동 (`timing_overrides.popout_from_model`) |
| 라이다 | 20000 광선/프레임, 10 Hz, 수직 −7°–52°, 비반복(시드), 거리 노이즈 0 | Mid-360 근사 | `sim.yaml` `lidar` |
| 시뮬 스텝, AUTO 시각, 최대 시간 | 0.01 s, 0.3 s, 15 s | | `sim.yaml` `sim` |

## 6. 시뮬로 알 수 있는 것 / 없는 것

**알 수 있는 것**
- 파이프라인 연결 버그 (토픽, 파라미터 우선순위, 노드 내부 시간 로직, mux 우선순위, 트리거 계산)
- 같은 관측 조건에서 방법별 phantom 배치·v_cap·명령 속도의 상대 차이 (그림 5·6 시뮬판)
- 민감도의 방향 (w_min, v_imp, N, row_face_tol, k, B1 ROI)
- RQ4 기준선: 노이즈 0, 스캔 패턴만의 경계 흔들림
- 설계 가정 사이의 불일치 (OPEN_ITEMS 9–10절)

**알 수 없는 것 (실차 필요)**
- Mid-360 실제 스캔 패턴, 거리 노이즈, 반사율, 다중 반사
- 실제 a_b, t_r, τ, 바닥 슬립, VESC 속도 제어 응답, 최소 속도(4000 ERPM) 근처 떨림·출발 토크
- DDS 지연, 메시지 유실, CPU 부하에 따른 주기 흔들림, 시간 동기
- odom 오차(시뮬은 참값), IMU(시뮬에 없음 → jerk는 차량 모델 속도 미분)
- 실제 #2 진입 거동 (출발 가속, 직진성)

## 7. 시뮬 전용 차이

| 항목 | 실차 | 시뮬 |
|---|---|---|
| 조향 | exp_steering_node | 같음 (기본). `--steering simple`은 시뮬 전용 좌측 벽 P 제어 |
| 조이스틱 데드맨·발행자 점검 | 켬 | 끔 (`sim.yaml` `mux_overrides`). 나머지 mux 우선순위·래치는 같음 |
| AUTO 진입 | 조이스틱 LB | `auto_start_time`(0.3 s)에 `/autonomous_mode=true` |
| v_cap의 t_r | Exp0 측정값 | t_r + τ (`timing_overrides`) |
| #2 진입 시간·트리거 지연 | Exp0 측정값 | 시뮬 #2 모델 값, 0 (`timing_overrides`) |
| odom | VESC 휠 odom | 참값 |
| 2D scan | pointcloud_to_laserscan 노드 | 같은 규칙을 numpy로 (`lidar.cloud_to_scan`, 테스트로 확인) |
| 차량 | 실제 | 운동학 자전거 모델 (순수 지연 + 1차 지연 + 가감속 한계) |
| 충돌 | 천장 카메라·scan | footprint 다각형 참값 (천장 카메라 분석과 같은 함수) |

## 8. 테스트

```bash
python -m unittest discover -s sim/tests     # 라이다 해석해, scan 규칙, 박스 위 빔, 정지거리, 최소 ERPM, 뷰어·배치 비트 단위 동일
python -m unittest discover -s test          # 레포 전체 (분석 함수, 노드 로직)
```

## 9. 시간

배치 1회는 시뮬 5–9 s에 벽시계 4–6 s. sweep 72회는 M3 Pro 9 프로세스로 약 1분 (뷰어는 실시간).

## 10. 자주 나는 문제

| 증상 | 원인 · 해결 |
|---|---|
| `--viewer`인데 창이 안 뜨고 오류 (``RuntimeError: `launch_passive` requires that the Python script be run under `mjpython` on macOS``) | `python`으로 실행했다. `mjpython sim/runner.py ... --viewer` |
| 창이 안 보인다 | 다른 창 뒤에 있다. Dock의 Python(로켓) 아이콘 또는 Cmd+Tab |
| `ModuleNotFoundError: mujoco` / `yaml` | venv가 활성화되지 않았다 (프롬프트에 `(.venv)` 없음). `source sim/.venv/bin/activate` 또는 `sim/.venv/bin/python` 경로로 |
| conda의 python이 잡힌다 (`which python`이 miniconda) | venv 활성화를 conda 활성화 뒤에 하거나, 경로(`sim/.venv/bin/python`)로 직접 실행 |
| `uname -m`이 x86_64, 설치·실행이 매우 느림 | Rosetta 터미널. 터미널 앱 정보 가져오기에서 "Rosetta를 사용하여 열기" 해제 후 새 터미널 |
| `/opt/homebrew/bin/python3.12 없음` | `brew install python@3.12` 또는 `SIM_BASE_PYTHON=<python3.10 이상 경로> bash sim/setup_env.sh` |
| `[EXP-TUNE]가 아닌 값은 바꿀 수 없다` | 덮어쓸 수 있는 값은 2절 목록뿐. 다른 값은 config 파일 (실차와 공유)이나 `sim.yaml` |
| `경고: 트랙 배치 불일치` | scenario yaml의 row_x_end·collision_x 사본이 `track_layout` 계산값과 다르다. 계산값을 쓴다 (OPEN_ITEMS 8절) |
