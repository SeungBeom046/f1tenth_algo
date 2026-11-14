# MuJoCo 추월 시나리오 시뮬 (Mac, ROS 없음)

`my_algo.aur`(tracker / corridor / belief / FSM / 횡 제어)을 그대로 돌린다. 직선 통로에 ego와 선행차 1대.
scan마다 MuJoCo 레이캐스트 → `pointcloud_to_laserscan` 규칙의 scan → 판단 → ERPM·서보 → 운동학 차량 모델.

```bash
# 환경: Python 3.12 + requirements.txt (예전 시뮬의 sim/.venv와 같은 버전이면 그대로 쓸 수 있다)
python sim/mujoco_overtake/run.py --method d --leader Y                              # 1회, 창 없음
mjpython sim/mujoco_overtake/run.py --method d --leader Y --viewer --start-paused    # 뷰어 (macOS는 mjpython)
python sim/mujoco_overtake/batch.py belief      # belief 2종 × 방법 c·d × 200회 (약 6분)
python sim/mujoco_overtake/batch.py yaw         # 선행차 양보 yaw 0/10/20/30°에서 tracker 추적 유지
python sim/mujoco_overtake/batch.py matrix      # 방법 4 × 선행차 Y/NY
python sim/mujoco_overtake/batch.py aeb         # 방법 c·d × NY × AEB 조건 (N_AEB)
python sim/mujoco_overtake/batch.py treact      # t_react 0~1.6 s별 survival c·d 성공률 (약 15분)
python sim/mujoco_overtake/batch.py yr          # 양보 후 복귀, 복귀 시간별
python -m pytest sim/mujoco_overtake/test_sim.py -q
```

결과 CSV는 `sim_logs/mujoco_overtake/` (git에 올리지 않는다).

## 파일

| 파일 | 내용 |
|---|---|
| `scenario.yaml` | 시뮬 전용 값 (통로, 출발 간격, 선행차 행동, 차량 응답, 라이다). TUNE 태그 → `docs/TUNING.md` |
| `world.py` | MJCF 생성: 벽 2개 + 차 2대 (mocap box) |
| `sim.py` | `Sim`: 고정 스텝 루프, scan마다 판단, 충돌 판정, 회차 결과 |
| `run.py` / `batch.py` | 1회 실행(뷰어) / 헤드리스 일괄 실행 |
| `viewer_ui.py` | 뷰어 오버레이·키 |
| `lidar.py`, `vehicle.py` | 예전 시뮬(`sim/mujoco` 브랜치)에서 그대로 가져온 라이다·운동학 모델 |

## 선행차 행동 (`scenario.yaml`의 `leader`, 또는 `--leader`, `--d-y`, `--yield-yaw`)

| 값 | 동작 |
|---|---|
| `Y` | 후행차가 `d_y` 안으로 오면 오른쪽 차선(−δ)으로 비킨다 |
| `NY` | 비키지 않는다. 후행차가 1.0 m 안으로 온 뒤 0.5~2.0 s 후 급정지 |
| `YR` | 비켰다가 `return_after`초 뒤 중앙으로 돌아온다 |
| `d_y` | 양보 반응 거리. `null`이면 seed로 U[d_y_min, d_y_max] |
| `yield_yaw_deg` | `null`: 선행차 자신의 횡 제어기가 차선을 바꾼다. 숫자: 횡위치를 `yield_lat_speed`로 옮기면서 차체 yaw를 이 값으로 고정 (tracker 강건성 테스트용, 이때 선행차-벽 충돌 판정은 뺀다) |

## 뷰어 조작 (예전 시뮬과 같은 키)

| 키 | 동작 | 예전 시뮬에서 |
|---|---|---|
| Enter | 일시정지 / 재개 | 같음 |
| `.` | 일시정지 중 scan 1프레임(0.1 s) 진행 | 같음 |
| 9 | 배속 1× ↔ 0.25× | 같음 |
| 6 | 라이다 점 (초록) 켜기/끄기 | 같음 |
| 7 | 추적 중인 선행차 표시 (주황) 켜기/끄기 | 가림 경계 표시였음 |
| 8 | 차선 ±δ (보라)·y_ref (노랑) 켜기/끄기 | phantom 표시였음 |
| End (fn + →) | 글자 패널 켜기/끄기 | 같음 |
| Home (fn + ←) | 재시작: 같은 설정·시드로 처음부터 | 같음 |
| 마우스 | 끌기 회전, 오른쪽 끌기 이동, 스크롤 확대 | 같음 |

PgDn(미관측 바닥 칠하기)은 이 시나리오에 해당 기능이 없어 쓰지 않는다.
MuJoCo 기본 Reset/Run/Pause(Backspace, Space, 왼쪽 패널)는 이 시뮬에서 동작하지 않는다.

## 실차와 다른 점 (결과를 읽을 때)

- AEB: 실차 `AEBSafetyModel`(legacy, launch 기본값)을 50 Hz로 최종 명령에 적용한다 (`scenario.yaml`의 `aeb`). 속도 입력은 `zero`(현재 실차, odom 없음) 또는 `true`. 기본은 켜짐·`zero`. **양보·추월 차선에서 벽 때문에 AEB가 서는 알려진 문제가 있어** `batch.py`는 기본으로 AEB를 끈다 (`--aeb`로 선택). N_AEB = AEB 상승 에지 수.
- 충돌은 차체 사각형으로 판정한다. 결과의 `gap_min`은 x 방향 간격이라 나란할 때 음수가 된다.
- 판단에 쓰는 속도는 차량 모델의 참값이다 (실차는 `/sensors/core`).
- 차량은 운동학 자전거 모델이다 (미끄러짐 없음). 응답 값(`t_r`, `tau`, `a_b` 등)은 가정값이다.
- 라이다는 수평 한 줄, 기본 무잡음이다 (`lidar.range_noise_std`).
- `params.t_d`는 시뮬 차량 모델의 유효 지연(0.35 s)으로 덮어쓴다. 실차 값은 보정 실험으로 정한다.
- 뷰어 창은 아직 실제로 띄워 확인하지 않았다 (오버레이 그리기와 키 처리는 창 없이 테스트).
