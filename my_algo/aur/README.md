# my_algo/aur — 추월 판단 로직

rclpy 없는 순수 파이썬. 실차 노드(`my_algo/exp_planner.py`, 지금은 leader 역할만)와 시뮬(`sim/aur_overtake/sim.py`)이 같은 코드를 import한다.
흐름·식·상태 전이 조건은 [docs/PIPELINE.md](../../docs/PIPELINE.md). 파라미터는 `config/exp_common.yaml` (→ [docs/TUNING.md](../../docs/TUNING.md)).

scan 한 프레임의 처리 순서대로:

| 단계 | 파일 | 역할 |
|---|---|---|
| 공통 | `params.py` | `AurParams` 모든 파라미터, `load_params`(YAML 덮어쓰기), `EXPERIMENT_OVERRIDES` |
| [1 인지] | `tracker.py` `scan_to_points` | LaserScan → 각도순 점 (라이다 좌표) |
| [1 인지] | `corridor.py` | 다른 차 점을 뺀 벽 거리, ego 횡위치, 통로 기준 헤딩 (벽 직선 맞춤), 선행차 전용 실측 W |
| [2 추적] | `tracker.py` `VehicleTracker` | 상대 차 클러스터: 거리 d, 중심 y_B, 왼쪽 모서리 y_L, 속도 v_B (α-β). 헤딩 보정, 측면 추적(PASS·MERGE·RETREAT), 후방 뷰(선행차용) |
| [3 belief] | `belief.py` | 양보 의도 belief λ, 식 1~5 (`original` / `survival`) |
| [4 판단] | `safety.py` | 식 6~11: d_safe, h, v_lim, d*, 횡 여유, 차선 오프셋 δ |
| [4 판단] | `ego_fsm.py` | 후행차 상태 기계 WAIT → APPROACH → PROBE → PASS → MERGE → DONE / FOLLOW / RETREAT, 방법 a·b·c·d의 속도 명령 |
| [4 판단] | `leader_logic.py` | 선행차 행동 Y(양보) / NY(급정지 이벤트) / 양보 후 복귀(시뮬용) |
| [1~5] | `leader_runner.py` | 선행차 한 scan: 인지 → 후방 추적 → `leader_logic` → 횡 제어. 실차 `exp_planner`(role=leader)가 부른다 |
| [5 제어] | `lateral.py` | 횡: y_ref를 따라가는 조향 (오차 필터, P + 헤딩 감쇠, deadband, rate limit) |
| [5 제어] | `speed.py` | 종: m/s ↔ ERPM, 최소 속도 규칙 (0 < v < v_min 처리) |
| [6 AEB] | `../baseline/aeb.py` | AEB 판정 (실차에서는 `autonomous_drive`가 적용). 추월 실험은 `geometric` 모드 |

단위 테스트: `test/test_aur_*.py`, `test/test_aeb_geometric.py` (`python3 -m pytest -q`).
