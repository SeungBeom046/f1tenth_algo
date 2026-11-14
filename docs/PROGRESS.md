# exp/aur-overtake 진행 상황

다음 세션은 `CLAUDE_TASK.md`와 이 파일만 읽고 시작한다.

## 끝낸 단계
- 세션 1 (2026-11-14): 0단계(점검), 1단계(`my_algo/aur/` 순수 모듈 + 테스트 38개 통과, Python 3.8.20). 커밋은 `git log --oneline -3` 참고.

## 남은 일
- 세션 2: 4단계(속도 소스), 5단계(AEB relative). 이후 지시서 표대로.
- 0단계에서 고른 재사용 자산은 아직 가져오지 않았다. 쓰는 단계에서 `git checkout origin/exp/occlusion-speed -- <path>`:
  `scripts/bag_player.py`(+`test/test_bag_player.py`), `analysis/bag_io.py`(sqlite3 bag 읽기, 8단계), `scripts/env_ego.sh`(`ROS_DOMAIN_ID` 방식, 7단계).

## 다음 세션이 알아야 할 결정사항
1. 작업 위치는 worktree `f1tenth_gym_ros/my_algo_aur` (원래 체크아웃 `my_algo`는 sim/mujoco에 미커밋 변경이 있어 건드리지 않음). 테스트는 Python 3.8 환경으로 `python -m pytest test/ -q` (Mac 기본 python에는 pytest 없음).
2. 함수는 모두 파라미터 객체 `p`(AurParams)를 마지막 인자로 받는다: `d_safe(v_e, v_b, p)` 등. YAML은 최상위 `aur:` 키, 모르는 키는 오류.
3. **[사용자 확인 필요] belief 모델(식 2~5)이 Y를 NY로 오판한다.** 접근 중 "아직 양보 안 함"(z=0)을 scan마다 독립 증거로 곱해서, 선행차가 반응하기 전에 b ≤ 1−η가 된다. 노이즈 없는 폐루프에서 방법 c·d의 Y 성공률 115/200 (d_y < 0.87 m이면 전부 FOLLOW). 양보 동작이 끝난 뒤의 z=0도 NY 증거로 쌓인다. 식은 기획안대로 두었다.
4. 판단은 APPROACH·PROBE 모두에서 하고, b가 η 또는 1−η를 처음 넘은 쪽으로 고정(`fsm.decided`)한다. PROBE 진입은 `d ≤ d_ref + probe_tol`. T_wait_a·T_max는 PROBE 진입부터 센다 (PROBE에 못 들어가면 타이머가 안 돈다).
5. 최소 속도 규칙의 hold는 달리는 중(v_E > 0.3)에만 한다. 정지 중에는 h가 항상 커서, 그대로면 선 차 뒤에서 v_min으로 튀어나갔다 서기를 반복한다. `hold_count`는 분기 진입 횟수.
6. `corridor`의 `min_point_x` 기본값을 −1.0으로 바꿨다 (기존 −0.05면 벽 거리 < 0.25 m에서 벽을 놓치고, 추월 차선의 벽 거리가 0.235 m다). 기존 `lidar_reactive_drive`는 그대로.
7. PASS에서 선행차와 중심 간격이 w + c가 되기 전에는 v_pass가 아니라 방법별 추종 속도를 쓴다 (W=1.0이면 중앙의 ego와 양보한 선행차가 횡으로 겹친다).
8. tracker는 헤딩이 통로와 나란하다고 가정하고 `|y + ego_lateral| ≥ W/2 − 0.05`인 점을 벽으로 버린다. 선행차가 양보하며 20° 넘게 틀면 클러스터 폭이 0.45 m를 넘어 추적이 끊길 수 있다 (미확인, 실차·시뮬에서 볼 것). 측면 추적의 y_B는 보이는 면 중앙이라 최대 w/2 치우친다.
9. `exp_planner`가 매 scan에 부를 순서: `tracker.update(points, t, v_E, corridor.ego_lateral, W)` → `corridor.update(points, track.idx)` → `fsm.step(Obs(...))`. PASS·MERGE에서는 `update_side`. 2단계의 "차량이 아닌 정지 물체 → STOP"은 아직 없다.
10. `CLAUDE_TASK.md`와 기획안 PDF는 커밋하지 않았다 (PDF는 저장소에 없고, 공개 저장소에 연구 계획을 올릴지는 사용자 결정).
