"""의도 불확실 선행차 안전 추월 실험(exp/aur-overtake)의 순수 파이썬 판단 로직.

rclpy에 의존하지 않는다. 실차 노드(exp_planner, 아직 없음)와 시뮬(sim/aur_overtake)이 같은 코드를 import한다.
식 번호는 기획안과 같다. 좌표 규약: x 전방, y 왼쪽 양수. 흐름 전체는 docs/PIPELINE.md.

scan 한 프레임의 처리 순서 (모듈 머리의 [n]):
    [1 인지]   tracker.scan_to_points (scan → 점), corridor.Corridor (벽 거리, ego 횡위치, 헤딩)
    [2 추적]   tracker.VehicleTracker (상대 차 d, y_B, v_B)
    [3 belief] belief.Belief (식 1~5)
    [4 판단]   ego_fsm.EgoFSM (후행차 상태 기계), leader_logic.Leader (선행차), safety (식 6~11)
    [5 제어]   lateral.Lateral (횡), speed.min_speed_rule (종 출력 규칙)
    [6 AEB]    my_algo/baseline/aeb.py (ROS 쪽 autonomous_drive가 적용)
    params: 모든 단계의 파라미터 (config/exp_common.yaml)
"""
