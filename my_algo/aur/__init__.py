"""의도 불확실 선행차 안전 추월 실험(exp/aur-overtake)의 순수 파이썬 판단 로직.

rclpy에 의존하지 않는다. 실차 노드(exp_planner)와 시뮬(sim/aur)이 같은 코드를 import한다.
식 번호는 기획안과 같다. 좌표 규약: x 전방, y 왼쪽 양수.
"""
