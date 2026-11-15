"""트리거 거리 계산 (ROS 비의존). trigger_node와 시뮬이 같이 쓴다.

d_e 정의: #2의 앞부분이 차선 경계에 닿는 순간, 에고 차체 앞 끝과 충돌 지점 사이 거리.
trigger_distance = collision_x - d_e - v_ref * (trigger_latency + popout_enter_time)
"""


def trigger_distance(collision_x, d_e, v_ref, trigger_latency, popout_enter_time):
    return collision_x - d_e - v_ref * (trigger_latency + popout_enter_time)
