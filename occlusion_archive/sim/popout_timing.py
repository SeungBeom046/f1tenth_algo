"""시뮬 #2 모델의 진입 시간 (Exp0 f의 시뮬판). 시뮬에서만 trigger_node의 popout_enter_time으로 쓴다.

실차 Exp0 f와 같은 방법으로 잰다: 실차 popout_vehicle_node를 가짜 rclpy로 띄우고 /trigger를 보낸 뒤,
노드가 기록하는 reach_lane_s(트리거 수신 → lane_boundary_distance 주행)를 읽는다. #2 차량은 runner와 같은
vehicle.Kinematic (sim.yaml car2·vehicle 값), 스텝 순서도 runner와 같다 (차량 갱신 → odom → 라우팅 → 타이머 → 명령).
노드 타이머(50 Hz)와 트리거 시각의 위상에 따라 최대 한 주기 차이가 나므로 위상 5개의 평균을 쓴다.
시뮬의 /trigger 전달 지연은 0이다 (같은 프로세스) → 시뮬 trigger_latency = 0.
"""
import math

import rosstub
import vehicle


def enter_time(cfg, dt, phases=5):
    """반환 {'popout_enter_time': 평균 [s], 'samples': [...], 'trigger_latency': 0.0}."""
    import my_algo.popout_vehicle_node as po
    from nav_msgs.msg import Odometry
    from std_msgs.msg import Header

    rn = rosstub.install(force=False)
    c, s = cfg.common, cfg.sim
    c2, vc = s['car2'], s['vehicle']
    samples = []
    for phase in range(phases):
        rosstub.set_time(rn, 0.0)
        rn.BUS.clear()
        node = rosstub.make_node(rn, po.PopoutVehicleNode, cfg.popout_params(''))
        node.finish = lambda reason, elapsed: setattr(node, 'state', 'stopped')   # 파일 기록 없이
        car = vehicle.Kinematic(0.0, 0.0, math.pi / 2, 0.33, c2['t_r'], vc['tau'], vc['a_max'], float(c['a_b']),
                                vc['steer_rate'], erpm_gain=float(cfg.popout['erpm_gain']),
                                servo_center=float(cfg.popout['servo_center']))
        t_trig = 0.5 + phase * dt
        for k in range(int(3.0 / dt)):
            t = round((k + 1) * dt, 9)
            rosstub.set_time(rn, t)
            car.step(t, dt)
            o = Odometry()
            o.pose.pose.position.x, o.pose.pose.position.y = car.x, car.y
            o.twist.twist.linear.x = car.v
            rn.BUS.append(('odom', o))
            if abs(t - t_trig) < 1e-9:
                h = Header()
                h.stamp.sec, h.stamp.nanosec = int(t), int(round((t % 1) * 1e9))
                rn.BUS.append(('/trigger', h))
            motor = []
            for _ in range(2):      # 라우팅 → 타이머 → 라우팅 (runner와 같은 순서)
                for topic, msg in rosstub.take(rn):
                    if topic in node.subs:
                        node.subs[topic](msg)
                    if topic == 'commands/motor/speed':
                        motor.append(msg.data)
                if _ == 0:
                    rosstub.run_timers(rn, node, t)
            if motor:
                car.command(t, motor[-1], car.servo_now)
            if 'reach_lane_s' in node.event:
                samples.append(node.event['reach_lane_s'])
                break
    rosstub.set_time(rn, 0.0)
    rn.BUS.clear()
    return {'popout_enter_time': sum(samples) / len(samples) if samples else None,
            'samples': samples, 'trigger_latency': 0.0}
