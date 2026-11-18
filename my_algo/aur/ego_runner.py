"""후행차(ego, 추월하는 차) 한 scan 처리: 인지 → 전방/측면 추적 → EgoFSM(belief·안전 거리·상태 기계) → 횡 제어. rclpy 없음.

실차 노드(my_algo/exp_planner.py role=ego)가 /scan마다 step()을 부른다. 순서는 시뮬 Car.perceive + Sim._ego_frame과 같다
(sim/aur_overtake/test_sim.py가 같은 scan을 넣어 시뮬 ego와 명령이 같은지 확인한다).
"""
from my_algo.aur.corridor import Corridor
from my_algo.aur.ego_fsm import EgoFSM, Obs
from my_algo.aur.lateral import Lateral
from my_algo.aur.tracker import VehicleTracker, scan_to_points


class EgoRunner:
    def __init__(self, p, method):
        """method: 'a' 고정 규칙 | 'b' 소극적 belief | 'c' 공격적 탐색 | 'd' 제안."""
        self.p = p
        self.fsm = EgoFSM(p, method)
        self.tracker = VehicleTracker(p, view='front')
        self.corridor = Corridor(p)
        self.lateral = Lateral(p)
        self.track = None

    def step(self, t, scan, v):
        """scan: LaserScan 필드. v: 자기 속도 [m/s]. 반환: (v_cmd [m/s], 조향 [rad], y_ref [m], 상태, reason)."""
        p, cor = self.p, self.corridor
        points = scan_to_points(list(scan.ranges), scan.angle_min, scan.angle_increment,
                                scan.range_min, scan.range_max)
        if cor.W_est is None:
            cor.update(points)
        W = p.W if p.W is not None else cor.W_est
        side = self.fsm.state in ('PASS', 'MERGE', 'RETREAT')     # 옆으로 나란해지면 측면 추적
        args = (points, t, v, cor.ego_lateral, W, cor.heading)
        trk = self.track = self.tracker.update_side(*args) if side else self.tracker.update(*args)
        cor.update(points, trk.idx)
        if p.corridor_guardrail and not cor.valid:
            return 0.0, self.lateral.steer, self.lateral.y_ref, self.fsm.state, 'corridor_invalid'
        v_cmd, y_ref, state, reason = self.fsm.step(Obs(
            t, v, trk.valid, trk.d, trk.y_b, trk.v_b, cor.ego_lateral, W, trk.front_rel, trk.y_l))
        steer = self.lateral.step(cor.ego_lateral, y_ref, p.scan_period, cor.heading)
        return v_cmd, steer, y_ref, state, reason
