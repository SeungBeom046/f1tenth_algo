"""선행차(leader) 한 scan 처리: 인지 → 후방 추적 → 양보/급정지 시나리오 → 횡 제어. rclpy 없음.

실차 노드(my_algo/exp_planner.py)가 /scan마다 step()을 부른다. 순서는 시뮬 Sim._leader_frame과 같다
(sim/aur_overtake/test_sim.py가 같은 scan을 넣어 시뮬 선행차와 명령이 같은지 확인한다).
"""
from my_algo.aur.corridor import Corridor
from my_algo.aur.lateral import Lateral
from my_algo.aur.leader_logic import Leader
from my_algo.aur.tracker import VehicleTracker, scan_to_points


class LeaderRunner:
    def __init__(self, p, intent, run_id, d_y=None):
        """intent: 'Y'(양보) | 'NY'(양보 안 함, 급정지). run_id: d_y·급정지 지연 seed. d_y: 주면 seed 대신 이 값 [m]."""
        self.p = p
        self.logic = Leader(p, intent, run_id)
        if d_y is not None:
            self.logic.d_y = float(d_y)
        self.tracker = VehicleTracker(p, view='rear')
        self.corridor = Corridor(p)
        self.lateral = Lateral(p)
        self.W_fixed = p.W_leader
        self.track = None

    def step(self, t, scan, v):
        """scan: LaserScan 필드(ranges, angle_min, angle_increment, range_min, range_max). v: 자기 속도 [m/s].
        반환: (v_cmd [m/s], 조향 [rad], y_ref [m])."""
        p, cor = self.p, self.corridor
        points = scan_to_points(list(scan.ranges), scan.angle_min, scan.angle_increment,
                                scan.range_min, scan.range_max)
        if cor.W_est is None:
            cor.update(points, W_fixed=self.W_fixed)
        W = self.W_fixed or (p.W if p.W is not None else cor.W_est)
        self.track = self.tracker.update(points, t, v, cor.ego_lateral, W, cor.heading)
        cor.update(points, self.track.idx, W_fixed=self.W_fixed)
        if not self.track.valid:    # 후행차가 옆으로 오면 이어서 추적해 그 점을 벽 계산에서 뺀다
            side = self.tracker.update_side(points, t, v, cor.ego_lateral, W, cor.heading)
            cor.update(points, side.idx, W_fixed=self.W_fixed)
        if p.corridor_guardrail and not cor.valid:
            return 0.0, self.lateral.steer, self.lateral.y_ref
        v_cmd, y_ref = self.logic.step(t, v, self.track.d if self.track.valid else None, W)
        steer = self.lateral.step(cor.ego_lateral, y_ref, p.scan_period, cor.heading)
        return v_cmd, steer, y_ref
