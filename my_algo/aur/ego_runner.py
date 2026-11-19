"""후행차(ego, 추월하는 차) 한 scan 처리: 인지 → 전방/측면 추적 → EgoFSM(belief·안전 거리·상태 기계) → 횡 제어. rclpy 없음.

실차 노드(my_algo/exp_planner.py role=ego)가 /scan마다 step()을 부른다. 순서는 시뮬 Car.perceive + Sim._ego_frame과 같다
(sim/aur_overtake/test_sim.py가 같은 scan을 넣어 시뮬 ego와 명령이 같은지 확인한다).
"""
from my_algo.aur.corridor import Corridor
from my_algo.aur.ego_fsm import EgoFSM, Obs
from my_algo.aur.fallback import free_space_lateral
from my_algo.aur.lateral import Lateral
from my_algo.aur.pose import PoseFilter
from my_algo.aur.tracker import VehicleTracker, scan_to_points


class EgoRunner:
    def __init__(self, p, method):
        """method: 'a' 고정 규칙 | 'b' 소극적 belief | 'c' 공격적 탐색 | 'd' 제안."""
        self.p = p
        self.fsm = EgoFSM(p, method)
        self.tracker = VehicleTracker(p, view='front')
        self.corridor = Corridor(p)
        self.lateral = Lateral(p)
        self.pose = PoseFilter(p)
        self.track = None
        self.t_rail = None          # 마지막으로 레일이 유효했던 시각 (rail_fallback)

    def step(self, t, scan, v, yaw_rate=None):
        """scan: LaserScan 필드. v: 자기 속도 [m/s]. yaw_rate: 자이로 [rad/s] (이 scan 주기 평균, 없으면 None).
        반환: (v_cmd [m/s], 조향 [rad], y_ref [m], 상태, reason)."""
        p, cor = self.p, self.corridor

        def update(*args, **kwargs):
            cor.update(*args, **kwargs)
            self.pose.apply(cor, t, v, yaw_rate)
        points = scan_to_points(list(scan.ranges), scan.angle_min, scan.angle_increment,
                                scan.range_min, scan.range_max)
        fresh_wall = p.track_recovery and p.experiment_safety
        if cor.W_est is None and not (fresh_wall and p.guardrail_local_view):
            update(points, stamp=t)
        W = p.W if p.W is not None else cor.W_est
        tracking_points = points
        if fresh_wall:
            update(points, stamp=t)
            tracking_points = [q for q in points if q.i not in cor.wall_idx]
            if p.guardrail_local_view and not cor.valid:
                tracking_points = []
        side = self.fsm.state in ('PASS', 'MERGE', 'RETREAT')     # 옆으로 나란해지면 측면 추적
        args = (tracking_points, t, v, cor.ego_lateral, W, cor.heading)
        trk = self.track = self.tracker.update_side(*args) if side else self.tracker.update(*args)
        if not (fresh_wall and p.guardrail_local_view) or cor.sector_idx.intersection(trk.idx):
            update(points, trk.idx, stamp=t)
        if cor.valid:
            self.t_rail = t
        if p.corridor_guardrail and not cor.valid:
            fallback = rail_fallback(self, t, points, W, v)
            if fallback is not None:
                return fallback
            reason = 'corridor_initializing' if cor.initializing else 'corridor_invalid'
            return 0.0, self.lateral.steer, self.lateral.y_ref, self.fsm.state, reason
        if self.tracker.pending_d is not None:
            return 0.0, self.lateral.steer, self.lateral.y_ref, self.fsm.state, 'front_track_pending'
        v_cmd, y_ref, state, reason = self.fsm.step(Obs(
            t, v, trk.valid, trk.d, trk.y_b, trk.v_b, cor.ego_lateral, W, trk.front_rel, trk.y_l))
        steer = self.lateral.step(cor.ego_lateral, y_ref, p.scan_period, cor.heading,
                                  integrate=not p.guardrail_local_view or v > p.t0_speed_threshold)
        return v_cmd, steer, y_ref, state, reason


def rail_fallback(car, t, points, W, v):
    """레일 무효 동안의 대체 주행 (실차 runner·시뮬 공용). car: fsm·tracker·corridor·lateral·t_rail를 가진 객체.
    앞차를 아직 못 봤고(추적 이력 없음) 추월 전이며 마지막 레일 뒤 rail_fallback_s 안일 때만,
    빈 공간 횡위치로 기존 횡 제어기를 돌려 rail_fallback_v로 간다. 반환: runner.step과 같은 튜플 또는 None."""
    p, fsm = car.p, car.fsm
    if p.rail_fallback_v is None or car.t_rail is None or t - car.t_rail > p.rail_fallback_s:
        return None
    if fsm.state not in ('WAIT', 'APPROACH') or fsm._last is not None or car.tracker.pending_d is not None:
        return None
    lateral = free_space_lateral(points, W, p)
    if lateral is None:
        return None
    steer = car.lateral.step(lateral, 0.0, p.scan_period, car.corridor.heading, integrate=False)
    return p.rail_fallback_v, steer, car.lateral.y_ref, fsm.state, 'rail_fallback'
