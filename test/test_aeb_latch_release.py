"""실험 AEB 고정 해제(latch_release_sec)와 ego 순항 속도(v_cruise)."""
from dataclasses import replace

from my_algo.aur.ego_fsm import EgoFSM, Obs
from my_algo.aur.emergency import ExperimentAEB
from my_algo.aur.params import AurParams, EXPERIMENT_OVERRIDES
from my_algo.baseline.vehicle_utils import ReactiveDriveCommand
from test_aeb_geometric import scan_with_point

GO = ReactiveDriveCommand(0.0, 4153.0, 'EGO_APPROACH')
STOP = ReactiveDriveCommand(0.0, 0.0, 'EGO_FOLLOW')


def aeb(release):
    return ExperimentAEB(latch_release_sec=release, model='geometric', path_margin=0.0, vehicle_half_width=0.135,
                         ttc_threshold=0.35, bumper_emergency_distance=0.20, lidar_to_bumper_dist=0.28)


def test_latch_releases_after_planner_stops():
    m = aeb(1.0)
    scan = scan_with_point(0.25, 0.0, lidar_to_bumper=0.28)
    assert m.check_emergency_braking(scan, 0.9, GO) and m.latched
    far = scan_with_point(0.60, 0.0, lidar_to_bumper=0.28)
    for _ in range(49):
        assert m.check_emergency_braking(far, 0.0, STOP)
    assert not m.check_emergency_braking(far, 0.0, STOP) and not m.latched


def test_latch_kept_while_moving_or_still_dangerous():
    m = aeb(1.0)
    near = scan_with_point(0.15, 0.0, lidar_to_bumper=0.28)
    assert m.check_emergency_braking(near, 0.9, GO)
    for _ in range(200):
        assert m.check_emergency_braking(near, 0.0, STOP)     # 0.20 안: 정지해도 비상
    far = scan_with_point(0.60, 0.0, lidar_to_bumper=0.28)
    for _ in range(200):
        assert m.check_emergency_braking(far, 0.3, STOP)      # 아직 움직임
    off = aeb(0.0)
    off.check_emergency_braking(near, 0.9, GO)
    for _ in range(200):
        assert off.check_emergency_braking(far, 0.0, STOP)


def test_cruise_speed_only_before_first_lead():
    p = replace(AurParams(), **dict(EXPERIMENT_OVERRIDES, W=1.0, v_cruise=1.2))
    fsm = EgoFSM(p, 'd')
    assert fsm.step(Obs(0.0, 0.0, False, None, None, None, 0.0, 1.0))[0] == 1.2
    fsm.step(Obs(0.1, 1.0, True, 2.0, 0.0, 0.0, 0.0, 1.0))
    assert fsm.step(Obs(0.2, 1.0, False, None, None, None, 0.0, 1.0))[0] == 0.0     # 본 뒤 놓치면 정지
    assert EgoFSM(replace(p, v_cruise=None), 'd').step(Obs(0.0, 0.0, False, None, None, None, 0.0, 1.0))[0] == p.v_B_nominal
