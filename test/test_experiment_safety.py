from dataclasses import replace

from _aur_synth import box, scan
from my_algo.aur.emergency import ExperimentAEB
from my_algo.aur.ego_fsm import EgoFSM, Obs
from my_algo.aur.lateral import Lateral
from my_algo.aur.params import AurParams
from my_algo.aur.tracker import VehicleTracker
from my_algo.baseline.vehicle_utils import ReactiveDriveCommand
from test_aeb_geometric import scan_with_point
from test_aur_tracker import lead_box

P = AurParams(experiment_safety=True)


def test_command_speed_catches_obstacle_with_zero_odom_and_latches():
    model = ExperimentAEB(model='geometric', ttc_threshold=0.35, path_margin=0.0)
    cmd = ReactiveDriveCommand(0.0, 1.5 * P.erpm_gain)
    assert model.check_emergency_braking(scan_with_point(0.4, 0.0), 0.0, cmd)
    empty = scan_with_point(10.0, 2.0)
    assert model.check_emergency_braking(empty, 0.0, cmd)
    assert model.last_reason.startswith('latched ')
    model.reset()
    assert not model.check_emergency_braking(empty, 0.0, cmd)


def test_new_target_speed_is_conservative_and_far_jump_is_rejected():
    tracker = VehicleTracker(P)
    target = tracker.update(scan(1.0, 0.0, [lead_box(0.4, 0.0)]), 0., 0.9, 0., 1.)
    assert target.valid and target.v_b == 0.0
    assert not tracker.update(scan(1.0, 0.0, [lead_box(1.5, 0.0)]), .1, .9, 0., 1.).valid


def test_track_loss_does_not_return_to_cruise_after_timeout():
    fsm = EgoFSM(P, 'd')
    fsm.step(Obs(0., .9, True, .4, 0., 0., 0., 1.))
    for t in (.1, .6, 3.):
        v, _, _, reason = fsm.step(Obs(t, .9, False, None, None, 0., 0., 1.))
        assert v == 0.0 and reason == 'front_track_lost'


def test_narrow_same_mount_lidar_return_has_rear_bumper_margin():
    tracker = VehicleTracker(P)
    tracker.update(scan(1., 0., [lead_box(.8, 0.)]), 0., 0., 0., 1.)
    part = box(P.lidar_to_front + .82, P.lidar_to_front + .88, -.03, .03)
    target = tracker.update(scan(1., 0., [part]), .1, 0., 0., 1.)
    assert target.valid and abs(target.d - .60) < .02


def test_partial_target_margin_stops_before_lidar_only_surface():
    from test_aur_guardrail import scan_message
    points = scan(1., 0., [box(.7, .76, -.03, .03)], n=1440)
    msg = scan_message(points)
    model = ExperimentAEB(model='geometric', path_margin=0., ttc_threshold=.35)
    cmd = ReactiveDriveCommand(0., .9 * P.erpm_gain)
    assert model.check_emergency_braking(msg, 0., cmd)
    assert 'partial_target_margin' in model.last_reason


def test_car_corner_clipped_by_aeb_strip_is_not_a_narrow_sensor():
    from test_aur_guardrail import scan_message
    points = scan(1., 0., [box(.7, .76, -.05, .22)], n=1440)
    model = ExperimentAEB(model='geometric', path_margin=0., ttc_threshold=.35)
    assert not model.check_emergency_braking(scan_message(points), 0.,
                                            ReactiveDriveCommand(0., .9 * P.erpm_gain))


def test_heading_jitter_is_reduced_without_ignoring_turns():
    # 감쇠항을 실제로 사용하는 실험 설정으로 비교한다.
    old = Lateral(replace(P, experiment_safety=False, heading_gain=1., steering_deadband=0.))
    new = Lateral(replace(P, heading_gain=1., steering_deadband=0.))
    old_values, new_values = [], []
    for k in range(60):
        heading = .06 if k % 2 else -.06
        old_values.append(old.step(0., 0., .1, heading))
        new_values.append(new.step(0., 0., .1, heading))
    assert sum(abs(x) for x in new_values[10:]) < .6 * sum(abs(x) for x in old_values[10:])
    for _ in range(10):
        steer = new.step(0., 0., .1, .1)
    assert steer < -.09
