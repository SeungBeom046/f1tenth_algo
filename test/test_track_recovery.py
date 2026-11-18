from dataclasses import replace

from _aur_synth import scan
from my_algo.aur.corridor import Corridor
from my_algo.aur.ego_runner import EgoRunner
from my_algo.aur.params import AurParams, EXPERIMENT_OVERRIDES
from my_algo.aur.tracker import VehicleTracker
from test_aur_guardrail import scan_message
from test_aur_tracker import lead_box

P = replace(AurParams(), **dict(EXPERIMENT_OVERRIDES, track_recovery=True, W=1.0))


def test_short_false_detection_does_not_lock_empty_corridor():
    runner = EgoRunner(P, 'd')
    hit = scan_message(scan(1., 0., [lead_box(1., 0.)], n=1440))
    empty = scan_message(scan(1., 0., n=1440))
    assert runner.step(0., hit, 0.)[-1] == 'front_track_pending'
    assert runner.fsm._last is None
    for t in (.1, .6, 3.):
        result = runner.step(t, empty, 0.)
        assert result[0] > 0.0 and result[-1] != 'front_track_lost'
        assert runner.corridor.valid


def test_initial_vehicle_requires_consecutive_frames_and_stops_during_confirmation():
    runner = EgoRunner(P, 'd')
    msg = scan_message(scan(1., 0., [lead_box(1., 0.)], n=1440))
    for t in (0., .1):
        result = runner.step(t, msg, 0.)
        assert result[0] == 0.0 and result[-1] == 'front_track_pending'
    runner.step(.2, msg, 0.)
    assert runner.track.valid and runner.fsm._last is not None
    empty = scan_message(scan(1., 0., n=1440))
    assert runner.step(.3, empty, .9)[-1] == 'front_track_lost'


def test_same_close_target_can_be_reacquired_after_long_loss_with_speed_input():
    points = scan(1., 0., [lead_box(.4, 0.)])
    old = VehicleTracker(replace(P, track_recovery=False))
    old.update(points, 0., 0., 0., 1.)
    assert not old.update(points, 1.5, .9, 0., 1.).valid
    tracker = VehicleTracker(P)
    for t in (0., .1, .2):
        tracker.update(points, t, 0., 0., 1.)
    assert tracker.update(points, 1.5, .9, 0., 1.).valid
    far = scan(1., 0., [lead_box(1.5, 0.)])
    assert not tracker.update(far, 1.6, 0., 0., 1.).valid
    assert tracker.status == 'distance_jump'


def test_selected_guardrails_are_removed_from_vehicle_input():
    points = scan(1., .2, n=1440)
    cor = Corridor(P).update(points)
    assert cor.valid and cor.wall_idx
    filtered = [q for q in points if q.i not in cor.wall_idx]
    assert not VehicleTracker(P).update(filtered, 0., 0., cor.ego_lateral, 1., cor.heading).valid
