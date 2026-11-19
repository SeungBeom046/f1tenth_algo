"""시뮬 자체 점검 (mujoco가 있는 환경에서만): python -m pytest sim/aur_overtake/test_sim.py -q"""
import math
import pytest

mujoco = pytest.importorskip('mujoco')

from sim import Sim, load_cfg       # noqa: E402


def run(method, behavior, **over):
    """판단 로직 점검용: AEB 끔 (AEB는 아래 별도 테스트)."""
    base = {'leader.behavior': behavior, 'leader.d_y': 1.0, 'aeb.enabled': False}
    return Sim(method, load_cfg(**dict(base, **over))).run()


def test_events_show_leader_yield_before_ego_pass():
    sim = Sim('d', load_cfg(**{'leader.behavior': 'Y', 'leader.d_y': 1.0, 'aeb.enabled': False}))
    sim.run()
    ev = sim.events
    yield_i = next(i for i, e in enumerate(ev) if '양보 시작' in e)
    pass_i = next(i for i, e in enumerate(ev) if '→ PASS' in e)
    assert yield_i < pass_i and any('belief 확정 Y' in e for e in ev)


@pytest.mark.parametrize('behavior', ['Y', 'NY'])
def test_leader_runner_matches_sim_leader(behavior):
    # 실차 노드가 쓰는 LeaderRunner에 시뮬 선행차와 같은 scan·속도를 넣으면 같은 명령이 나와야 한다
    from my_algo.aur.leader_runner import LeaderRunner
    sim = Sim('d', load_cfg(**{'leader.behavior': behavior, 'leader.d_y': 1.0}))
    runner = LeaderRunner(sim.p, behavior, sim.seed, d_y=1.0)
    frame, out = sim._leader_frame, []

    def checked():
        t, v = sim.t, sim.leader.kin.v
        frame()
        rv, steer, _ = runner.step(t, sim.leader.scan_ns, v, sim.leader.yaw_rate)
        out.append((rv * sim.p.erpm_gain, steer) == sim.leader.cmd)
    sim._leader_frame = checked
    sim.run()
    assert len(out) > 50 and all(out)
    assert runner.logic.yield_triggered == (behavior == 'Y') and runner.logic.brake_fired == (behavior == 'NY')


@pytest.mark.parametrize('behavior', ['Y', 'NY'])
@pytest.mark.parametrize('recovery', [False, True])
@pytest.mark.parametrize('local_view', [False, True])
def test_ego_runner_matches_sim_ego(behavior, recovery, local_view):
    # 실차 노드가 쓰는 EgoRunner에 시뮬 ego와 같은 scan·속도를 넣으면 같은 명령·상태가 나와야 한다
    from my_algo.aur.ego_runner import EgoRunner
    sim = Sim('d', load_cfg(**{'leader.behavior': behavior, 'leader.d_y': 1.0}),
              track_recovery=recovery, guardrail_local_view=local_view)
    runner = EgoRunner(sim.p, 'd')
    frame, out = sim._ego_frame, []

    def checked():
        t, v = sim.t, sim.ego.kin.v
        res = frame()
        rv, steer, y_ref, state, reason = runner.step(t, sim.ego.scan_ns, v, sim.ego.yaw_rate)
        out.append((rv, y_ref, state, reason) == res and (rv * sim.p.erpm_gain, steer) == sim.ego.cmd)
        return res
    sim._ego_frame = checked
    r = sim.run()
    assert len(out) > 50 and all(out)
    assert runner.fsm.state == r['final_state'] and runner.fsm.decided == behavior


def test_legacy_aeb_stops_on_wall_in_yield_lane():
    # 기존 주행 AEB(띠 ±0.30 m)는 양보 차선 벽(여유 0.235 m)에서 선다. 그래서 실험은 geometric(±0.16)을 쓴다.
    r = Sim('d', load_cfg(**{'leader.behavior': 'Y', 'leader.d_y': 1.0, 'sim.max_time': 8.0, 'aeb.model': 'legacy',
                             'aeb.path_margin': 0.15, 'aeb.speed_source': 'zero'})).run()
    assert r['N_AEB_leader'] >= 1 and not r['success']


def test_geometric_aeb_does_not_stop_leader_on_wall():
    # 기하 AEB(±0.16)는 양보 차선 벽에서 선행차를 세우지 않는다 (ego MERGE 전까지 선행차 AEB 0).
    sim = Sim('d', load_cfg(**{'leader.behavior': 'Y', 'leader.d_y': 1.0}))
    fired = []

    def hook(s):
        if s.leader.aeb_active and s.fsm.state not in ('MERGE', 'DONE'):
            fired.append(s.t)
    r = sim.run(hook=hook)
    assert not fired and r['collision'] == ''


def test_merge_does_not_trigger_leader_aeb():
    # merge_gap 0.35 + merge_v 1.5 전에는 0.15 m 간격·v_B_nominal로 끼어들어 선행차 AEB(0.31 m)가 0.27 m에서 걸렸다
    r = Sim('d', load_cfg(**{'leader.behavior': 'Y', 'leader.d_y': 1.0})).run()
    assert r['success'] and r['N_AEB_leader'] == 0


def test_proposed_passes_yielding_leader():
    r = run('d', 'Y')
    assert r['success'] and r['decided'] == 'Y' and not r['collision'] and r['track_lost'] == 0


def test_proposed_follows_and_survives_brake():
    r = run('d', 'NY')
    assert r['final_state'] == 'FOLLOW' and not r['pass_entered'] and not r['collision']
    assert r['gap_min'] > 0.15


def test_aeb_flicker_while_stopped_not_counted_as_moving():
    # NY 급정지 뒤 앞차 바로 뒤에 서 있는 동안 라이다 잡음으로 AEB가 깜빡인다: N_AEB에는 들어가고 N_AEB_moving에는 안 들어간다
    # 정지 거리 0.25 m (2026-11-17 이전 값): 0.20이면 정지 간격 약 0.27 m 밖이라 깜빡임이 안 생긴다
    # 이 검사는 정지 중 상승 에지 집계용이다. 깜빡임을 재현하는 기존 벽 추정 조건도 고정한다.
    r = Sim('d', load_cfg(**{'leader.behavior': 'NY', 'leader.d_y': 1.0, 'lidar.range_noise_std': 0.02,
                             'aeb.bumper_emergency_distance': 0.25}), corridor_guardrail=False,
            experiment_safety=False, guardrail_local_view=False, eps_follow=0.05).run()
    assert r['N_AEB'] >= 3 and r['N_AEB_moving'] <= 1 and not r['collision']


def test_passive_times_out():
    r = run('b', 'Y')
    assert r['cause'] == 'T_max' and r['decided'] == ''


def test_deterministic():
    assert run('d', 'Y') == run('d', 'Y')


def test_viewer_overlay_draws_without_window():
    import contextlib
    from viewer_ui import KEY_6, KEY_ENTER, ViewerUI
    sim = Sim('d', load_cfg())

    class Handle:
        user_scn = mujoco.MjvScene(sim.m, 3000)
        texts = None
        def lock(self): return contextlib.nullcontext()
        def set_texts(self, t): self.texts = t
        def clear_texts(self): self.texts = None
        def sync(self): pass
        def is_running(self): return True

    ui = ViewerUI(sim, sleep=lambda s: None)
    ui.attach(Handle())
    for _ in range(30):
        sim.step()
        assert ui.after_step(sim)
    assert ui.handle.user_scn.ngeom > 100 and ui.handle.texts
    ui.on_key(KEY_6)
    ui.after_step(sim)
    assert ui.handle.user_scn.ngeom < 10            # 라이다 점 끔
    ui.on_key(KEY_ENTER)
    ui._apply_keys()
    assert ui.paused


def test_viewer_draws_at_render_rate_and_cycles_speed():
    import contextlib
    from viewer_ui import KEY_0, KEY_9, RENDER_HZ, ViewerUI
    sim = Sim('d', load_cfg())
    syncs = []

    class Handle:
        user_scn = mujoco.MjvScene(sim.m, 3000)
        def lock(self): return contextlib.nullcontext()
        def set_texts(self, t): pass
        def clear_texts(self): pass
        def sync(self): syncs.append(now[0])
        def is_running(self): return True

    now = [0.0]

    def sleep(dt):
        now[0] += dt
    ui = ViewerUI(sim, clock=lambda: now[0], sleep=sleep)
    ui.attach(Handle())
    for _ in range(100):                # 시뮬 1 s, 1x: 스텝마다(100번)가 아니라 약 RENDER_HZ번만 그린다
        sim.step()
        assert ui.after_step(sim)
    assert 0.8 * RENDER_HZ <= len(syncs) <= 1.1 * RENDER_HZ and abs(now[0] - 1.0) < 0.02
    speeds = []
    for _ in range(4):
        ui.on_key(KEY_9)
        ui._apply_keys()
        speeds.append(ui.speed)
    assert speeds == [2.0, 5.0, 0.5, 1.0]
    ui.on_key(KEY_9)
    ui.on_key(KEY_0)
    ui._apply_keys()
    assert ui.speed == 1.0


def test_level_scan_removes_floor_seen_by_pitched_lidar():
    # 실측 라이다: 바닥에서 0.24 m, 앞으로 3.2° 숙임. 그대로 자르면 앞 약 4.3 m 바닥이 /scan에 장애물로 들어온다
    import math
    import numpy as np
    over = {'lidar.mount_xyz': [0.13, 0.0, 0.24], 'lidar.mount_rpy': [0.0, 0.0557, 0.0], 'start.gap0': 12.0,
            'lidar.rays_per_frame': 1440, 'lidar.vfov_min_deg': 0.0, 'lidar.vfov_max_deg': 0.0}     # 수평 1줄

    def ahead(level):
        sim = Sim('d', load_cfg(**dict(over, **{'scan.level': level})))
        sim.ego.perceive(sim, False)
        r = np.array(sim.ego.scan_ns.ranges)
        i = int(round((0.0 - sim.ego.scan_ns.angle_min) / sim.ego.scan_ns.angle_increment))
        return float(np.min(r[i - 5:i + 6]))
    assert abs(ahead(False) - 0.24 / math.tan(0.0557)) < 0.2       # 바닥 (약 4.3 m)
    assert ahead(True) > 8.0                                         # 앞차(12 m)까지 비어 있음


@pytest.mark.parametrize('behavior', ['Y', 'NY'])
def test_low_guardrails_inside_wider_hallway(behavior):
    # 실제 높이로 레이캐스트한다. 위로 지나간 광선은 바깥 복도 벽에 맞아 같은 scan에 섞인다.
    cfg = load_cfg(**{'track.wall_height': 0.20, 'leader.behavior': behavior, 'leader.d_y': 1.0})
    cfg['track'].update(outer_width=2.7, outer_center_y=0.35)
    sim = Sim('d', cfg, W=1.0, corridor_guardrail=True)
    r = sim.run()
    assert not r['collision'] and not r['deadlock']
    if behavior == 'Y':
        assert r['success'] and r['N_AEB_moving'] == r['N_AEB_leader_moving'] == 0
    else:
        assert r['decided'] == 'NY' and not r['pass_entered'] and r['stop_gap_ok']


def test_low_guardrail_wall_collision_is_reproduced_when_disabled():
    cfg = load_cfg(**{'track.wall_height': 0.20, 'leader.d_y': 1.0})
    cfg['track'].update(outer_width=2.7, outer_center_y=0.35)
    r = Sim('d', cfg, W=1.0, corridor_guardrail=False, experiment_safety=False, guardrail_local_view=False).run()
    assert r['collision'] == 'ego_wall'


@pytest.mark.parametrize('recovery', [False, True])
@pytest.mark.parametrize('local_view', [False, True])
@pytest.mark.parametrize('pose_filter', [False, True])
def test_guardrail_runners_match_sim_with_hallway_points(recovery, local_view, pose_filter):
    from my_algo.aur.ego_runner import EgoRunner
    from my_algo.aur.leader_runner import LeaderRunner
    cfg = load_cfg(**{'track.wall_height': 0.20, 'leader.d_y': 1.0})
    cfg['track'].update(outer_width=2.7, outer_center_y=0.35)
    sim = Sim('d', cfg, W=1.0, corridor_guardrail=True, track_recovery=recovery,
              guardrail_local_view=local_view, pose_filter=pose_filter)
    ego = EgoRunner(sim.p, 'd')
    leader = LeaderRunner(sim.p, 'Y', sim.seed, d_y=1.0)
    ego_frame, leader_frame = sim._ego_frame, sim._leader_frame
    frames = []

    def checked_ego():
        result = ego_frame()
        rv, steer, y_ref, state, reason = ego.step(sim.t, sim.ego.scan_ns, sim.ego.kin.v, sim.ego.yaw_rate)
        assert (rv, y_ref, state, reason) == result
        assert (rv * sim.p.erpm_gain, steer) == sim.ego.cmd
        frames.append(sim.t)
        return result

    def checked_leader():
        leader_frame()
        rv, steer, _ = leader.step(sim.t, sim.leader.scan_ns, sim.leader.kin.v, sim.leader.yaw_rate)
        assert (rv * sim.p.erpm_gain, steer) == sim.leader.cmd

    sim._ego_frame, sim._leader_frame = checked_ego, checked_leader
    assert sim.run()['success'] and len(frames) > 100


@pytest.mark.parametrize('height', [0.12, 0.25])
def test_stationary_vehicle_bypass_with_same_lidar_mount(height):
    # 전후 차량의 라이다 장착은 동일하다. 낮은 차체도 높이 필터 안에 들어와야 한다.
    cfg = load_cfg(**{'start.gap0': 2.5, 'vehicle.height': height})
    sim = Sim('a', cfg, experiment_safety=True, stationary_pass=True)
    sim.leader.kin.y = -0.265
    sim.place_bodies()

    def stationary():
        sim.leader.perceive(sim, False)
        sim.leader.drive(sim, 0.0, -0.265)
    sim._leader_frame = stationary
    r = sim.run()
    assert r['success'] and not r['collision'] and not r['deadlock']
    assert r['N_AEB_moving'] == 0


@pytest.mark.parametrize('height', [0.12, 0.25])
def test_stationary_vehicle_stops_method_d_with_zero_aeb_speed(height):
    sim = Sim('d', load_cfg(**{'aeb.speed_source': 'zero', 'vehicle.height': height}), experiment_safety=True)
    sim.leader.kin.y = -0.265
    sim.place_bodies()

    def stationary():
        sim.leader.perceive(sim, False)
        sim.leader.drive(sim, 0.0, -0.265)
    sim._leader_frame = stationary
    r = sim.run()
    assert not r['collision'] and not r['pass_entered'] and r['gap_min'] >= sim.p.m_stop
    assert sim.ego.kin.v < 0.01


@pytest.mark.parametrize('seed', [0, 1])
def test_noisy_low_guardrails_do_not_create_permanent_front_loss(seed):
    cfg = load_cfg(**{'track.wall_height': 0.20, 'start.gap0': 20.0,
                      'start.leader_delay': 30.0, 'sim.max_time': 6.0,
                      'lidar.range_noise_std': 0.02})
    cfg['track'].update(outer_width=2.7, outer_center_y=0.35)
    sim = Sim('d', cfg, seed=seed, track_recovery=True)
    r = sim.run()
    assert not r['collision'] and sim.ego.kin.travel > 3.0
    assert not any(row['reason'] == 'front_track_lost' for row in sim.log)


def test_pose_filter_steadies_heading_and_passes_stationary_car_with_lidar_noise():
    # 실차 조건(20 cm 가드레일 + 2.7 m 복도, 라이다 거리 잡음 0.02 m): 자이로 + 속도 예측으로 헤딩 잡음을 줄여
    # 정지 차 우회(method a stationary_pass)가 된다. 끄면 헤딩 잡음으로 추월을 중단한다 (2026-11-19)
    def run(pose_filter):
        cfg = load_cfg(**{'track.wall_height': 0.20, 'lidar.range_noise_std': 0.02, 'start.gap0': 2.5})
        cfg['track'].update(outer_width=2.7, outer_center_y=0.35)
        sim = Sim('a', cfg, W=1.0, experiment_safety=True, stationary_pass=True, pose_filter=pose_filter)
        sim.leader.kin.y = -0.2
        sim.place_bodies()

        def stationary():
            sim.leader.perceive(sim, False)
            sim.leader.drive(sim, 0.0, -0.2)
        sim._leader_frame = stationary
        err = []

        def hook(s):
            if s.ego.corridor.valid and s.t > 1.0:
                err.append(abs(s.ego.corridor.heading - s.ego.kin.yaw))
        return sim.run(hook=hook), max(err)
    on, err_on = run(True)
    off, err_off = run(False)
    assert on["success"] and not on["collision"] and err_on < math.radians(4.0)   # 차선 변경 포함 최대 3.4°
    assert not off['success'] and err_off > err_on
