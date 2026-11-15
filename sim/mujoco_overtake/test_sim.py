"""시뮬 자체 점검 (mujoco가 있는 환경에서만): python -m pytest sim/mujoco_overtake/test_sim.py -q"""
import pytest

mujoco = pytest.importorskip('mujoco')

from sim import Sim, load_cfg       # noqa: E402


def run(method, behavior, **over):
    """판단 로직 점검용: AEB 끔 (AEB는 아래 별도 테스트)."""
    base = {'leader.behavior': behavior, 'leader.d_y': 1.0, 'aeb.enabled': False}
    return Sim(method, load_cfg(**dict(base, **over))).run()


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


@pytest.mark.xfail(strict=True, reason='열린 문제 (docs/PROGRESS.md): MERGE가 m = 0.15 m 간격으로 끼어들고 v_B_nominal로 가서 '
                   '선행차 AEB(0.9 × 0.35 = 0.31 m)가 0.27 m에서 걸린다. 고치면 이 표시를 지운다')
def test_merge_does_not_trigger_leader_aeb():
    r = Sim('d', load_cfg(**{'leader.behavior': 'Y', 'leader.d_y': 1.0})).run()
    assert r['success'] and r['N_AEB_leader'] == 0


def test_proposed_passes_yielding_leader():
    r = run('d', 'Y')
    assert r['success'] and r['decided'] == 'Y' and not r['collision'] and r['track_lost'] == 0


def test_proposed_follows_and_survives_brake():
    r = run('d', 'NY')
    assert r['final_state'] == 'FOLLOW' and not r['pass_entered'] and not r['collision']
    assert r['gap_min'] > 0.15


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
