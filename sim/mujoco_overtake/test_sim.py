"""시뮬 자체 점검 (mujoco가 있는 환경에서만): python -m pytest sim/mujoco_overtake/test_sim.py -q"""
import pytest

mujoco = pytest.importorskip('mujoco')

from sim import Sim, load_cfg       # noqa: E402


def run(method, behavior, **over):
    return Sim(method, load_cfg(**dict({'leader.behavior': behavior, 'leader.d_y': 1.0}, **over))).run()


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
