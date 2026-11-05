"""뷰어 모드(오버레이·배속·일시정지·프레임 진행·토글)와 배치 모드가 같은 시드에서 비트 단위로 같은지.

창 없이 돈다: 가짜 Handle(user_scn = 실제 MjvScene, lock/sync/set_texts는 기록만)과 가짜 시계로
ViewerUI.after_step 경로 전체(그리기 포함)를 실행한다. 진짜 창에서만 다른 것은 화면 출력뿐이다.
"""
import contextlib
import os
import sys
import tempfile
import unittest

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
SIM = os.path.dirname(HERE)
sys.path[:0] = [SIM, os.path.dirname(SIM)]

import runner  # noqa: E402
import viewer_ui  # noqa: E402


class FakeHandle:
    def __init__(self, m):
        import mujoco
        self.user_scn = mujoco.MjvScene(m, maxgeom=100000)
        self.syncs = 0
        self.texts = None
        self.max_ngeom = 0

    def lock(self):
        return contextlib.nullcontext()

    def sync(self, state_only=False):
        self.syncs += 1
        self.max_ngeom = max(self.max_ngeom, self.user_scn.ngeom)

    def is_running(self):
        return True

    def set_texts(self, texts):
        self.texts = texts

    def clear_texts(self):
        self.texts = None


class FakeClock:
    def __init__(self):
        self.now = 0.0
        self.slept = 0.0

    def time(self):
        return self.now

    def sleep(self, dt):
        self.now += dt
        self.slept += dt


class ScriptedUI(viewer_ui.ViewerUI):
    """시뮬 시각에 맞춰 키를 누른다. 일시정지 중에는 가짜 sleep이 프레임 진행·재개 키를 누른다."""

    def __init__(self, sim, clock, script, pause_script):
        super().__init__(sim, max_points=1500, clock=clock.time, sleep=self._sleep)
        self.fake = clock
        self.script = sorted(script)
        self.pause_script = list(pause_script)
        self.pause_sleeps = 0

    def _sleep(self, dt):
        self.fake.sleep(dt)
        if self.paused and self.step_until is None:
            self.pause_sleeps += 1
            if self.pause_sleeps % 3 == 0 and self.pause_script:
                self.on_key(self.pause_script.pop(0))

    def after_step(self, sim):
        while self.script and sim.t + 1e-9 >= self.script[0][0]:
            self.on_key(self.script.pop(0)[1])
        return super().after_step(sim)


def run(viewer, out_dir):
    args = runner.parse(['--method', 'P', '--scenario', 'S1', '--d-e', '1.0', '--seed', '3', '--out-dir', out_dir])
    sim = runner.Sim(args)
    if not viewer:
        log, frames = sim.run()
        return sim, log, frames, None
    clock = FakeClock()
    K = viewer_ui
    script = [(0.5, K.KEY_9), (0.8, K.KEY_6), (1.0, K.KEY_7), (1.2, K.KEY_8), (1.4, K.KEY_END),
              (1.6, K.KEY_6), (1.7, K.KEY_9), (2.0, K.KEY_ENTER), (2.9, K.KEY_ENTER), (3.05, K.KEY_7)]
    pause_script = [K.KEY_PERIOD, K.KEY_PERIOD, K.KEY_8, K.KEY_PERIOD, K.KEY_ENTER, K.KEY_ENTER, K.KEY_ENTER]
    ui = ScriptedUI(sim, clock, script, pause_script)
    h = FakeHandle(sim.m)
    ui.attach(h)
    log, frames = sim.run(hook=ui)
    return sim, log, frames, (ui, h, clock)


class ViewerDeterminismTest(unittest.TestCase):
    def test_viewer_equals_batch_bitwise(self):
        with tempfile.TemporaryDirectory() as d:
            with open(os.devnull, 'w') as null, contextlib.redirect_stdout(null):
                sa, la, fa, _ = run(False, os.path.join(d, 'a'))
                sb, lb, fb, (ui, h, clock) = run(True, os.path.join(d, 'b'))
        # 뷰어 경로가 실제로 돌았는지: 그리기, 배속 대기, 일시정지 중 프레임 진행
        self.assertGreater(h.syncs, len(la['t']))
        self.assertGreater(h.max_ngeom, 100)
        self.assertGreater(ui.pause_sleeps, 3)
        self.assertGreater(clock.slept, 4.0)
        self.assertLessEqual(ui.drawn_points, 1500)
        # 궤적·명령·로그 전부 비트 단위 동일
        self.assertEqual(sorted(la), sorted(lb))
        for k in la:
            a, b = np.asarray(la[k]), np.asarray(lb[k])
            self.assertEqual(a.shape, b.shape, k)
            self.assertTrue(np.array_equal(a, b, equal_nan=a.dtype.kind == 'f'), k)
        self.assertEqual(fa['cloud_digest'], fb['cloud_digest'])     # 라이다 광선 패턴·거리까지 같음
        for k in ('t', 'phantoms', 'boundary', 'grid', 'grid_origin'):
            self.assertEqual(len(fa[k]), len(fb[k]), k)
            for x, y in zip(fa[k], fb[k]):
                self.assertTrue(np.array_equal(np.asarray(x), np.asarray(y)), k)
        self.assertEqual([(e['t'], e['topic']) for e in sa.events], [(e['t'], e['topic']) for e in sb.events])


if __name__ == '__main__':
    unittest.main()
