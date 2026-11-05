"""뷰어 재시작 후 화면 밝기·오버레이 도형 수가 처음과 같은지 (오프스크린 렌더, 창 없음).

원인 기록: mj_resetData가 조명 위치·방향(light_xpos/xdir)을 0으로 만들고 시뮬 루프는 mj_kinematics만 불러
재시작 후 조명이 원점·방향 0이 되어 화면 밝기가 약 절반이 됐다 (64.8 → 33.6). 재시작 때 mj_forward로 고침.
"""
import contextlib
import os
import sys
import tempfile
import unittest

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
SIM = os.path.dirname(HERE)
sys.path[:0] = [SIM, os.path.dirname(SIM), HERE]

import mujoco  # noqa: E402

import runner  # noqa: E402
import viewer_ui  # noqa: E402
from test_viewer_determinism import FakeClock, FakeHandle  # noqa: E402


class CaptureUI(viewer_ui.ViewerUI):
    """각 회차 첫 스텝에서 (모델 장면 + 오버레이)를 오프스크린으로 그려 밝기를 잰다. t 0.3 s에 재시작, 4회차에서 창 닫기."""

    def __init__(self, sim, clock, renderer, cam):
        super().__init__(sim, max_points=1500, clock=clock.time, sleep=clock.sleep)
        self.r, self.cam = renderer, cam
        self.first = {}
        self.ngeom = {}
        self.restart_ngeom = []

    def after_step(self, sim):
        out = super().after_step(sim)
        if self.run_index not in self.first:
            self.r.update_scene(sim.d, self.cam)
            self.fill_scene(self.r.scene)
            img = self.r.render()
            self.first[self.run_index] = (float(img.mean()), img)
            self.ngeom[self.run_index] = self.handle.user_scn.ngeom
            self.light = (sim.d.light_xpos.copy(), sim.d.light_xdir.copy())
        if sim.t >= 0.3 - 1e-9 and out is not False:
            if self.run_index < 3:
                self.on_key(viewer_ui.KEY_HOME)
            else:
                self.handle.closed = True
        return out


class ViewerRenderTest(unittest.TestCase):
    def test_brightness_and_overlay_count_same_after_restarts(self):
        with tempfile.TemporaryDirectory() as d, open(os.devnull, 'w') as null, contextlib.redirect_stdout(null):
            args = runner.parse(['--method', 'P', '--scenario', 'S2', '--seed', '0', '--out-dir', os.path.join(d, 'v')])
            sim = runner.Sim(args)
            mujoco.mj_forward(sim.m, sim.d)          # launch_passive가 창을 띄울 때 하는 것과 같다
            r = mujoco.Renderer(sim.m, 240, 320)
            cam = mujoco.MjvCamera()
            cam.lookat[:], cam.distance, cam.elevation, cam.azimuth = [3.0, 0.0, 0.0], 8.0, -45.0, 180.0
            ui = CaptureUI(sim, FakeClock(), r, cam)
            h = FakeHandle(sim.m)
            ui.attach(h)
            done = runner.viewer_session(args, sim, ui, echo=lambda m: None)
            r.close()
        self.assertEqual(len(done), 4)                      # 처음 + 재시작 3회
        b0, img0 = ui.first[0]
        for i in (1, 2, 3):
            bi, imgi = ui.first[i]
            self.assertAlmostEqual(bi, b0, delta=0.5, msg=f'재시작 {i} 밝기 {bi:.2f} vs 처음 {b0:.2f}')
            self.assertTrue(np.array_equal(imgi, img0), f'재시작 {i} 첫 프레임 픽셀')
            self.assertEqual(ui.ngeom[i], ui.ngeom[0], f'재시작 {i} 오버레이 도형 수 (누적되면 늘어남)')
        self.assertTrue(np.any(ui.light[1] != 0), '조명 방향이 0 (mj_resetData 후 미갱신)')


if __name__ == '__main__':
    unittest.main()
