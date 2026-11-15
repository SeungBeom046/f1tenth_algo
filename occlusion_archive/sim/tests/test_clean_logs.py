"""sim/clean_logs.py: 미리보기는 아무것도 안 지움, 조건 조합, KEEP 보호, 휴지통 이동, sweep 통째, mp4만."""
import contextlib
import io
import json
import os
import sys
import tempfile
import unittest

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path[:0] = [os.path.dirname(HERE)]

import clean_logs as C  # noqa: E402


def make_run(path, complete=True, video=False):
    os.makedirs(path)
    meta = {'config': {'sim': {'sim': {'max_time': 15.0, 'dt': 0.01}}}}
    with open(os.path.join(path, 'meta.json'), 'w') as f:
        json.dump(meta, f)
    t = np.arange(1, 501) * 0.01
    np.savez(os.path.join(path, 'log.npz'), t=t, v=np.full(500, 0.0 if complete else 1.5),
             stop_reason=np.array(['stop_line' if complete else ''] * 500))
    if video:
        open(os.path.join(path, 'topdown.mp4'), 'wb').write(b'x' * 1000)


class CleanLogsTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        r = self.root = os.path.join(self.tmp.name, 'sim_logs')
        make_run(f'{r}/20261101/S1_P_de1.0_seed00_dev', video=True)
        make_run(f'{r}/20261101/S1_P_de1.0_seed00_dev_r1', complete=False)
        make_run(f'{r}/20261105/S2_B2_dena_seed00')
        make_run(f'{r}/20261105/S2_B2_dena_seed00_r1')
        make_run(f'{r}/20261105/sweep_a/S3_P_dena_seed00')
        make_run(f'{r}/20261105/sweep_a/S3_P_dena_seed01')
        os.makedirs(f'{r}/20261105/sweep_a/analysis')
        make_run(f'{r}/20261105/sweep_k/S3_P_dena_seed00')
        open(f'{r}/20261105/sweep_k/KEEP', 'w').close()
        self.home = os.environ.get('HOME')
        os.environ['HOME'] = self.tmp.name            # 휴지통 = 임시 폴더/.Trash (실제 휴지통 건드리지 않음)

    def tearDown(self):
        os.environ['HOME'] = self.home
        self.tmp.cleanup()

    def run_c(self, *a):
        out = io.StringIO()
        with contextlib.redirect_stdout(out), contextlib.redirect_stderr(out):
            code = C.main(['--root', self.root, *a])
        return code, out.getvalue()

    def exists(self, rel):
        return os.path.exists(os.path.join(self.root, rel))

    def test_preview_deletes_nothing_and_needs_condition(self):
        code, out = self.run_c('--restarts')
        self.assertIn('미리보기', out)
        self.assertTrue(self.exists('20261105/S2_B2_dena_seed00_r1'))
        self.assertEqual(self.run_c()[0], 1)          # 조건 없으면 거부

    def test_conditions_combine(self):
        args = C.argparse.Namespace
        sel = lambda **kw: [os.path.relpath(p, self.root) for p in C.select(args(root=self.root, **{  # noqa: E731
            'before': None, 'tag': None, 'restarts': False, 'incomplete': False, 'keep_latest': None,
            'videos_only': False, **kw}))[0]]
        self.assertEqual(sel(restarts=True), ['20261101/S1_P_de1.0_seed00_dev_r1', '20261105/S2_B2_dena_seed00_r1'])
        self.assertEqual(sel(restarts=True, before='20261105'), ['20261101/S1_P_de1.0_seed00_dev_r1'])
        self.assertEqual(sel(tag='dev'), ['20261101/S1_P_de1.0_seed00_dev', '20261101/S1_P_de1.0_seed00_dev_r1'])
        self.assertEqual(sel(incomplete=True), ['20261101/S1_P_de1.0_seed00_dev_r1'])
        self.assertEqual(sel(videos_only=True, before='20261105'), ['20261101/S1_P_de1.0_seed00_dev/topdown.mp4'])
        # 날짜 조건으로 sweep 안 회차가 모두 골라지면 sweep 폴더 통째, KEEP sweep은 제외
        self.assertIn('20261105/sweep_a', sel(before='20261106'))
        self.assertNotIn('20261105/sweep_k', sel(before='20261106'))
        self.assertFalse(any('sweep_k' in p for p in sel(before='20261106')))
        # KEEP을 뺀 후보 중 최근 2개(sweep_a의 두 회차)는 남는다
        os.utime(os.path.join(self.root, '20261101/S1_P_de1.0_seed00_dev/meta.json'), (1, 1))
        kept = sel(before='20261106', keep_latest=2)
        self.assertFalse(any('sweep_a' in p for p in kept))
        self.assertEqual(len(kept), 4)

    def test_trash_and_permanent_and_keep(self):
        self.run_c('--restarts', '--yes')
        self.assertFalse(self.exists('20261105/S2_B2_dena_seed00_r1'))
        self.assertTrue(any('S2_B2_dena_seed00_r1' in n for n in os.listdir(os.path.join(self.tmp.name, '.Trash'))))
        self.run_c('--keep', os.path.join(self.root, '20261105/S2_B2_dena_seed00'))
        self.run_c('--before', '20261106', '--yes', '--permanent')
        self.assertTrue(self.exists('20261105/S2_B2_dena_seed00'))     # KEEP
        self.assertTrue(self.exists('20261105/sweep_k'))               # KEEP (상위)
        self.assertFalse(self.exists('20261105/sweep_a'))
        self.assertFalse(self.exists('20261101/S1_P_de1.0_seed00_dev'))

    def test_du(self):
        code, out = self.run_c('--du')
        self.assertIn('sweep_a/', out)
        self.assertIn('[KEEP]', out)


if __name__ == '__main__':
    unittest.main()
