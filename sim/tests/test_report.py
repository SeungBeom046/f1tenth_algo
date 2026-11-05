"""회차 판단 리포트: 배치 회차가 끝나면 report.html이 생기고, 판정 항목이 기준대로 나오는지."""
import contextlib
import json
import os
import sys
import tempfile
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
SIM = os.path.dirname(HERE)
sys.path[:0] = [SIM, os.path.dirname(SIM)]

import report  # noqa: E402
import runner  # noqa: E402


class ReportTest(unittest.TestCase):
    def test_batch_run_writes_report_and_index(self):
        with tempfile.TemporaryDirectory() as d, open(os.devnull, 'w') as null, contextlib.redirect_stdout(null):
            for method in ('B0', 'P'):
                runner.main(['--method', method, '--scenario', 'S3', '--seed', '0', '--out-dir', os.path.join(d, method)])
            with open(os.path.join(d, 'B0', 'analysis', 'report_summary.json'), encoding='utf-8') as f:
                b0 = json.load(f)
            html = open(os.path.join(d, 'P', 'report.html'), encoding='utf-8').read()
            idx = report.index(d)
            self.assertTrue(os.path.exists(idx))
        self.assertEqual(set(b0['checks']), {'collision', 'cmd_le_vcap', 'tracking', 'discard', 'aeb_malfunction',
                                             'vref', 'min_speed_flag', 'phantom_pos'})
        # B0, S3: 충돌 없음, 폐기 아님, 오작동 없음, v_ref 도달, phantom 없음 (기대대로)
        for k in ('collision', 'discard', 'aeb_malfunction', 'vref', 'phantom_pos', 'cmd_le_vcap'):
            self.assertEqual(b0['checks'][k], 'pass', k)
        self.assertEqual(b0['checks']['min_speed_flag'], 'info')
        self.assertIn('같은 조건 비교', html)              # P 리포트에 B0 회차가 겹쳐짐
        self.assertNotIn('cdn', html.lower())              # 외부 라이브러리 없이 열림


if __name__ == '__main__':
    unittest.main()
