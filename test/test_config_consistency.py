"""설정 파일 일관성: 방법 간 통제변인이 한 곳에만 있고 drive 모드 값과 같은지 확인한다."""
import ast
import os
import unittest

from my_algo.speed_policy import V_MIN

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..')
BOX_HEIGHT = 0.35


def scalar(text):
    text = text.strip()
    if text in ('true', 'false'):
        return text == 'true'
    try:
        return int(text)
    except ValueError:
        pass
    try:
        return float(text)
    except ValueError:
        return text


def load_params(path):
    """이 레포의 ROS 파라미터 YAML(섹션 → ros__parameters → key: scalar)만 읽는 작은 파서."""
    out, section = {}, None
    with open(path, encoding='utf-8') as f:
        for raw in f:
            line = raw.split('#', 1)[0].rstrip()
            if not line.strip():
                continue
            indent = len(line) - len(line.lstrip())
            key, _, value = line.strip().partition(':')
            if indent == 0:
                section = key.strip("'\"")
                out[section] = {}
            elif indent >= 4 and value.strip():
                out[section][key] = scalar(value)
    return out


def launch_dicts(path):
    with open(path, encoding='utf-8') as f:
        tree = ast.parse(f.read())
    found = {}
    for node in ast.walk(tree):
        if isinstance(node, ast.Call) and getattr(node.func, 'id', '') == 'Node':
            kw = {k.arg: k.value for k in node.keywords}
            exe = kw.get('executable')
            if isinstance(exe, ast.Constant) and 'parameters' in kw:
                found[exe.value] = ast.literal_eval(kw['parameters'].elts[0])
    return found


class ConfigConsistencyTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cfg = os.path.join(ROOT, 'config')
        cls.common = load_params(os.path.join(cfg, 'common.yaml'))
        cls.c = cls.common['/**']
        cls.drive = launch_dicts(os.path.join(ROOT, 'launch', 'lidar_drive.launch.py'))
        cls.cfg = cfg

    def test_aeb_thresholds_equal_drive_mode(self):
        d = self.drive['autonomous_drive']
        for key in ('aeb_ttc_threshold', 'aeb_path_margin', 'aeb_bumper_emergency_distance',
                    'aeb_bumper_overlap_tolerance'):
            self.assertEqual(self.c[key], d[key], key)
        # aeb.py 기존 하드코딩 값 (autonomous_drive 파라미터 기본값)
        self.assertEqual(self.c['aeb_comfortable_decel'], 2.8)
        self.assertEqual(self.c['aeb_stopping_margin'], 0.20)

    def test_scan_params_equal_drive_mode(self):
        self.assertEqual(self.common['pointcloud_to_laserscan'],
                         self.drive['pointcloud_to_laserscan_node'])

    def test_planner_params_equal_drive_mode_except_documented(self):
        exp = self.common['exp_steering_node']
        drive = self.drive['lidar_reactive_drive']
        allowed_diff = {'target_wall_distance', 'lidar_to_bumper_dist'}  # 좌측 벽 목표, 앞 끝 기준
        for key, value in drive.items():
            if key in allowed_diff:
                continue
            self.assertEqual(exp.get(key), value, key)

    def test_method_files_only_select_method(self):
        for m in ('B0', 'B1', 'B2', 'P'):
            p = load_params(os.path.join(self.cfg, f'method_{m}.yaml'))
            self.assertEqual(p, {'/**': {'method': m}})

    def test_scenario_files_same_keys(self):
        keys = None
        for s in ('S1', 'S2', 'S3', 'S4'):
            p = load_params(os.path.join(self.cfg, f'scenario_{s}.yaml'))['/**']
            self.assertEqual(p['scenario'], s)
            self.assertEqual(p['trigger_enabled'], s == 'S1')
            keys = keys or set(p)
            self.assertEqual(set(p), keys)

    def test_physical_constraints(self):
        self.assertGreaterEqual(self.c['v_imp'], V_MIN)
        self.assertLess(self.c['band_z_max'], BOX_HEIGHT)
        self.assertTrue(1 <= self.c['known_off'] <= self.c['known_on'] <= self.c['n_frames'])
        self.assertEqual(self.c['min_speed_policy'], 'hold')


if __name__ == '__main__':
    unittest.main()
