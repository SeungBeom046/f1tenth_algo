"""시뮬 설정 로더. 실차와 같은 config 파일을 같은 우선순위로 읽어 노드별 파라미터를 만든다.

우선순위 (bringup.launch.py와 같음): common.yaml '/**' < common.yaml 노드 섹션 < method_*.yaml < scenario_*.yaml
< 회차 값(d_e, run_id) < [EXP-TUNE] 인자 < 시뮬 전용(sim.yaml mux_overrides, 지오메트리 파일 경로).
"""
import os
import sys
import types

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
CONFIG = os.path.join(ROOT, 'config')
TUNE_KEYS = ('v_imp', 'w_min', 'n_frames', 'known_on', 'known_off', 'b1_k', 'b1_roi_length',
             'b1_roi_half_width', 'row_face_tol')


def _load(path):
    with open(path, encoding='utf-8') as f:
        return yaml.safe_load(f)


def load_sim(path=None):
    return _load(path or os.path.join(HERE, 'config', 'sim.yaml'))


def mount():
    """launch/tf_static.launch.py를 가짜 launch 모듈로 실행해 장착값을 읽는다 (값 하드코딩 없음)."""
    launch = types.ModuleType('launch')
    launch.LaunchDescription = lambda xs: xs
    lr = types.ModuleType('launch_ros')
    lra = types.ModuleType('launch_ros.actions')
    lra.Node = lambda **kw: kw
    lr.actions = lra
    saved = {k: sys.modules.get(k) for k in ('launch', 'launch_ros', 'launch_ros.actions')}
    sys.modules.update({'launch': launch, 'launch_ros': lr, 'launch_ros.actions': lra})
    try:
        ns = {}
        with open(os.path.join(ROOT, 'launch', 'tf_static.launch.py'), encoding='utf-8') as f:
            exec(compile(f.read(), 'tf_static.launch.py', 'exec'), ns)
    finally:
        for k, v in saved.items():
            if v is None:
                sys.modules.pop(k, None)
            else:
                sys.modules[k] = v
    return {'lidar_xyz': (ns['LIDAR_X'], ns['LIDAR_Y'], ns['LIDAR_Z']),
            'lidar_rpy': (ns['LIDAR_ROLL'], ns['LIDAR_PITCH'], ns['LIDAR_YAW'])}


class RunConfig:
    def __init__(self, method, scenario, d_e=None, run_id=0, tune=None, sim_path=None):
        self.method, self.scenario, self.d_e, self.run_id = method, scenario, d_e, int(run_id)
        self.tune = {k: v for k, v in (tune or {}).items() if v is not None}
        bad = set(self.tune) - set(TUNE_KEYS)
        if bad:
            raise ValueError(f'[EXP-TUNE]가 아닌 값은 바꿀 수 없다: {sorted(bad)}')
        self.common_doc = _load(os.path.join(CONFIG, 'common.yaml'))
        self.common = self.common_doc['/**']['ros__parameters']
        self.method_p = _load(os.path.join(CONFIG, f'method_{method}.yaml'))['/**']['ros__parameters']
        self.scenario_p = _load(os.path.join(CONFIG, f'scenario_{scenario}.yaml'))['/**']['ros__parameters']
        self.popout = _load(os.path.join(CONFIG, 'popout.yaml'))['/**']['ros__parameters']
        self.geometry_path = os.path.join(CONFIG, 'vehicle_geometry.yaml')
        self.sim = load_sim(sim_path)
        self.mount = mount()
        if scenario == 'S1' and d_e is None:
            raise ValueError('S1은 d_e가 필요하다')

    def section(self, name):
        doc = self.common_doc.get(name)
        return dict(doc['ros__parameters']) if doc else {}

    def node_params(self, node_name):
        p = dict(self.common)
        p.update(self.section(node_name))
        p.update(self.method_p)
        p.update(self.scenario_p)
        p.update({'run_id': str(self.run_id), 'run_dir': '', 'vehicle_geometry_file': self.geometry_path})
        if self.d_e is not None:
            p['d_e'] = float(self.d_e)
        p.update(self.tune)
        if node_name == 'exp_drive_mux':
            p.update(self.sim['mux_overrides'])
        return p

    def popout_params(self, log_dir):
        return dict(self.popout, log_dir=log_dir)

    def pointcloud_to_laserscan(self):
        return self.section('pointcloud_to_laserscan')

    def summary(self):
        return {'method': self.method, 'scenario': self.scenario, 'd_e': self.d_e, 'run_id': self.run_id,
                'tune': self.tune, 'common': self.common, 'method_params': self.method_p,
                'scenario_params': self.scenario_p, 'popout': self.popout, 'sim': self.sim,
                'mount': self.mount, 'steering_section': self.section('exp_steering_node'),
                'pointcloud_to_laserscan': self.pointcloud_to_laserscan()}
