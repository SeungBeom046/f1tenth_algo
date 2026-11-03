"""테스트용 가짜 rclpy/메시지 모듈. ROS가 없는 PC에서 노드 로직을 단위 테스트한다.

scripts/verify_refactor_offline.py의 STUBS를 바탕으로 exp 노드에 필요한 메시지를 더한다.
import하면 임시 폴더에 가짜 모듈을 만들고 sys.path 맨 앞에 넣는다. 실제 ROS가 있으면 아무것도 하지 않는다.
"""
import importlib.util
import os
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
ACTIVE = False

EXTRA = {
    'rclpy/node.py_extra': '''
_Pub.topic_name = property(lambda self: self.topic)
''',
    'std_msgs/msg.py': '''
class _D:
    def __init__(self, data=None): self.data = data
class Bool(_D): pass
class Float64(_D): pass
class String(_D): pass
class _Stamp:
    def __init__(self, sec=0, nanosec=0): self.sec = sec; self.nanosec = nanosec
class Header:
    def __init__(self): self.stamp = _Stamp(); self.frame_id = ''
''',
    'sensor_msgs/msg.py': '''
class LaserScan:
    def __init__(self):
        self.angle_min = 0.0; self.angle_max = 0.0; self.angle_increment = 0.0
        self.range_min = 0.0; self.range_max = 0.0; self.ranges = []
class Joy:
    def __init__(self): self.axes = []; self.buttons = []
''',
    'nav_msgs/msg.py': '''
class _V:
    def __init__(self): self.x = 0.0; self.y = 0.0; self.z = 0.0
class _T:
    def __init__(self): self.linear = _V(); self.angular = _V()
class _TC:
    def __init__(self): self.twist = _T()
class _P:
    def __init__(self): self.position = _V()
class _PC:
    def __init__(self): self.pose = _P()
class Odometry:
    def __init__(self): self.twist = _TC(); self.pose = _PC()
''',
}


def _install():
    global ACTIVE
    if importlib.util.find_spec('rclpy') is not None:
        return
    spec = importlib.util.spec_from_file_location(
        'verify_refactor_offline', os.path.join(HERE, '..', 'scripts', 'verify_refactor_offline.py'))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    files = dict(mod.STUBS)
    files['rclpy/node.py'] = files['rclpy/node.py'] + EXTRA['rclpy/node.py_extra']
    for k, v in EXTRA.items():
        if not k.endswith('_extra'):
            files[k] = v
    root = tempfile.mkdtemp(prefix='rosstub_')
    for rel, content in files.items():
        path = os.path.join(root, rel)
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, 'w', encoding='utf-8') as f:
            f.write(content)
    sys.path.insert(0, root)
    ACTIVE = True


_install()
