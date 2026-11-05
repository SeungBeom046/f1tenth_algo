"""가짜 rclpy·메시지 모듈 (시뮬·리팩터 비교용). ROS 없이 my_algo 노드 클래스를 그대로 실행한다.

test/_rosstub.py(단위 테스트용)를 복사해 넓힌 공용판이다. test 쪽은 바꾸지 않는다.
- 시계: 벽시계를 쓰지 않는다. set_time(t)로 시뮬 시간을 넣으면 노드의 get_clock().now()가 그 값을 돌려준다
  (모드 전환 0.45 s, 명령 만료 1 s 같은 노드 내부 시간이 시뮬 스텝에 묶인다 → 배치·실시간 결과가 같다).
- 타이머: 등록만 한다. 호출은 run_timers(node)로 시뮬이 직접 한다.
- 발행: BUS 목록에 (토픽, 메시지)로 쌓인다. take(topic)으로 꺼낸다.
- install()은 실제 rclpy가 있어도 가짜를 강제로 쓴다 (force=True 기본). 프로세스 단위로만 쓸 것.
"""
import importlib
import os
import sys
import tempfile

STUBS = {
    'rclpy/__init__.py': '''
from . import node, qos, parameter, time, duration
def init(args=None): pass
def shutdown(): pass
def ok(): return True
def create_node(name, **kw): return node.Node(name, **kw)
''',
    'rclpy/node.py': '''
CLOCK = {'ns': 0}
OVERRIDES = {}
BUS = []
from rclpy.time import Time
from rclpy.duration import Duration
class _Clock:
    def now(self): return Time(nanoseconds=CLOCK['ns'])
class _Param:
    def __init__(self, v): self.value = v
class _Logger:
    def __init__(self): self.lines = []
    def info(self, msg, **k): self.lines.append(('info', msg))
    def warn(self, msg, **k): self.lines.append(('warn', msg))
    warning = warn
    def error(self, msg, **k): self.lines.append(('error', msg))
    def debug(self, msg, **k): pass
class _Pub:
    def __init__(self, topic): self.topic = topic; self.topic_name = topic
    def publish(self, msg): BUS.append((self.topic, msg))
class Node:
    def __init__(self, name, **kw):
        self._name = name
        self._params, self.subs, self.timers = {}, {}, []
        self._logger = _Logger()
    def get_name(self): return self._name
    def declare_parameter(self, name, value=None, *a, **k):
        self._params[name] = OVERRIDES.get(name, value)
        return _Param(self._params[name])
    def get_parameter(self, name): return _Param(self._params[name])
    def set_parameters(self, ps): pass
    def create_subscription(self, t, topic, cb, qos): self.subs[topic] = cb
    def create_publisher(self, t, topic, qos): return _Pub(topic)
    def create_timer(self, period, cb):
        self.timers.append([period, cb, None]); return None
    def get_clock(self): return _Clock()
    def get_logger(self): return self._logger
    def get_publishers_info_by_topic(self, topic): return []
    def get_topic_names_and_types(self): return []
    def destroy_node(self): pass
''',
    'rclpy/time.py': '''
class Time:
    def __init__(self, seconds=0, nanoseconds=0):
        self.nanoseconds = int(seconds * 1e9) + int(nanoseconds)
    def __sub__(self, other):
        from rclpy.duration import Duration
        return Duration(nanoseconds=self.nanoseconds - other.nanoseconds)
    def to_msg(self):
        from builtin_interfaces.msg import Time as T
        t = T(); t.sec, t.nanosec = divmod(self.nanoseconds, 1_000_000_000); return t
    @classmethod
    def from_msg(cls, msg): return cls(nanoseconds=msg.sec * 1_000_000_000 + msg.nanosec)
''',
    'rclpy/duration.py': '''
class Duration:
    def __init__(self, seconds=0, nanoseconds=0):
        self.nanoseconds = int(seconds * 1e9) + int(nanoseconds)
''',
    'rclpy/qos.py': '''
class QoSProfile:
    def __init__(self, **kw): pass
class QoSHistoryPolicy: KEEP_LAST = 1
class QoSReliabilityPolicy: BEST_EFFORT = 2; RELIABLE = 1
ReliabilityPolicy = QoSReliabilityPolicy
class DurabilityPolicy: TRANSIENT_LOCAL = 1; VOLATILE = 2
QoSDurabilityPolicy = DurabilityPolicy
qos_profile_sensor_data = None
''',
    'rclpy/parameter.py': '''
class Parameter:
    class Type: BOOL = 1
    def __init__(self, *a): pass
''',
    'builtin_interfaces/__init__.py': '',
    'builtin_interfaces/msg.py': '''
class Time:
    def __init__(self, sec=0, nanosec=0): self.sec = sec; self.nanosec = nanosec
''',
    'std_msgs/__init__.py': '',
    'std_msgs/msg.py': '''
from builtin_interfaces.msg import Time as _T
class _D:
    def __init__(self, data=None): self.data = data
class Bool(_D): pass
class Float64(_D): pass
class String(_D): pass
class Header:
    def __init__(self): self.stamp = _T(); self.frame_id = ''
''',
    'geometry_msgs/__init__.py': '',
    'geometry_msgs/msg.py': '''
from std_msgs.msg import Header
class Vector3:
    def __init__(self, x=0.0, y=0.0, z=0.0): self.x = x; self.y = y; self.z = z
class Point(Vector3): pass
class Quaternion:
    def __init__(self, x=0.0, y=0.0, z=0.0, w=1.0): self.x = x; self.y = y; self.z = z; self.w = w
class Pose:
    def __init__(self): self.position = Point(); self.orientation = Quaternion()
class Vector3Stamped:
    def __init__(self): self.header = Header(); self.vector = Vector3()
''',
    'sensor_msgs/__init__.py': '',
    'sensor_msgs/msg.py': '''
from std_msgs.msg import Header
class LaserScan:
    def __init__(self):
        self.header = Header()
        self.angle_min = 0.0; self.angle_max = 0.0; self.angle_increment = 0.0
        self.time_increment = 0.0; self.scan_time = 0.0
        self.range_min = 0.0; self.range_max = 0.0; self.ranges = []; self.intensities = []
class Joy:
    def __init__(self): self.header = Header(); self.axes = []; self.buttons = []
class PointField:
    INT8, UINT8, INT16, UINT16, INT32, UINT32, FLOAT32, FLOAT64 = 1, 2, 3, 4, 5, 6, 7, 8
    def __init__(self, name='', offset=0, datatype=7, count=1):
        self.name = name; self.offset = offset; self.datatype = datatype; self.count = count
class PointCloud2:
    def __init__(self):
        self.header = Header(); self.height = 1; self.width = 0; self.fields = []
        self.is_bigendian = False; self.point_step = 0; self.row_step = 0; self.data = b''; self.is_dense = True
class Imu:
    def __init__(self): self.header = Header()
''',
    'nav_msgs/__init__.py': '',
    'nav_msgs/msg.py': '''
from std_msgs.msg import Header
from geometry_msgs.msg import Vector3, Point, Pose
class _T:
    def __init__(self): self.linear = Vector3(); self.angular = Vector3()
class _TC:
    def __init__(self): self.twist = _T()
class _PC:
    def __init__(self): self.pose = Pose()
class Odometry:
    def __init__(self): self.header = Header(); self.twist = _TC(); self.pose = _PC()
class MapMetaData:
    def __init__(self): self.resolution = 0.0; self.width = 0; self.height = 0; self.origin = Pose()
class OccupancyGrid:
    def __init__(self): self.header = Header(); self.info = MapMetaData(); self.data = []
class GridCells:
    def __init__(self): self.header = Header(); self.cell_width = 0.0; self.cell_height = 0.0; self.cells = []
''',
    'visualization_msgs/__init__.py': '',
    'visualization_msgs/msg.py': '''
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, Vector3
class _C:
    def __init__(self): self.r = self.g = self.b = self.a = 0.0
class Marker:
    ADD, DELETE, DELETEALL = 0, 2, 3
    CYLINDER = 3
    def __init__(self):
        self.header = Header(); self.ns = ''; self.id = 0; self.type = 0; self.action = 0
        self.pose = Pose(); self.scale = Vector3(); self.color = _C()
class MarkerArray:
    def __init__(self): self.markers = []
''',
    'tf2_ros/__init__.py': '''
class TransformException(Exception): pass
class Buffer:
    def __init__(self, *a, **k): pass
    def lookup_transform(self, *a, **k): raise TransformException('stub: TF 없음 (시뮬은 조회 함수를 바꿔 끼운다)')
class TransformListener:
    def __init__(self, *a, **k): pass
''',
    'ament_index_python/__init__.py': '',
    'ament_index_python/packages.py': '''
import os
def get_package_share_directory(name):
    # 시뮬·비교에서는 레포 루트를 share처럼 쓴다 (config/ 경로가 같다)
    return os.environ['MY_ALGO_REPO_ROOT']
''',
}

_root = None


def install(force=True):
    """가짜 모듈을 임시 폴더에 쓰고 sys.path 맨 앞에 넣는다. 반환: rclpy.node 모듈 (CLOCK, BUS, OVERRIDES)."""
    global _root
    if _root is None:
        _root = tempfile.mkdtemp(prefix='sim_rosstub_')
        for rel, content in STUBS.items():
            path = os.path.join(_root, rel)
            os.makedirs(os.path.dirname(path), exist_ok=True)
            with open(path, 'w', encoding='utf-8') as f:
                f.write(content)
    if force:
        for name in list(sys.modules):
            if name.split('.')[0] in ('rclpy', 'std_msgs', 'geometry_msgs', 'sensor_msgs', 'nav_msgs',
                                      'visualization_msgs', 'tf2_ros', 'builtin_interfaces',
                                      'ament_index_python'):
                del sys.modules[name]
    if _root not in sys.path:
        sys.path.insert(0, _root)
    repo = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    os.environ.setdefault('MY_ALGO_REPO_ROOT', repo)
    if repo not in sys.path:
        sys.path.insert(1, repo)
    return importlib.import_module('rclpy.node')


def set_time(rn, t_sec):
    rn.CLOCK['ns'] = int(round(t_sec * 1e9))


def make_node(rn, cls, overrides=None):
    rn.OVERRIDES = dict(overrides or {})
    return cls()


def take(rn, topic=None):
    """BUS에서 꺼낸다 (topic 지정 시 그 토픽만, 나머지는 남김)."""
    if topic is None:
        out = list(rn.BUS)
        rn.BUS.clear()
        return out
    out = [m for t, m in rn.BUS if t == topic]
    rn.BUS[:] = [(t, m) for t, m in rn.BUS if t != topic]
    return out


def run_timers(rn, node, t_sec):
    """시뮬 시각 t_sec까지 밀린 타이머를 주기대로 호출한다 (시뮬 시간 기준)."""
    for timer in node.timers:
        period, cb, last = timer
        if last is None:
            timer[2] = t_sec
            continue
        while t_sec - timer[2] >= period - 1e-9:
            timer[2] += period
            cb()
