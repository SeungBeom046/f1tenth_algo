"""scripts/bag_player.py의 sqlite3 읽기·일정 계산 (ROS 없이)."""
import importlib.util
import os
import sqlite3
import tempfile
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
spec = importlib.util.spec_from_file_location('bag_player', os.path.join(HERE, '..', 'scripts', 'bag_player.py'))
bp = importlib.util.module_from_spec(spec)
spec.loader.exec_module(bp)


def foxy_bag(d):
    """Foxy rosbag2 sqlite3 스키마와 같은 테이블."""
    with sqlite3.connect(os.path.join(d, 'bag_0.db3')) as db:
        db.execute('CREATE TABLE topics(id INTEGER PRIMARY KEY, name TEXT NOT NULL, type TEXT NOT NULL, '
                   'serialization_format TEXT NOT NULL, offered_qos_profiles TEXT NOT NULL)')
        db.execute('CREATE TABLE messages(id INTEGER PRIMARY KEY, topic_id INTEGER NOT NULL, '
                   'timestamp INTEGER NOT NULL, data BLOB NOT NULL)')
        db.execute("INSERT INTO topics VALUES (1, '/livox/lidar', 'sensor_msgs/msg/PointCloud2', 'cdr', '')")
        db.execute("INSERT INTO topics VALUES (2, '/vesc/odom', 'nav_msgs/msg/Odometry', 'cdr', '')")
        db.execute("INSERT INTO topics VALUES (3, '/commands/motor/speed', 'std_msgs/msg/Float64', 'cdr', '')")
        for i, (t, tid) in enumerate([(300, 1), (100, 2), (200, 3), (400, 2)]):
            db.execute('INSERT INTO messages VALUES (?, ?, ?, ?)', (i, tid, t, b'x'))


class BagPlayerTest(unittest.TestCase):
    def test_read_filter_order(self):
        with tempfile.TemporaryDirectory() as d:
            foxy_bag(d)
            self.assertEqual(bp.read_topics(d)['/vesc/odom'], 'nav_msgs/msg/Odometry')
            msgs = bp.iter_messages(d, {'/livox/lidar', '/vesc/odom'})
            self.assertEqual([(t, n) for t, n, _ in msgs],
                             [(100, '/vesc/odom'), (300, '/livox/lidar'), (400, '/vesc/odom')])

    def test_schedule_rate(self):
        self.assertEqual(bp.schedule([1_000_000_000, 2_000_000_000], 0.5), [0.0, 2.0])

    @unittest.skipUnless(importlib.util.find_spec('rosbags'), 'rosbags 없음')
    def test_reads_bag_written_by_rosbags(self):
        from rosbags.rosbag2 import Writer
        from rosbags.typesys import Stores, get_typestore
        ts = get_typestore(Stores.ROS2_FOXY)
        F = ts.types['std_msgs/msg/Float64']
        with tempfile.TemporaryDirectory() as d:
            path = os.path.join(d, 'bag')
            with Writer(path) as w:
                c = w.add_connection('/x', 'std_msgs/msg/Float64', typestore=ts)
                w.write(c, 5, ts.serialize_cdr(F(data=2.5), 'std_msgs/msg/Float64'))
            msgs = bp.iter_messages(path)
            self.assertEqual(msgs[0][:2], (5, '/x'))
            self.assertEqual(ts.deserialize_cdr(msgs[0][2], 'std_msgs/msg/Float64').data, 2.5)


if __name__ == '__main__':
    unittest.main()
