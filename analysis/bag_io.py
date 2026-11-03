"""rosbag2(mcap/sqlite3) 읽기. ROS 환경(rosbag2_py, rclpy)이 있는 PC나 Jetson에서 실행한다."""
import os

import yaml


def storage_id(bag_dir):
    with open(os.path.join(bag_dir, 'metadata.yaml'), encoding='utf-8') as f:
        return yaml.safe_load(f)['rosbag2_bagfile_information']['storage_identifier']


def read_bag(bag_dir, topics=None):
    """{topic: [(t_bag_sec, msg), ...]}. topics=None이면 전부."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag_dir, storage_id=storage_id(bag_dir)),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    want = set(types) if topics is None else set(topics) & set(types)
    if topics is not None:
        reader.set_filter(rosbag2_py.StorageFilter(topics=sorted(want)))
    classes = {t: get_message(types[t]) for t in want}
    out = {t: [] for t in want}
    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        if topic in classes:
            out[topic].append((t_ns * 1e-9, deserialize_message(data, classes[topic])))
    return out


def stamp(msg):
    s = msg.header.stamp
    return s.sec + s.nanosec * 1e-9
