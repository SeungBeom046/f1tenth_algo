"""rosbag2(sqlite3) 읽기 — rosbags 0.9.23 (순수 파이썬, ROS 설치 불필요).

노트북(분석 PC)과 Jetson(Python 3.8) 모두에서 돈다. 버전 고정 이유: rosbags 0.10부터 Python 3.10 이상 필요.
    pip install -r analysis/requirements.txt
메시지 형식은 ROS2 Foxy 기준(Stores.ROS2_FOXY). vesc_msgs 같은 비표준 타입 토픽은 읽지 않는다
(분석에 필요한 토픽은 모두 표준 타입).
"""
import os

from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore

TYPESTORE = get_typestore(Stores.ROS2_FOXY)


def read_bag(bag_dir, topics=None):
    """{topic: [(t_bag_sec, msg), ...]}. topics=None이면 해석 가능한 표준 타입 토픽 전부."""
    if not os.path.isdir(bag_dir):
        raise FileNotFoundError(bag_dir)
    out = {}
    with Reader(bag_dir) as reader:
        conns = [c for c in reader.connections
                 if (topics is None or c.topic in topics) and c.msgtype in TYPESTORE.types]
        for c in conns:
            out.setdefault(c.topic, [])
        for conn, t_ns, raw in reader.messages(connections=conns):
            out[conn.topic].append((t_ns * 1e-9, TYPESTORE.deserialize_cdr(raw, conn.msgtype)))
    return out


def stamp(msg):
    s = msg.header.stamp
    return s.sec + s.nanosec * 1e-9


def clock_mapper(bag):
    """bag의 /clock으로 기록 시각(wall) → sim 시각 변환 함수. /clock이 없으면 항등 함수.

    Foxy `ros2 bag record`에는 --use-sim-time이 없어 오프라인 재생 결과는 wall 시각으로 찍힌다.
    """
    import numpy as np
    clk = bag.get('/clock', [])
    if not clk:
        return lambda t: t
    tw = np.array([t for t, _ in clk])
    ts = np.array([m.clock.sec + m.clock.nanosec * 1e-9 for _, m in clk])
    return lambda t: np.interp(t, tw, ts)
