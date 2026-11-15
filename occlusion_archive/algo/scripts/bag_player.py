#!/usr/bin/env python3
"""rosbag2(sqlite3) 재생 + bag 시간으로 /clock 발행. Foxy `ros2 bag play`에 --clock이 없어서 만든 대체품.

    python3 bag_player.py <bag 폴더> [--rate 0.5] [--topics /livox/lidar /vesc/odom] [--delay 2]

- 메시지는 기록된 수신 시각 순서로, (기록 간격 / rate) 간격에 맞춰 발행한다.
- 매 메시지 직전과 그 사이 clock_hz 주기로 /clock(rosgraph_msgs/Clock) = 그 시점의 bag 시각을 발행한다.
  use_sim_time:=true인 노드는 bag 시각으로 동작한다 (replay_drive.sh, offline_replay.sh).
- /tf_static은 transient_local로 발행한다 (늦게 붙은 노드도 받게).
- my_algo 패키지에 의존하지 않는다 (리팩터링 전 기준선 태그를 빌드한 상태에서도 그대로 쓰려고).
"""

import argparse
import glob
import os
import sqlite3
import sys
import time


def bag_files(bag_dir):
    files = sorted(glob.glob(os.path.join(bag_dir, '*.db3')))
    if not files:
        raise FileNotFoundError(f'{bag_dir}에 .db3 파일이 없다 (Foxy 기본 저장소 sqlite3)')
    return files


def read_topics(bag_dir):
    """{토픽: 타입}"""
    out = {}
    for f in bag_files(bag_dir):
        with sqlite3.connect(f) as db:
            for name, typ in db.execute('SELECT name, type FROM topics'):
                out[name] = typ
    return out


def iter_messages(bag_dir, topics=None):
    """(timestamp_ns, 토픽, 직렬화 데이터)를 시간순으로. topics가 있으면 그 토픽만."""
    rows = []
    for f in bag_files(bag_dir):
        with sqlite3.connect(f) as db:
            names = dict(db.execute('SELECT id, name FROM topics'))
            ids = [i for i, n in names.items() if topics is None or n in topics]
            if not ids:
                continue
            q = ('SELECT timestamp, topic_id, data FROM messages WHERE topic_id IN (%s)'
                 % ','.join('?' * len(ids)))
            rows.extend((t, names[tid], bytes(d)) for t, tid, d in db.execute(q, ids))
    rows.sort(key=lambda r: r[0])
    return rows


def schedule(stamps_ns, rate):
    """첫 메시지 기준 wall 시각 오프셋 [s] 목록. rate 0.5 = 절반 속도."""
    if not stamps_ns:
        return []
    t0 = stamps_ns[0]
    return [(t - t0) * 1e-9 / rate for t in stamps_ns]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument('bag')
    ap.add_argument('--rate', type=float, default=1.0)
    ap.add_argument('--topics', nargs='*', default=None)
    ap.add_argument('--delay', type=float, default=1.0, help='발행 시작 전 대기 (구독 연결) [s]')
    ap.add_argument('--clock-hz', type=float, default=100.0)
    args = ap.parse_args()

    import rclpy
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from rclpy.serialization import deserialize_message
    from rosgraph_msgs.msg import Clock
    from rosidl_runtime_py.utilities import get_message

    types = read_topics(args.bag)
    wanted = set(args.topics) if args.topics else set(types)
    missing = wanted - set(types)
    if missing:
        print(f'경고: bag에 없는 토픽 {sorted(missing)}', file=sys.stderr)
    msgs = iter_messages(args.bag, wanted)
    if not msgs:
        sys.exit('재생할 메시지가 없다')

    rclpy.init()
    node = rclpy.create_node('bag_player')
    clock_pub = node.create_publisher(Clock, '/clock', 10)
    pubs, classes = {}, {}
    for topic in sorted(wanted & set(types)):
        classes[topic] = get_message(types[topic])
        qos = (QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                          reliability=ReliabilityPolicy.RELIABLE)
               if topic == '/tf_static' else QoSProfile(depth=100))
        pubs[topic] = node.create_publisher(classes[topic], topic, qos)

    def publish_clock(t_ns):
        c = Clock()
        c.clock.sec, c.clock.nanosec = divmod(int(t_ns), 1_000_000_000)
        clock_pub.publish(c)

    publish_clock(msgs[0][0])
    time.sleep(args.delay)
    offsets = schedule([m[0] for m in msgs], args.rate)
    start = time.monotonic()
    clock_period = 1.0 / args.clock_hz
    for (t_ns, topic, data), off in zip(msgs, offsets):
        # 다음 메시지까지 /clock을 일정 주기로 진행시킨다 (타이머 기반 노드가 멈추지 않게)
        while True:
            now = time.monotonic() - start
            if now >= off:
                break
            frac_ns = msgs[0][0] + int(now * args.rate * 1e9)
            publish_clock(min(frac_ns, t_ns))
            time.sleep(min(clock_period, off - now))
        publish_clock(t_ns)
        pubs[topic].publish(deserialize_message(data, classes[topic]))
    # 마지막 메시지 뒤 1초(bag 시간)만큼 clock을 더 진행해 남은 타이머 출력을 받는다
    t_last = msgs[-1][0]
    for k in range(int(args.clock_hz)):
        publish_clock(t_last + int(k / args.clock_hz * 1e9))
        time.sleep(clock_period / args.rate)
    print(f'재생 완료: {len(msgs)}개 메시지, bag 시간 {(t_last - msgs[0][0]) * 1e-9:.1f} s')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
