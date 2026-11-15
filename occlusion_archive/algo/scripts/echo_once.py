#!/usr/bin/env python3
"""토픽 메시지 1개를 받아 출력 (Foxy `ros2 topic echo`에는 --once가 없다).

    python3 echo_once.py <토픽> [--timeout 5] [--transient-local] [--field a.b.c]
타입은 그래프에서 자동으로 찾는다. 0 = 수신, 1 = 시간 초과.
"""
import argparse
import sys
import time

import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rosidl_runtime_py import message_to_yaml
from rosidl_runtime_py.utilities import get_message


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('topic')
    ap.add_argument('--timeout', type=float, default=5.0)
    ap.add_argument('--transient-local', action='store_true', help='/tf_static, /exp/preflight 등 latched 토픽')
    ap.add_argument('--field', default='', help='점으로 구분한 필드 경로 (예: twist.twist.linear)')
    args = ap.parse_args()
    rclpy.init()
    node = rclpy.create_node('echo_once')
    end = time.time() + 2.0
    types = {}
    while time.time() < end and args.topic not in types:
        types = dict(node.get_topic_names_and_types())
        time.sleep(0.1)
    if args.topic not in types:
        print(f'{args.topic}: 토픽 없음', file=sys.stderr)
        return 1
    cls = get_message(types[args.topic][0])
    qos = (QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                      reliability=ReliabilityPolicy.RELIABLE)
           if args.transient_local else qos_profile_sensor_data)
    got = []
    node.create_subscription(cls, args.topic, got.append, qos)
    end = time.time() + args.timeout
    while not got and time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()
    if not got:
        print(f'{args.topic}: {args.timeout} s 안에 수신 없음', file=sys.stderr)
        return 1
    value = got[0]
    for part in filter(None, args.field.split('.')):
        value = getattr(value, part)
    print(f'# {args.topic} [{types[args.topic][0]}]')
    print(message_to_yaml(value) if hasattr(value, 'get_fields_and_field_types') else value)
    return 0


if __name__ == '__main__':
    sys.exit(main())
