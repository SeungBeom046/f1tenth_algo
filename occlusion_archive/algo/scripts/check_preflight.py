#!/usr/bin/env python3
"""/exp/preflight (transient_local)를 읽어 발행자 점검 통과 여부를 종료 코드로 알린다. record_run.sh가 호출.

    python3 check_preflight.py [--timeout 5]   # 0 = 통과, 1 = 위반 또는 결과 없음
"""
import argparse
import json
import sys
import time

import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--timeout', type=float, default=5.0)
    args = ap.parse_args()
    rclpy.init()
    node = rclpy.create_node('check_preflight')
    got = []
    node.create_subscription(String, '/exp/preflight', got.append,
                             QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                        reliability=ReliabilityPolicy.RELIABLE))
    end = time.time() + args.timeout
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
        if got and json.loads(got[-1].data).get('problems') != ['그래프 탐색 대기 중']:
            break
    node.destroy_node()
    rclpy.shutdown()
    if not got:
        print('발행자 점검 결과 없음: bringup(mode:=exp)이 떠 있는지 확인', file=sys.stderr)
        return 1
    info = json.loads(got[-1].data)
    for p in info.get('problems', []):
        print(f'  위반: {p}', file=sys.stderr)
    if not info.get('ok'):
        print('회차 시작 거부: 발행자 점검 실패 (docs/RUNBOOK.md 문제 해결)', file=sys.stderr)
        return 1
    if info.get('problems'):
        print('경고: mode=warn이라 위반이 있어도 통과시킴', file=sys.stderr)
    return 0


if __name__ == '__main__':
    sys.exit(main())
