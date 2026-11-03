#!/usr/bin/env python3
"""전방 카메라 영상에 AEB 상태를 오프라인으로 입힌다 (온보드에서는 그리지 않는다).

    python3 analysis/aeb_overlay.py <run_dir> [--camera-topic /camera/camera/color/image_raw/compressed]

입력: <run_dir>/bag 의 압축 카메라, /aeb/active, /aeb/status, /exp/final_cmd, /exp/stop_reason
출력: <run_dir>/camera/front_aeb_overlay.mp4 (bag 수신 시각 기준으로 상태를 맞춘다)
"""

import argparse
import bisect
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from bag_io import read_bag  # noqa: E402


def latest(series, t):
    i = bisect.bisect_right([x for x, _ in series], t) - 1
    return series[i][1] if i >= 0 else None


def main():
    import cv2
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument('run_dir')
    ap.add_argument('--camera-topic', default=os.environ.get(
        'CAMERA_TOPIC', '/camera/camera/color/image_raw/compressed'))
    args = ap.parse_args()
    bag = read_bag(os.path.join(args.run_dir, 'bag'),
                   [args.camera_topic, '/aeb/active', '/aeb/status', '/exp/final_cmd', '/exp/stop_reason'])
    frames = bag.get(args.camera_topic, [])
    if not frames:
        sys.exit(f'{args.camera_topic} 없음')
    active = [(t, m.data) for t, m in bag.get('/aeb/active', [])]
    status = [(t, m.data) for t, m in bag.get('/aeb/status', [])]
    speed = [(t, m.data) for t, m in bag.get('/exp/final_cmd', [])]
    reason = [(t, m.data) for t, m in bag.get('/exp/stop_reason', [])]
    fps = 1.0 / float(np.median(np.diff([t for t, _ in frames])))
    first = cv2.imdecode(np.frombuffer(bytes(frames[0][1].data), np.uint8), cv2.IMREAD_COLOR)
    h, w = first.shape[:2]
    out_path = os.path.join(args.run_dir, 'camera', 'front_aeb_overlay.mp4')
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    writer = cv2.VideoWriter(out_path, cv2.VideoWriter_fourcc(*'mp4v'), fps, (w, h))
    t0 = frames[0][0]
    for t, msg in frames:
        img = cv2.imdecode(np.frombuffer(bytes(msg.data), np.uint8), cv2.IMREAD_COLOR)
        v = latest(speed, t)
        r = latest(reason, t) or ''
        text = f't={t - t0:6.2f}s  cmd={0.0 if v is None else v:4.2f} m/s  {r}'
        cv2.rectangle(img, (0, 0), (w, 34), (0, 0, 0), -1)
        cv2.putText(img, text, (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)
        if latest(active, t):
            cv2.rectangle(img, (int(w * 0.25), 44), (int(w * 0.75), 110), (0, 0, 210), -1)
            cv2.putText(img, 'AEB ACTIVE', (int(w * 0.30), 82), cv2.FONT_HERSHEY_SIMPLEX, 1.0,
                        (255, 255, 255), 2)
            cv2.putText(img, (latest(status, t) or '')[:48], (int(w * 0.27), 104),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        writer.write(img)
    writer.release()
    print(f'저장: {out_path} ({len(frames)} 프레임, {fps:.1f} fps)')


if __name__ == '__main__':
    main()
