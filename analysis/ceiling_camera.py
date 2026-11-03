#!/usr/bin/env python3
"""천장 카메라(60 fps) 처리: ArUco 검출 → 바닥 호모그래피 → 두 차량 위치 → 통과시각·실제 d_e·최소 거리.

    python3 analysis/ceiling_camera.py --video run.mp4 --calib floor_calib.json --run-dir <회차 폴더>

floor_calib.json (트랙 좌표 = 에고 시작 시 앞 끝이 x=0, 진행 +x, 왼쪽 +y; 바닥 테이프 0.5 m 눈금으로 측정):
    {"image_points": [[u,v],[u,v],[u,v],[u,v]], "world_points": [[x,y],[x,y],[x,y],[x,y]],
     "aruco_dict": "DICT_4X4_50", "ego_id": 1, "car2_id": 2,
     "ego_marker_to_front": 0.20, "car2_marker_to_front": 0.20,   # 마커 중심 → 차체 앞 끝 [m] (마커 x축 방향)
     "car2_footprint": [[0.25,0.15],[-0.25,0.15],[-0.25,-0.15],[0.25,-0.15]],  # 마커 중심 기준
     "ego_marker_x_in_base": 0.15,  # 에고 마커 중심의 base_link x [m] (footprint 변환용)
     "lane_boundary_y": -0.35, "collision_x": 5.0, "lines_x": [2.5, 7.0]}

산출 (<run_dir>/analysis/ceiling.json, ceiling_track.csv):
    lines_x 각 선을 에고 앞 끝이 지나는 시각 (전후 두 프레임 선형 보간 → 서브프레임 시각; 2 m/s에서 프레임당 3.3 cm)
    d_e_actual: #2 앞 끝이 차선 경계(lane_boundary_y)에 닿는 순간 에고 앞 끝과 collision_x의 거리 (2.10절 정의)
    min_distance: 두 차체 다각형 사이 최소 거리 (footprint 기준), collision
"""

import argparse
import csv
import json
import math
import os

import numpy as np

from my_algo.geometry import load_geometry, point_polygon_distance


def detector(dict_name):
    import cv2
    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dict_name))
    if hasattr(cv2.aruco, 'ArucoDetector'):          # OpenCV >= 4.7
        det = cv2.aruco.ArucoDetector(dictionary, cv2.aruco.DetectorParameters())
        return lambda img: det.detectMarkers(img)[:2]
    params = cv2.aruco.DetectorParameters_create()
    return lambda img: cv2.aruco.detectMarkers(img, dictionary, parameters=params)[:2]


def to_world(H, pts):
    p = np.column_stack([pts, np.ones(len(pts))]) @ H.T
    return p[:, :2] / p[:, 2:3]


def crossing_time(t, x, line):
    """x(t)가 line을 처음 지나는 시각 (선형 보간)."""
    for k in range(len(x) - 1):
        if np.isfinite(x[k]) and np.isfinite(x[k + 1]) and (x[k] - line) * (x[k + 1] - line) <= 0 \
                and x[k] != x[k + 1]:
            return float(t[k] + (line - x[k]) / (x[k + 1] - x[k]) * (t[k + 1] - t[k]))
    return None


def transform(poly, x, y, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return [(x + c * px - s * py, y + s * px + c * py) for px, py in poly]


def polygon_distance(a, b, samples=20):
    """두 다각형 최소 거리 (변 위 표본점 → 상대 다각형, 양방향)."""
    best = math.inf
    for p, q in ((a, b), (b, a)):
        for i in range(len(p)):
            (x1, y1), (x2, y2) = p[i], p[(i + 1) % len(p)]
            for f in np.linspace(0, 1, samples):
                best = min(best, point_polygon_distance(x1 + f * (x2 - x1), y1 + f * (y2 - y1), q))
    return best


def main():
    import cv2
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument('--video', required=True)
    ap.add_argument('--calib', required=True)
    ap.add_argument('--run-dir', required=True)
    args = ap.parse_args()
    with open(args.calib, encoding='utf-8') as f:
        cal = json.load(f)
    H, _ = cv2.findHomography(np.array(cal['image_points'], float), np.array(cal['world_points'], float))
    detect = detector(cal.get('aruco_dict', 'DICT_4X4_50'))
    geom = load_geometry(os.path.join(args.run_dir, 'config_snapshot', 'vehicle_geometry.yaml'))
    # 에고 footprint를 마커 중심 기준으로 옮긴다 (base_link → 마커)
    ego_fp = [(x - cal['ego_marker_x_in_base'], y) for x, y in geom.footprint]

    cap = cv2.VideoCapture(args.video)
    fps = cap.get(cv2.CAP_PROP_FPS) or 60.0
    rows = []
    k = 0
    while True:
        ok, img = cap.read()
        if not ok:
            break
        corners, ids = detect(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY))
        row = {'frame': k, 't': k / fps}
        for name, mid, off in (('ego', cal['ego_id'], cal['ego_marker_to_front']),
                               ('car2', cal['car2_id'], cal['car2_marker_to_front'])):
            row.update({f'{name}_{q}': math.nan for q in ('x', 'y', 'yaw', 'front_x', 'front_y')})
            if ids is None:
                continue
            for c, i in zip(corners, ids.flatten()):
                if int(i) != mid:
                    continue
                w = to_world(H, c.reshape(4, 2))
                cx, cy = w.mean(axis=0)
                yaw = math.atan2(w[1, 1] - w[0, 1], w[1, 0] - w[0, 0])  # 마커 x축 = 차량 전방으로 부착
                row.update({f'{name}_x': cx, f'{name}_y': cy, f'{name}_yaw': yaw,
                            f'{name}_front_x': cx + off * math.cos(yaw),
                            f'{name}_front_y': cy + off * math.sin(yaw)})
        rows.append(row)
        k += 1
    cap.release()

    t = np.array([r['t'] for r in rows])
    ego_front = np.array([r['ego_front_x'] for r in rows])
    car2_front_y = np.array([r['car2_front_y'] for r in rows])
    result = {'fps': fps, 'frames': len(rows),
              'line_times': {str(x): crossing_time(t, ego_front, x) for x in cal.get('lines_x', [])}}
    t_enter = crossing_time(t, car2_front_y, cal['lane_boundary_y'])
    result['car2_enter_time'] = t_enter
    if t_enter is not None:
        result['d_e_actual'] = float(cal['collision_x'] - np.interp(t_enter, t, ego_front))
    dists = []
    for r in rows:
        if all(np.isfinite(r[q]) for q in ('ego_x', 'ego_y', 'ego_yaw', 'car2_x', 'car2_y', 'car2_yaw')):
            dists.append(polygon_distance(transform(ego_fp, r['ego_x'], r['ego_y'], r['ego_yaw']),
                                          transform(cal['car2_footprint'], r['car2_x'], r['car2_y'],
                                                    r['car2_yaw'])))
    result['min_distance'] = float(min(dists)) if dists else None
    result['collision'] = bool(dists) and min(dists) <= 0.0
    adir = os.path.join(args.run_dir, 'analysis')
    os.makedirs(adir, exist_ok=True)
    with open(os.path.join(adir, 'ceiling_track.csv'), 'w', newline='', encoding='utf-8') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    with open(os.path.join(adir, 'ceiling.json'), 'w', encoding='utf-8') as f:
        json.dump(result, f, indent=2)
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
