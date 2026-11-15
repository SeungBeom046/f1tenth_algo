#!/usr/bin/env python3
"""시뮬 회차의 위에서 본 영상 (mp4): 점유격자·가림 경계·phantom·차량·v_cap을 겹쳐 그린다.

    sim/.venv/bin/python sim/video.py <run_dir> [--fps 10]
"""
import argparse
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path[:0] = [HERE, os.path.dirname(HERE)]


def render(run_dir, fps=10):
    import imageio.v2 as imageio
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Polygon, Rectangle

    import vehicle
    from my_algo.geometry import load_geometry
    with open(os.path.join(run_dir, 'meta.json'), encoding='utf-8') as f:
        meta = json.load(f)
    z = np.load(os.path.join(run_dir, 'log.npz'))
    g = np.load(os.path.join(run_dir, 'grids.npz'))
    geom = load_geometry(os.path.join(os.path.dirname(HERE), 'config', 'vehicle_geometry.yaml'))
    c2 = meta['config']['sim']['car2']
    car2_fp = [(c2['length'] - c2['rear_overhang'], c2['width'] / 2), (-c2['rear_overhang'], c2['width'] / 2),
               (-c2['rear_overhang'], -c2['width'] / 2), (c2['length'] - c2['rear_overhang'], -c2['width'] / 2)]
    tr = meta['config']['sim']['track']
    out_path = os.path.join(run_dir, 'topdown.mp4')
    writer = imageio.get_writer(out_path, fps=fps)
    fig, ax = plt.subplots(figsize=(12, 4), dpi=100)
    origins = g['origin']
    for i, ft in enumerate(z['frame_t']):
        k = int(np.searchsorted(z['t'], ft))
        k = min(k, len(z['t']) - 1)
        ax.clear()
        key = f'g{i:04d}'
        if key in g and i < len(origins):
            grid = g[key].astype(float)
            ox, oy, res = origins[i]
            img = np.where(grid < 0, 0.5, np.where(grid >= 100, 0.0, 1.0))
            ax.imshow(img, cmap='gray', vmin=0, vmax=1, origin='lower', alpha=0.6,
                      extent=[ox, ox + grid.shape[1] * res, oy, oy + grid.shape[0] * res])
        for b in meta['world_boxes']:
            (cx, cy, _), (hx, hy, _) = b['center'], b['half']
            ax.add_patch(Rectangle((cx - hx, cy - hy), 2 * hx, 2 * hy, fc='none', ec='#8a6d3b', lw=1))
        bxy = z['boundary_xy'][z['boundary_frame'] == i]
        if len(bxy):
            ax.scatter(bxy[:, 0], bxy[:, 1], s=2, c='#eda100', label='가림 경계')
        pxy = z['phantoms_xy'][z['phantoms_frame'] == i]
        if len(pxy):
            ax.scatter(pxy[:, 0], pxy[:, 1], s=25, c='#e34948', marker='x', label='phantom')
        ego = vehicle.transform_polygon(geom.footprint, z['x'][k], z['y'][k], z['yaw'][k])
        ax.add_patch(Polygon(ego, fc='#2a78d6', ec='k'))
        if np.isfinite(z['car2_x'][k]):
            p2 = vehicle.transform_polygon(car2_fp, z['car2_x'][k], z['car2_y'][k], math.pi / 2)
            ax.add_patch(Polygon(p2, fc='#e34948', ec='k'))
        ax.set_xlim(tr['start_x'], tr['end_x'] + 0.3)
        ax.set_ylim(0.35 - tr['track_width'] - 0.1, 0.6)
        ax.set_aspect('equal')
        vcap = z['v_cap'][k]
        ax.set_title(f"{meta['scenario']} {meta['method']} t={ft:4.1f}s  v={z['v'][k]:.2f}  "
                     f"v_cap={'inf' if not np.isfinite(vcap) else f'{vcap:.2f}'}  d={z['d'][k]:.2f}  "
                     f"{z['stop_reason'][k]}", fontsize=10)
        fig.canvas.draw()
        frame = np.asarray(fig.canvas.buffer_rgba())[:, :, :3]
        writer.append_data(frame)
    writer.close()
    plt.close(fig)
    return out_path


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('run_dir')
    ap.add_argument('--fps', type=int, default=10)
    a = ap.parse_args()
    print(render(a.run_dir, a.fps))
