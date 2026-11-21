"""D455 깊이 카메라 융합 (my_algo/aur/camera.py)과 YOLO 앞차 확인 (tracker camera_lead).

합성 깊이 영상: 라이다 수평 프레임 원점, 바닥 z = 0, 카메라는 라이다 앞 0.27 m·높이 0.16 m에서 17.5° 숙임
(실측 2026-11-21: 앞바퀴 축 7 cm 앞, 바닥에서 0.16 m, 15~20° 숙임). D455 깊이 잡음 σ = 0.005·z² [m].
"""
import math
from dataclasses import replace

import numpy as np

from my_algo.aur import camera
from my_algo.aur.params import AurParams, EXPERIMENT_OVERRIDES
from my_algo.aur.tracker import Pt, VehicleTracker

K = (380.0, 380.0, 320.0, 240.0)            # 640×480, 수평 시야 약 80°
MOUNT = (0.27, 0.0, 0.0)
TRUE = camera.Calib(math.radians(17.5), math.radians(1.0), 0.16)
RAIL_Y, RAIL_H = 0.5, 0.30                  # 통로 폭 1.0 m, 덕트 높이
CAR = (1.5, 2.0, -0.135, 0.135, 0.02, 0.25)  # 앞차 상자 x0 x1 y0 y1 z0 z1 (라이다 프레임)
GRID = (-math.pi, 0.00436, int(math.ceil(2 * math.pi / 0.00436)))


def render(boxes, calib=TRUE, noise=0.0, seed=0):
    """바닥 + 상자들에 광선을 쏜 깊이 영상 [m] (optical z)."""
    v, u = np.mgrid[0:480, 0:640]
    d_opt = np.stack([(u - K[2]) / K[0], (v - K[3]) / K[1], np.ones_like(u, float)], -1).reshape(-1, 3)
    d = camera.to_body(d_opt) @ camera.rotation(calib.pitch, calib.roll).T     # level, optical z = 1 기준
    o = np.array([MOUNT[0], MOUNT[1], calib.height])
    t = np.full(len(d), np.inf)
    down = d[:, 2] < 0
    t[down] = -o[2] / d[down, 2]
    for x0, x1, y0, y1, z0, z1 in boxes:
        with np.errstate(divide='ignore', invalid='ignore'):
            lo = (np.array([x0, y0, z0]) - o) / d
            hi = (np.array([x1, y1, z1]) - o) / d
        tn = np.nanmax(np.minimum(lo, hi), axis=1)
        tf = np.nanmin(np.maximum(lo, hi), axis=1)
        hit = (tn <= tf) & (tn > 0)
        t[hit] = np.minimum(t[hit], tn[hit])
    z = t.reshape(480, 640)
    z[z > 6.0] = 0.0
    if noise:
        z = z + np.random.default_rng(seed).normal(0.0, 1.0, z.shape) * noise * z ** 2
    return z


def rails():
    return [(-1.0, 10.0, RAIL_Y, RAIL_Y + 0.3, 0.0, RAIL_H), (-1.0, 10.0, -RAIL_Y - 0.3, -RAIL_Y, 0.0, RAIL_H)]


def body_of(z):
    opt, u, v = camera.deproject(z, K, stride=4, scale=1.0)
    return camera.to_body(opt), u, v


def ranges_of(z, calib):
    b, _, _ = body_of(z)
    return camera.obstacle_ranges(b, calib, MOUNT, *GRID)[0]


def bin_angle(i):
    return GRID[0] + (i + 0.5) * GRID[1]


def test_floor_fit_recovers_pitch_roll_height_with_noise():
    b, _, _ = body_of(render(rails() + [CAR], noise=0.005))
    calib, frac = camera.fit_floor(b)
    assert abs(math.degrees(calib.pitch - TRUE.pitch)) < 0.3
    assert abs(math.degrees(calib.roll - TRUE.roll)) < 0.3
    assert abs(calib.height - TRUE.height) < 0.005 and frac > 0.3


def test_noisy_floor_alone_gives_no_obstacle():
    for seed in range(3):
        b, _, _ = body_of(render([], noise=0.005, seed=seed))
        calib, _ = camera.fit_floor(b)
        assert not np.isfinite(camera.obstacle_ranges(b, calib, MOUNT, *GRID)[0]).any()


def test_rails_and_car_ranges_match_geometry():
    z = render(rails() + [CAR], noise=0.005)
    calib, _ = camera.fit_floor(body_of(z)[0])
    r = ranges_of(z, calib)
    hits = np.nonzero(np.isfinite(r))[0]
    a = np.array([bin_angle(i) for i in hits])
    car = np.abs(a) < math.atan2(0.10, 1.5)
    assert car.sum() > 5 and np.all(np.abs(r[hits][car] * np.cos(a[car]) - CAR[0]) < 0.04)
    rail = np.abs(a) > math.radians(25)
    lat = np.abs(r[hits][rail] * np.sin(a[rail]))
    assert rail.sum() > 20 and np.all(np.abs(lat - RAIL_Y) < 0.04)


def test_lead_from_yolo_box():
    z = render(rails() + [CAR], noise=0.005)
    calib, _ = camera.fit_floor(body_of(z)[0])
    b, u, v = body_of(z)
    # 상자 = 앞차 꼭짓점 투영 (YOLO가 낼 상자)
    corners = np.array([[x, y, h] for x in CAR[:2] for y in CAR[2:4] for h in CAR[4:]])
    q = corners - np.array([MOUNT[0], MOUNT[1], TRUE.height])
    q = q @ camera.rotation(TRUE.pitch, TRUE.roll)        # level → body
    uu = K[2] - q[:, 1] / q[:, 0] * K[0]
    vv = K[3] - q[:, 2] / q[:, 0] * K[1]
    lead = camera.lead_from_box(b, u, v, calib, MOUNT, (uu.min(), vv.min(), uu.max(), vv.max()))
    assert abs(lead['r'] - CAR[0]) < 0.04
    assert lead['a_lo'] < 0 < lead['a_hi'] and lead['a_hi'] - lead['a_lo'] < 2 * math.atan2(0.14, 1.5)
    assert camera.lead_from_box(b, u, v, calib, MOUNT, (0, 0, 5, 5)) is None


def test_fuse_takes_nearer_per_bin_across_grids():
    lidar = [np.inf, 2.0, 1.0, np.inf]
    out = camera.fuse_ranges(lidar, 0.0, 0.1, [1.5, np.inf, 1.2, 0.8, 0.5, np.inf, 0.3, 9.0], 0.0, 0.05)
    assert list(out) == [1.5, 0.8, 0.5, 0.3]


P = replace(AurParams(), **dict(EXPERIMENT_OVERRIDES, W=1.0))
C = replace(P, camera_lead=True)


def narrow_car(x_bumper=1.5, width=0.05, n=5):
    """검은 차 위 모서리만 보인 좁은 클러스터 (라이다 x = 범퍼 + lidar_to_front + 0.2)."""
    x = x_bumper + P.lidar_to_front + 0.2
    return [Pt(i, math.atan2(y, x), x, y) for i, y in enumerate(np.linspace(-width / 2, width / 2, n))]


def test_camera_confirms_narrow_lead_and_uses_camera_face():
    pts = narrow_car()
    cam = {'r': 1.5 + P.lidar_to_front, 'a_lo': -0.09, 'a_hi': 0.09}
    trk = VehicleTracker(C)
    res = [trk.update(pts, 0.1 * k, 0.0, 0.0, 1.0, cam=cam) for k in range(3)]
    assert [r.valid for r in res] == [False, False, True]      # 라이다만이면 좁아서 영영 후보가 아니다
    assert abs(res[-1].d - 1.5) < 1e-9 and trk.cam_confirmed
    # 카메라가 다른 곳(옆 레일 방향)을 차로 봤으면 기존 규칙 (좁아서 후보 아님)
    other = VehicleTracker(C)
    assert not other.update(pts, 0.0, 0.0, 0.0, 1.0, cam={'r': 1.78, 'a_lo': 0.4, 'a_hi': 0.5}).valid
    # 플래그 off면 카메라를 넘겨도 기존과 같다
    off = VehicleTracker(P)
    assert not off.update(pts, 0.0, 0.0, 0.0, 1.0, cam=cam).valid and not off.cam_confirmed


def test_camera_confirmation_waits_three_scans_unless_instant():
    pts = narrow_car(width=0.2)
    cam = {'r': 1.5 + P.lidar_to_front + 0.2, 'a_lo': -0.07, 'a_hi': 0.07}
    for trk in (VehicleTracker(C), VehicleTracker(C)):
        assert [trk.update(pts, 0.1 * k, 0.0, 0.0, 1.0, cam=cam).valid for k in range(3)] == [False, False, True]
    assert VehicleTracker(replace(C, cam_instant_confirm=True)).update(pts, 0.0, 0.0, 0.0, 1.0, cam=cam).valid


def test_node_image_decoding_handles_row_padding_and_rgb():
    """camera_fusion.image_array (ROS 없이 함수만 떼어 검사)."""
    import ast
    import os
    from types import SimpleNamespace
    src = os.path.join(os.path.dirname(__file__), '..', 'my_algo', 'camera_fusion.py')
    tree = ast.parse(open(src).read())
    keep = [n for n in tree.body if (isinstance(n, ast.FunctionDef) and n.name == 'image_array')
            or (isinstance(n, ast.Assign) and getattr(n.targets[0], 'id', '') == '_DTYPE')]
    ns = {'np': np}
    exec(compile(ast.Module(body=keep, type_ignores=[]), src, 'exec'), ns)
    depth = np.arange(12, dtype=np.uint16).reshape(3, 4)
    padded = np.zeros((3, 6), np.uint16)
    padded[:, :4] = depth
    msg = SimpleNamespace(encoding='16UC1', height=3, width=4, step=12, data=padded.tobytes())
    assert (ns['image_array'](msg) == depth).all()
    rgb = np.arange(2 * 2 * 3, dtype=np.uint8).reshape(2, 2, 3)
    msg = SimpleNamespace(encoding='rgb8', height=2, width=2, step=6, data=rgb.tobytes())
    assert (ns['image_array'](msg) == rgb[..., ::-1]).all()
