"""카메라 융합 비교 (2026-11-21). 실차 조건(20 cm 가드레일 + 2.7 m 복도, 라이다 잡음 0.02 m, 서보 오차 0.01 rad)에서
레이스 시나리오 Y(S1)·NYR(S2)·S(S3)를 센서 조건별로 돌린다.

    cd sim/aur_overtake && ./.venv/bin/python analysis/camera_eval.py --n 16 --methods d a

조건: lidar(기존) / black(검은 앞차: 라이다가 차체 가운데 ±5 cm만 맞힘, 실차 test_01) / black+depth / black+depth+yolo /
depth+yolo(검지 않은 앞차, 회귀 확인). 카메라 잡음·검출률은 scenario.yaml camera 절의 임시값이다 (실차 미측정).
"""
import argparse
import collections
import os
import sys
from multiprocessing import Pool

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, HERE)

from sim import Sim, load_cfg     # noqa: E402

REAL = {'track.wall_height': 0.2, 'track.outer_width': 2.7, 'lidar.range_noise_std': 0.02, 'vehicle.steer_offset': 0.01}
BLACK = {'lidar.leader_visible_half_width': 0.05}
CAM = {'camera.enabled': True}
CONDS = {
    'lidar': ({}, {}),
    'black': (BLACK, {}),
    'black+depth': (dict(BLACK, **CAM), {'camera_fusion': True}),
    'black+depth+yolo': (dict(BLACK, **CAM), {'camera_fusion': True, 'camera_lead': True}),
    'depth+yolo': (CAM, {'camera_fusion': True, 'camera_lead': True}),
    # YOLO 앞차 범위만 깊이를 합침 (레일 근처 카메라 점은 안 씀), +aeb = AEB도 합친 scan
    'black+yolo': (dict(BLACK, **CAM), {'camera_lead': True}),
    'black+yolo+aeb': (dict(BLACK, **CAM), {'camera_lead': True, 'camera_aeb': True}),
    'yolo': (CAM, {'camera_lead': True}),
    'yolo+aeb': (CAM, {'camera_lead': True, 'camera_aeb': True}),
    'black+yolo+aeb-wait': (dict(BLACK, **CAM), {'camera_lead': True, 'camera_aeb': True, 'cam_instant_confirm': False}),
    'yolo+aeb-wait': (CAM, {'camera_lead': True, 'camera_aeb': True, 'cam_instant_confirm': False}),
}


def run_one(job):
    cond, method, beh, seed = job
    over, params = CONDS[cond]
    r = Sim(method, load_cfg(**dict(REAL, **over, **{'leader.behavior': beh})), seed=seed, **params).run()
    return job, r


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('--n', type=int, default=16)
    ap.add_argument('--methods', nargs='*', default=['d'])
    ap.add_argument('--conds', nargs='*', default=list(CONDS))
    ap.add_argument('--behaviors', nargs='*', default=['Y', 'NYR', 'S'])
    args = ap.parse_args(argv)
    jobs = [(c, m, b, k) for c in args.conds for m in args.methods for b in args.behaviors for k in range(args.n)]
    with Pool() as pool:
        results = pool.map(run_one, jobs)
    agg = collections.defaultdict(collections.Counter)
    for (cond, method, beh, _), r in results:
        a = agg[(cond, method, beh)]
        a['n'] += 1
        a['success'] += bool(r['success'])
        a['pass'] += bool(r['pass_entered'])
        a['coll_cars'] += r['collision'] == 'cars'
        a['coll_ego_wall'] += r['collision'] == 'ego_wall'
        a['coll_leader_wall'] += r['collision'] == 'leader_wall'
        a['aeb_moving'] += r['N_AEB_moving']
        a['deadlock'] += bool(r['deadlock'])
    print('| 조건 | 방법 | 앞차 | 성공 | 추월 진입 | 차끼리 충돌 | ego 벽 | 앞차 벽 | 주행 중 AEB | 교착 |')
    print('| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |')
    for (cond, method, beh), a in agg.items():
        print('| %s | %s | %s | %d/%d | %d | %d | %d | %d | %d | %d |' % (
            cond, method, beh, a['success'], a['n'], a['pass'], a['coll_cars'], a['coll_ego_wall'],
            a['coll_leader_wall'], a['aeb_moving'], a['deadlock']))


if __name__ == '__main__':
    main()
