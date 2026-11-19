#!/usr/bin/env python3
"""추월 시나리오 1회 실행.

    python sim/aur_overtake/run.py --method d --leader Y                       # 창 없음, 결과 한 줄
    mjpython sim/aur_overtake/run.py --method d --leader Y --viewer --start-paused   # 대화형 뷰어 (macOS는 mjpython)
"""
import argparse
import json
import sys

from batch import AEB
from sim import Sim, load_cfg


def parse(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('--method', default='d', choices=list('abcd'))
    ap.add_argument('--leader', default=None, choices=['Y', 'NY', 'NYR', 'YR', 'S'],
                    help='선행차 행동 (기본: scenario.yaml). 레이스: Y=S1 비켜 줌, NYR=S2 급정지 뒤 재출발, S=S3 멈춘 차')
    ap.add_argument('--belief', default=None, choices=['original', 'survival'])
    ap.add_argument('--aeb', default=None, choices=list(AEB), help='AEB 조건 (batch.py와 같은 이름, 기본: scenario.yaml)')
    ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--W', type=float, default=None, help='통로 폭 [m]')
    ap.add_argument('--d-y', type=float, default=None, help='선행차 양보 반응 거리 [m]')
    ap.add_argument('--yield-yaw', type=float, default=None, help='양보 중 선행차 yaw 고정 [deg]')
    ap.add_argument('--scenario', default=None, help='scenario yaml 경로')
    ap.add_argument('--set', nargs='*', default=[], metavar='K=V', help="덮어쓰기: 'start.gap0=2.0'(scenario) 또는 'eta=0.95'(AurParams)")
    ap.add_argument('--viewer', action='store_true')
    ap.add_argument('--start-paused', action='store_true')
    ap.add_argument('--speed', type=float, default=1.0, help='(뷰어) 재생 배속. 실행 중에는 키 9 / 0')
    ap.add_argument('--quiet', action='store_true', help='상태 전이·양보·AEB 이벤트 줄을 찍지 않는다')
    return ap.parse_args(argv)


def make_sim(args, reuse=None):
    import yaml
    over, params = {}, {}
    for kv in args.set:
        k, v = kv.split('=', 1)
        (over if '.' in k else params)[k] = yaml.safe_load(v)
    if args.aeb:
        over.update(AEB[args.aeb])
    for key, v in (('leader.behavior', args.leader), ('track.W', args.W), ('leader.d_y', args.d_y),
                   ('leader.yield_yaw_deg', args.yield_yaw)):
        if v is not None:
            over[key] = v
    if args.belief:
        params['belief_model'] = args.belief
    sim = Sim(args.method, load_cfg(args.scenario, **over), seed=args.seed, reuse=reuse, **params)
    if not args.quiet:      # stderr: 마지막 결과 JSON 한 줄(stdout)은 그대로 파싱할 수 있게
        sim.on_event = lambda line: print(line, file=sys.stderr, flush=True)
    return sim


def main(argv=None):
    args = parse(argv)
    sim = make_sim(args)
    if not args.viewer:
        print(json.dumps(sim.run(), ensure_ascii=False))
        return
    import mujoco.viewer
    from viewer_ui import ViewerUI
    ui = ViewerUI(sim, start_paused=args.start_paused)
    ui.speed = args.speed
    with mujoco.viewer.launch_passive(sim.m, sim.d, key_callback=ui.on_key) as handle:
        ui.attach(handle)
        handle.cam.lookat[:] = [3.0, 0.0, 0.0]
        handle.cam.distance, handle.cam.elevation, handle.cam.azimuth = 6.0, -55.0, 90.0
        while handle.is_running():
            import time
            w0, t_start = time.monotonic(), sim.t
            result = sim.run(hook=ui.after_step)
            if not args.start_paused:
                print('[viewer] 재생: 시뮬 %.2f s / 실제 %.2f s = %.2fx (설정 %gx, 그리기 평균 %s ms)' % (
                    sim.t - t_start, time.monotonic() - w0, (sim.t - t_start) / max(1e-6, time.monotonic() - w0),
                    ui.speed, '%.1f' % ui.draw_ms if ui.draw_ms else '-'), file=sys.stderr, flush=True)
            print(json.dumps(result, ensure_ascii=False))
            if not ui.restart_requested:
                ui.hold_until_closed()
            if not ui.restart_requested:
                break
            sim = make_sim(args, reuse=(sim.m, sim.d))      # Home: 같은 설정·시드로 처음부터
            ui.reset(sim)


if __name__ == '__main__':
    main()
