#!/usr/bin/env python3
"""시뮬 환경 점검: mujoco 버전, mjpython, my_algo 순수 모듈 import, 오프스크린 렌더, mj_multiRay.

    sim/.venv/bin/python sim/check_env.py
"""
import importlib
import os
import platform
import shutil
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)

PURE = ['vehicle_utils', 'geometry', 'occupancy', 'phantom', 'speed_policy', 'aeb', 'aeb_event',
        'metrics', 'braking', 'pc2', 'track_layout', 'trigger_math', 'mux_logic', 'perception']


def main():
    ok = True
    print(f'python {sys.version.split()[0]} {platform.machine()} {sys.executable}')
    try:
        import mujoco
        print(f'mujoco {mujoco.__version__} {mujoco.__file__}')
    except ImportError as e:
        print(f'mujoco 없음: {e}')
        return 1
    mjpy = os.path.join(os.path.dirname(sys.executable), 'mjpython')
    print(f'mjpython {"있음 " + mjpy if os.path.exists(mjpy) else "없음 (뷰어 모드 불가)"}'
          f'{" | PATH: " + shutil.which("mjpython") if shutil.which("mjpython") else ""}')
    print(f'mj_multiRay {"있음" if hasattr(mujoco, "mj_multiRay") else "없음"}')
    for name in PURE:
        try:
            importlib.import_module('my_algo.' + name)
            print(f'  my_algo.{name}: OK')
        except Exception as e:  # 아직 없는 모듈(이 브랜치 이전)도 보고만 한다
            print(f'  my_algo.{name}: 실패 {type(e).__name__}: {e}')
            ok = False
    try:
        m = mujoco.MjModel.from_xml_string(
            '<mujoco><worldbody><light pos="0 0 3"/><geom type="box" size=".1 .1 .1"/></worldbody></mujoco>')
        d = mujoco.MjData(m)
        r = mujoco.Renderer(m, 64, 64)
        mujoco.mj_forward(m, d)
        r.update_scene(d)
        print(f'오프스크린 렌더 OK {r.render().shape}')
        r.close()
    except Exception as e:
        print(f'오프스크린 렌더 실패: {e}')
        ok = False
    print('점검 결과:', '통과' if ok else '문제 있음')
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
