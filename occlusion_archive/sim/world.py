"""MuJoCo 월드(MJCF) 생성. 트랙 좌표는 scenario yaml + track_layout, 치수는 sim.yaml·vehicle_geometry.yaml.

좌표: 월드 x = 트랙 x (AUTO 시작 시 에고 차체 앞 끝 = 0), y = 왼쪽 +, 차선 중앙 y = 0, z = 바닥 0.
장애물은 모두 box 도형 (메시 없음, 레이캐스트 정확도). 에고·#2는 mocap 물체로 위치를 직접 정한다.
"""
from dataclasses import dataclass, field
from typing import List, Tuple

from my_algo.track_layout import from_params

STATIC_GROUP = 0     # 장애물 (라이다·충돌 대상)
VEHICLE_GROUP = 1    # 차량 (#2는 라이다 대상, 에고는 광선에서 제외)


@dataclass
class Box:
    name: str
    center: Tuple[float, float, float]
    half: Tuple[float, float, float]
    rgba: str = '0.6 0.6 0.6 1'


@dataclass
class World:
    xml: str
    boxes: List[Box] = field(default_factory=list)   # 정적 장애물 (충돌 판정용 사각형)
    layout: object = None
    lane_half: float = 0.35
    car2_start: Tuple[float, float, float] = None    # (x, y, yaw) 뒤축 기준, S1만


def build(cfg):
    sim, scen, geom = cfg.sim, cfg.scenario_p, None
    t = sim['track']
    lane_half = 0.5 * t['lane_width']
    left_y = lane_half + t['left_wall_offset']
    right_room_y = left_y - t['track_width']
    x0, x1 = t['start_x'], t['end_x']
    lay = from_params(scen)
    boxes = []

    def add(name, xa, xb, ya, yb, h, rgba='0.65 0.62 0.55 1'):
        boxes.append(Box(name, (0.5 * (xa + xb), 0.5 * (ya + yb), 0.5 * h),
                         (0.5 * abs(xb - xa), 0.5 * abs(yb - ya), 0.5 * h), rgba))

    th = 0.05  # 벽 두께
    add('left_wall', x0, x1, left_y, left_y + th, t['left_wall_height'], '0.55 0.6 0.7 1')
    add('room_right', x0, x1, right_room_y - th, right_room_y, t['room_wall_height'], '0.5 0.5 0.5 1')
    add('back_wall', x0 - th, x0, right_room_y, left_y, t['room_wall_height'], '0.5 0.5 0.5 1')
    add('barrier', x1, x1 + 0.2, right_room_y, left_y, t['barrier_height'], '0.9 0.5 0.2 1')
    face = -lane_half - t['box_offset_from_lane']           # 박스 차선 쪽 면 y
    if scen.get('right_wall'):
        add('right_wall', x0, x1, face - t['box_depth'], face, t['right_wall_height'], '0.6 0.55 0.5 1')
    else:
        for i, (bx0, bx1) in enumerate(lay.boxes):
            add(f'P{i + 1}', bx0, bx1, face - t['box_depth'], face, t['box_height'])

    car2_start = None
    c2 = sim['car2']
    if scen.get('trigger_enabled') and lay.collision_x is not None:
        # #2 앞 끝이 차선 경계에서 lane_boundary_distance만큼 들어간 곳, 차선 쪽(+y)을 향함, 틈 중앙
        nose_y = -lane_half - float(cfg.popout['lane_boundary_distance'])
        rear_axle_y = nose_y - (c2['length'] - c2['rear_overhang'])
        car2_start = (lay.collision_x, rear_axle_y, 1.5707963267948966)

    geoms = '\n'.join(
        f'    <geom name="{b.name}" type="box" pos="{b.center[0]:.4f} {b.center[1]:.4f} {b.center[2]:.4f}" '
        f'size="{b.half[0]:.4f} {b.half[1]:.4f} {b.half[2]:.4f}" rgba="{b.rgba}" group="{STATIC_GROUP}"/>'
        for b in boxes)
    eh = sim['ego']['body_height']
    car2_xml = ''
    if car2_start:
        L, W, H, ro = c2['length'], c2['width'], c2['height'], c2['rear_overhang']
        car2_xml = (f'    <body name="car2" mocap="true" pos="0 0 0">\n'
                    f'      <geom name="car2_body" type="box" pos="{0.5 * L - ro:.4f} 0 {0.5 * H:.4f}" '
                    f'size="{0.5 * L:.4f} {0.5 * W:.4f} {0.5 * H:.4f}" rgba="0.9 0.2 0.2 1" group="{VEHICLE_GROUP}"/>\n'
                    f'    </body>')
    xml = f'''<mujoco model="occlusion_track_{cfg.scenario}">
  <visual><global offwidth="1280" offheight="720"/></visual>
  <asset><texture name="grid" type="2d" builtin="checker" width="512" height="512" rgb1=".85 .85 .85" rgb2=".75 .75 .75"/>
    <material name="floor" texture="grid" texrepeat="18 5"/></asset>
  <worldbody>
    <light pos="4 0 6" dir="0 0 -1"/>
    <geom name="floor" type="plane" pos="4 -0.9 0" size="{0.5 * (x1 - x0) + 1:.2f} {0.5 * t['track_width'] + 1:.2f} 0.1" material="floor"/>
{geoms}
    <body name="ego" mocap="true" pos="0 0 0">
      {{EGO_GEOM}}
    </body>
{car2_xml}
  </worldbody>
</mujoco>'''
    return World(xml=xml, boxes=boxes, layout=lay, lane_half=lane_half, car2_start=car2_start), eh


def ego_geom_xml(footprint, height):
    xs = [p[0] for p in footprint]
    ys = [p[1] for p in footprint]
    cx, cy = 0.5 * (max(xs) + min(xs)), 0.5 * (max(ys) + min(ys))
    return (f'<geom name="ego_body" type="box" pos="{cx:.4f} {cy:.4f} {0.5 * height:.4f}" '
            f'size="{0.5 * (max(xs) - min(xs)):.4f} {0.5 * (max(ys) - min(ys)):.4f} {0.5 * height:.4f}" '
            f'rgba="0.2 0.4 0.9 1" group="{VEHICLE_GROUP}"/>')
