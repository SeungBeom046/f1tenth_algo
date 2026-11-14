"""추월 시나리오 MuJoCo 월드(MJCF): 직선 통로 + 차 2대(mocap box).

좌표: x 전방, y 왼쪽 +, 통로 중심 y = 0, 바닥 z = 0. 차 body 원점 = 뒤축 중심 (base_link).
라이다 광선 대상은 벽과 상대 차뿐이다 (자기 차는 bodyexclude).
"""


def build_xml(cfg):
    t, v = cfg['track'], cfg['vehicle']
    W, x0, x1, h = t['W'], t['x_start'], t['x_end'], t['wall_height']
    th = 0.05
    L, wcar, H, ro = v['length'], cfg['w_car'], v['height'], v['rear_overhang']

    def wall(name, y):
        return (f'    <geom name="{name}" type="box" pos="{0.5 * (x0 + x1):.3f} {y:.4f} {0.5 * h:.3f}" '
                f'size="{0.5 * (x1 - x0):.3f} {0.5 * th:.3f} {0.5 * h:.3f}" rgba="0.55 0.6 0.7 1"/>')

    def car(name, rgba):
        return (f'    <body name="{name}" mocap="true" pos="0 0 0">\n'
                f'      <geom name="{name}_body" type="box" pos="{0.5 * L - ro:.4f} 0 {0.5 * H:.4f}" '
                f'size="{0.5 * L:.4f} {0.5 * wcar:.4f} {0.5 * H:.4f}" rgba="{rgba}"/>\n    </body>')

    return f'''<mujoco model="overtake_corridor">
  <visual><global offwidth="1280" offheight="720"/></visual>
  <asset><texture name="grid" type="2d" builtin="checker" width="512" height="512" rgb1=".85 .85 .85" rgb2=".75 .75 .75"/>
    <material name="floor" texture="grid" texrepeat="60 4"/></asset>
  <worldbody>
    <light pos="5 0 6" dir="0 0 -1"/>
    <geom name="floor" type="plane" pos="{0.5 * (x0 + x1):.2f} 0 0" size="{0.5 * (x1 - x0) + 1:.2f} {W:.2f} 0.1" material="floor"/>
{wall('left_wall', 0.5 * W + 0.5 * th)}
{wall('right_wall', -0.5 * W - 0.5 * th)}
{car('ego', '0.2 0.4 0.9 1')}
{car('leader', '0.9 0.2 0.2 1')}
  </worldbody>
</mujoco>'''
