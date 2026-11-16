"""추월 시나리오 MuJoCo 월드(MJCF): 직선 통로 + 차 2대(mocap).

좌표: x 전방, y 왼쪽 +, 통로 중심 y = 0, 바닥 z = 0. 차 body 원점 = 뒤축 중심 (base_link).

geom 그룹 두 가지:
- SENSE_GROUP(4): 라이다가 맞히는 판정용 모양 (벽 판, 차 상자). 예전 월드와 같은 크기·위치라 scan이 그대로다.
  뷰어 기본 표시에서는 숨는다 (뷰어 키 5로 보기).
- 0: 보기용 (도로·가장자리선·가드레일·바퀴·차체). 라이다(Lidar의 geomgroup)와 충돌 판정(sim.check_collision, 기하 계산)에 안 쓴다.
"""
SENSE_GROUP = 4


def _box(name, pos, size, rgba, extra=''):
    return (f'    <geom name="{name}" type="box" pos="{pos[0]:.4f} {pos[1]:.4f} {pos[2]:.4f}" '
            f'size="{size[0]:.4f} {size[1]:.4f} {size[2]:.4f}" rgba="{rgba}" {extra}/>')


def _corridor_visual(W, x0, x1):
    """가드레일(연석 + 기둥 + 레일 2단), 가장자리 실선, 5 m 눈금. 중앙선 없음 (차선 하나짜리 통로)."""
    out = []
    xm, half = 0.5 * (x0 + x1), 0.5 * (x1 - x0)
    vis = 'contype="0" conaffinity="0"'
    for side, s in (('l', 1), ('r', -1)):
        y_face = s * 0.5 * W
        out.append(_box(f'curb_{side}', (xm, y_face + s * 0.03, 0.02), (half, 0.03, 0.02), '0.78 0.78 0.76 1',
                        f'material="concrete" {vis}'))
        for z in (0.13, 0.25):
            out.append(_box(f'rail_{side}_{z:.2f}', (xm, y_face + s * 0.008, z), (half, 0.007, 0.035),
                            '0.80 0.82 0.85 1', f'material="steel" {vis}'))
        k = 0
        x = x0
        while x <= x1 + 1e-9:
            out.append(_box(f'post_{side}_{k}', (x, y_face + s * 0.035, 0.16), (0.015, 0.02, 0.16),
                            '0.35 0.37 0.40 1', f'material="steel" {vis}'))
            if k % 2 == 0:      # 반사판
                out.append(_box(f'refl_{side}_{k}', (x, y_face + s * 0.004, 0.20), (0.012, 0.002, 0.012),
                                '1 0.75 0.1 1', f'material="reflector" {vis}'))
            x += 0.5
            k += 1
    for side, s in (('l', 1), ('r', -1)):
        out.append(_box(f'edge_{side}', (xm, s * (0.5 * W - 0.03), 0.0012), (half, 0.008, 0.0012),
                        '0.95 0.95 0.95 1', vis))
    # 중앙선은 그리지 않는다: 이 통로는 차선 하나짜리 좁은 길이다 (2차선이면 앞차가 비킬 필요 없이 추월된다)
    for k in range(int(x1) // 5 + 1):     # 5 m마다 거리 표시 (가장자리 짧은 가로줄)
        for s in (1, -1):
            out.append(_box(f'tick_{k}_{s}', (5.0 * k, s * (0.5 * W - 0.09), 0.0012), (0.006, 0.05, 0.0012),
                            '0.95 0.95 0.95 0.9', vis))
    return '\n'.join(out)


def _car_visual(name, L, ro, wb, wcar, shell, accent):
    """F1TENTH 비슷한 모양 (뒤축 원점). 보기용이라 라이다·충돌에 안 쓴다."""
    vis = 'contype="0" conaffinity="0"'
    g = []
    xc = 0.5 * L - ro
    yw = 0.5 * wcar - 0.022
    for i, xw in enumerate((0.0, wb)):
        for s in (1, -1):
            g.append(f'      <geom name="{name}_tire{i}{s}" type="cylinder" pos="{xw:.3f} {s * yw:.3f} 0.05" '
                     f'size="0.05 0.022" euler="90 0 0" material="rubber" {vis}/>')
            g.append(f'      <geom name="{name}_hub{i}{s}" type="cylinder" pos="{xw:.3f} {s * (yw + 0.004):.3f} 0.05" '
                     f'size="0.026 0.02" euler="90 0 0" rgba="0.75 0.76 0.78 1" material="steel" {vis}/>')
    g.append(f'      <geom name="{name}_chassis" type="box" pos="{xc:.3f} 0 0.045" size="{0.5 * L - 0.01:.3f} 0.085 0.008" '
             f'material="carbon" {vis}/>')
    g.append(f'      <geom name="{name}_shell" type="ellipsoid" pos="{xc + 0.01:.3f} 0 0.075" size="0.21 0.095 0.05" '
             f'rgba="{shell}" material="paint" {vis}/>')
    g.append(f'      <geom name="{name}_stripe" type="box" pos="{xc + 0.01:.3f} 0 0.1245" size="0.15 0.014 0.0015" '
             f'rgba="{accent}" {vis}/>')
    g.append(f'      <geom name="{name}_bumper" type="box" pos="{L - ro - 0.01:.3f} 0 0.04" size="0.012 0.115 0.014" '
             f'material="carbon" {vis}/>')
    g.append(f'      <geom name="{name}_wing" type="box" pos="{-ro + 0.03:.3f} 0 0.165" size="0.03 0.12 0.004" '
             f'rgba="{shell}" material="paint" {vis}/>')
    for s in (1, -1):
        g.append(f'      <geom name="{name}_strut{s}" type="box" pos="{-ro + 0.035:.3f} {s * 0.07:.3f} 0.13" '
                 f'size="0.006 0.004 0.033" material="carbon" {vis}/>')
        g.append(f'      <geom name="{name}_head{s}" type="box" pos="{L - ro - 0.025:.3f} {s * 0.065:.3f} 0.075" '
                 f'size="0.004 0.018 0.008" material="headlight" {vis}/>')
        g.append(f'      <geom name="{name}_tail{s}" type="box" pos="{-ro + 0.012:.3f} {s * 0.07:.3f} 0.07" '
                 f'size="0.004 0.02 0.007" material="taillight" {vis}/>')
    # Livox Mid-360: 뒤축 앞 0.13 m (scenario lidar.mount_xyz와 같은 x)
    g.append(f'      <geom name="{name}_lidar_base" type="cylinder" pos="0.13 0 0.105" size="0.03 0.012" '
             f'material="carbon" {vis}/>')
    g.append(f'      <geom name="{name}_lidar" type="cylinder" pos="0.13 0 0.135" size="0.033 0.02" '
             f'rgba="0.15 0.16 0.18 1" material="steel" {vis}/>')
    g.append(f'      <geom name="{name}_lidar_ring" type="cylinder" pos="0.13 0 0.137" size="0.0335 0.006" '
             f'material="lidar_ring" {vis}/>')
    return '\n'.join(g)


def build_xml(cfg):
    t, v = cfg['track'], cfg['vehicle']
    W, x0, x1, h = t['W'], t['x_start'], t['x_end'], t['wall_height']
    th = 0.05
    L, wcar, H, ro, wb = v['length'], cfg['w_car'], v['height'], v['rear_overhang'], v['wheelbase']
    sense = f'group="{SENSE_GROUP}" contype="0" conaffinity="0"'

    def wall(name, y):
        return (f'    <geom name="{name}" type="box" pos="{0.5 * (x0 + x1):.3f} {y:.4f} {0.5 * h:.3f}" '
                f'size="{0.5 * (x1 - x0):.3f} {0.5 * th:.3f} {0.5 * h:.3f}" rgba="0.55 0.6 0.7 0.35" {sense}/>')

    def car(name, rgba, shell, accent):
        return (f'    <body name="{name}" mocap="true" pos="0 0 0">\n'
                f'      <geom name="{name}_body" type="box" pos="{0.5 * L - ro:.4f} 0 {0.5 * H:.4f}" '
                f'size="{0.5 * L:.4f} {0.5 * wcar:.4f} {0.5 * H:.4f}" rgba="{rgba}" {sense}/>\n'
                f'{_car_visual(name, L, ro, wb, wcar, shell, accent)}\n    </body>')

    xm = 0.5 * (x0 + x1)
    return f'''<mujoco model="overtake_corridor">
  <visual>
    <global offwidth="1920" offheight="1080"/>
    <quality shadowsize="8192" offsamples="8"/>
    <headlight ambient="0.35 0.35 0.37" diffuse="0.45 0.45 0.45" specular="0.1 0.1 0.1"/>
    <map znear="0.01" zfar="200" haze="0.25"/>
    <rgba haze="0.82 0.87 0.93 1"/>
  </visual>
  <asset>
    <texture name="sky" type="skybox" builtin="gradient" rgb1="0.62 0.76 0.92" rgb2="0.95 0.97 1" width="512" height="512"/>
    <texture name="asphalt" type="2d" builtin="flat" mark="random" random="0.25" rgb1="0.27 0.28 0.30"
             rgb2="0.27 0.28 0.30" markrgb="0.40 0.40 0.42" width="1024" height="1024"/>
    <texture name="ground" type="2d" builtin="flat" mark="random" random="0.5" rgb1="0.44 0.55 0.36" rgb2="0.44 0.55 0.36" markrgb="0.36 0.47 0.30" width="1024" height="1024"/>
    <material name="asphalt" texture="asphalt" texrepeat="40 2" specular="0.05" shininess="0.1"/>
    <material name="ground" texture="ground" texrepeat="30 8" specular="0"/>
    <material name="concrete" specular="0.05" shininess="0.1"/>
    <material name="steel" specular="0.8" shininess="0.9" reflectance="0.05"/>
    <material name="rubber" rgba="0.06 0.06 0.07 1" specular="0.1" shininess="0.2"/>
    <material name="carbon" rgba="0.10 0.10 0.11 1" specular="0.4" shininess="0.6"/>
    <material name="paint" specular="0.9" shininess="0.95" reflectance="0.08"/>
    <material name="reflector" rgba="1 0.75 0.1 1" emission="0.6"/>
    <material name="headlight" rgba="1 0.97 0.85 1" emission="1"/>
    <material name="taillight" rgba="1 0.1 0.1 1" emission="0.9"/>
    <material name="lidar_ring" rgba="0.2 0.85 1 1" emission="0.8"/>
  </asset>
  <worldbody>
    <light name="sun" directional="true" pos="{xm:.1f} -4 8" dir="0.25 0.45 -1" diffuse="0.75 0.75 0.72" specular="0.3 0.3 0.3" castshadow="true"/>
    <light name="fill" directional="true" pos="{xm:.1f} 4 6" dir="-0.2 -0.4 -1" diffuse="0.25 0.25 0.28" castshadow="false"/>
    <geom name="ground" type="plane" pos="{xm:.2f} 0 -0.001" size="{0.5 * (x1 - x0) + 20:.2f} 12 0.1" material="ground" contype="0" conaffinity="0"/>
    <geom name="floor" type="box" pos="{xm:.2f} 0 -0.0005" size="{0.5 * (x1 - x0) + 1:.2f} {0.5 * W + 0.12:.3f} 0.0005" material="asphalt" contype="0" conaffinity="0"/>
{_corridor_visual(W, x0, x1)}
{wall('left_wall', 0.5 * W + 0.5 * th)}
{wall('right_wall', -0.5 * W - 0.5 * th)}
{car('ego', '0.2 0.4 0.9 0.35', '0.08 0.32 0.85 1', '0.95 0.95 0.95 1')}
{car('leader', '0.9 0.2 0.2 0.35', '0.06 0.06 0.07 1', '0.95 0.75 0.1 1')}
  </worldbody>
</mujoco>'''
