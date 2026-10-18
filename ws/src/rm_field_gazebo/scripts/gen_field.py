#!/usr/bin/env python3
"""生成 RMUC 2027 简化战场：Gazebo 世界、纹理材质、规划用场地地图。

坐标系 map(ENU)：原点为战场中心，x 沿长边（28 m）指向蓝方，y 沿宽边（15 m），z 向上。
战场中心对称：蓝方物体 = 红方物体绕 z 轴旋转 180°。
布局尺寸为依据规则手册文字描述的近似值，拿到官方图纸后只需修改 RED_SIDE / SHARED 中的数值并重新运行：
    python3 gen_field.py
"""
import math
import os
import random

import numpy as np
from PIL import Image, ImageDraw

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MODEL_DIR = os.path.join(PKG, 'models', 'rm_field')
WORLD_PATH = os.path.join(PKG, 'worlds', 'rmuc2027.world')
MAP_PATH = os.path.join(PKG, 'config', 'field_map.yaml')

FIELD_L, FIELD_W, WALL_H, WALL_T = 28.0, 15.0, 2.4, 0.1

SMALL_ARMOR = (0.135, 0.125)  # 宽, 高
LARGE_ARMOR = (0.230, 0.127)
LIGHT_BAR = (0.012, 0.056)    # 宽, 高

# 静态障碍物（红方一侧）：name, center(x,y), size(sx,sy,sz)，底面在 z=0
RED_SIDE_BOXES = [
    ('fortress', (-9.0, -3.5), (2.0, 1.6, 0.4)),
    ('trapezoid_highland', (-8.3, 4.6), (3.0, 3.0, 0.2)),
    ('high_point', (-11.6, 5.6), (2.2, 1.8, 0.6)),
    ('supply_wall', (-12.6, -5.2), (2.2, 0.2, 1.2)),
    ('workbench', (-13.2, -6.4), (0.8, 0.6, 0.8)),
    ('dart_station', (-13.40, 3.30), (1.0, 1.2, 1.2)),
    ('road_tunnel', (-5.5, -5.8), (3.2, 1.4, 0.55)),
    ('road_bump_zone', (-3.0, 6.3), (2.5, 1.6, 0.05)),
    ('outpost_pedestal', (-3.10, -3.84), (0.9, 0.9, 0.2)),
    ('base_pedestal', (-11.50, 0.0), (1.6, 1.6, 0.2)),
]

# 中央共享结构（已经是中心对称的，不再镜像）
SHARED_BOXES = [
    ('central_highland', (0.0, 0.0), (5.4, 3.2, 0.6)),
    ('energy_pillar', (0.0, 0.0), (0.3, 0.3, 2.1)),
]

CENTRAL_H = 0.6
# 机库平台（放在中央高地上）+ 机库，红方
RED_HANGAR_PLATFORM = ((-1.9, -1.0), (0.7, 0.5, 0.25))
HANGAR_BOX = (0.32, 0.32, 0.12)

# 基地：中心、主体尺寸、装甲高度、装甲半径
# 中心由 RMUC 2026 网格量出来，装甲贴在基地/前哨站主体上
RED_BASE = dict(center=(-11.60, 0.01), z0=0.2, body=(0.7, 0.7, 1.2), armor_z=1.00, armor_r=0.34)
RED_OUTPOST = dict(center=(-3.09, -3.84), z0=0.2, pillar_h=1.15, hub_z=1.30, armor_r=0.20, spin=0.0)
ENERGY = dict(z=2.0, arm_len=0.70, target_r=0.15, offset=0.22, spin=math.pi / 3.0)


def mirror_xy(x, y):
    return -x, -y


# ---------------------------------------------------------------- 纹理 ---
def gen_textures(tex_dir):
    os.makedirs(tex_dir, exist_ok=True)
    rng = np.random.default_rng(2027)

    def speckle(base, n, size, val_rng, w=1024, h=1024, seed=0):
        img = Image.new('RGB', (w, h), base)
        d = ImageDraw.Draw(img)
        r = random.Random(seed)
        for _ in range(n):
            x, y = r.randrange(w), r.randrange(h)
            s = r.randint(*size)
            v = r.randint(*val_rng)
            c = (v, v, v) if isinstance(base, tuple) and base[0] == base[1] else (
                min(255, base[0] + v - 128), min(255, base[1] + v - 128), min(255, base[2] + v - 128))
            if r.random() < 0.5:
                d.ellipse([x, y, x + s, y + s], fill=c)
            else:
                d.rectangle([x, y, x + s, y + int(s * r.uniform(0.3, 2.0))], fill=c)
        return img

    # 地胶：深灰 + 斑点 + 拼缝线，给 VIO 提供角点
    floor = speckle((70, 72, 75), 9000, (2, 9), (30, 150), seed=1)
    d = ImageDraw.Draw(floor)
    for i in range(0, 1024, 128):
        d.line([(i, 0), (i, 1023)], fill=(110, 110, 110), width=2)
        d.line([(0, i), (1023, i)], fill=(110, 110, 110), width=2)
    noise = rng.normal(0, 6, (1024, 1024, 1))
    floor = Image.fromarray(np.clip(np.asarray(floor, np.float32) + noise, 0, 255).astype(np.uint8))
    floor.save(os.path.join(tex_dir, 'rm_floor.png'))

    # 木质结构：木纹 + 节疤 + 喷绘块
    yy, xx = np.mgrid[0:1024, 0:1024]
    grain = 0.5 + 0.5 * np.sin(xx / 7.0 + 4.0 * np.sin(yy / 90.0) + rng.normal(0, 0.3, (1024, 1024)))
    wood = np.stack([150 + 50 * grain, 110 + 35 * grain, 70 + 20 * grain], -1)
    wood_img = Image.fromarray(np.clip(wood, 0, 255).astype(np.uint8))
    d = ImageDraw.Draw(wood_img)
    r = random.Random(3)
    for _ in range(220):
        x, y, s = r.randrange(1024), r.randrange(1024), r.randint(6, 40)
        c = r.choice([(40, 30, 20), (230, 230, 230), (20, 20, 20), (200, 180, 60)])
        if r.random() < 0.5:
            d.ellipse([x, y, x + s, y + s], fill=c)
        else:
            d.rectangle([x, y, x + s * 2, y + s], fill=c)
    wood_img.save(os.path.join(tex_dir, 'rm_wood.png'))

    # 围挡：黑色钢板 + 广告/横幅块
    wall = speckle((25, 25, 28), 3000, (2, 6), (40, 120), seed=5)
    d = ImageDraw.Draw(wall)
    r = random.Random(7)
    for _ in range(60):
        x, y = r.randrange(1024), r.randrange(1024)
        w, h = r.randint(40, 260), r.randint(20, 120)
        c = tuple(r.randint(60, 255) for _ in range(3))
        d.rectangle([x, y, x + w, y + h], fill=c)
        for _ in range(r.randint(2, 8)):
            tx, ty = x + r.randint(0, max(1, w - 20)), y + r.randint(0, max(1, h - 10))
            d.rectangle([tx, ty, tx + r.randint(5, 25), ty + r.randint(4, 12)], fill=(10, 10, 10))
    wall.save(os.path.join(tex_dir, 'rm_wall.png'))

    # 装甲板中心（模拟点阵屏数字区域）
    armor = Image.new('RGB', (256, 256), (35, 35, 38))
    d = ImageDraw.Draw(armor)
    d.rectangle([96, 48, 160, 208], outline=(200, 200, 200), width=10)
    armor.save(os.path.join(tex_dir, 'rm_armor.png'))


MATERIAL_SCRIPT = """
material RM/Floor
{{
  technique
  {{
    pass
    {{
      ambient 0.9 0.9 0.9 1
      diffuse 1 1 1 1
      specular 0.05 0.05 0.05 1 8
      texture_unit
      {{
        texture rm_floor.png
        filtering anisotropic
        max_anisotropy 16
        scale {fx} {fy}
      }}
    }}
  }}
}}

material RM/Wood
{{
  technique
  {{
    pass
    {{
      ambient 0.9 0.9 0.9 1
      diffuse 1 1 1 1
      texture_unit
      {{
        texture rm_wood.png
        filtering anisotropic
        max_anisotropy 16
        scale 0.5 0.5
      }}
    }}
  }}
}}

material RM/Wall
{{
  technique
  {{
    pass
    {{
      ambient 0.9 0.9 0.9 1
      diffuse 1 1 1 1
      texture_unit
      {{
        texture rm_wall.png
        filtering anisotropic
        max_anisotropy 16
        scale 0.08 1.0
      }}
    }}
  }}
}}

material RM/ArmorPlate
{{
  technique
  {{
    pass
    {{
      ambient 1 1 1 1
      diffuse 1 1 1 1
      texture_unit
      {{
        texture rm_armor.png
      }}
    }}
  }}
}}

material RM/RedLightBar
{{
  technique
  {{
    pass
    {{
      lighting off
      ambient 1 0.15 0.15 1
      diffuse 1 0.15 0.15 1
      emissive 1 0.15 0.15 1
    }}
  }}
}}

material RM/BlueLightBar
{{
  technique
  {{
    pass
    {{
      lighting off
      ambient 0.15 0.45 1 1
      diffuse 0.15 0.45 1 1
      emissive 0.15 0.45 1 1
    }}
  }}
}}

material RM/WhiteLightBar
{{
  technique
  {{
    pass
    {{
      lighting off
      ambient 0.9 0.9 0.9 1
      diffuse 0.9 0.9 0.9 1
      emissive 0.9 0.9 0.9 1
    }}
  }}
}}
"""


def mat(name):
    return ('<material><script><uri>model://rm_field/materials/scripts</uri>'
            '<uri>model://rm_field/materials/textures</uri>'
            f'<name>{name}</name></script></material>')


def pose(x, y, z, roll=0.0, pitch=0.0, yaw=0.0):
    return f'<pose>{x:.4f} {y:.4f} {z:.4f} {roll:.4f} {pitch:.4f} {yaw:.4f}</pose>'


def box_geom(sx, sy, sz):
    return f'<geometry><box><size>{sx:.4f} {sy:.4f} {sz:.4f}</size></box></geometry>'


def static_box(name, cx, cy, cz, sx, sy, sz, material='RM/Wood', yaw=0.0, visual=False):
    vis = f'\n        <visual name="visual">{box_geom(sx, sy, sz)}{mat(material)}</visual>' if visual else ''
    return f"""
    <model name="{name}">
      <static>true</static>
      {pose(cx, cy, cz, 0, 0, yaw)}
      <link name="link">
        <collision name="collision">{box_geom(sx, sy, sz)}</collision>{vis}
      </link>
    </model>"""


def armor_link(name, parent, x, y, z, yaw, size, team, plugin_name, extra_pitch=0.0):
    """装甲板：一个 link（板 + 左右灯条），法向为 link 局部 +x，附带冲击检测传感器。"""
    w, h = size
    bar_mat = 'RM/RedLightBar' if team == 'red' else 'RM/BlueLightBar'
    bw, bh = LIGHT_BAR
    t = 0.01
    return f"""
      <link name="{name}">
        {pose(x, y, z, 0, extra_pitch, yaw)}
        <inertial><mass>0.2</mass><inertia><ixx>1e-4</ixx><iyy>1e-4</iyy><izz>1e-4</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>
        <gravity>false</gravity>
        <collision name="{name}_collision">{box_geom(t, w + 2 * bw, h)}</collision>
        <visual name="plate">{box_geom(t, w, h)}{mat('RM/ArmorPlate')}</visual>
        <visual name="bar_l"><pose>{t / 2 + 0.002:.4f} {w / 2 + bw / 2:.4f} 0 0 0 0</pose>{box_geom(0.004, bw, bh)}{mat(bar_mat)}</visual>
        <visual name="bar_r"><pose>{t / 2 + 0.002:.4f} {-(w / 2 + bw / 2):.4f} 0 0 0 0</pose>{box_geom(0.004, bw, bh)}{mat(bar_mat)}</visual>
        <sensor name="{name}_contact" type="contact">
          <always_on>true</always_on>
          <update_rate>50</update_rate>
          <contact><collision>{name}_collision</collision></contact>
          <plugin name="{plugin_name}" filename="libarmor_impact_plugin.so">
            <armor_name>{name}</armor_name>
            <link_name>{name}</link_name>
            <force_threshold>2.0</force_threshold>
          </plugin>
        </sensor>
      </link>
      <joint name="{name}_joint" type="fixed"><parent>{parent}</parent><child>{name}</child></joint>"""


def base_model(team, cx, cy, yaw0):
    b = RED_BASE
    bx, by, bz = b['body']
    links = []
    for i in range(6):
        a = yaw0 + i * math.pi / 3.0
        ax = cx + b['armor_r'] * math.cos(a)
        ay = cy + b['armor_r'] * math.sin(a)
        links.append(armor_link(f'{team}_base_armor_{i}', 'body', ax, ay, b['armor_z'], a,
                                LARGE_ARMOR, team, f'{team}_base_armor_{i}_impact'))
    return f"""
    <model name="{team}_base">
      <pose>0 0 0 0 0 0</pose>
      <link name="body">
        {pose(cx, cy, b['z0'] + bz / 2)}
        <inertial><mass>50</mass><inertia><ixx>5</ixx><iyy>5</iyy><izz>5</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>
        <gravity>false</gravity>
        <collision name="collision"><geometry><cylinder><radius>{bx / 2 + 0.03:.3f}</radius><length>{bz:.3f}</length></cylinder></geometry></collision>
      </link>
      <joint name="fix_world" type="fixed"><parent>world</parent><child>body</child></joint>
      {''.join(links)}
    </model>"""


def outpost_model(team, cx, cy, yaw0):
    o = RED_OUTPOST
    top = o['z0'] + o['pillar_h']
    links = []
    for i in range(3):
        a = yaw0 + i * 2.0 * math.pi / 3.0
        ax = o['armor_r'] * math.cos(a)
        ay = o['armor_r'] * math.sin(a)
        # 装甲固定在旋转 hub 上，pose 以世界系给出（SDF 1.6 link pose 相对 model）
        links.append(armor_link(f'{team}_outpost_armor_{i}', 'hub', cx + ax, cy + ay, o['hub_z'], a,
                                SMALL_ARMOR, team, f'{team}_outpost_armor_{i}_impact'))
    return f"""
    <model name="{team}_outpost">
      <pose>0 0 0 0 0 0</pose>
      <link name="pillar">
        {pose(cx, cy, top / 2 + o['z0'] / 2)}
        <inertial><mass>20</mass><inertia><ixx>1</ixx><iyy>1</iyy><izz>1</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>
        <gravity>false</gravity>
        <collision name="collision">{box_geom(0.22, 0.22, top - o['z0'])}</collision>
      </link>
      <joint name="fix_world" type="fixed"><parent>world</parent><child>pillar</child></joint>
      <link name="hub">
        {pose(cx, cy, o['hub_z'])}
        <inertial><mass>1.0</mass><inertia><ixx>0.02</ixx><iyy>0.02</iyy><izz>0.02</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>
        <gravity>false</gravity>
        <collision name="collision"><geometry><cylinder><radius>0.16</radius><length>0.22</length></cylinder></geometry></collision>
      </link>
      <joint name="spin" type="fixed">
        <parent>pillar</parent><child>hub</child>
      </joint>
      {''.join(links)}
    </model>"""


def energy_rotor(team, side_sign):
    """能量机关一侧转子：5 灯臂 + 圆靶（仅视觉/碰撞，不做击打检测）。"""
    e = ENERGY
    cx = side_sign * e['offset']
    yaw = 0.0 if side_sign > 0 else math.pi
    bar_mat = 'RM/RedLightBar' if team == 'red' else 'RM/BlueLightBar'
    visuals = []
    for i in range(5):
        a = i * 2 * math.pi / 5
        ly, lz = math.cos(a) * e['arm_len'] / 2, math.sin(a) * e['arm_len'] / 2
        visuals.append(f'<visual name="arm_{i}"><pose>0 {ly:.3f} {lz:.3f} {a:.4f} 0 0</pose>'
                       f'{box_geom(0.02, e["arm_len"], 0.04)}{mat(bar_mat)}</visual>')
        ty, tz = math.cos(a) * e['arm_len'], math.sin(a) * e['arm_len']
        visuals.append(f'<visual name="target_{i}"><pose>0 {ty:.3f} {tz:.3f} 0 1.5708 0</pose>'
                       f'<geometry><cylinder><radius>{e["target_r"]}</radius><length>0.02</length></cylinder></geometry>'
                       f'{mat(bar_mat)}</visual>')
        visuals.append(f'<collision name="target_{i}_col"><pose>0 {ty:.3f} {tz:.3f} 0 1.5708 0</pose>'
                       f'<geometry><cylinder><radius>{e["target_r"]}</radius><length>0.02</length></cylinder></geometry></collision>')
    return f"""
      <link name="{team}_rotor">
        {pose(cx, 0, e['z'], 0, 0, yaw)}
        <inertial><mass>2.0</mass><inertia><ixx>0.3</ixx><iyy>0.3</iyy><izz>0.3</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>
        <gravity>false</gravity>
        <visual name="hub"><pose>0 0 0 0 1.5708 0</pose><geometry><cylinder><radius>0.12</radius><length>0.05</length></cylinder></geometry>{mat(bar_mat)}</visual>
        {''.join(visuals)}
      </link>
      <joint name="{team}_rotor_spin" type="fixed">
        <parent>mount</parent><child>{team}_rotor</child>
      </joint>"""


def energy_model():
    e = ENERGY
    return f"""
    <model name="energy_mechanism">
      <pose>0 0 0 0 0 0</pose>
      <link name="mount">
        {pose(0, 0, e['z'])}
        <inertial><mass>10</mass><inertia><ixx>1</ixx><iyy>1</iyy><izz>1</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>
        <gravity>false</gravity>
        <collision name="collision">{box_geom(0.36, 0.36, 0.36)}</collision>
      </link>
      <joint name="fix_world" type="fixed"><parent>world</parent><child>mount</child></joint>
      {energy_rotor('blue', 1)}
      {energy_rotor('red', -1)}
    </model>"""


def world_sdf(boxes):
    parts = ["""
    <include>
      <uri>model://rmuc_2026</uri>
      <name>rmuc_2026_visual</name>
      <pose>0 0 0 0 0 0</pose>
    </include>"""]
    # 地面：只保留碰撞，外观用 RMUC 2026 网格
    parts.append(f"""
    <model name="field_floor">
      <static>true</static>
      <pose>0 0 -0.05 0 0 0</pose>
      <link name="link">
        <collision name="collision">{box_geom(FIELD_L + 2, FIELD_W + 2, 0.1)}
          <surface><friction><ode><mu>0.8</mu><mu2>0.8</mu2></ode></friction></surface></collision>
        <visual name="texture">
          <pose>0 0 0.08 0 0 0</pose>
          {box_geom(FIELD_L, FIELD_W, 0.01)}{mat('RM/Floor')}
        </visual>
      </link>
    </model>""")
    # 围挡
    walls = [
        ('wall_xp', FIELD_L / 2 + WALL_T / 2, 0, WALL_T, FIELD_W + 2 * WALL_T),
        ('wall_xn', -FIELD_L / 2 - WALL_T / 2, 0, WALL_T, FIELD_W + 2 * WALL_T),
        ('wall_yp', 0, FIELD_W / 2 + WALL_T / 2, FIELD_L, WALL_T),
        ('wall_yn', 0, -FIELD_W / 2 - WALL_T / 2, FIELD_L, WALL_T),
    ]
    for n, x, y, sx, sy in walls:
        parts.append(static_box(n, x, y, WALL_H / 2, sx, sy, WALL_H, 'RM/Wall'))
    for b in boxes:
        parts.append(static_box(b['name'], b['center'][0], b['center'][1], b['z0'] + b['size'][2] / 2,
                                b['size'][0], b['size'][1], b['size'][2]))
    # 目标
    rb = RED_BASE['center']
    parts.append(base_model('red', rb[0], rb[1], 0.0))
    bbx, bby = mirror_xy(*rb)
    parts.append(base_model('blue', bbx, bby, math.pi))
    ro = RED_OUTPOST['center']
    parts.append(outpost_model('red', ro[0], ro[1], 0.0))
    box_, boy = mirror_xy(*ro)
    parts.append(outpost_model('blue', box_, boy, math.pi))
    # 2026 网格里已经有能量机关，不再叠加一套白色圆靶

    lights = []
    for i, (lx, ly) in enumerate([(-9, -4), (-9, 4), (0, 0), (9, -4), (9, 4)]):
        lights.append(f"""
    <light name="lamp_{i}" type="point">
      <pose>{lx} {ly} 6 0 0 0</pose>
      <diffuse>0.25 0.25 0.25 1</diffuse><specular>0.02 0.02 0.02 1</specular>
      <attenuation><range>25</range><constant>0.6</constant><linear>0.02</linear><quadratic>0.0</quadratic></attenuation>
      <cast_shadows>false</cast_shadows>
    </light>""")

    return f"""<?xml version="1.0"?>
<!-- 由 scripts/gen_field.py 自动生成，请勿手动修改 -->
<sdf version="1.6">
  <world name="rmuc2027">
    <physics name="default" type="ode">
      <max_step_size>0.002</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>500</real_time_update_rate>
      <ode><solver><type>quick</type><iters>50</iters></solver></ode>
    </physics>
    <gravity>0 0 -9.81</gravity>
    <scene>
      <ambient>0.35 0.35 0.35 1</ambient>
      <background>0.15 0.15 0.18 1</background>
      <shadows>false</shadows>
      <grid>false</grid>
    </scene>
    <light name="sun" type="directional">
      <pose>0 0 20 0 0 0</pose>
      <diffuse>0.35 0.35 0.35 1</diffuse><specular>0.02 0.02 0.02 1</specular>
      <direction>-0.3 0.2 -1.0</direction>
      <cast_shadows>false</cast_shadows>
    </light>
    {''.join(lights)}
    {''.join(parts)}
    <gui fullscreen="0">
      <camera name="user_camera">
        <pose>-5.2 -4.6 2.4 0 0.55 0.72</pose>
      </camera>
    </gui>
  </world>
</sdf>
"""


def collect_boxes():
    boxes = []
    for n, (x, y), s in RED_SIDE_BOXES:
        boxes.append(dict(name=f'red_{n}', center=(x, y), size=s, z0=0.0))
        mx, my = mirror_xy(x, y)
        boxes.append(dict(name=f'blue_{n}', center=(mx, my), size=s, z0=0.0))
    for n, c, s in SHARED_BOXES:
        z0 = CENTRAL_H if n == 'energy_pillar' else 0.0
        boxes.append(dict(name=n, center=c, size=(s[0], s[1], s[2] - z0), z0=z0))
    (px, py), ps = RED_HANGAR_PLATFORM
    for team, (x, y) in (('red', (px, py)), ('blue', mirror_xy(px, py))):
        boxes.append(dict(name=f'{team}_hangar_platform', center=(x, y), size=ps, z0=CENTRAL_H))
        boxes.append(dict(name=f'{team}_hangar', center=(x, y), size=HANGAR_BOX, z0=CENTRAL_H + ps[2]))
    return boxes


def hangar_pose(team):
    (px, py), ps = RED_HANGAR_PLATFORM
    x, y = (px, py) if team == 'red' else mirror_xy(px, py)
    z = CENTRAL_H + ps[2] + HANGAR_BOX[2]
    yaw = 0.0 if team == 'red' else math.pi
    return x, y, z, yaw


def write_map(boxes):
    lines = ['# 由 scripts/gen_field.py 自动生成：规划器/任务/裁判模拟共用的场地地图', 'field:',
             f'  length: {FIELD_L}', f'  width: {FIELD_W}', f'  wall_height: {WALL_H}', 'obstacles:']
    for b in boxes:
        cz = b['z0'] + b['size'][2] / 2
        lines.append(f"  - {{name: {b['name']}, center: [{b['center'][0]:.3f}, {b['center'][1]:.3f}, {cz:.3f}], "
                     f"size: [{b['size'][0]:.3f}, {b['size'][1]:.3f}, {b['size'][2]:.3f}]}}")
    # 基地/前哨站本体、能量机关也作为障碍物
    rb, ro = RED_BASE['center'], RED_OUTPOST['center']
    bz = RED_BASE['z0'] + RED_BASE['body'][2]
    for team, c in (('red', rb), ('blue', mirror_xy(*rb))):
        lines.append(f"  - {{name: {team}_base_body, center: [{c[0]:.3f}, {c[1]:.3f}, {bz / 2:.3f}], size: [1.0, 1.0, {bz:.3f}]}}")
    oz = RED_OUTPOST['hub_z'] + 0.4
    for team, c in (('red', ro), ('blue', mirror_xy(*ro))):
        lines.append(f"  - {{name: {team}_outpost_body, center: [{c[0]:.3f}, {c[1]:.3f}, {oz / 2:.3f}], size: [0.4, 0.4, {oz:.3f}]}}")
    e = ENERGY
    lines.append(f"  - {{name: energy_rotors, center: [0.0, 0.0, {e['z']:.3f}], size: [0.7, {2 * e['arm_len'] + 0.4:.3f}, {2 * e['arm_len'] + 0.4:.3f}]}}")

    lines.append('targets:')
    for team, sign in (('red', 1), ('blue', -1)):
        c = rb if team == 'red' else mirror_xy(*rb)
        yaw0 = 0.0 if team == 'red' else math.pi
        for i in range(6):
            a = yaw0 + i * math.pi / 3
            x = c[0] + RED_BASE['armor_r'] * math.cos(a)
            y = c[1] + RED_BASE['armor_r'] * math.sin(a)
            lines.append(f"  - {{name: {team}_base_armor_{i}, team: {team}, kind: base, armor_type: large, "
                         f"position: [{x:.3f}, {y:.3f}, {RED_BASE['armor_z']:.3f}], normal_yaw: {math.atan2(math.sin(a), math.cos(a)):.4f}}}")
        oc = ro if team == 'red' else mirror_xy(*ro)
        lines.append(f"  - {{name: {team}_outpost, team: {team}, kind: outpost, armor_type: small, "
                     f"position: [{oc[0]:.3f}, {oc[1]:.3f}, {RED_OUTPOST['hub_z']:.3f}], radius: {RED_OUTPOST['armor_r']}, "
                     f"spin_speed: {RED_OUTPOST['spin']:.4f}}}")
    lines.append('hangars:')
    for team in ('red', 'blue'):
        x, y, z, yaw = hangar_pose(team)
        lines.append(f'  {team}: {{position: [{x:.3f}, {y:.3f}, {z:.3f}], yaw: {yaw:.4f}}}')
    with open(MAP_PATH, 'w') as f:
        f.write('\n'.join(lines) + '\n')


def main():
    os.makedirs(os.path.join(MODEL_DIR, 'materials', 'scripts'), exist_ok=True)
    os.makedirs(os.path.dirname(WORLD_PATH), exist_ok=True)
    os.makedirs(os.path.dirname(MAP_PATH), exist_ok=True)
    gen_textures(os.path.join(MODEL_DIR, 'materials', 'textures'))
    with open(os.path.join(MODEL_DIR, 'materials', 'scripts', 'rm_field.material'), 'w') as f:
        f.write(MATERIAL_SCRIPT.format(fx=1.0 / 14.0, fy=1.0 / 7.5))
    with open(os.path.join(MODEL_DIR, 'model.config'), 'w') as f:
        f.write('<?xml version="1.0"?>\n<model><name>rm_field</name><version>1.0</version>'
                '<sdf version="1.6">model.sdf</sdf><description>RMUC2027 材质资源</description></model>\n')
    with open(os.path.join(MODEL_DIR, 'model.sdf'), 'w') as f:
        f.write('<?xml version="1.0"?>\n<sdf version="1.6"><model name="rm_field_assets"><static>true</static>'
                '<link name="l"/></model></sdf>\n')
    boxes = collect_boxes()
    with open(WORLD_PATH, 'w') as f:
        f.write(world_sdf(boxes))
    write_map(boxes)
    print('world :', WORLD_PATH)
    print('map   :', MAP_PATH)
    for t in ('red', 'blue'):
        print(f'{t} hangar pose (x y z yaw):', ['%.3f' % v for v in hangar_pose(t)])


if __name__ == '__main__':
    main()
