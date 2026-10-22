#!/usr/bin/env python3
"""生成 RMUC 2027 简化战场：Gazebo 世界、纹理材质、规划用场地地图。

坐标系 map(ENU)：原点为战场中心，x 沿长边（28 m）指向蓝方，y 沿宽边（15 m），z 向上。
战场中心对称：蓝方物体 = 红方物体绕 z 轴旋转 180°。
布局尺寸为依据规则手册文字描述的近似值，拿到官方图纸后只需修改 RED_SIDE / SHARED 中的数值并重新运行：
    python3 gen_field.py
"""
import argparse
import math
import os
import random

import numpy as np
from PIL import Image, ImageDraw

from asset_meshes import build_dart_assets, build_energy_assets
from field_appearance import SURFACES, surface_materials, surface_textures, write_surface_meshes

PKG = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
if not os.path.isdir(os.path.join(PKG, 'models')):
    # 普通安装时脚本位于 lib/<pkg>，资源位于 share/<pkg>；symlink 安装则直接解析源码位置。
    from ament_index_python.packages import get_package_share_directory
    PKG = get_package_share_directory('rm_field_gazebo')
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
]

CENTRAL_H = 0.6
# 机库平台（放在中央高地上）+ 机库，红方
RED_HANGAR_PLATFORM = ((-1.9, -1.0), (0.7, 0.5, 0.25))
HANGAR_BOX = (0.32, 0.32, 0.12)

# 基地碰撞圆柱停在装甲下沿以下。三块大装甲在 z≈1.26–1.32，圆柱顶取 1.18 m。
RED_BASE = dict(center=(-11.595, 0.004), z0=0.2, body=(0.7, 0.7, 0.98), body_r=0.16)
# 前哨站固定，不旋转。hub 只挡柱心，半径小于三块板里最近的一块。
RED_OUTPOST = dict(center=(-3.090, -3.840), z0=0.2, pillar_h=1.15, hub_z=1.65, hub_r=0.10, spin=0.0)

# 网格外表面中心（世界坐标）。rmuc_2026 link 已有 z=-0.23、scale 1、无旋转，这里不再减 0.23。
# 每项：x, y, z, 朝外法向水平角(度), 法向仰角(度，正值朝上)。
# 基地网格上与大装甲尺寸吻合的外表面只有三块：0.227×0.128 m，法向仰角 +62.5°，
# 间隔 120°，高度相同，但水平半径约 0.13 m，不是六等分圆周。
RED_BASE_FACES = [
    (-11.4623, 0.0013, 1.2866, 0.0, 62.50),
    (-11.6582, 0.1143, 1.2866, 120.0, 62.50),
    (-11.6581, -0.1119, 1.2866, -120.0, 62.50),
]
# 前哨站三块小装甲外表面，尺寸 0.135×0.125 m，法向俯角 15°。高度和半径都不同。
RED_OUTPOST_FACES = [
    (-3.3639, -3.8559, 1.6130, 180.0, -15.0),
    (-2.9494, -3.6166, 1.7130, 60.0, -15.0),
    (-2.9493, -4.0952, 1.8131, -60.0, -15.0),
]
PLATE_T = 0.012
# 内侧面比网格外表面再向外 2 mm，避免和网格共面闪烁。碰撞仍从这块板开始。
MESH_CLEARANCE = 0.002
# 灯条盒子中心相对装甲 link 原点，沿朝外 +X。与 armor_link() 里的局部 pose 相同。
BAR_CENTER_X = PLATE_T / 2.0 + 0.002
BAR_DEPTH = 0.004
# PnP 的四点位于灯条朝外可见面，地图目标也使用这个平面的中心。
BAR_SURFACE_X = BAR_CENTER_X + BAR_DEPTH / 2.0
ARMOR_PATTERNS = {
    'base': ('RM/BaseArmorFace', 'rm_armor_base.png', LARGE_ARMOR),
    'outpost': ('RM/OutpostArmorFace', 'rm_armor_outpost.png', SMALL_ARMOR),
}
# 训练类别按颜色和用途固定，不依赖模型生成/目标遍历顺序；每块板的 name 是实例 ID。
ARMOR_CLASSES = {('red', 'base'): 0, ('blue', 'base'): 1,
                 ('red', 'outpost'): 2, ('blue', 'outpost'): 3}
# 规则图4-31/32：轴心距300mm台面2300mm，靶心回转直径1400mm，外径308mm。
# 45°朝向适配当前场地外观；红蓝正面共用同一物理转轴。
ENERGY = dict(z=2.60, arm_len=0.70, target_r=0.154, offset=0.50, yaw=-math.pi / 4, spin=0.0)
DART = dict(z=1.48, travel=0.280, width=0.242, height=0.239, depth=0.18)
DETAILED_ENERGY = {}


def mirror_xy(x, y):
    return -x, -y


def outward_normal(yaw_deg, pitch_deg):
    y, p = math.radians(yaw_deg), math.radians(pitch_deg)
    return (math.cos(y) * math.cos(p), math.sin(y) * math.cos(p), math.sin(p))


def mirror_face(face):
    """绕场地中心转 180°。高度和法向仰角不变，水平法向一起转。"""
    x, y, z, yaw, pitch = face
    return (-x, -y, z, yaw + 180.0, pitch)


def team_faces(faces, team):
    return list(faces) if team == 'red' else [mirror_face(f) for f in faces]


def armor_placement(face):
    """安装面中心、板厚中点、灯条可见面中心，由同一个刚体变换得到。"""
    x, y, z, yaw_deg, pitch_deg = face
    n = outward_normal(yaw_deg, pitch_deg)
    link = tuple(c + ni * (PLATE_T / 2.0 + MESH_CLEARANCE) for c, ni in zip((x, y, z), n))
    target = tuple(c + ni * BAR_SURFACE_X for c, ni in zip(link, n))
    return {
        'link': link,
        'target': target,
        'yaw': math.radians(yaw_deg),
        # SDF 先绕 Y 再绕 Z。正的 SDF pitch 会让局部 +X 得到负的 z。
        'sdf_pitch': -math.radians(pitch_deg),
        'normal_yaw': math.atan2(n[1], n[0]),
        'normal': n,
    }


def armor_keypoints(face, size):
    """灯条端点，顺序与 ArmorDetection.corners/PnP 一致：左上、左下、右下、右上。"""
    place = armor_placement(face)
    yaw, pitch = place['yaw'], place['sdf_pitch']
    cy, sy, cp, sp = math.cos(yaw), math.sin(yaw), math.cos(pitch), math.sin(pitch)
    hw, hh = (size[0] + LIGHT_BAR[0]) / 2, LIGHT_BAR[1] / 2
    points = []
    # 从 +X 一侧看向板面，+Y 在图像右侧，+Z 在图像上方。
    for ly, lz in ((-hw, hh), (-hw, -hh), (hw, -hh), (hw, hh)):
        x1 = cp * BAR_SURFACE_X + sp * lz
        points.append((place['link'][0] + cy * x1 - sy * ly,
                       place['link'][1] + sy * x1 + cy * ly,
                       place['link'][2] - sp * BAR_SURFACE_X + cp * lz))
    return points


def write_armor_assets():
    """共用正面网格有明确 UV；贴纸是规范附录原图，不在生成时重画或拉伸。"""
    texture_dir = os.path.join(MODEL_DIR, 'materials', 'textures')
    for _, filename, _ in ARMOR_PATTERNS.values():
        path = os.path.join(texture_dir, filename)
        if not os.path.isfile(path):
            raise FileNotFoundError(f'缺少装甲贴纸资源：{path}')
        with Image.open(path) as image:
            image.verify()
    mesh_dir = os.path.join(MODEL_DIR, 'meshes')
    os.makedirs(mesh_dir, exist_ok=True)
    # TL, BL, BR, TR；两三角形朝 +X，正面图像右向为 +Y，上向为 +Z。
    mesh = '''<?xml version="1.0" encoding="utf-8"?>
<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1">
  <asset><unit name="meter" meter="1"/><up_axis>Z_UP</up_axis></asset>
  <library_geometries><geometry id="face"><mesh>
    <source id="positions"><float_array id="positions-array" count="12">0 -0.5 0.5 0 -0.5 -0.5 0 0.5 -0.5 0 0.5 0.5</float_array>
      <technique_common><accessor source="#positions-array" count="4" stride="3"><param name="X" type="float"/><param name="Y" type="float"/><param name="Z" type="float"/></accessor></technique_common></source>
    <source id="normals"><float_array id="normals-array" count="3">1 0 0</float_array>
      <technique_common><accessor source="#normals-array" count="1" stride="3"><param name="X" type="float"/><param name="Y" type="float"/><param name="Z" type="float"/></accessor></technique_common></source>
    <source id="uv"><float_array id="uv-array" count="8">0 1 0 0 1 0 1 1</float_array>
      <technique_common><accessor source="#uv-array" count="4" stride="2"><param name="S" type="float"/><param name="T" type="float"/></accessor></technique_common></source>
    <vertices id="vertices"><input semantic="POSITION" source="#positions"/></vertices>
    <triangles count="2"><input semantic="VERTEX" source="#vertices" offset="0"/><input semantic="NORMAL" source="#normals" offset="1"/><input semantic="TEXCOORD" source="#uv" offset="2" set="0"/>
      <p>0 0 0 1 0 1 2 0 2 0 0 0 2 0 2 3 0 3</p></triangles>
  </mesh></geometry></library_geometries>
  <library_visual_scenes><visual_scene id="Scene"><node id="face-node"><instance_geometry url="#face"/></node></visual_scene></library_visual_scenes>
  <scene><instance_visual_scene url="#Scene"/></scene>
</COLLADA>
'''
    with open(os.path.join(mesh_dir, 'armor_face.dae'), 'w') as f:
        f.write(mesh)


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

    # 装甲图案为独立的规范贴纸资源，不能被背景纹理生成覆盖。


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

material RM/ArmorBody
{{
  technique
  {{
    pass
    {{
      ambient 0.05 0.05 0.06 1
      diffuse 0.05 0.05 0.06 1
    }}
  }}
}}

material RM/BaseArmorFace
{{
  technique
  {{
    pass
    {{
      ambient 1 1 1 1
      diffuse 0.6 0.6 0.6 1
      texture_unit
      {{
        texture rm_armor_base.png
        tex_address_mode clamp
      }}
    }}
  }}
}}

material RM/OutpostArmorFace
{{
  technique
  {{
    pass
    {{
      ambient 1 1 1 1
      diffuse 0.6 0.6 0.6 1
      texture_unit
      {{
        texture rm_armor_outpost.png
        tex_address_mode clamp
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
      lighting on
      ambient 0 0 0 1
      diffuse 0 0 0 1
      emissive 0.85 0.01 0.01 1
    }}
  }}
}}

material RM/BlueLightBar
{{
  technique
  {{
    pass
    {{
      lighting on
      ambient 0 0 0 1
      diffuse 0 0 0 1
      emissive 0.01 0.03 0.85 1
    }}
  }}
}}

material RM/WhiteLightBar
{{
  technique
  {{
    pass
    {{
      lighting on
      ambient 0 0 0 1
      diffuse 0 0 0 1
      emissive 0.9 0.9 0.9 1
    }}
  }}
}}
"""

MATERIAL_SCRIPT += """
material RM/GreenGuide
{{ technique {{ pass {{ lighting on
      ambient .01 .06 .01 1
      diffuse .02 .15 .02 1
      specular .35 .5 .35 1 32
      emissive .003 .65 .003 1 }} }} }}

material RM/RingMark
{{ technique {{ pass {{ lighting on
      ambient 0.12 0.12 0.12 1
      diffuse 0.12 0.12 0.12 1 }} }} }}

material RM/RedPaint
{{ technique {{ pass {{ lighting on
      ambient 0.65 0.025 0.025 1
      diffuse 0.65 0.025 0.025 1 }} }} }}

material RM/BluePaint
{{ technique {{ pass {{ lighting on
      ambient 0.025 0.1 0.65 1
      diffuse 0.025 0.1 0.65 1 }} }} }}

material RM/WhitePaint
{{ technique {{ pass {{ lighting on
      ambient 0.85 0.85 0.85 1
      diffuse 0.85 0.85 0.85 1 }} }} }}

"""

MATERIAL_SCRIPT += """
material RM/MechanismShell {{ technique {{ pass {{ ambient 0.11 0.115 0.12 1
 diffuse 0.11 0.115 0.12 1
 specular .08 .08 .08 1 16 }} }} }}
material RM/MechanismEdge {{ technique {{ pass {{ ambient 0.19 0.2 0.21 1
 diffuse 0.19 0.2 0.21 1
 specular .08 .08 .08 1 16 }} }} }}
material RM/Rubber {{ technique {{ pass {{ ambient 0.025 0.027 0.03 1
 diffuse 0.025 0.027 0.03 1
 specular .08 .08 .08 1 16 }} }} }}
material RM/Screw {{ technique {{ pass {{ ambient 0.28 0.29 0.3 1
 diffuse 0.28 0.29 0.3 1
 specular .08 .08 .08 1 16 }} }} }}
material RM/Plinth {{ technique {{ pass {{ ambient .35 .35 .36 1
 diffuse .46 .46 .48 1 }} }} }}
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


def armor_link(name, parent, x, y, z, yaw, size, team, plugin_name, extra_pitch=0.0, kind='base'):
    """装甲板：一个 link（板 + 左右灯条），法向为 link 局部 +x，附带冲击检测传感器。"""
    w, h = size
    bar_mat = 'RM/RedLightBar' if team == 'red' else 'RM/BlueLightBar'
    bw, bh = LIGHT_BAR
    t = PLATE_T
    face_material, face_texture, expected_size = ARMOR_PATTERNS[kind]
    if team not in ('red', 'blue') or size != expected_size:
        raise ValueError(f'装甲颜色/几何与类型不一致：{name}, {team}, {kind}, {size}')
    with Image.open(os.path.join(MODEL_DIR, 'materials', 'textures', face_texture)) as icon:
        aspect = icon.width / icon.height
    face_h = min(0.96 * h, 0.96 * w / aspect)
    face_w = face_h * aspect
    return f"""
      <link name="{name}">
        {pose(x, y, z, 0, extra_pitch, yaw)}
        <inertial><mass>0.2</mass><inertia><ixx>1e-4</ixx><iyy>1e-4</iyy><izz>1e-4</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>
        <gravity>false</gravity>
        <collision name="{name}_collision">{box_geom(t, w + 2 * bw, h)}</collision>
        <visual name="plate">{box_geom(t, w, h)}{mat('RM/ArmorBody')}</visual>
        <visual name="face">
          <pose>{t / 2 + 0.0005:.4f} 0 0 0 0 0</pose>
          <geometry><mesh><uri>model://rm_field/meshes/armor_face.dae</uri><scale>1 {face_w:.6f} {face_h:.6f}</scale></mesh></geometry>
          {mat(face_material)}
        </visual>
        <visual name="bar_l"><pose>{BAR_CENTER_X:.4f} {-(w + bw) / 2:.4f} 0 0 0 0</pose>{box_geom(BAR_DEPTH, bw, bh)}{mat(bar_mat)}</visual>
        <visual name="bar_r"><pose>{BAR_CENTER_X:.4f} {(w + bw) / 2:.4f} 0 0 0 0</pose>{box_geom(BAR_DEPTH, bw, bh)}{mat(bar_mat)}</visual>
        <sensor name="{name}_contact" type="contact">
          <always_on>true</always_on>
          <update_rate>500</update_rate>
          <contact><collision>{name}_collision</collision></contact>
          <plugin name="{plugin_name}" filename="libarmor_impact_plugin.so">
            <armor_name>{name}</armor_name>
            <link_name>{name}</link_name>
            <striker_entity>rm_drone</striker_entity>
            <!-- 仿真假设，不是该版规则的官方冲击阈值。 -->
            <force_threshold>2.0</force_threshold>
          </plugin>
        </sensor>
      </link>
      <joint name="{name}_joint" type="fixed"><parent>{parent}</parent><child>{name}</child></joint>"""


def base_model(team, cx, cy, yaw0):
    b = RED_BASE
    bx, by, bz = b['body']
    links = []
    for i, face in enumerate(team_faces(RED_BASE_FACES, team)):
        place = armor_placement(face)
        lx, ly, lz = place['link']
        links.append(armor_link(f'{team}_base_armor_{i}', 'body', lx, ly, lz, place['yaw'],
                                LARGE_ARMOR, team, f'{team}_base_armor_{i}_impact', place['sdf_pitch'], kind='base'))
    return f"""
    <model name="{team}_base">
      <pose>0 0 0 0 0 0</pose>
      <link name="body">
        {pose(cx, cy, b['z0'] + bz / 2)}
        <inertial><mass>50</mass><inertia><ixx>5</ixx><iyy>5</iyy><izz>5</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>
        <gravity>false</gravity>
        <collision name="collision"><geometry><cylinder><radius>{b['body_r']:.3f}</radius><length>{bz:.3f}</length></cylinder></geometry></collision>
      </link>
      <joint name="fix_world" type="fixed"><parent>world</parent><child>body</child></joint>
      {''.join(links)}
    </model>"""


def outpost_model(team, cx, cy, yaw0):
    o = RED_OUTPOST
    top = o['z0'] + o['pillar_h']
    links = []
    for i, face in enumerate(team_faces(RED_OUTPOST_FACES, team)):
        place = armor_placement(face)
        lx, ly, lz = place['link']
        links.append(armor_link(f'{team}_outpost_armor_{i}', 'hub', lx, ly, lz, place['yaw'],
                                SMALL_ARMOR, team, f'{team}_outpost_armor_{i}_impact', place['sdf_pitch'], kind='outpost'))
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
        <collision name="collision"><geometry><cylinder><radius>{o['hub_r']:.3f}</radius><length>0.22</length></cylinder></geometry></collision>
      </link>
      <joint name="hub_fixed" type="fixed"><parent>pillar</parent><child>hub</child></joint>
      {''.join(links)}
    </model>"""


def cylinder(radius, length):
    return f'<geometry><cylinder><radius>{radius:.4f}</radius><length>{length:.4f}</length></cylinder></geometry>'


def inertia(mass=1.0):
    return (f'<inertial><mass>{mass}</mass><inertia><ixx>0.1</ixx><iyy>0.1</iyy><izz>0.1</izz>'
            '<ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial><gravity>false</gravity>'
            '<kinematic>true</kinematic>')


def visual(name, geometry, material, xyz=(0, 0, 0), rpy=(0, 0, 0)):
    return f'<visual name="{name}">{pose(*xyz, *rpy)}{geometry}{mat(material)}</visual>'


def ring_mesh():
    """能量机关300mm靶面之外的308mm外缘；环网格法向+X。"""
    vertices, faces = [], []
    for i in range(64):
        a = i * 2 * math.pi / 64
        for r in (0.150, 0.154):
            vertices.append((0, r * math.cos(a), r * math.sin(a)))
    for i in range(64):
        a, b = 2 * i, 2 * ((i + 1) % 64)
        faces.extend(((a, a + 1, b + 1), (a, b + 1, b)))
    # STL可直接使用，不依赖外部建模工具。
    write_stl(os.path.join(MODEL_DIR, 'meshes', 'energy_ring.stl'), vertices, faces)


def write_stl(path, vertices, faces):
    dtype = np.dtype([('normal', '<f4', 3), ('vertices', '<f4', (3, 3)), ('attribute', '<u2')])
    triangles = np.zeros(len(faces), dtype=dtype)
    triangles['vertices'] = np.asarray(vertices)[np.asarray(faces)]
    v = triangles['vertices']
    normals = np.cross(v[:, 1] - v[:, 0], v[:, 2] - v[:, 0])
    triangles['normal'] = normals / np.maximum(np.linalg.norm(normals, axis=1, keepdims=True), 1e-12)
    with open(path, 'wb') as f:
        f.write(b'RM27 generated geometry'.ljust(80, b' '))
        f.write(np.asarray([len(triangles)], dtype='<u4').tobytes())
        f.write(triangles.tobytes())


def prepare_scene_mesh():
    """保留上游网格，按完整连通部件移除旧机关和基地顶靶，避免叠影/残留半片三角形。"""
    source = os.path.join(PKG, 'models', 'rmuc_2026', 'meshes', 'rmuc_2026.stl')
    dtype = np.dtype([('normal', '<f4', 3), ('vertices', '<f4', (3, 3)), ('attribute', '<u2')])
    with open(source, 'rb') as f:
        header = f.read(80)
        count = int(np.frombuffer(f.read(4), dtype='<u4')[0])
        triangles = np.fromfile(f, dtype=dtype, count=count)
    if len(triangles) != count:
        raise ValueError('场地STL损坏')
    verts, inverse = np.unique(triangles['vertices'].reshape(-1, 3), axis=0, return_inverse=True)
    indices = inverse.reshape(-1, 3)
    parents = list(range(len(verts)))
    def root(i):
        while parents[i] != i:
            parents[i] = parents[parents[i]]
            i = parents[i]
        return i
    for a, b, c in indices:
        ra, rb, rc = root(a), root(b), root(c)
        parents[rb] = parents[rc] = ra
    groups = np.asarray([root(i) for i in range(len(verts))])
    lo = np.full((len(verts), 3), np.inf)
    hi = np.full((len(verts), 3), -np.inf)
    world = verts.copy()
    world[:, 2] -= 0.23
    np.minimum.at(lo, groups, world)
    np.maximum.at(hi, groups, world)
    central = (lo[:, 0] > -1.7) & (hi[:, 0] < 1.7) & (lo[:, 1] > -1.7) & (hi[:, 1] < 1.7) & (hi[:, 2] > 1.0)
    # 低处机库平台等静态细节保留；只替换上方旧机关/支撑结构。
    central &= hi[:, 2] > 1.85
    top_targets = np.zeros(len(verts), dtype=bool)
    for team in ('red', 'blue'):
        cx, cy = RED_BASE['center'] if team == 'red' else mirror_xy(*RED_BASE['center'])
        top_targets |= ((lo[:, 0] > cx - 0.55) & (hi[:, 0] < cx + 0.55) &
                        (lo[:, 1] > cy - 0.55) & (hi[:, 1] < cy + 0.55) & (lo[:, 2] > 1.37))
    keep = ~(central | top_targets)[groups[indices[:, 0]]]
    if not 1000 < np.count_nonzero(~keep) < len(triangles) / 3:
        raise ValueError('静态部件替换范围异常，检查上游网格版本')
    write_surface_meshes(MODEL_DIR, triangles[keep], header, groups[indices[keep, 0]], lo, hi)
    print(f'static mesh: replaced {np.count_nonzero(~keep)} triangles; original mesh preserved')


def mesh_geom(filename):
    return f'<geometry><mesh><uri>model://rm_field/meshes/{filename}.stl</uri></mesh></geometry>'


def energy_model():
    """复用POWER.glb详细几何；静止展示，保留原灯效控制接口。"""
    e = ENERGY
    parts = [visual('housing', mesh_geom('energy_body'), 'RM/MechanismShell')]
    collisions = []
    for team, sign in (('red', -1), ('blue', 1)):
        material = 'RM/RedLightBar' if team == 'red' else 'RM/BlueLightBar'
        for i, center in enumerate(DETAILED_ENERGY['centers'][team]):
            parts.append(visual(f'{team}_arm_{i}', mesh_geom(f'energy_{team}_arm_{i}'), material))
            parts.append(visual(f'{team}_rim_{i}', mesh_geom(f'energy_{team}_rim_{i}'), material))
            # 与源模型圆靶中心一致，不再按简化五角星猜测靶面位置。
            collisions.append(f'<collision name="{team}_target_{i}">{pose(*center, 0, math.pi / 2)}{cylinder(.154,.024)}</collision>')
            for ring in range(1, 10):
                x, y, z = center
                parts.append(visual(f'{team}_ring_{i}_{ring}',
                                    f'<geometry><mesh><uri>model://rm_field/meshes/energy_ring.stl</uri>'
                                    f'<scale>1 {ring / 10:.3f} {ring / 10:.3f}</scale></mesh></geometry>',
                                    'RM/RingMark', (x + sign * .0004, y, z), (0, 0, math.pi if sign < 0 else 0)))
        parts.append(visual(f'{team}_center', mesh_geom(f'energy_{team}_r'), material))
    return f"""
    <model name="energy_mechanism">
      {pose(0, 0, 0, 0, 0, e['yaw'])}
      <link name="mount">{inertia(50)}
        {visual('detailed_mount', mesh_geom('energy_mount'), 'RM/MechanismShell')}
        <collision name="mount_collision">{mesh_geom('energy_mount')}</collision>
        {visual('pedestal', mesh_geom('energy_plinth'), 'RM/Plinth')}
        <collision name="pedestal_collision">{mesh_geom('energy_plinth')}</collision>
      </link>
      <joint name="fix_world" type="fixed"><parent>world</parent><child>mount</child></joint>
      <link name="rotor">{pose(0, 0, e['z'])}{inertia(2)}{''.join(parts)}{''.join(collisions)}
        <collision name="frame_collision">{mesh_geom('energy_body')}</collision>
      </link>
      <joint name="shaft" type="revolute"><parent>mount</parent><child>rotor</child>
        <axis><xyz>1 0 0</xyz><limit><lower>-1e16</lower><upper>1e16</upper></limit></axis>
      </joint>
      <plugin name="energy_motion" filename="libfield_motion_plugin.so"><kind>energy</kind><joint>shaft</joint>
        <motion_enabled>false</motion_enabled><initial_state>activated</initial_state>
      </plugin>
    </model>
    <model name="technology_core"><static>true</static>{pose(0,0,0,0,0,e['yaw'])}<link name="body">
      {visual('core_and_arms', mesh_geom('technology_core'), 'RM/MechanismShell')}
      {visual('red_core_lights', mesh_geom('technology_core_red'), 'RM/RedLightBar')}
      {visual('blue_core_lights', mesh_geom('technology_core_blue'), 'RM/BlueLightBar')}
      <collision name="core_collision">{mesh_geom('technology_core')}</collision>
    </link></model>"""


def dart_target_model(team):
    cx, cy = RED_BASE['center'] if team == 'red' else mirror_xy(*RED_BASE['center'])
    yaw = 0.0 if team == 'red' else math.pi
    light = 'RM/RedLightBar' if team == 'red' else 'RM/BlueLightBar'
    pieces = [visual('beveled_casing', mesh_geom('dart_casing'), 'RM/MechanismShell'),
              visual('armor_module', mesh_geom('dart_module'), 'RM/MechanismEdge'),
              visual('recessed_impact_surface', mesh_geom('dart_impact'), 'RM/Rubber'),
              visual('guide_hood', mesh_geom('dart_hood'), 'RM/MechanismShell')]
    # 中部140mm方形模块，两侧护边和细长灯条，与图4-13一致。
    for sign in (-1, 1):
        pieces.append(visual(f'side_rail_{sign}', box_geom(.012,.025,.158), 'RM/MechanismEdge',(.017,sign*.103,.031)))
        pieces.append(visual(f'bar_recess_{sign}', box_geom(.002,.011,.067), 'RM/Rubber',(.029,sign*.0735,.032)))
        pieces.append(visual(f'bar_{sign}', box_geom(.003,.005,.056), light, (.031,sign*.0735,.032)))
        for z in (-.026,.096):
            pieces.append(visual(f'fastener_{sign}_{z}', cylinder(.003,.0018),'RM/Screw',(.030,sign*.058,z),(0,math.pi/2,0)))
    for y in (-.040,.040):
        pieces.append(visual(f'panel_groove_{y}',box_geom(.0008,.001,.098),'RM/MechanismEdge',(.0295,y,.032)))
    pieces.append(visual('sensor_aperture',box_geom(.002,.014,.008),'RM/Rubber',(.029,0,.097)))
    tilt=math.radians(48)
    # 下方灯座和55mm曲面透镜，法向向下倾斜48°，正面看到的是带边框的椭圆。
    pieces.append(visual('guide_bezel',cylinder(.034,.014),'RM/Rubber',(.050,0,-.087),(0,math.pi/2+tilt,0)))
    pieces.append(visual('green_lens',mesh_geom('dart_lens'),'RM/GreenGuide',(.056,0,-.094),(0,tilt,0)))
    pieces.append(visual('lens_retainer',cylinder(.029,.002),'RM/MechanismEdge',(.054,0,-.091),(0,math.pi/2+tilt,0)))
    return f"""
    <model name="dart_target_{team}">
      {pose(cx, cy, 0)}
      <link name="rail">{inertia(10)}
        {visual('rail', box_geom(0.045, 0.84, 0.022), 'RM/MechanismEdge', (-.045, 0, DART['z']-.05))}
        <collision name="rail_collision">{pose(-.045,0,DART['z']-.05)}{box_geom(.045,.84,.022)}</collision>
      </link>
      <joint name="fixed_rail" type="fixed"><parent>world</parent><child>rail</child></joint>
      <link name="target">{pose(0, 0, DART['z'], 0, 0, yaw)}{inertia(.3)}
        {''.join(pieces)}
        <collision name="target_collision">{box_geom(DART['depth'],DART['width'],DART['height'])}</collision>
      </link>
      <joint name="slide" type="prismatic"><parent>rail</parent><child>target</child>
        <axis><xyz>0 1 0</xyz><use_parent_model_frame>true</use_parent_model_frame>
          <limit><lower>-.280</lower><upper>.280</upper></limit></axis>
      </joint>
      <plugin name="dart_motion_{team}" filename="libfield_motion_plugin.so"><kind>dart</kind><joint>slide</joint></plugin>
    </model>"""


def markings():
    """场景导航细节；不添加虚构的官方视觉标签。"""
    models = []
    for team in ('red', 'blue'):
        material = 'RM/RedPaint' if team == 'red' else 'RM/BluePaint'
        cx, cy = RED_BASE['center'] if team == 'red' else mirror_xy(*RED_BASE['center'])
        segments = []
        for i in range(6):
            a, b = i * math.pi / 3, (i + 1) * math.pi / 3
            p1, p2 = (cx + 1.4 * math.cos(a), cy + 1.4 * math.sin(a)), (cx + 1.4 * math.cos(b), cy + 1.4 * math.sin(b))
            # 旧网格可见地面z约0.198；油漆抬至0.204，避免被地面遮住。
            segments.append(visual(f'base_boundary_{i}', box_geom(1.4, 0.025, 0.003), 'RM/WhitePaint',
                                   ((p1[0]+p2[0])/2, (p1[1]+p2[1])/2, .204), (0, 0, math.atan2(p2[1]-p1[1], p2[0]-p1[0]))))
        sign = -1 if team == 'red' else 1
        for row in (-1, 0, 1):
            segments.append(visual(f'team_lane_{row}', box_geom(2.2, .025, .003), material,
                                   (cx + sign * .8, cy + row * .08, .204)))
        hx, hy, hz, _ = hangar_pose(team)
        segments.append(visual('hangar_top', box_geom(0.30, 0.30, 0.004), 'RM/ArmorBody', (hx, hy, hz + 0.002)))
        for name, dx, dy, sx, sy in [('h_left',-.075,0,.018,.18),('h_right',.075,0,.018,.18),('h_cross',0,0,.15,.018)]:
            segments.append(visual(name, box_geom(sx, sy, .003), 'RM/WhitePaint', (hx+dx, hy+dy, hz+.006)))
        for sign in (-1, 1):
            segments.append(visual(f'hangar_edge_{sign}', box_geom(.32,.015,.003),material,(hx,hy+sign*.155,hz+.006)))
        models.append(f'<model name="{team}_navigation_markings"><static>true</static><link name="paint">{"".join(segments)}</link></model>')
    return ''.join(models)


def world_sdf(boxes):
    field_visuals = ''.join(
        f'<visual name="{name}"><geometry><mesh><uri>model://rm_field/meshes/field_{name}.{extension}</uri>'
        f'</mesh></geometry>{mat(material)}</visual>' for name, extension, material in SURFACES)
    parts = [f"""
    <model name="field_static_visual"><static>true</static><link name="body">
      <pose>0 0 -0.23 0 0 0</pose>
      {field_visuals}</link></model>"""]
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
    parts.append(energy_model())
    parts.extend(dart_target_model(team) for team in ('red', 'blue'))
    parts.append(markings())

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
        br = RED_BASE['body_r']
        lines.append(f"  - {{name: {team}_base_body, center: [{c[0]:.3f}, {c[1]:.3f}, {bz / 2:.3f}], "
                     f"size: [{2 * br:.3f}, {2 * br:.3f}, {bz:.3f}]}}")
    oz = RED_OUTPOST['hub_z'] + 0.4
    for team, c in (('red', ro), ('blue', mirror_xy(*ro))):
        lines.append(f"  - {{name: {team}_outpost_body, center: [{c[0]:.3f}, {c[1]:.3f}, {oz / 2:.3f}], size: [0.4, 0.4, {oz:.3f}]}}")
    e = ENERGY
    radius = DETAILED_ENERGY['radius']
    # 45°转轴的完整扫掠AABB；与生成的支撑柱/科技核心共用尺寸。
    extent = abs(math.cos(e['yaw'])) * (DETAILED_ENERGY['half_depth'] + 0.01) + abs(math.sin(e['yaw'])) * radius
    lines.append(f"  - {{name: energy_rotors, center: [0.0, 0.0, {e['z']:.3f}], size: [{2*extent:.3f}, {2*extent:.3f}, {2*radius:.3f}]}}")
    # 源模型静态支架/科技核心使用完整网格碰撞；规划侧保守包络同一个网格。
    bounds = np.asarray(DETAILED_ENERGY['mount_bounds'])
    corners = np.asarray([(x,y,z) for x in bounds[:,0] for y in bounds[:,1] for z in bounds[:,2]])
    yaw=e['yaw'];r=np.array([[math.cos(yaw),-math.sin(yaw),0],[math.sin(yaw),math.cos(yaw),0],[0,0,1]])
    corners=corners@r.T
    center=(corners.min(0)+corners.max(0))/2
    size=corners.max(0)-corners.min(0)+.002
    lines.append(f'  - {{name: energy_mount, center: [{center[0]:.3f}, {center[1]:.3f}, {center[2]:.3f}], size: [{size[0]:.3f}, {size[1]:.3f}, {size[2]:.3f}]}}')
    lines.append('  - {name: energy_plinth, center: [0.0, 0.0, 0.550], size: [3.114, 3.114, 0.300]}')
    core_bounds = np.asarray(DETAILED_ENERGY['core_bounds'])
    core_corners = np.asarray([(x,y,z) for x in core_bounds[:,0] for y in core_bounds[:,1] for z in core_bounds[:,2]]) @ r.T
    core_center = (core_corners.min(0) + core_corners.max(0)) / 2
    core_size = core_corners.max(0) - core_corners.min(0) + .002
    lines.append(f'  - {{name: technology_core, center: [{core_center[0]:.3f}, {core_center[1]:.3f}, {core_center[2]:.3f}], size: [{core_size[0]:.3f}, {core_size[1]:.3f}, {core_size[2]:.3f}]}}')
    for team, c in (('red', rb), ('blue', mirror_xy(*rb))):
        lines.append(f"  - {{name: dart_target_{team}_sweep, center: [{c[0]:.3f}, {c[1]:.3f}, {DART['z']:.3f}], "
                     f"size: [{DART['depth']:.3f}, {max(0.86,2*DART['travel']+DART['width']):.3f}, {DART['height']:.3f}]}}")

    lines.append('nofly:')
    for b in boxes:
        if not b['name'].endswith('dart_station'):
            continue
        lines.append(f"  - {{name: {b['name']}, center: [{b['center'][0]:.3f}, {b['center'][1]:.3f}, 0.0], "
                     f"size: [{b['size'][0]:.3f}, {b['size'][1]:.3f}, 0.0]}}")
    lines.append('armor_classes:')
    for (team, kind), class_id in ARMOR_CLASSES.items():
        lines.append(f'  {class_id}: {team}_{kind}')
    lines.append('targets:')
    def target_line(team, kind, i, face):
        place = armor_placement(face)
        size = ARMOR_PATTERNS[kind][2]
        def vector(v):
            return '[' + ', '.join(f'{x:.6f}' for x in v) + ']'
        points = '[' + ', '.join(vector(p) for p in armor_keypoints(face, size)) + ']'
        armor_type = 'large' if kind == 'base' else 'small'
        return (f"  - {{name: {team}_{kind}_armor_{i}, team: {team}, kind: {kind}, armor_type: {armor_type}, "
                f"class_id: {ARMOR_CLASSES[(team, kind)]}, pattern: {kind}, "
                f"position: {vector(place['target'])}, normal_yaw: {place['normal_yaw']:.6f}, "
                f"normal: {vector(place['normal'])}, spin_speed: 0.0000, "
                f"light_bar_corners: {points}}}")
    for team in ('red', 'blue'):
        for i, face in enumerate(team_faces(RED_BASE_FACES, team)):
            lines.append(target_line(team, 'base', i, face))
        for i, face in enumerate(team_faces(RED_OUTPOST_FACES, team)):
            lines.append(target_line(team, 'outpost', i, face))
    # 动态道具只供场景/离线数据集使用，绝不混入无人机攻击目标。
    lines.append('scene_objects:')
    lines.append('  - {name: energy_mechanism, role: energy_mechanism, drone_attackable: false, center: [0.0, 0.0, 2.600], target_orbit_radius: 0.700, target_diameter: 0.308, effective_diameter: 0.300}')
    for team in ('red', 'blue'):
        cx, cy = rb if team == 'red' else mirror_xy(*rb)
        lines.append(f'  - {{name: dart_target_{team}, role: dart_target, team: {team}, drone_attackable: false, center: [{cx:.3f}, {cy:.3f}, {DART["z"]:.3f}], axis: [0, 1, 0], travel: [-0.280, 0.280]}}')
    lines.append('hangars:')
    for team in ('red', 'blue'):
        x, y, z, yaw = hangar_pose(team)
        lines.append(f'  {team}: {{position: [{x:.3f}, {y:.3f}, {z:.3f}], yaw: {yaw:.4f}}}')
    with open(MAP_PATH, 'w') as f:
        f.write('\n'.join(lines) + '\n')


def _ray_vertical_cylinder(origin, direction, axis_xy, radius, z0, z1):
    ox, oy = origin[0] - axis_xy[0], origin[1] - axis_xy[1]
    dx, dy = direction[0], direction[1]
    a = dx * dx + dy * dy
    b = 2 * (ox * dx + oy * dy)
    c = ox * ox + oy * oy - radius * radius
    disc = b * b - 4 * a * c
    if a < 1e-9 or disc < 0:
        return None
    t = (-b - math.sqrt(disc)) / (2 * a)
    if t < 0:
        return None
    z = origin[2] + t * direction[2]
    if z0 <= z <= z1:
        return t
    return None


def check_armor_clearance():
    """沿朝外法向的反向射线必须先打到装甲，再打到主体圆柱。"""
    specs = []
    bz = RED_BASE['body'][2]
    specs.append(('base', RED_BASE_FACES, RED_BASE['center'], RED_BASE['body_r'],
                  RED_BASE['z0'], RED_BASE['z0'] + bz, LARGE_ARMOR))
    specs.append(('outpost', RED_OUTPOST_FACES, RED_OUTPOST['center'], RED_OUTPOST['hub_r'],
                  RED_OUTPOST['hub_z'] - 0.11, RED_OUTPOST['hub_z'] + 0.11, SMALL_ARMOR))
    bw = LIGHT_BAR[0]
    for label, faces, hub, radius, z0, z1, size in specs:
        half_y = (size[0] + 2 * bw) / 2
        half_z = size[1] / 2
        for team in ('red', 'blue'):
            origin = hub if team == 'red' else mirror_xy(*hub)
            for i, face in enumerate(team_faces(faces, team)):
                place = armor_placement(face)
                n = place['normal']
                start = tuple(place['link'][k] + n[k] * 0.30 for k in range(3))
                direction = tuple(-n[k] for k in range(3))
                # 装甲盒子在 link 上，射线从外侧 0.30 m 出发，命中厚度中心附近应为 0.30 m
                ta = 0.30 - PLATE_T / 2.0
                tc = _ray_vertical_cylinder(start, direction, origin, radius, z0, z1)
                print(f'{team} {label} {i}: link=({place["link"][0]:.3f},{place["link"][1]:.3f},{place["link"][2]:.3f}) '
                      f'armor_t={ta:.3f} body_t={None if tc is None else round(tc, 3)} '
                      f'sdf_pitch={place["sdf_pitch"]:.3f}')
                if tc is not None and not ta < tc - 1e-3:
                    raise SystemExit(f'{team} {label} {i} 主体碰撞先于装甲')
                yaw, pitch = place['yaw'], place['sdf_pitch']
                cy, sy = math.cos(yaw), math.sin(yaw)
                cp, sp = math.cos(pitch), math.sin(pitch)
                # R = Rz(yaw) Ry(pitch)，局部点变到世界
                for ly, lz in ((half_y, half_z), (half_y, -half_z), (-half_y, half_z), (-half_y, -half_z)):
                    lx = -PLATE_T / 2.0
                    x1 = cp * lx + sp * lz
                    y1 = ly
                    z_local = -sp * lx + cp * lz
                    wx = place['link'][0] + cy * x1 - sy * y1
                    wy = place['link'][1] + sy * x1 + cy * y1
                    wz = place['link'][2] + z_local
                    if z0 <= wz <= z1 and math.hypot(wx - origin[0], wy - origin[1]) < radius - 0.005:
                        raise SystemExit(f'{team} {label} {i} 装甲角点进入主体圆柱')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--skip-background-textures', action='store_true',
                        help='保留地面/木纹/围挡贴图，只重新生成模型、地图、装甲正面网格和材质')
    args = parser.parse_args()
    os.makedirs(os.path.join(MODEL_DIR, 'materials', 'scripts'), exist_ok=True)
    os.makedirs(os.path.dirname(WORLD_PATH), exist_ok=True)
    os.makedirs(os.path.dirname(MAP_PATH), exist_ok=True)
    write_armor_assets()
    ring_mesh()
    global DETAILED_ENERGY
    DETAILED_ENERGY = build_energy_assets(MODEL_DIR, ENERGY['z'])
    build_dart_assets(MODEL_DIR)
    prepare_scene_mesh()
    surface_textures(MODEL_DIR)
    if not args.skip_background_textures:
        gen_textures(os.path.join(MODEL_DIR, 'materials', 'textures'))
    else:
        for name in ('rm_floor.png', 'rm_wood.png', 'rm_wall.png'):
            if not os.path.isfile(os.path.join(MODEL_DIR, 'materials', 'textures', name)):
                parser.error(f'缺少背景贴图 {name}，请不带 --skip-background-textures 生成一次')
    with open(os.path.join(MODEL_DIR, 'materials', 'scripts', 'rm_field.material'), 'w') as f:
        f.write(MATERIAL_SCRIPT.format(fx=1.0 / 14.0, fy=1.0 / 7.5) + '\n' + surface_materials())
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
    check_armor_clearance()
    print('world :', WORLD_PATH)
    print('map   :', MAP_PATH)
    for t in ('red', 'blue'):
        print(f'{t} hangar pose (x y z yaw):', ['%.3f' % v for v in hangar_pose(t)])


if __name__ == '__main__':
    raise SystemExit('旧地图生成入口已停用；重建当前场地请使用 gen_official_field.py。'
                     '本模块仅保留供 STEP 生成器复用的装甲材质定义。')
