"""Surface materials for the existing field mesh; geometry is not reshaped.

The palette follows the local 2027 rules' figures 4-1/2/3. These are approximate
display colours, not an official colour specification or camera calibration.
"""
import os

import numpy as np
from PIL import Image


SURFACES = (
    ('ground', 'dae', 'RM/FieldGround'),
    ('platforms', 'dae', 'RM/FieldPlatform'),
    ('edges', 'stl', 'RM/FieldEdge'),
    ('hardware', 'stl', 'RM/FieldHardware'),
    ('barriers', 'stl', 'RM/FieldBarrier'),
)

PALETTE = {
    'ground': '#343638',
    'platforms': '#797D79',
    'edges': '#585D55',
    'hardware': '#303438',
    'barriers': '#191C20',
}


def surface_textures(model_dir):
    """Subtle PVC grain/scuffs; do not invent official tags or large coloured patches."""
    texture_dir = os.path.join(model_dir, 'materials', 'textures')
    rng = np.random.default_rng(20271020)
    yy, xx = np.mgrid[0:512, 0:512]
    for name, filename, amplitude in (
            ('ground', 'rm_competition_floor.png', 1.4),
            ('platforms', 'rm_competition_platform.png', 3.0)):
        base = np.array([int(PALETTE[name][i:i + 2], 16) for i in (1, 3, 5)])
        grain = rng.normal(0, amplitude, (512, 512))
        worn = 1.3 * np.sin(xx / 21 + .5 * np.sin(yy / 49)) + .8 * np.cos(yy / 37)
        # Sparse broad, low-contrast wear marks, instead of bright synthetic confetti.
        for _ in range(18):
            cx, cy = rng.uniform(0, 512, 2)
            sx, sy = rng.uniform(10, 80, 2)
            worn += rng.uniform(-3, 3) * np.exp(-((xx-cx)/sx)**2 - ((yy-cy)/sy)**2)
        pixels = np.clip(base + (grain + worn)[..., None], 0, 255).astype(np.uint8)
        Image.fromarray(pixels).save(os.path.join(texture_dir, filename))


def surface_materials():
    def rgb(hex_color):
        return ' '.join(f'{int(hex_color[i:i+2],16)/255:.5f}' for i in (1, 3, 5))
    parts = []
    for name, _, material in SURFACES:
        if name in ('ground', 'platforms'):
            texture = 'rm_competition_floor.png' if name == 'ground' else 'rm_competition_platform.png'
            body = (f'ambient .8 .8 .8 1\n diffuse .8 .8 .8 1\n'
                    f'texture_unit {{ texture {texture}\n tex_address_mode wrap\n'
                    'filtering anisotropic\n max_anisotropy 8\n }')
        else:
            color = rgb(PALETTE[name])
            body = f'ambient {color} 1\n diffuse {color} 1'
        parts.append(f'material {material} {{ technique {{ pass {{ lighting on\n'
                     f'{body}\n specular .02 .02 .02 1 8\n }} }} }}\n')
    return '\n'.join(parts)


def write_textured_mesh(path, triangles):
    """Bake world-planar UVs onto the same vertices; no vertex welding or simplification."""
    points = triangles['vertices'].reshape(-1, 3)
    n = len(triangles)
    cross = np.cross(points.reshape(-1, 3, 3)[:, 1] - points.reshape(-1, 3, 3)[:, 0],
                     points.reshape(-1, 3, 3)[:, 2] - points.reshape(-1, 3, 3)[:, 0])
    normals = cross / np.maximum(np.linalg.norm(cross, axis=1, keepdims=True), 1e-12)
    # One texture tile per 1.5m, continuous across disconnected faces.
    uv = points[:, :2] / 1.5
    def numbers(values):
        return ' '.join(f'{float(v):.9g}' for v in values.reshape(-1))
    faces = ' '.join(f'{3*i+j} {i} {3*i+j}' for i in range(n) for j in range(3))
    content = f'''<?xml version="1.0" encoding="utf-8"?>
<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1">
<asset><unit name="meter" meter="1"/><up_axis>Z_UP</up_axis></asset>
<library_geometries><geometry id="surface"><mesh>
<source id="pos"><float_array id="pos-array" count="{points.size}">{numbers(points)}</float_array>
<technique_common><accessor source="#pos-array" count="{len(points)}" stride="3"><param name="X" type="float"/><param name="Y" type="float"/><param name="Z" type="float"/></accessor></technique_common></source>
<source id="normal"><float_array id="normal-array" count="{normals.size}">{numbers(normals)}</float_array>
<technique_common><accessor source="#normal-array" count="{n}" stride="3"><param name="X" type="float"/><param name="Y" type="float"/><param name="Z" type="float"/></accessor></technique_common></source>
<source id="uv"><float_array id="uv-array" count="{uv.size}">{numbers(uv)}</float_array>
<technique_common><accessor source="#uv-array" count="{len(uv)}" stride="2"><param name="S" type="float"/><param name="T" type="float"/></accessor></technique_common></source>
<vertices id="vertices"><input semantic="POSITION" source="#pos"/></vertices>
<triangles count="{n}"><input semantic="VERTEX" source="#vertices" offset="0"/><input semantic="NORMAL" source="#normal" offset="1"/><input semantic="TEXCOORD" source="#uv" offset="2" set="0"/><p>{faces}</p></triangles>
</mesh></geometry></library_geometries>
<library_visual_scenes><visual_scene id="Scene"><node id="field"><instance_geometry url="#surface"/></node></visual_scene></library_visual_scenes>
<scene><instance_visual_scene url="#Scene"/></scene></COLLADA>
'''
    with open(path, 'w') as f:
        f.write(content)


def write_surface_meshes(model_dir, triangles, header, component_labels, low, high):
    """Partition by height/surface normal and bounded hardware components.

    Every source triangle is assigned exactly once. This is a visual material
    approximation because the source STL does not contain semantic materials.
    """
    world = triangles['vertices'].astype(np.float64).copy()
    world[:, :, 2] -= .23
    centers = world.mean(1)
    cross = np.cross(world[:, 1]-world[:, 0], world[:, 2]-world[:, 0])
    normals = cross / np.maximum(np.linalg.norm(cross, axis=1, keepdims=True), 1e-12)
    classes = np.full(len(triangles), 2, dtype=np.uint8)  # muted structural edges
    upper = normals[:, 2] > .5
    classes[upper & (centers[:, 2] <= .214)] = 0
    classes[upper & (centers[:, 2] > .214)] = 1
    hardware = np.zeros(len(low), dtype=bool)
    for cx, cy, radius in ((-11.595, .004, .95), (11.595, -.004, .95),
                           (-3.09, -3.84, .65), (3.09, 3.84, .65)):
        hardware |= ((low[:, 0] > cx-radius) & (high[:, 0] < cx+radius) &
                     (low[:, 1] > cy-radius) & (high[:, 1] < cy+radius) & (high[:, 2] > .65))
    classes[hardware[component_labels]] = 3
    boundary = ((np.abs(centers[:, 0]) > 13.75) | (np.abs(centers[:, 1]) > 7.25))
    classes[(boundary & (world[:, :, 2].max(1) > 1.8)) | (world[:, :, 2].max(1) > 5)] = 4
    mesh_dir = os.path.join(model_dir, 'meshes')
    counts = []
    for index, (name, extension, _) in enumerate(SURFACES):
        subset = triangles[classes == index]
        if not len(subset):
            raise ValueError(f'Expected nonempty field surface class: {name}')
        path = os.path.join(mesh_dir, f'field_{name}.{extension}')
        if extension == 'dae':
            write_textured_mesh(path, subset)
        else:
            with open(path, 'wb') as f:
                f.write(header)
                f.write(np.asarray([len(subset)], dtype='<u4').tobytes())
                f.write(subset.tobytes())
        counts.append(len(subset))
    assert sum(counts) == len(triangles)
    print('field material groups:', dict(zip((s[0] for s in SURFACES), counts)))
