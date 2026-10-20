"""Convert the pinned POWER.glb visual asset to Gazebo Classic meshes.

Converter code: this project's MIT license. Imported geometry: AGPL-3.0,
Blackjack200/bevy_robomaster_simulator, see meshes/source/LICENSE-POWER.
Only geometry is reused; no upstream simulation code is imported.
"""
import json
import os
import struct

import numpy as np


class Glb:
    def __init__(self, path):
        with open(path, 'rb') as f:
            data = f.read()
        if data[:4] != b'glTF' or struct.unpack_from('<I', data, 4)[0] != 2:
            raise ValueError('Expected glTF 2.0 binary asset')
        length, kind = struct.unpack_from('<II', data, 12)
        if kind != 0x4e4f534a:
            raise ValueError('Missing glTF JSON chunk')
        self.doc = json.loads(data[20:20 + length])
        self.binary = memoryview(data)[28 + length:]
        self.transforms = {}

        def walk(index, parent):
            node = self.doc['nodes'][index]
            m = np.eye(4)
            if 'matrix' in node:
                m = np.asarray(node['matrix']).reshape(4, 4).T
            else:
                x, y, z, w = node.get('rotation', [0, 0, 0, 1])
                m[:3, :3] = np.array([
                    [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                    [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                    [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)],
                ]) @ np.diag(node.get('scale', [1, 1, 1]))
                m[:3, 3] = node.get('translation', [0, 0, 0])
            self.transforms[index] = parent @ m
            for child in node.get('children', []):
                walk(child, parent @ m)

        for index in self.doc['scenes'][self.doc.get('scene', 0)]['nodes']:
            walk(index, np.eye(4))

    def accessor(self, index):
        a = self.doc['accessors'][index]
        view = self.doc['bufferViews'][a['bufferView']]
        dtype = np.dtype({5126: '<f4', 5125: '<u4', 5123: '<u2', 5121: 'u1'}[a['componentType']])
        width = {'VEC3': 3, 'VEC2': 2, 'VEC4': 4, 'SCALAR': 1}[a['type']]
        return np.ndarray(
            (a['count'], width), dtype=dtype, buffer=self.binary,
            offset=view.get('byteOffset', 0) + a.get('byteOffset', 0),
            strides=(view.get('byteStride', dtype.itemsize * width), dtype.itemsize))

    def triangles(self, index):
        node = self.doc['nodes'][index]
        m = self.transforms[index]
        out = []
        for primitive in self.doc['meshes'][node['mesh']]['primitives']:
            if primitive.get('mode', 4) != 4:
                raise ValueError('Expected triangle primitive')
            points = self.accessor(primitive['attributes']['POSITION'])
            points = points @ m[:3, :3].T + m[:3, 3]
            indices = self.accessor(primitive['indices']).reshape(-1, 3)
            out.append((primitive.get('material', 0), points[indices]))
        return out


def write_triangles(path, triangles):
    triangles = np.asarray(triangles, dtype=np.float32).reshape(-1, 3, 3)
    normal = np.cross(triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0])
    area = np.linalg.norm(normal, axis=1)
    keep = area > 1e-12
    triangles, normal, area = triangles[keep], normal[keep], area[keep]
    normal /= area[:, None]
    dtype = np.dtype([('normal', '<f4', 3), ('vertices', '<f4', (3, 3)), ('attribute', '<u2')])
    data = np.zeros(len(triangles), dtype=dtype)
    data['normal'], data['vertices'] = normal, triangles
    with open(path, 'wb') as f:
        f.write(b'RM27 generated mesh; attribution: meshes/source/README.md'.ljust(80, b' '))
        f.write(struct.pack('<I', len(data)))
        f.write(data.tobytes())


def build_energy_assets(model_dir, height):
    mesh_dir = os.path.join(model_dir, 'meshes')
    # 本项目补建的八边形台座，非POWER导入几何，给原模型的支脚提供承托面。
    outline = np.array([[-1.1, -1.1], [-.8, -1.4], [.8, -1.4], [1.1, -1.1],
                        [1.1, 1.1], [.8, 1.4], [-.8, 1.4], [-1.1, 1.1]])
    # 当前简化场地的中央高地表面约为0.6m，台座顶面露出该表面，避免藏在旧地形内。
    bottom = np.column_stack((outline, np.full(8, .40)))
    top = np.column_stack((outline * .96, np.full(8, .70)))
    facets = []
    for i in range(8):
        j = (i + 1) % 8
        facets.extend(((bottom[i], bottom[j], top[j]), (bottom[i], top[j], top[i]),
                       (bottom.mean(0), bottom[j], bottom[i]), (top.mean(0), top[i], top[j])))
    write_triangles(os.path.join(mesh_dir, 'energy_plinth.stl'), facets)
    glb = Glb(os.path.join(mesh_dir, 'source', 'POWER.glb'))
    names = {node['name']: i for i, node in enumerate(glb.doc['nodes']) if 'mesh' in node}
    groups = {node['name']: node for node in glb.doc['nodes']}
    center = (np.array(groups['FACE_1']['translation']) + np.array(groups['FACE_2']['translation'])) / 2
    # glTF Y-up -> local Z-up; local +X is blue-facing shaft normal.
    h = 2 ** -0.5
    rotation = np.array([[h, 0, h], [h, 0, -h], [0, 1, 0]])

    def node(name):
        return [(material, (points - center) @ rotation.T)
                for material, points in glb.triangles(names[name])]

    def export(filename, pieces):
        points = np.concatenate(pieces)
        write_triangles(os.path.join(mesh_dir, filename + '.stl'), points)
        return points

    mount = export('energy_mount', [p + [0, 0, height] for _, p in node('BASE')])
    body, extent_points, centers = [], [], {}
    for face, team in ((1, 'red'), (2, 'blue')):
        base_name = next(n for n in names if n.startswith(f'FACE_{face}_BASE'))
        body.extend(points for _, points in node(base_name))
        body.extend(points for _, points in node(f'FACE_{face}_R_PADDING'))
        export(f'energy_{team}_r', [p for _, p in node(f'FACE_{face}_R_POWERED')])
        centers[team] = []
        for i in range(5):
            prefix = f'FACE_{face}_TARGET_{i + 1}'
            target = np.concatenate([p for _, p in node(prefix + '_DISABLED')])
            body.append(target)
            flat = target.reshape(-1, 3)
            centers[team].append(((flat.min(0) + flat.max(0)) / 2).tolist())
            lights = []
            for part in ('_LEGGING_1', '_LEGGING_2', '_LEGGING_3', '_LEGGING_PROGRESSING'):
                for material, points in node(prefix + part):
                    (body if material == 0 else lights).append(points)
            extent_points.append(export(f'energy_{team}_arm_{i}', lights))
            extent_points.append(export(f'energy_{team}_rim_{i}', [p for _, p in node(prefix + '_PADDING')]))
    extent_points.append(export('energy_body', body))
    points = np.concatenate(extent_points).reshape(-1, 3)
    # 同一上游的科技核心与机械臂；不导入其中的GROUND场地，避免覆盖现有地形。
    core = Glb(os.path.join(mesh_dir, 'source', 'TECH_CORE.glb'))
    core_parts = {'body': [], 'red': [], 'blue': []}
    for i, n in enumerate(core.doc['nodes']):
        name = n.get('name', '')
        if 'mesh' not in n or not name.startswith(('RED_', 'BLUE_')):
            continue
        kind = ('red' if name.startswith('RED_') else 'blue') if '_LIGHT_' in name else 'body'
        core_parts[kind].extend((p - center) @ rotation.T + [0, 0, height] for _, p in core.triangles(i))
    core_body = export('technology_core', core_parts['body'])
    export('technology_core_red', core_parts['red'])
    export('technology_core_blue', core_parts['blue'])
    core_points = core_body.reshape(-1, 3)
    return dict(radius=float(np.linalg.norm(points[:, 1:], axis=1).max()),
                half_depth=float(np.abs(points[:, 0]).max()), centers=centers,
                core_bounds=[core_points.min(0).tolist(), core_points.max(0).tolist()],
                mount_bounds=[mount.reshape(-1, 3).min(0).tolist(), mount.reshape(-1, 3).max(0).tolist()])


def beveled_prism(path, outline, back, front, bevel=0.002):
    """Convex chamfered housing, local +X front, outline in Y/Z metres."""
    outline = np.asarray(outline, dtype=float)
    centroid = outline.mean(0)
    inset = centroid + (outline - centroid) * 0.96
    layers = [np.column_stack((np.full(len(outline), x), yz))
              for x, yz in ((back, inset), (back + bevel, outline), (front - bevel, outline), (front, inset))]
    triangles = []
    for a, b in zip(layers[:-1], layers[1:]):
        for i in range(len(outline)):
            j = (i + 1) % len(outline)
            triangles.extend(((a[i], b[j], b[i]), (a[i], a[j], b[j])))
    for layer, reverse in ((layers[0], True), (layers[-1], False)):
        c = layer.mean(0)
        for i in range(len(outline)):
            j = (i + 1) % len(outline)
            triangles.append((c, layer[j], layer[i]) if reverse else (c, layer[i], layer[j]))
    # Outlines are specified clockwise in the Y/Z drawing; fix all winding together.
    if np.cross(outline[1] - outline[0], outline[2] - outline[1]) < 0:
        triangles = np.asarray(triangles)[:, [0, 2, 1]]
    write_triangles(path, triangles)


def build_dart_assets(model_dir):
    mesh_dir = os.path.join(model_dir, 'meshes')
    casing = [(-.121, .105), (-.107, .1195), (.107, .1195), (.121, .105),
              (.121, -.055), (.075, -.1195), (-.075, -.1195), (-.121, -.055)]
    module = [(-.070, -.037), (-.065, -.042), (.065, -.042), (.070, -.037),
              (.070, .107), (.065, .112), (-.065, .112), (-.070, .107)]
    impact = [(-.043, -.022), (-.038, -.027), (.038, -.027), (.043, -.022),
              (.043, .086), (.038, .091), (-.038, .091), (-.043, .086)]
    hood = [(-.066, -.038), (.066, -.038), (.043, -.111), (-.043, -.111)]
    for name, outline, back, front in (
            ('dart_casing', casing, -.032, .010), ('dart_module', module, .010, .027),
            ('dart_impact', impact, .026, .029), ('dart_hood', hood, .012, .050)):
        beveled_prism(os.path.join(mesh_dir, name + '.stl'), outline, back, front)
    # A shallow curved lens catches oblique views; unlike a flat green dot it has a real bezel/cap.
    points, faces = [], []
    for ring in range(9):
        radius = .0275 * ring / 8
        for i in range(64):
            angle = 2 * np.pi * i / 64
            points.append((.006 * np.sqrt(max(0, 1 - (radius / .0275) ** 2)),
                           radius * np.cos(angle), radius * np.sin(angle)))
    for ring in range(8):
        for i in range(64):
            a, b = ring * 64 + i, ring * 64 + (i + 1) % 64
            faces.extend(((a, a + 64, b + 64), (a, b + 64, b)))
    write_triangles(os.path.join(mesh_dir, 'dart_lens.stl'), np.asarray(points)[np.asarray(faces)])
