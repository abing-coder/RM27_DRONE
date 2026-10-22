"""Small mesh utilities for the offline STEP field conversion (metres, Z up)."""
from pathlib import Path
import numpy as np


def simplify_by_color(vertices, faces, colors, max_error_mm=.5):
    """Reduce CAD tessellation with fixed colours and a bounded geometric error.

    Input remains in CAD millimetres. Armour mounting planes are extracted from
    the original BRep, independently of this visual/static-collision mesh.
    """
    if len(faces) < 20000:
        return vertices, faces, colors
    from vtkmodules.vtkCommonCore import vtkPoints
    from vtkmodules.vtkCommonDataModel import vtkCellArray, vtkPolyData
    from vtkmodules.vtkFiltersCore import vtkCleanPolyData, vtkDecimatePro
    from vtkmodules.util.numpy_support import numpy_to_vtk, numpy_to_vtkIdTypeArray, vtk_to_numpy
    palette, ids = np.unique(np.round(colors, 5), axis=0, return_inverse=True)
    vertex_sets, face_sets, color_sets = [], [], []
    offset = 0
    for index, color in enumerate(palette):
        group = faces[ids == index]
        points = vtkPoints()
        points.SetData(numpy_to_vtk(np.asarray(vertices, dtype=np.float64), deep=True))
        cells = vtkCellArray()
        cells.SetData(numpy_to_vtkIdTypeArray(np.arange(len(group) + 1, dtype=np.int64) * 3, deep=True),
                      numpy_to_vtkIdTypeArray(group.reshape(-1).astype(np.int64), deep=True))
        poly = vtkPolyData()
        poly.SetPoints(points)
        poly.SetPolys(cells)
        clean = vtkCleanPolyData()
        clean.SetInputData(poly)
        clean.ToleranceIsAbsoluteOn()
        clean.SetAbsoluteTolerance(.0001)
        decimate = vtkDecimatePro()
        decimate.SetInputConnection(clean.GetOutputPort())
        decimate.SetTargetReduction(.75)
        decimate.PreserveTopologyOn()
        decimate.BoundaryVertexDeletionOff()
        decimate.SetFeatureAngle(30.)
        decimate.SetErrorIsAbsolute(1)
        decimate.SetAbsoluteError(max_error_mm)
        decimate.AccumulateErrorOn()
        decimate.Update()
        result = decimate.GetOutput()
        v = vtk_to_numpy(result.GetPoints().GetData()).copy()
        f = vtk_to_numpy(result.GetPolys().GetConnectivityArray()).reshape(-1, 3).copy()
        if not len(f):
            raise ValueError('CAD simplification removed a material group')
        vertex_sets.append(v)
        face_sets.append(f + offset)
        color_sets.append(np.tile(color, (len(f), 1)))
        offset += len(v)
    return np.vstack(vertex_sets), np.vstack(face_sets), np.vstack(color_sets)


def transform_points(points, matrix):
    matrix = np.asarray(matrix, dtype=np.float64)
    points = np.asarray(points, dtype=np.float64)
    if matrix.shape != (4, 4) or not np.isfinite(matrix).all():
        raise ValueError("Invalid CAD transform")
    return points @ matrix[:3, :3].T + matrix[:3, 3]


def write_stl(path, vertices, faces):
    """Binary STL for static collision geometry, with the same vertices as visual."""
    points = np.asarray(vertices, dtype=np.float32)[faces]
    normal = np.cross(points[:, 1] - points[:, 0], points[:, 2] - points[:, 0])
    normal /= np.maximum(np.linalg.norm(normal, axis=1, keepdims=True), 1e-20)
    records = np.zeros(len(faces), dtype=[
        ('normal', '<f4', 3), ('vertices', '<f4', (3, 3)), ('attribute', '<u2')])
    records['normal'], records['vertices'] = normal, points
    with Path(path).open('wb') as stream:
        stream.write(b'RMUC2027 user-provided STEP; metres'.ljust(80, b' '))
        stream.write(np.array([len(faces)], dtype='<u4').tobytes())
        stream.write(records.tobytes())


def _numbers(stream, values, floating=True):
    flat = np.asarray(values).reshape(-1)
    for start in range(0, len(flat), 16384):
        chunk = flat[start:start + 16384]
        stream.write(' '.join(format(float(x), '.7g') for x in chunk) if floating
                     else ' '.join(str(int(x)) for x in chunk))
        stream.write(' ')


def write_collada(path, vertices, faces, colors):
    """Preserve STEP face RGBs; smooth within CAD faces, retain their sharp edges."""
    vertices = np.asarray(vertices, dtype=np.float64)
    faces = np.asarray(faces, dtype=np.int32)
    colors = np.asarray(colors, dtype=np.float64)
    if not len(faces) or vertices.shape[1:] != (3,) or colors.shape != (len(faces), 3):
        raise ValueError("Empty or malformed CAD mesh")
    if not np.isfinite(vertices).all() or not np.isfinite(colors).all():
        raise ValueError("Non-finite CAD mesh")
    if faces.min() < 0 or faces.max() >= len(vertices):
        raise ValueError("CAD triangle index out of bounds")
    normals = np.zeros_like(vertices)
    tri = vertices[faces]
    area_normal = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
    for column in range(3):
        np.add.at(normals, faces[:, column], area_normal)
    normals /= np.maximum(np.linalg.norm(normals, axis=1, keepdims=True), 1e-20)
    palette, material_ids = np.unique(np.round(colors, 5), axis=0, return_inverse=True)
    with Path(path).open('w', encoding='utf-8') as stream:
        stream.write('<?xml version="1.0" encoding="utf-8"?>\n'
                     '<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1">'
                     '<asset><unit name="meter" meter="1"/><up_axis>Z_UP</up_axis></asset>'
                     '<library_effects>')
        for index, rgb in enumerate(palette):
            color = ' '.join(f'{c:.5f}' for c in np.clip(rgb, 0, 1)) + ' 1'
            stream.write(f'<effect id="e{index}"><profile_COMMON><technique sid="common"><phong>'
                         f'<ambient><color>{color}</color></ambient>'
                         f'<diffuse><color>{color}</color></diffuse>'
                         '<specular><color>0.05 0.05 0.05 1</color></specular>'
                         '<shininess><float>8</float></shininess></phong></technique>'
                         '</profile_COMMON></effect>')
        stream.write('</library_effects><library_materials>')
        for index in range(len(palette)):
            stream.write(f'<material id="m{index}"><instance_effect url="#e{index}"/></material>')
        stream.write('</library_materials><library_geometries><geometry id="geometry"><mesh>')
        for name, data in [('positions', vertices), ('normals', normals)]:
            stream.write(f'<source id="{name}"><float_array id="{name}-array" count="{data.size}">')
            _numbers(stream, data)
            stream.write(f'</float_array><technique_common><accessor source="#{name}-array" '
                         f'count="{len(data)}" stride="3"><param name="X" type="float"/>'
                         '<param name="Y" type="float"/><param name="Z" type="float"/>'
                         '</accessor></technique_common></source>')
        stream.write('<vertices id="vertices"><input semantic="POSITION" source="#positions"/></vertices>')
        for index in range(len(palette)):
            group = faces[material_ids == index]
            stream.write(f'<triangles material="mat{index}" count="{len(group)}">'
                         '<input semantic="VERTEX" source="#vertices" offset="0"/>'
                         '<input semantic="NORMAL" source="#normals" offset="1"/><p>')
            _numbers(stream, np.repeat(group.reshape(-1), 2), floating=False)
            stream.write('</p></triangles>')
        stream.write('</mesh></geometry></library_geometries><library_visual_scenes>'
                     '<visual_scene id="scene"><node id="node"><instance_geometry url="#geometry">'
                     '<bind_material><technique_common>')
        for index in range(len(palette)):
            stream.write(f'<instance_material symbol="mat{index}" target="#m{index}"/>')
        stream.write('</technique_common></bind_material></instance_geometry></node></visual_scene>'
                     '</library_visual_scenes><scene><instance_visual_scene url="#scene"/></scene></COLLADA>')
