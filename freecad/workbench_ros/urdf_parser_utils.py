# Utility function depending on the `urdf_parser_py` module (provided by ROS).

from __future__ import annotations

from pathlib import Path
from typing import Optional

import FreeCAD as fc

import Mesh as fcmesh # FreeCAD.

from ament_index_python.packages import PackageNotFoundError
from ament_index_python.packages import get_package_share_directory

from urdf_parser_py import xml_reflection as xmlr
from urdf_parser_py.urdf import Box
from urdf_parser_py.urdf import Cylinder
from urdf_parser_py.urdf import Mesh
from urdf_parser_py.urdf import Pose
from urdf_parser_py.urdf import Sphere

from .utils import is_group
from .utils import scale_mesh_object
from .export_urdf import rotation_from_rpy

# Typing hints.
Doc = fc.Document
DO = fc.DocumentObject
Shape = [Box, Cylinder, Mesh, Pose, Sphere]


def obj_from_geometry(
        geometry: Shape,
        doc_or_group: [Doc | DO],
        ) -> tuple[DO, Optional[Path]]:
    """Return a FreeCAD object for the URDF shape with the path for meshes."""
    if isinstance(geometry, Mesh):
        return obj_from_mesh(geometry, doc_or_group)
    else:
        raise NotImplementedError('Primitive not implemented')


def mesh_path_from_urdf(
        mesh_path: str,
        ) -> Optional[Path]:
    if not mesh_path.startswith('package://'):
        return
    try:
        pkg, _, rel_path = mesh_path[len('package://'):].partition('/')
    except ValueError:
        return
    try:
        pkg_path = get_package_share_directory(pkg)
    except PackageNotFoundError:
        return
    return Path(pkg_path) / rel_path


def placement_from_origin(
        origin: Pose,
        ) -> fc.Placement:
    if origin is None:
        return fc.Placement()
    return fc.Placement(fc.Vector(origin.position),
                        rotation_from_rpy(origin.rpy))


def obj_from_mesh(
        geometry: xmlr.Object,
        doc_or_group: [Doc | DO],
        ) -> tuple[DO, Path]:
    mesh_path = mesh_path_from_urdf(geometry.filename)
    if not mesh_path:
        return
    raw_mesh = fcmesh.read(str(mesh_path))
    if is_group(doc_or_group):
        doc = doc_or_group.Document
    else:
        doc = doc_or_group
    mesh_obj = doc.addObject('Mesh::Feature', mesh_path.name)
    mesh_obj.Label = mesh_path.name
    if is_group(doc_or_group):
        doc_or_group.addObject(mesh_obj)
    mesh_obj.Mesh = raw_mesh
    if mesh_path.suffix.lower() in ['.stl', '.obj']:
        scale_mesh_object(mesh_obj, 1000.0)  # m to mm.
    if (geometry.scale is not None) and (geometry.scale != 1.0):
        scale_mesh_object(mesh_obj, geometry.scale)
    return mesh_obj, mesh_path
