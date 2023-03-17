# Utility function depending on the `urdf_parser_py` module (provided by ROS).

from __future__ import annotations

from pathlib import Path
from typing import Optional

import FreeCAD as fc

import Mesh as fcmesh  # FreeCAD.

from ament_index_python.packages import PackageNotFoundError
from ament_index_python.packages import get_package_share_directory

from urdf_parser_py.urdf import Box
from urdf_parser_py.urdf import Cylinder
from urdf_parser_py.urdf import Joint as UrdfJoint
from urdf_parser_py.urdf import Joint as UrdfLink
from urdf_parser_py.urdf import Mesh
from urdf_parser_py.urdf import Pose
from urdf_parser_py.urdf import Sphere

from .utils import add_object
from .utils import is_group
from .utils import read_mesh_dae
from .utils import scale_mesh_object
from .export_urdf import rotation_from_rpy

# Typing hints.
Doc = fc.Document
DO = fc.DocumentObject
Shape = [Box, Cylinder, Mesh, Sphere]


def obj_from_geometry(
        geometry: Shape,
        doc_or_group: [Doc | DO],
        ) -> tuple[DO, Optional[Path]]:
    """Return a FreeCAD object for the URDF shape with the path for meshes."""
    if isinstance(geometry, Box):
        return obj_from_box(geometry, doc_or_group)
    if isinstance(geometry, Cylinder):
        return obj_from_cylinder(geometry, doc_or_group)
    if isinstance(geometry, Mesh):
        return obj_from_mesh(geometry, doc_or_group)
    if isinstance(geometry, Sphere):
        return obj_from_sphere(geometry, doc_or_group)
    raise NotImplementedError('Primitive not implemented')


def mesh_path_from_urdf(
        mesh_path: str,
        ) -> Optional[Path]:
    if (not (mesh_path.startswith('package://')
             or mesh_path.startswith('file://'))):
        return
    if mesh_path.startswith('package://'):
        try:
            pkg, _, rel_path = mesh_path[len('package://'):].partition('/')
        except ValueError:
            return
        try:
            pkg_path = get_package_share_directory(pkg)
        except PackageNotFoundError:
            return
        return Path(pkg_path) / rel_path
    elif mesh_path.startswith('file://'):
        return Path(mesh_path[len('file://'):])


def placement_from_origin(
        origin: Pose,
        ) -> fc.Placement:
    """Return the FreeCAD placement corresponding to URDF origin."""
    placement = fc.Placement()
    if origin is None:
        return placement
    if hasattr(origin, 'position'):
        # Convert from meters to millimeters.
        placement.Base = fc.Vector(origin.position) * 1000.0
    if hasattr(origin, 'rpy'):
        placement.Rotation = rotation_from_rpy(origin.rpy)
    return placement


def placement_from_link(
        link: UrdfLink,
        ) -> fc.Placement:
    """Return the FreeCAD placement corresponding to the URDF link."""
    if not hasattr(link, 'origin'):
        return fc.Placement()
    return placement_from_origin(link.origin)


def placement_from_joint(
        joint: UrdfJoint,
        ) -> fc.Placement:
    """Return the FreeCAD placement corresponding to the URDF joint."""
    if not hasattr(joint, 'origin'):
        return fc.Placement()
    return placement_from_origin(joint.origin)


def axis_to_z(
        joint: UrdfJoint,
        ) -> fc.Placement:
    """Return the rotation to bring `joint.axis` to z."""
    if hasattr(joint, 'axis') and (joint.axis is not None):
        axis = fc.Vector(joint.axis)
    else:
        # URDF's default axis.
        if joint.type in ['prismatic', 'revolute', 'continuous', 'planar']:
            axis = fc.Vector(1.0, 0.0, 0.0)
        else:
            axis = fc.Vector(0.0, 0.0, 1.0)
    zaxis = fc.Vector(0.0, 0.0, 1.0)
    # Orient to bring the joint axis along z.
    return fc.Rotation(zaxis, axis)


def placement_along_z_from_joint(
        joint: UrdfJoint,
        ) -> fc.Placement:
    """Return the joint placement so that its axis is along z."""
    placement = placement_from_joint(joint)
    placement.Rotation *= axis_to_z(joint)
    return placement


def obj_from_box(
        geometry: Box,
        doc_or_group: [Doc | DO],
        ) -> tuple[DO, None]:
    obj = add_object(doc_or_group, 'Part::Box', 'box')
    obj.Length = geometry.size[0] * 1000.0  # m to mm.
    obj.Width = geometry.size[1] * 1000.0
    obj.Height = geometry.size[2] * 1000.0
    obj.Placement.Base -= fc.Vector(geometry.size) * 1000.0 / 2.0
    return obj, None


def obj_from_cylinder(
        geometry: Cylinder,
        doc_or_group: [Doc | DO],
        ) -> tuple[DO, None]:
    obj = add_object(doc_or_group, 'Part::Cylinder', 'cylinder')
    obj.Radius = geometry.radius * 1000.0  # m to mm.
    obj.Height = geometry.length * 1000.0  # m to mm.
    obj.Placement.Base.z -= geometry.length * 1000.0 / 2.0
    return obj, None


def obj_from_mesh(
        geometry: Mesh,
        doc_or_group: [Doc | DO],
        ) -> tuple[DO, Path]:
    mesh_path = mesh_path_from_urdf(geometry.filename)
    if not mesh_path:
        return None, None
    if mesh_path.suffix.lower() == '.dae':
        raw_mesh = read_mesh_dae(mesh_path)
    else:
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
    if ((geometry.scale is not None)
            and (not (geometry.scale == 1.0)
                 or geometry.scale == [1.0, 1.0, 1.0])):
        scale_mesh_object(mesh_obj, geometry.scale)
    return mesh_obj, mesh_path


def obj_from_sphere(
        geometry: Sphere,
        doc_or_group: [Doc | DO],
        ) -> tuple[DO, None]:
    obj = add_object(doc_or_group, 'Part::Sphere', 'sphere')
    obj.Radius = geometry.radius * 1000.0  # m to mm.
    return obj, None
