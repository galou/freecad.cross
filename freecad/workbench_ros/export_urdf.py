"""Contains various helper functions to export to URDF from FreeCAD.
"""

import math
from pathlib import Path
from typing import Any, Iterable, Tuple, Union
import xml.etree.ElementTree as et

import FreeCAD as fc

import numpy as np

from .utils import label_or
from .utils import warn
from .utils import xml_comment


# Hint for a URDF rotation.
Rpy = Tuple[float, float, float]

# Small number to test whether a number is close to zero.
_EPS = np.finfo(float).eps * 4.0


def quaternion_matrix(quaternion: Iterable[float]) -> np.ndarray:
    """Return the homogeneous rotation matrix from quaternion.

    The quaternion must have the format (qx, qy, qz, qw).

    Inspired from quaternion_matrix in
    https://github.com/ros/geometry/blob/noetic-devel/tf/src/tf/transformations.py.

    """
    q = np.array(quaternion[:4], dtype=np.float64, copy=True)
    nq = np.dot(q, q)
    if nq < _EPS:
        return np.identity(4)
    q *= math.sqrt(2.0 / nq)
    q = np.outer(q, q)
    return np.array((
        (1.0-q[1, 1]-q[2, 2],     q[0, 1]-q[2, 3],     q[0, 2]+q[1, 3], 0.0),
        (    q[0, 1]+q[2, 3], 1.0-q[0, 0]-q[2, 2],     q[1, 2]-q[0, 3], 0.0),
        (    q[0, 2]-q[1, 3],     q[1, 2]+q[0, 3], 1.0-q[0, 0]-q[1, 1], 0.0),
        (                0.0,                 0.0,                 0.0, 1.0)
        ), dtype=np.float64)


def euler_from_matrix(matrix) -> Rpy:
    """Convert a 3x3 rotation matrix to Euler angles (X, Y, Z) in fixed frame.

    This Euler angle convention is the one used in URDF format.

    The quaternion must have the format (qx, qy, qz, qw).

    Inspired from euler_from_matrix(matrix, axes='sxyz') in
    https://github.com/ros/geometry/blob/noetic-devel/tf/src/tf/transformations.py.

    """
    i, j, k = 0, 1, 2

    M = np.array(matrix, dtype=np.float64, copy=False)[:3, :3]
    cy = math.sqrt(M[i, i]*M[i, i] + M[j, i]*M[j, i])
    if cy > _EPS:
        ax = math.atan2( M[k, j],  M[k, k])
        ay = math.atan2(-M[k, i],  cy)
        az = math.atan2( M[j, i],  M[i, i])
    else:
        ax = math.atan2(-M[j, k],  M[j, j])
        ay = math.atan2(-M[k, i],  cy)
        az = 0.0

    return ax, ay, az


def urdf_from_quaternion(q) -> Rpy:
    """Convert quaternion to rpy (URDF convention).

    The quaternion must have the format (qx, qy, qz, qw).

    """
    return euler_from_matrix(quaternion_matrix(q))


def urdf_origin_from_placement(p: fc.Placement) -> et.Element:
    """Return an xml element 'origin'."""
    rpy = urdf_from_quaternion(p.Rotation.Q)
    pattern = '<origin xyz="{v.x:.6} {v.y:.6} {v.z:.6}" rpy="{r[0]} {r[1]} {r[2]}" />'
    return et.fromstring(pattern.format(v=p.Base * 1e-3, r=rpy))


def urdf_geometry_box(length_x: float, length_y: float, length_z: float) -> et.Element:
    """Return an xml element 'geometry' with a box.

    Lengths must be given in meters.

    """
    geometry = et.fromstring('<geometry></geometry>')
    geometry.append(et.fromstring(
        f'<box size="{length_x} {length_y} {length_z}" />'))
    return geometry


def urdf_box_placement_from_fc(box: 'PrimitivePy') -> fc.Placement:
    """Return the FreeCAD placement of the box center.

    Return the placement of the box center for a FreeCAD box with the placement
    at its lower-left-bottom point.

    """
    if (not hasattr(box, 'TypeId')) and (box.TypeId != 'Part::Box'):
        raise RuntimeError("Argument must be a 'Part::Box'")
    offset = box.Placement.Rotation.multVec(fc.Vector(box.Length.Value, box.Width.Value, box.Height.Value) / 2.0)
    center_placement = fc.Placement(box.Placement)
    center_placement.Base += offset
    return center_placement


def urdf_geometry_sphere(radius: float) -> et.Element:
    """Return an xml element 'geometry' with a sphere.

    The radius must be given in meters.

    """
    geometry = et.fromstring('<geometry></geometry>')
    geometry.append(et.fromstring(
        f'<sphere radius="{radius}" />'))
    return geometry


def urdf_geometry_cylinder(radius: float, length: float) -> et.Element:
    """Return an xml element 'geometry' with a cylinder.

    Lengths must be given in meters.

    """
    geometry = et.fromstring('<geometry></geometry>')
    geometry.append(et.fromstring(
        f'<cylinder radius="{radius}" length="{length}" />'))
    return geometry


def urdf_cylinder_placement_from_fc(cyl: 'PrimitivePy') -> fc.Placement:
    """Return the FreeCAD placement of the cylinder center.

    Return the placement of the cylinder center for a FreeCAD cylinder with the
    placement at the center of its bottom disc.

    """
    if (not hasattr(cyl, 'TypeId')) and (cyl.TypeId != 'Part::Cylinder'):
        raise RuntimeError("Argument must be a 'Part::Cylinder'")
    offset = cyl.Placement.Rotation.multVec(fc.Vector(0.0, 0.0, cyl.Height.Value / 2.0))
    center_placement = fc.Placement(cyl.Placement)
    center_placement.Base += offset
    return center_placement


def _urdf_generic_from_box(box: 'PrimitivePy', generic: str) -> et.Element:
    """Return the xml element for visual or collision for a FreeCAD's box.

    Parameters
    ----------
    - cyl: the FreeCAD cylinder Part object
    - generic: {'visual', 'collision'}.

    """
    if (not hasattr(box, 'TypeId')) and (box.TypeId != 'Part::Box'):
        raise RuntimeError("Argument must be a 'Part::Box'")
    parent = et.fromstring('<{0}></{0}>'.format(generic))
    # Correct the placement to fit the box center.
    center_placement = urdf_box_placement_from_fc(box)
    parent.append(urdf_origin_from_placement(center_placement))  # TODO: handle links
    parent.append(urdf_geometry_box(
        box.Length.getValueAs('m'),
        box.Width.getValueAs('m'),
        box.Height.getValueAs('m'),
        ))
    return parent


def urdf_visual_from_box(box: 'PrimitivePy') -> et.Element:
    """Return the xml element for visual for a FreeCAD's box.

    Parameters
    ----------
    - cyl: the FreeCAD cylinder Part object
    - generic: {'visual', 'collision'}.

    """
    return _urdf_generic_from_box(box, 'visual')


def urdf_collision_from_box(box: 'PrimitivePy') -> et.Element:
    """Return the xml element for collision for a FreeCAD's box.

    Parameters
    ----------
    - cyl: the FreeCAD cylinder Part object
    - generic: {'visual', 'collision'}.

    """
    return _urdf_generic_from_box(box, 'collision')


def _urdf_generic_from_sphere(sphere: 'PrimitivePy', generic: str) -> et.Element:
    """Return the xml element for visual or collision for a FreeCAD's sphere.

    Parameters
    ----------
    - sphere: the FreeCAD sphere Part object
    - generic: {'visual', 'collision'}.

    """
    if (not hasattr(sphere, 'TypeId')) and (sphere.TypeId != 'Part::Sphere'):
        raise RuntimeError("Argument must be a 'Part::Sphere'")
    parent = et.fromstring('<{0}></{0}>'.format(generic))
    parent.append(urdf_origin_from_placement(sphere.Placement))  # TODO: handle links
    parent.append(urdf_geometry_sphere(
        sphere.Radius.getValueAs('m'),
        ))
    return parent


def urdf_visual_from_sphere(sphere: 'PrimitivePy') -> et.Element:
    """Return the xml element for visual for a FreeCAD's sphere.

    Parameters
    ----------
    - sphere: the FreeCAD sphere Part object
    - generic: {'visual', 'collision'}.

    """
    return _urdf_generic_from_sphere(sphere, 'visual')


def urdf_collision_from_sphere(sphere: 'PrimitivePy') -> et.Element:
    """Return the xml element for collision for a FreeCAD's sphere.

    Parameters
    ----------
    - sphere: the FreeCAD sphere Part object
    - generic: {'visual', 'collision'}.

    """
    return _urdf_generic_from_sphere(sphere, 'collision')


def _urdf_generic_from_cylinder(cyl: 'PrimitivePy', generic: str) -> et.Element:
    """Return the xml element for visual or collision for a FreeCAD's cylinder.

    Parameters
    ----------
    - cyl: the FreeCAD cylinder Part object
    - generic: {'visual', 'collision'}.

    """
    if (not hasattr(cyl, 'TypeId')) and (cyl.TypeId != 'Part::Cylinder'):
        raise RuntimeError("Argument must be a 'Part::Cylinder'")
    parent = et.fromstring('<{0}></{0}>'.format(generic))
    # Correct the placement to fit the cylinder center.
    center_placement = urdf_cylinder_placement_from_fc(cyl)
    parent.append(urdf_origin_from_placement(center_placement))  # TODO: handle links
    parent.append(urdf_geometry_cylinder(
        cyl.Radius.getValueAs('m'),
        cyl.Height.getValueAs('m'),
        ))
    return parent


def urdf_visual_from_cylinder(cyl: 'PrimitivePy') -> et.Element:
    """Return the xml element for visual for a FreeCAD's cylinder.

    Parameters
    ----------
    - cyl: the FreeCAD cylinder Part object
    - generic: {'visual', 'collision'}.

    """
    return _urdf_generic_from_cylinder(cyl, 'visual')


def urdf_collision_from_cylinder(cyl: 'PrimitivePy') -> et.Element:
    """Return the xml element for collision for a FreeCAD's cylinder.

    Parameters
    ----------
    - cyl: the FreeCAD cylinder Part object
    - generic: {'visual', 'collision'}.

    """
    return _urdf_generic_from_cylinder(cyl, 'collision')


def urdf_geometry_mesh(mesh_name: str, package_name: str) -> et.Element:
    """Return an xml element 'geometry' with a mesh.

    """
    geometry = et.fromstring('<geometry></geometry>')
    geometry.append(et.fromstring(
        f'<mesh filename="package://{package_name}/{mesh_name}" />'))
    return geometry


def _urdf_generic_from_object(obj: fc.DocumentObject, mesh_name: str, package_name: str, generic: str) -> et.Element:
    """Return the xml element for visual or collision for a FreeCAD object.

    The URDF just contains a reference to the object.

    The mesh is not exported.

    Parameters
    ----------
    - obj: the FreeCAD object (must have a `Placement` attribute.
    - generic: {'visual', 'collision'}.

    """
    if (not hasattr(obj, 'Placement')) or (not isinstance(obj.Placement, fc.Placement)):
        raise RuntimeError("Argument must be a FreeCAD object with a 'Placement' attribute")
    parent = et.fromstring('<{0}></{0}>'.format(generic))
    parent.append(et.Comment(xml_comment(obj.Label)))
    parent.append(urdf_origin_from_placement(obj.Placement))  # TODO: handle links
    parent.append(urdf_geometry_mesh(mesh_name, package_name))
    return parent


def urdf_visual_from_object(obj: fc.DocumentObject, mesh_name: str, package_name: str) -> et.Element:
    """Return the xml element for visual for a FreeCAD object.

    The URDF just contains a reference to the object.

    The mesh is not exported.

    Parameters
    ----------
    - obj: the FreeCAD object (must have a `Placement` attribute.

    """
    return _urdf_generic_from_object(obj, mesh_name, package_name, 'visual')


def urdf_collision_from_object(obj: fc.DocumentObject, mesh_name: str, package_name: str) -> et.Element:
    """Return the xml element for collision for a FreeCAD object.

    The URDF just contains a reference to the object.

    The mesh is not exported.

    Parameters
    ----------
    - obj: the FreeCAD object (must have a `Placement` attribute.

    """
    return _urdf_generic_from_object(obj, mesh_name, package_name, 'collision')


def export_group_with_lcs(group: fc.DocumentObjectGroup, package_path: Union[str,Path]) -> bool:
    """Export a group containing a link to an assembly and links to LCS.

    Export a group containing a link to an assembly and links to local
    coordinate systems (LCS). Each LCS represent a joint. The assembly is a
    part with part children (or links to parts). It will be used to get the
    link associated to each joint as well as the relative link poses.

    """
    if (not hasattr(group, 'TypeId')) or (group.TypeId != 'App::DocumentObjectGroup'):
         raise ValueError('First argument must be a group, i.e. "App::DocumentObjectGroup"')
    fc_links = group.Group
    urdf_joints: List[fc.DocumentObject] = []
    for fc_link in fc_links:
        if not hasattr(fc_link, 'TypeId'):
            # I don't know what `fc_link` is.
            warn(f'"{label_or(fc_link)}" has unexpected type, ignoring')
            continue
        if fc_link.TypeId != 'App::Link':
            # fc_link is not a link, cannot handle it.
            warn(f'"{label_or(fc_link)}" is not a link, ignoring')
            continue
        linked_object = fc_link.LinkedObject
        if ((not hasattr(linked_object, 'TypeId'))
                or (not linked_object.TypeId in ('PartDesign::CoordinateSystem', 'App::Part'))):
            # linked_object is not a coordinate system, cannot handle it.
            warn(f'Linked object of "{label_or(fc_link)}" is not a coordinate system or part, ignoring')
            continue
    return True
