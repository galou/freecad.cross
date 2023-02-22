"""Contains various helper functions to export to URDF from FreeCAD.
"""

import math
from pathlib import Path
from typing import Iterable, List, Optional, Tuple, Union
import xml.etree.ElementTree as et

import FreeCAD as fc

import numpy as np

from .utils import has_placement
from .utils import is_box
from .utils import is_cylinder
from .utils import is_sphere
from .utils import label_or
from .utils import warn
from .utils import xml_comment


# Hint for a URDF rotation.
Rpy = Tuple[float, float, float]
QuatList = Tuple[float, float, float, float]  # (qx, qy, qz, qw).

# Small number to test whether a number is close to zero.
_EPS = np.finfo(float).eps * 4.0


def quaternion_matrix(quaternion: QuatList) -> np.ndarray:
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


def rpy_from_quaternion(q: QuatList) -> Rpy:
    """Convert quaternion to rpy (URDF convention).

    The quaternion must have the format (qx, qy, qz, qw).

    Cf. `rotation_from_rpy` for the "inverse" function.

    """
    return euler_from_matrix(quaternion_matrix(q))


def rotation_from_rpy(rpy: Rpy) -> fc.Rotation:
    """Convert rpy (URDF convention) to a FreeCAD's rotation (quaternion).

    Cf. `rpy_from_quaternion` for the "inverse" function.

    """
    # Inverse the order to get from URDF's rotations about absolute axes to
    # relative axes.
    return (
            fc.Rotation(fc.Vector(0.0, 0.0, 1.0), np.degrees(rpy[2]))
            * fc.Rotation(fc.Vector(0.0, 1.0, 0.0), np.degrees(rpy[1]))
            * fc.Rotation(fc.Vector(1.0, 0.0, 0.0), np.degrees(rpy[0])))


def urdf_origin_from_placement(p: fc.Placement) -> et.Element:
    """Return an xml element 'origin'."""
    rpy = rpy_from_quaternion(p.Rotation.Q)
    pattern = '<origin xyz="{v.x:.6} {v.y:.6} {v.z:.6}" rpy="{r[0]} {r[1]} {r[2]}" />'
    return et.fromstring(pattern.format(v=p.Base * 1e-3, r=rpy))


def urdf_geometry_box(length_x: float, length_y: float, length_z: float) -> et.Element:
    """Return an xml element 'geometry' with a box.

    Lengths must be given in meters.

    """
    geometry = et.fromstring('<geometry/>')
    geometry.append(et.fromstring(
        f'<box size="{length_x} {length_y} {length_z}" />'))
    return geometry


def urdf_box_placement_from_object(
        box: 'PrimitivePy',
        placement: Optional[fc.Placement] = None) -> fc.Placement:
    """Return the FreeCAD placement of the box center.

    Return the placement of the box center for a FreeCAD box with the placement
    at its lower-left-bottom point.

    """
    if not is_box(box):
        raise RuntimeError("First argument must be a 'Part::Box'")
    if not placement:
        placement = box.Placement
    to_center = fc.Placement(fc.Vector(box.Length.Value,
                                       box.Width.Value,
                                       box.Height.Value) / 2.0,
                             fc.Rotation())
    return placement * to_center


def urdf_geometry_sphere(radius: float) -> et.Element:
    """Return an xml element 'geometry' with a sphere.

    The radius must be given in meters.

    """
    geometry = et.fromstring('<geometry/>')
    geometry.append(et.fromstring(
        f'<sphere radius="{radius}" />'))
    return geometry


def urdf_sphere_placement_from_object(
        sphere: 'PrimitivePy',
        placement: Optional[fc.Placement] = None) -> fc.Placement:
    """Return the FreeCAD placement of the sphere center.

    Return the placement of the sphere center for a FreeCAD sphere.

    """
    if not is_sphere(sphere):
        raise RuntimeError("First argument must be a 'Part::Sphere'")
    if not placement:
        placement = sphere.Placement
    return placement


def urdf_geometry_cylinder(radius: float, length: float) -> et.Element:
    """Return an xml element 'geometry' with a cylinder.

    Lengths must be given in meters.

    """
    geometry = et.fromstring('<geometry/>')
    geometry.append(et.fromstring(
        f'<cylinder radius="{radius}" length="{length}" />'))
    return geometry


def urdf_cylinder_placement_from_object(
        cyl: 'PrimitivePy',
        placement: Optional[fc.Placement] = None) -> fc.Placement:
    """Return the FreeCAD placement of the cylinder center.

    Return the placement of the cylinder center for a FreeCAD cylinder with the
    placement at the center of its bottom disc.

    """
    if not is_cylinder(cyl):
        raise RuntimeError("Argument must be a 'Part::Cylinder'")
    if not placement:
        placement = cyl.Placement
    to_center = fc.Placement(fc.Vector(0.0, 0.0, cyl.Height.Value / 2.0), fc.Rotation())
    return placement * to_center


def _urdf_generic_from_box(
        box: 'PrimitivePy',
        generic: str,
        placement: fc.Placement = fc.Placement(),
        ignore_obj_placement: bool = False) -> et.Element:
    """Return the xml element for visual or collision for a FreeCAD's box.

    Parameters
    ----------
    - cyl: the FreeCAD cylinder Part object
    - generic: {'visual', 'collision'}.
    - placement:
      - additional displacement of the box, the box' original
        placement is added to this if ``ignore_obj_placement`` is False.
      - box' placement otherwise.
    - ignore_obj_placement: cf. ``placement``.

    """
    if not is_box(box):
        raise RuntimeError("Argument must be a 'Part::Box'")
    parent = et.fromstring(f'<{generic}/>')
    if not ignore_obj_placement:
        placement = placement * box.Placement
    center_placement = urdf_box_placement_from_object(box, placement)
    parent.append(urdf_origin_from_placement(center_placement))
    parent.append(urdf_geometry_box(
        box.Length.getValueAs('m'),
        box.Width.getValueAs('m'),
        box.Height.getValueAs('m'),
        ))
    return parent


def urdf_visual_from_box(
        box: 'PrimitivePy',
        placement: fc.Placement = fc.Placement(),
        ignore_obj_placement: bool = False) -> et.Element:
    """Return the xml element for visual for a FreeCAD's box.

    Parameters
    ----------
    - cyl: the FreeCAD cylinder Part object
    - placement:
      - additional displacement of the box, the box' original
        placement is added to this if ``ignore_obj_placement`` is False.
      - box' placement otherwise.
    - ignore_obj_placement: cf. ``placement``.

    """
    return _urdf_generic_from_box(box, 'visual',
                                  placement, ignore_obj_placement)


def urdf_collision_from_box(
        box: 'PrimitivePy',
        placement: fc.Placement = fc.Placement(),
        ignore_obj_placement: bool = False) -> et.Element:
    """Return the xml element for collision for a FreeCAD's box.

    Parameters
    ----------
    - cyl: the FreeCAD cylinder Part object
    - placement:
      - additional displacement of the box, the box' original
        placement is added to this if ``ignore_obj_placement`` is False.
      - box' placement otherwise.
    - ignore_obj_placement: cf. ``placement``.

    """
    return _urdf_generic_from_box(box, 'collision',
                                  placement, ignore_obj_placement)


def _urdf_generic_from_sphere(
        sphere: 'PrimitivePy',
        generic: str,
        placement: fc.Placement = fc.Placement(),
        ignore_obj_placement: bool = False) -> et.Element:
    """Return the xml element for visual or collision for a FreeCAD's sphere.

    Parameters
    ----------
    - sphere: the FreeCAD sphere Part object
    - generic: {'visual', 'collision'}.
    - placement:
      - additional displacement of the sphere, the sphere's original
        placement is added to this if ``ignore_obj_placement`` is False.
      - sphere's placement otherwise.
    - ignore_obj_placement: cf. ``placement``.

    """
    if not is_sphere(sphere):
        raise RuntimeError("Argument must be a 'Part::Sphere'")
    parent = et.fromstring(f'<{generic}/>')
    if not ignore_obj_placement:
        placement = placement * sphere.Placement
    parent.append(urdf_geometry_sphere(
        sphere.Radius.getValueAs('m'),
        ))
    return parent


def urdf_visual_from_sphere(
        sphere: 'PrimitivePy',
        placement: fc.Placement = fc.Placement(),
        ignore_obj_placement: bool = False) -> et.Element:
    """Return the xml element for visual for a FreeCAD's sphere.

    Parameters
    ----------
    - sphere: the FreeCAD sphere Part object
    - placement:
      - additional displacement of the sphere, the sphere's original
        placement is added to this if ``ignore_obj_placement`` is False.
      - sphere's placement otherwise.
    - ignore_obj_placement: cf. ``placement``.

    """
    return _urdf_generic_from_sphere(sphere, 'visual',
                                     placement, ignore_obj_placement)


def urdf_collision_from_sphere(
        sphere: 'PrimitivePy',
        placement: fc.Placement = fc.Placement(),
        ignore_obj_placement: bool = False) -> et.Element:
    """Return the xml element for collision for a FreeCAD's sphere.

    Parameters
    ----------
    - sphere: the FreeCAD sphere Part object
    - placement:
      - additional displacement of the sphere, the sphere's original
        placement is added to this if ``ignore_obj_placement`` is False.
      - sphere's placement otherwise.
    - ignore_obj_placement: cf. ``placement``.

    """
    return _urdf_generic_from_sphere(sphere, 'collision',
                                     placement, ignore_obj_placement)


def _urdf_generic_from_cylinder(
        cyl: 'PrimitivePy',
        generic: str,
        placement: fc.Placement = fc.Placement(),
        ignore_obj_placement: bool = False) -> et.Element:
    """Return the xml element for visual or collision for a FreeCAD's cylinder.

    Parameters
    ----------
    - cyl: the FreeCAD cylinder Part object
    - generic: {'visual', 'collision'}.
    - placement:
      - additional displacement of the cylinder, the cylinder's original
        placement is added to this if ``ignore_obj_placement`` is False.
      - cylinder's placement otherwise.
    - ignore_obj_placement: cf. ``placement``.

    """
    if not is_cylinder(cyl):
        raise RuntimeError("Argument must be a 'Part::Cylinder'")
    parent = et.fromstring(f'<{generic}/>')
    if not ignore_obj_placement:
        placement = placement * cyl.Placement
    center_placement = urdf_cylinder_placement_from_object(cyl, placement)
    parent.append(urdf_origin_from_placement(center_placement))
    parent.append(urdf_geometry_cylinder(
        cyl.Radius.getValueAs('m'),
        cyl.Height.getValueAs('m'),
        ))
    return parent


def urdf_visual_from_cylinder(
        cyl: 'PrimitivePy',
        placement: fc.Placement = fc.Placement(),
        ignore_obj_placement: bool = False) -> et.Element:
    """Return the xml element for visual for a FreeCAD's cylinder.

    Parameters
    ----------
    - cyl: the FreeCAD cylinder Part object
    - placement:
      - additional displacement of the cylinder, the cylinder's original
        placement is added to this if ``ignore_obj_placement`` is False.
      - cylinder's placement otherwise.
    - ignore_obj_placement: cf. ``placement``.

    """
    return _urdf_generic_from_cylinder(cyl, 'visual',
                                       placement, ignore_obj_placement)


def urdf_collision_from_cylinder(
        cyl: 'PrimitivePy',
        placement: fc.Placement = fc.Placement(),
        ignore_obj_placement: bool = False) -> et.Element:
    """Return the xml element for collision for a FreeCAD's cylinder.

    Parameters
    ----------
    - cyl: the FreeCAD cylinder Part object
    - placement:
      - additional displacement of the cylinder, the cylinder's original
        placement is added to this if ``ignore_obj_placement`` is False.
      - cylinder's placement otherwise.
    - ignore_obj_placement: cf. ``placement``.

    """
    return _urdf_generic_from_cylinder(cyl, 'collision',
                                       placement, ignore_obj_placement)


def urdf_geometry_mesh(mesh_name: str, package_name: str) -> et.Element:
    """Return an xml element 'geometry' with a mesh.

    Parameters
    ----------
    - mesh_name: name of the mesh file without directory, so that the final
        mesh reference is `package://{package_name}/meshes/{mesh_name}`.
    - package_name: name of the ROS package.

    """
    geometry = et.fromstring('<geometry/>')
    geometry.append(et.fromstring(
        f'<mesh filename="package://{package_name}/meshes/{mesh_name}" />'))
    return geometry


def _urdf_generic_mesh(
        obj: fc.DocumentObject,
        mesh_name: str,
        package_name: str,
        generic: str,
        placement: fc.Placement = fc.Placement()) -> et.Element:
    """Return the xml element for visual or collision mesh for a FreeCAD object.

    The URDF just contains a reference to the object.
    The mesh is not exported.

    Parameters
    ----------
    - obj: the FreeCAD object.
    - mesh_name: name of the mesh file without directory, so that the final
        mesh reference is `package://{package_name}/meshes/{mesh_name}`.
    - package_name: name of the ROS package.
    - generic: {'visual', 'collision'}.
    - placement: additional displacement of the mesh, the mesh's
        original placement is added to this.

    """
    if (not placement) and (not has_placement(obj)):
        raise RuntimeError("Argument must be a FreeCAD object with a 'Placement' attribute")
    parent = et.fromstring(f'<{generic}/>')
    # TODO: handle links
    parent.append(et.Comment(xml_comment(obj.Label)))
    parent.append(urdf_origin_from_placement(placement))  # TODO: handle links
    parent.append(urdf_geometry_mesh(mesh_name, package_name))
    return parent


def urdf_visual_mesh(
        obj: fc.DocumentObject,
        mesh_name: str,
        package_name: str,
        placement: fc.Placement = fc.Placement()) -> et.Element:
    """Return the xml element for visual mesh for a FreeCAD object.

    The URDF just contains a reference to the object.
    The mesh is not exported.

    Parameters
    ----------
    - obj: the FreeCAD object.
    - mesh_name: name of the mesh file without directory, so that the final
        mesh reference is `package://{package_name}/meshes/{mesh_name}`.
    - package_name: name of the ROS package.
    - placement: additional displacement of the mesh, the mesh's
        original placement is added to this.

    """
    return _urdf_generic_mesh(obj, mesh_name, package_name, 'visual', placement)


def urdf_collision_mesh(
        obj: fc.DocumentObject,
        mesh_name: str,
        package_name: str,
        placement: fc.Placement = fc.Placement()) -> et.Element:
    """Return the xml element for collision mesh for a FreeCAD object.

    The URDF just contains a reference to the mesh.
    The mesh is not exported.

    Parameters
    ----------
    - obj: the FreeCAD object.
    - mesh_name: name of the mesh file without directory, so that the final
        mesh reference is `package://{package_name}/meshes/{mesh_name}`.
    - package_name: name of the ROS package.
    - placement: additional displacement of the mesh, the mesh's
        original placement is added to this.

    """
    return _urdf_generic_mesh(obj, mesh_name, package_name, 'collision', placement)


def _urdf_generic_from_object(
        obj: fc.DocumentObject,
        generic: str,
        mesh_name: Optional[str] = None,
        package_name: Optional[str] = None,
        placement: fc.Placement = fc.Placement(),
        ) -> et.ElementTree:
    """Return the xml element for visual or collision for a FreeCAD object.

    For meshes, the URDF just contains a reference to the object, the mesh is
    not exported.

    Parameters
    ----------
    - obj: the FreeCAD object.
    - mesh_name: name of the mesh file without directory, so that the final
        mesh reference is `package://{package_name}/meshes/{mesh_name}`.
    - package_name: name of the ROS package.
    - generic: {'visual', 'collision'}.
    - placement: optional additional displacement. The object's placement will
        be added to this.

    """
    if is_box(obj):
        return _urdf_generic_from_box(obj, generic, placement)
    elif is_sphere(obj):
        return _urdf_generic_from_sphere(obj, generic, placement)
    elif is_cylinder(obj):
        return _urdf_generic_from_cylinder(obj, generic, placement)
    elif mesh_name and package_name:
        return _urdf_generic_mesh(obj, mesh_name, package_name, generic, placement)


def urdf_visual_from_object(
        obj: fc.DocumentObject,
        mesh_name: Optional[str] = None,
        package_name: Optional[str] = None,
        placement: Optional[fc.Placement] = None,
        ) -> et.ElementTree:
    """Return the xml element for visual or collision for a FreeCAD object.

    For meshes, the URDF just contains a reference to the object, the mesh is
    not exported.

    Parameters
    ----------
    - obj: the FreeCAD object.
    - mesh_name: name of the mesh file without directory, so that the final
        mesh reference is `package://{package_name}/meshes/{mesh_name}`.
    - package_name: name of the ROS package.
    - placement: optional placement of the mesh relative to the URDF link.
        If not given, obj.Placement will be used.

    """
    return _urdf_generic_from_object(obj, 'visual', mesh_name, package_name, placement)


def urdf_collision_from_object(
        obj: fc.DocumentObject,
        mesh_name: Optional[str] = None,
        package_name: Optional[str] = None,
        placement: Optional[fc.Placement] = None,
        ) -> et.ElementTree:
    """Return the xml element for collision or collision for a FreeCAD object.

    For meshes, the URDF just contains a reference to the object, the mesh is
    not exported.

    Parameters
    ----------
    - obj: the FreeCAD object.
    - mesh_name: name of the mesh file without directory, so that the final
        mesh reference is `package://{package_name}/meshes/{mesh_name}`.
    - package_name: name of the ROS package.
    - placement: optional placement of the mesh relative to the URDF link.
        If not given, obj.Placement will be used.

    """
    return _urdf_generic_from_object(obj, 'collision', mesh_name, package_name, placement)


def export_group_with_lcs(group: fc.DocumentObjectGroup, package_path: Union[str, Path]) -> bool:
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
                or (linked_object.TypeId not in ('PartDesign::CoordinateSystem', 'App::Part'))):
            # linked_object is not a coordinate system, cannot handle it.
            warn(f'Linked object of "{label_or(fc_link)}" is not a coordinate system or part, ignoring')
            continue
    return True
