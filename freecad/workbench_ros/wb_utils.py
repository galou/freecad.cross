"""Utility function specific to this workbench."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Iterable, Optional

import FreeCAD as fc

from .freecad_utils import is_box
from .freecad_utils import is_cylinder
from .freecad_utils import is_sphere
from .freecad_utils import label_or
from .freecad_utils import warn
from .utils import attr_equals

# Typing hints.
DO = fc.DocumentObject
RosJoint = DO  # A Ros::Joint, i.e. a DocumentObject with Proxy "Joint".
RosLink = DO  # A Ros::Link, i.e. a DocumentObject with Proxy "Link".
RosRobot = DO  # A Ros::Robot, i.e. a DocumentObject with Proxy "Robot".
RosXacroObject = DO  # Ros::XacroObject, i.e. DocumentObject with Proxy "XacroObject".
DOList = Iterable[DO]

MOD_PATH = Path(fc.getUserAppDataDir()) / 'Mod/freecad.workbench_ros'
RESOURCES_PATH = MOD_PATH / 'resources'
UI_PATH = RESOURCES_PATH / 'ui'
ICON_PATH = RESOURCES_PATH / 'icons'


def is_robot(obj: DO) -> bool:
    """Return True if the object is a Ros::Robot."""
    return _has_ros_type(obj, 'Ros::Robot')


def is_link(obj: DO) -> bool:
    """Return True if the object is a Ros::Link."""
    return _has_ros_type(obj, 'Ros::Link')


def is_joint(obj: DO) -> bool:
    """Return True if the object is a Ros::Link."""
    return _has_ros_type(obj, 'Ros::Joint')


def is_xacro_object(obj: DO) -> bool:
    """Return True if the object is a Ros::Xacro."""
    return _has_ros_type(obj, 'Ros::XacroObject')


def is_workcell(obj: DO) -> bool:
    """Return True if the object is a Ros::Workcell."""
    return _has_ros_type(obj, 'Ros::Workcell')


def is_simple_joint(obj: DO) -> bool:
    """Return True if prismatic, revolute, or continuous."""
    return (is_joint(obj)
            and (obj.Type in ['prismatic', 'revolute', 'continuous']))


def is_primitive(obj: DO) -> bool:
    """Return True if the object is a 'Part::{Box,Cylinder,Sphere}'."""
    return is_box(obj) or is_sphere(obj) or is_cylinder(obj)


def is_robot_selected() -> bool:
    """Return True if the first selected object is a Ros::Object."""
    return is_selected_from_lambda(is_robot)


def is_workcell_selected() -> bool:
    """Return True if the first selected object is a Ros::Workcell."""
    return is_selected_from_lambda(is_workcell)


def get_links(objs: DOList) -> list[RosLink]:
    """Return only the objects that are Ros::Link instances."""
    return [o for o in objs if is_link(o)]


def get_joints(objs: DOList) -> list[RosJoint]:
    """Return only the objects that are Ros::Joint instances."""
    return [o for o in objs if is_joint(o)]


def get_xacro_objects(objs: DOList) -> list[RosXacroObject]:
    """Return only the objects that are Ros::XacroObject instances."""
    return [o for o in objs if is_xacro_object(o)]


def get_chains(
        links: list[RosLink],
        joints: list[RosJoint],
        ) -> list[DOList]:
    base_links: list[RosLink] = []
    tip_links: list[RosLink] = []
    for link in links:
        if link.Proxy.may_be_base_link():
            base_links.append(link)
        if link.Proxy.is_tip_link():
            tip_links.append(link)
    if len(base_links) > 1:
        # At least two root links found, not supported.
        return []
    chains: list[DOList] = []
    for link in tip_links:
        chain = get_chain(link)
        if chain:
            chains.append(chain)
    return chains


def get_chain(link: RosLink) -> DOList:
    """Return the chain from base_link to link, excluded.

    The chain start with the base link, then alternates a joint and a link. The
    last item is the joint that has `link` as child.

    """
    chain: DOList = []
    ref_joint = link.Proxy.get_ref_joint()
    if not ref_joint:
        # A root link.
        return [link]
    if not ref_joint.Parent:
        warn(f'{label_or(ref_joint)} has no parent', True)
        # Return only ref_joint to indicate an error.
        return [ref_joint]
    robot = ref_joint.Proxy.get_robot()
    subchain = get_chain(robot.Proxy.get_link(ref_joint.Parent))
    if subchain and is_joint(subchain[0]):
        # Propagate the error of missing joint.Parent.
        return subchain
    chain += subchain + [ref_joint] + [link]
    return chain


def is_subchain(subchain: DOList, chain: DOList) -> bool:
    """Return True if all items in `subchain` are in `chain`."""
    for link_or_joint in subchain:
        if link_or_joint not in chain:
            return False
    return True


def ros_name(obj: DO):
    """Return in order obj.Label2, obj.Label, obj.Name."""
    if ((not hasattr(obj, 'isDerivedFrom')
         or (not obj.isDerivedFrom('App::DocumentObject')))):
        return 'not_a_FreeCAD_object'
    if obj.Label2:
        return obj.Label2
    if obj.Label:
        return obj.Label
    return obj.Name


def get_valid_urdf_name(name: str) -> str:
    if not name:
        return 'no_name'
    # TODO: special XML characters must be escaped.
    return name.replace('"', '&quot;')


def export_templates(
        template_files: list[str],
        package_parent: [Path | str],
        **keys: dict[str, str],
        ) -> None:
    """Export generated files.

    Parameters
    ----------

    - template_files: list of files to export, relative to
                      `RESOURCES_PATH/templates`, where RESOURCES_PATH is the
                      directory `resources` of this workbench.
    - package_name: the directory containing the directory called
                    `package_name`, usually "$ROS_WORKSPACE/src".
    - keys: dictionnary of replacement string in templates.
            - package_name (compulsary): name of the ROS package and its containing
                                         directory.
            - urdf_file: name of the URDF/xacro file without directory.
                         Used in `launch/display.launch.py`.

    """
    try:
        package_name: str = keys['package_name']
    except KeyError:
        raise RuntimeError('Parameter "package_name" must be given')

    package_parent = Path(package_parent)
    meshes_dir = ('meshes '
                  if _has_meshes_directory(package_parent, package_name)
                  else '')
    for f in template_files:
        template_file_path = RESOURCES_PATH / 'templates' / f
        template = template_file_path.read_text()
        txt = template.format(meshes_dir=meshes_dir, **keys)
        output_path = package_parent / package_name / f
        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_text(txt)


def _has_ros_type(obj: DO, type_: str) -> bool:
    """Return True if the object is an object from this workbench."""
    if not isinstance(obj, DO):
        return False
    return attr_equals(obj, '_Type', type_)


def _has_meshes_directory(
        package_parent: [Path | str],
        package_name: str,
        ) -> bool:
    """Return True if the directory "meshes" exists in the package."""
    meshes_directory = Path(package_parent) / package_name / 'meshes'
    return meshes_directory.exists()


def is_selected_from_lambda(
        is_type_fun: Callable[[DO], bool],
        ) -> bool:
    if not fc.GuiUp:
        return False
    if fc.activeDocument() is None:
        return False
    import FreeCADGui as fcgui
    sel = fcgui.Selection.getSelection()
    if not sel:
        return False
    return is_type_fun(sel[0])
