"""Utility function specific to this workbench."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable, List, Optional, Protocol, Union

import FreeCAD as fc

from . import wb_globals
from .freecad_utils import get_param
from .freecad_utils import is_box
from .freecad_utils import is_cylinder
from .freecad_utils import is_sphere
from .freecad_utils import message
from .freecad_utils import set_param
from .freecad_utils import warn
from .ros.utils import get_ros_workspace_from_file
from .ros.utils import without_ros_workspace
from .utils import attr_equals
from .utils import values_from_string

# Stubs and typing hints.
from .attached_collision_object import AttachedCollisionObject as CrossAttachedCollisionObject  # A Cross::AttachedCollisionObject, i.e. a DocumentObject with Proxy "AttachedCollisionObject". # noqa: E501
from .joint import Joint as CrossJoint  # A Cross::Joint, i.e. a DocumentObject with Proxy "Joint". # noqa: E501
from .link import Link as CrossLink  # A Cross::Link, i.e. a DocumentObject with Proxy "Link". # noqa: E501
from .robot import Robot as CrossRobot  # A Cross::Robot, i.e. a DocumentObject with Proxy "Robot". # noqa: E501
from .workcell import Workcell as CrossWorkcell  # A Cross::Workcell, i.e. a DocumentObject with Proxy "Workcell". # noqa: E501
from .xacro_object import XacroObject as CrossXacroObject  # A Cross::XacroObject, i.e. a DocumentObject with Proxy "XacroObject". # noqa: E501
DO = fc.DocumentObject
CrossBasicElement = Union[CrossJoint, CrossLink]
CrossObject = Union[CrossJoint, CrossLink, CrossRobot, CrossXacroObject, CrossWorkcell]
DOList = List[DO]

MOD_PATH = Path(fc.getUserAppDataDir()) / 'Mod/freecad.cross'
RESOURCES_PATH = MOD_PATH / 'resources'
UI_PATH = RESOURCES_PATH / 'ui'
ICON_PATH = RESOURCES_PATH / 'icons'


class SupportsStr(Protocol):
    def __str__(self) -> str:
        ...


@dataclass
class XacroObjectAttachment:
    xacro_object: CrossXacroObject
    attached_to: Optional[CrossLink] = None
    attached_by: Optional[CrossJoint] = None
    # ROS object `attached_to` belongs to.
    attachement_ros_object: Optional[CrossXacroObject | CrossRobot] = None


def get_workbench_param(
    param_name: str,
    default: Any,
) -> Any:
    """Return the value of a workbench parameter."""
    param_grp = fc.ParamGet(
            f'User parameter:BaseApp/Preferences/Mod/{wb_globals.PREFS_CATEGORY}',
    )
    return get_param(param_grp, param_name, default)


def set_workbench_param(
    param_name: str,
    value: Any,
) -> None:
    """Set the value of a workbench parameter."""
    param_grp = fc.ParamGet(
            f'User parameter:BaseApp/Preferences/Mod/{wb_globals.PREFS_CATEGORY}',
    )
    set_param(param_grp, param_name, value)


def is_attached_collision_object(obj: DO) -> bool:
    """Return True if the object is a Cross::AttachedCollisionObject."""
    return _has_ros_type(obj, 'Cross::AttachedCollisionObject')


def is_robot(obj: DO) -> bool:
    """Return True if the object is a Cross::Robot."""
    return _has_ros_type(obj, 'Cross::Robot')


def is_link(obj: DO) -> bool:
    """Return True if the object is a Cross::Link."""
    return _has_ros_type(obj, 'Cross::Link')


def is_joint(obj: DO) -> bool:
    """Return True if the object is a Cross::Link."""
    return _has_ros_type(obj, 'Cross::Joint')


def is_xacro_object(obj: DO) -> bool:
    """Return True if the object is a Cross::Xacro."""
    return _has_ros_type(obj, 'Cross::XacroObject')


def is_workcell(obj: DO) -> bool:
    """Return True if the object is a Cross::Workcell."""
    return _has_ros_type(obj, 'Cross::Workcell')


def is_planning_scene(obj: DO) -> bool:
    """Return True if the object is a Cross::PlanningScene."""
    return _has_ros_type(obj, 'Cross::PlanningScene')


def is_simple_joint(obj: DO) -> bool:
    """Return True if prismatic, revolute, or continuous."""
    return (
        is_joint(obj)
        and (obj.Type in ['prismatic', 'revolute', 'continuous'])
    )


def is_primitive(obj: DO) -> bool:
    """Return True if the object is a 'Part::{Box,Cylinder,Sphere}'."""
    return is_box(obj) or is_sphere(obj) or is_cylinder(obj)


def is_robot_selected() -> bool:
    """Return True if the first selected object is a Cross::Robot."""
    return is_selected_from_lambda(is_robot)


def is_joint_selected() -> bool:
    """Return True if the first selected object is a Cross::Joint."""
    return is_selected_from_lambda(is_joint)


def is_link_selected() -> bool:
    """Return True if the first selected object is a Cross::Link."""
    return is_selected_from_lambda(is_link)


def is_workcell_selected() -> bool:
    """Return True if the first selected object is a Cross::Workcell."""
    return is_selected_from_lambda(is_workcell)


def is_planning_scene_selected() -> bool:
    """Return True if the first selected object is a Cross::PlanningScene."""
    return is_selected_from_lambda(is_planning_scene)


def get_attached_collision_objects(objs: DOList) -> list[CrossAttachedCollisionObject]:
    """Return only the objects that are Cross::AttachedCollisionObject instances."""
    return [o for o in objs if is_attached_collision_object(o)]


def get_links(objs: DOList) -> list[CrossLink]:
    """Return only the objects that are Cross::Link instances."""
    return [o for o in objs if is_link(o)]


def get_joints(objs: DOList) -> list[CrossJoint]:
    """Return only the objects that are Cross::Joint instances."""
    return [o for o in objs if is_joint(o)]


def get_xacro_objects(objs: DOList) -> list[CrossXacroObject]:
    """Return only the objects that are Cross::XacroObject instances."""
    return [o for o in objs if is_xacro_object(o)]


def get_chains(
        links: list[CrossLink],
        joints: list[CrossJoint],
) -> list[list[CrossBasicElement]]:
    """Return the list of chains.

    A chain starts at the root link, alternates links and joints, and ends
    at the last joint of the chain.

    If the last element of a chain would be a joint, that chain is not
    considered.

    """
    # TODO: Make the function faster.
    base_links: list[CrossLink] = []
    tip_links: list[CrossLink] = []
    for link in links:
        if link.Proxy.may_be_base_link():
            base_links.append(link)
        if link.Proxy.is_tip_link():
            tip_links.append(link)
    if len(base_links) > 1:
        # At least two root links found, not supported.
        return []
    chains: list[list[CrossBasicElement]] = []
    for link in tip_links:
        chain = get_chain(link)
        if chain:
            chains.append(chain)
    return chains


def get_chain(link: CrossLink) -> list[CrossBasicElement]:
    """Return the chain from base link to link, excluded.

    The chain starts with the base link, then alternates a joint and a link.
    The last item is the joint that has `link` as child.

    """
    chain: list[CrossBasicElement] = []
    ref_joint = link.Proxy.get_ref_joint()
    if not ref_joint:
        # A root link.
        return [link]
    if not ref_joint.Parent:
        warn(f'Joint `{ros_name(ref_joint)}` has no parent', False)
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


def get_xacro_object_attachments(
        xacro_objects: list[CrossXacroObject],
        joints: list[CrossJoint],
) -> list[XacroObjectAttachment]:
    """Return attachment details of xacro objects."""
    attachments: list[XacroObjectAttachment] = []
    for xacro_object in xacro_objects:
        attachment = XacroObjectAttachment(xacro_object)
        attachments.append(attachment)
        if not hasattr(xacro_object, 'Proxy'):
            continue
        root_link = xacro_object.Proxy.root_link
        for joint in joints:
            if joint.Child == root_link:
                attachment.attached_by = joint
        if not attachment.attached_by:
            continue
        for xo in [x for x in xacro_objects if x is not xacro_object]:
            parent: str = attachment.attached_by.Parent
            if xo.Proxy.has_link(parent):
                attachment.attached_to = xo.Proxy.get_link(parent)
                attachment.attachement_ros_object = xo
    return attachments


def get_xacro_chains(
        xacro_objects: list[CrossXacroObject],
        joints: list[CrossJoint],
) -> list[list[XacroObjectAttachment]]:
    """Return the list of chains.

    A chain starts at a xacro object that is not attached to any other xacro
    object and contains all xacro objects that form an attachment chain, up to
    the xacro object to which no other xacro object is attached.

    """
    def is_parent(
        xacro_object: CrossXacroObject,
        attachments: list[XacroObjectAttachment],
    ) -> bool:
        for attachment in attachments:
            if attachment.attachement_ros_object is xacro_object:
                return True
        return False

    def get_chain(
        xacro_object: CrossXacroObject,
        attachments: list[XacroObjectAttachment],
    ) -> list[XacroObjectAttachment]:
        for attachment in attachments:
            if attachment.xacro_object is xacro_object:
                break
        if attachment.attachement_ros_object:
            return (
                get_chain(attachment.attachement_ros_object, attachments)
                + [attachment]
            )
        else:
            return [attachment]

    attachments = get_xacro_object_attachments(xacro_objects, joints)
    tip_xacros: list[CrossXacroObject] = []
    for attachment in attachments:
        if not is_parent(attachment.xacro_object, attachments):
            tip_xacros.append(attachment.xacro_object)
    chains: list[list[XacroObjectAttachment]] = []
    for xacro_object in tip_xacros:
        chains.append(get_chain(xacro_object, attachments))
    return chains


def ros_name(obj: DO) -> str:
    """Return in order obj.Label2, obj.Label, obj.Name."""
    if ((
        not hasattr(obj, 'isDerivedFrom')
        or (not obj.isDerivedFrom('App::DocumentObject'))
    )):
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


def get_rel_and_abs_path(path: str) -> tuple[str, Path]:
    """Return the path relative to src and the absolute path.

    Return the path relative to the `src` folder in the  ROS workspace and
    the absolute path to the file.
    The input path can be a path relative to the ROS workspace or an absolute
    path.
    If the input path is relative, it is returned as-is.

    For example, if the file path is `my_package/file.py`, return
    `('my_package/file.py', Path('/home/.../ros2_ws/src/my_package/file.py`)`,
    supposing that `my_package` is a ROS package in the ROS workspace
    `/home/.../ros2_ws`.

    The existence of the given path is not checked.

    If `wb_globals.g_ros_workspace` is not set, ask the user to configure it.

    """
    # Import here to avoid circular import.
    from .wb_gui_utils import get_ros_workspace

    if not wb_globals.g_ros_workspace.name:
        ws = get_ros_workspace()
        wb_globals.g_ros_workspace = ws
    p = without_ros_workspace(path)
    full_path = (
        wb_globals.g_ros_workspace.expanduser() / 'src' / p
    )
    return p, full_path


def remove_ros_workspace(path) -> str:
    """Modify `path` to remove $ROS_WORKSPACE/src.

    Return the path relative to `wb_globals.g_ros_workspace / 'src'`.
    Doesn't use any ROS functionalities and, thus, doesn't support finding any
    underlay workspace.
    Modify `wb_globals.g_ros_workspace` if a workspace was guessed from `path`.

    """
    rel_path = without_ros_workspace(path)
    if wb_globals.g_ros_workspace.samefile(Path()):
        # g_ros_workspace was not defined yet.
        ws = get_ros_workspace_from_file(path)
        if not ws.samefile(Path()):
            # A workspace was found.
            wb_globals.g_ros_workspace = ws
            message(
                'ROS workspace was set to'
                f' {wb_globals.g_ros_workspace},'
                ' change if not correct.'
                ' Note that packages in this workspace will NOT be'
                ' found, though, but only by launching FreeCAD from a'
                ' sourced workspace',
                True,
            )
            rel_path = without_ros_workspace(path)
    return rel_path


def export_templates(
        template_files: list[str],
        package_parent: [Path | str],
        **keys: SupportsStr,
) -> None:
    """Export generated files.

    Parameters
    ----------

    - template_files: list of files to export, relative to
                      `RESOURCES_PATH/templates`, where RESOURCES_PATH is the
                      directory `resources` of this workbench.
    - package_name: the directory containing the directory called
                    `package_name`, usually "$ROS_WORKSPACE/src".
    - keys: dictionary of replacement string in templates.
            - package_name (compulsory): name of the ROS package and its containing
                                         directory.
            - urdf_file: name of the URDF/xacro file without directory.
                         Used in `launch/display.launch.py`.
            - fixed_frame: parameter "Global Options / Fixed Frame" in RViz.

    """
    try:
        package_name: str = keys['package_name']
    except KeyError:
        raise RuntimeError('Parameter "package_name" must be given')

    package_parent = Path(package_parent)
    meshes_dir = (
        'meshes '
        if _has_meshes_directory(package_parent, package_name)
        else ''
    )
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
        is_type_fun: Callable[DO, bool],
) -> bool:
    """Return True if the first selected object meets the given criteria.

    Return `is_type_fun("first_selected_object")`.

    """
    if not fc.GuiUp:
        return False
    if fc.activeDocument() is None:
        return False
    import FreeCADGui as fcgui
    sel = fcgui.Selection.getSelection()
    if not sel:
        return False
    return is_type_fun(sel[0])


def is_name_used(
        obj: CrossObject,
        container_obj: [CrossRobot | CrossWorkcell],
) -> bool:
    if not is_robot(container_obj) or is_workcell(container_obj):
        raise RuntimeError(
            'Second argument must be a'
            ' CrossRobot or a CrossWorkbench',
        )
    obj_name = ros_name(obj)
    if ((obj is not container_obj)
            and (ros_name(container_obj) == obj_name)):
        return True
    if is_robot(container_obj) and hasattr(container_obj, 'Proxy'):
        for link in container_obj.Proxy.get_links():
            if ((obj is not link)
                    and (ros_name(link) == obj_name)):
                return True
    elif is_workcell(container_obj) and hasattr(container_obj, 'Proxy'):
        for xacro_object in container_obj.Proxy.get_xacro_objects():
            if ((obj is not xacro_object)
                    and (ros_name(xacro_object) == obj_name)):
                return True
            if hasattr(xacro_object, 'Proxy'):
                robot = xacro_object.Proxy.get_robot()
                if robot and is_name_used(obj, robot):
                    return True
    if hasattr(container_obj, 'Proxy'):
        for joint in container_obj.Proxy.get_joints():
            if ((obj is not joint)
                    and (ros_name(joint) == obj_name)):
                return True
    return False


def placement_from_pose_string(pose: str) -> fc.Placement:
    """Return a FreeCAD Placement from a string pose `x, y, z; qw, qx, qy, qz`.

    The pose is a string of 2 semi-colon-separated groups: 3 floats
    representing the position in meters and 4 floats for the orientation
    as quaternions qw, qx, qy, qz. The values in a group can be separated
    by commas or spaces.
    A string of 7 floats is also accepted.

    """
    if ';' in pose:
        position_str, orientation_str = pose.split(';')
        try:
            x, y, z = values_from_string(position_str)
            qw, qx, qy, qz = values_from_string(orientation_str)
        except ValueError:
            raise ValueError('Pose must have the format `x, y, z; qw, qx, qy, qz`')
    else:
        try:
            x, y, z, qw, qx, qy, qz = values_from_string(pose)
        except ValueError:
            raise ValueError(
                    'Pose must have the format `x, y, z; qw, qx, qy, qz`'
                    ' or `x, y, z, qw, qx, qy, qz`',
            )
    ros_to_freecad_factor = 1000.0  # ROS uses meters, FreeCAD uses mm.
    return fc.Placement(
        fc.Vector(x, y, z) * ros_to_freecad_factor,
        fc.Rotation(qw, qx, qy, qz),
    )
