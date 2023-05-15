from __future__ import annotations

from math import radians
from pathlib import Path
from typing import Iterable, Optional
import xml.etree.ElementTree as et

import FreeCAD as fc

from . import wb_globals
from .freecad_utils import add_property
from .freecad_utils import error
from .freecad_utils import get_properties_of_category
from .freecad_utils import get_valid_property_name
from .freecad_utils import label_or
from .freecad_utils import message
from .freecad_utils import warn
from .ros_utils import get_ros_workspace_from_file
from .ros_utils import split_package_path
from .ros_utils import without_ros_workspace
from .utils import get_valid_filename
from .utils import grouper
from .utils import save_xml
from .utils import warn_unsupported
from .wb_gui_utils import get_ros_workspace
from .wb_utils import ICON_PATH
from .wb_utils import export_templates
from .wb_utils import get_chains
from .wb_utils import get_joints
from .wb_utils import get_links
from .wb_utils import get_valid_urdf_name
from .wb_utils import is_joint
from .wb_utils import is_link
from .wb_utils import is_robot
from .wb_utils import ros_name

# Typing hints.
DO = fc.DocumentObject
DOList = Iterable[DO]
VPDO = 'FreeCADGui.ViewProviderDocumentObject'
AppLink = DO  # TypeId == 'App::Link'.
RosLink = DO  # A Ros::Link, i.e. a DocumentObject with Proxy "Link".
RosJoint = DO  # A Ros::Joint, i.e. a DocumentObject with Proxy "Joint".
RosRobot = DO  # A Ros::Robot, i.e. a DocumentObject with Proxy "Robot".


def _existing_link(link: RosLink, obj: DO, lod: str) -> Optional[AppLink]:
    """Return the link to obj if it exists in link.Group with the given lod.

    Return the link to obj if it exists in link.Group with the given level of
    detail.

    Parameters
    ----------
    - link: a FreeCAD object of type Ros::Link.
    - obj: a FreeCAD object of which to search a link.
    - lod: string describing the level of details, {'real', 'visual',
            'collision'}.

    """
    for linked_lod in link.Group:
        if ((linked_lod.LinkedObject is obj)
                and linked_lod.Name.startswith(lod)):
            return linked_lod


def _add_links_lod(
        link: RosLink,
        objects: DOList,
        lod: str,
        ) -> list[AppLink]:
    """Add a level of detail as links to real, visual or collision elements.

    Return the full list of linked objects (existing + created).

    Parameters
    ----------
    - link: a FreeCAD object of type Ros::Link.
    - objects: the list of objects to potentially add.
    - lod: string describing the level of details, {'real', 'visual',
            'collision'}.

    """
    doc = link.Document
    old_and_new_objects: DOList = []
    for o in objects:
        link_to_o = _existing_link(link, o, lod)
        if link_to_o is not None:
            if link_to_o.LinkPlacement != link.Placement:
                # Avoid recursive recompute.
                link_to_o.LinkPlacement = link.Placement
            old_and_new_objects.append(link_to_o)
            continue
        name = f'{lod}_{link.Label}_'
        lod_link = doc.addObject('App::Link', name)
        lod_link.Label = name
        if lod_link.LinkPlacement != link.Placement:
            # Avoid recursive recompute.
            lod_link.LinkPlacement = link.Placement
        lod_link.setLink(o)
        lod_link.adjustRelativeLinks(link)
        link.addObject(lod_link)
        old_and_new_objects.append(lod_link)
    return old_and_new_objects


def _add_joint_variable(
        robot: RosRobot,
        joint: RosJoint,
        category: str,
        ) -> str:
    """Add a property for the actuator value to `robot` and return its name.

    Add a property starting with "joint.Label" to represent the actuation
    value of a joint. Supported types are 'prismatic', 'revolute', and
    'continuous'. There is no actuation value for mimicking joints.

    """
    if not is_joint(joint):
        warn(f'Wrong object type. {joint.Name} ({joint.Label})'
             ' is not a ROS::Joint')
        return
    if joint.Mimic:
        # No acuator for mimic joints.
        return ''
    if joint.Type == 'prismatic':
        unit = 'mm'
    elif joint.Type in ['revolute', 'continuous']:
        unit = 'deg'
    else:
        # Non-simple joints not supported yet.
        return ''
    rname = ros_name(joint)
    # e.g. name_candidate = "q0_deg" or "q0".
    name_candidate = f'{rname}{f"_{unit}" if unit else ""}'
    var_name = get_valid_property_name(name_candidate)
    # e.g. help_txt = "q0 in deg" or "q0".
    label = joint.Label
    id_ = rname if rname == label else f'{rname} ({label})'
    help_txt = f'{id_}{f" in {unit}" if unit else ""}'
    _, used_var_name = add_property(robot,
                                    'App::PropertyFloat', var_name,
                                    category, help_txt)
    value: Optional[float] = None
    if joint.Type == 'prismatic':
        value = robot.getPropertyByName(var_name) * 0.001
    elif joint.Type == 'revolute':
        value = radians(robot.getPropertyByName(var_name))
    if ((value is not None)
            and (joint.Position != value)
            and (not joint.Mimic)):
        # Avoid recursive recompute.
        joint.Position = value
    return used_var_name


class Robot:
    """The Robot proxy."""

    # The member is often used in workbenches, particularly in the Draft
    # workbench, to identify the object type.
    Type = 'Ros::Robot'

    # Name of the category (or group) for the joint values, which are saved as
    # properties of `self.robot`.
    _category_of_joint_values = 'JointValues'

    def __init__(self, obj: RosRobot):
        obj.Proxy = self
        self.robot = obj

        self.init_properties(obj)

    def init_properties(self, obj: RosRobot):
        add_property(obj, 'App::PropertyString', '_Type', 'Internal',
                     'The type')
        obj.setPropertyStatus('_Type', ['Hidden', 'ReadOnly'])
        obj._Type = self.Type

        # Managed in self.reset_group().
        obj.setPropertyStatus('Group', 'ReadOnly')

        add_property(obj, 'App::PropertyPath', 'OutputPath', 'Export',
                     'The path to the ROS package to export files to,'
                     ' relative to $ROS_WORKSPACE/src')

    def execute(self, obj: RosRobot) -> None:
        self.cleanup_group()
        self.set_joint_enum()
        self.add_joint_variables()
        self.compute_poses()
        self._fix_lost_fc_links()
        self.reset_group()

    def onChanged(self, obj: RosRobot, prop: str) -> None:
        if prop in ['Group']:
            self.execute(obj)
        if prop == 'OutputPath':
            self._remove_ros_workspace(obj)

    def onDocumentRestored(self, obj):
        """Restore attributes because __init__ is not called on restore."""
        obj.Proxy = self
        self.robot = obj
        self.init_properties(obj)

    def __getstate__(self):
        return self.Type,

    def __setstate__(self, state):
        if state:
            self.Type, = state

    def compute_poses(self) -> None:
        """Compute the pose of all joints and links.

        Compute the pose of all joints and links relative the the robot
        root link.

        """
        chains = self.get_chains()
        for chain in chains:
            placement = fc.Placement()
            for link, joint in grouper(chain, 2):
                # TODO: some links and joints are already placed, re-use.
                if hasattr(link, 'MountedPlacement'):
                    new_link_placement = placement * link.MountedPlacement
                else:
                    # TODO: find out why `MountedPlacement` is not set.
                    new_link_placement = link.Placement
                if link.Placement != new_link_placement:
                    # Avoid recursive recompute.
                    link.Placement = new_link_placement
                if joint:
                    new_joint_placement = placement * joint.Origin
                    if joint.Placement != new_joint_placement:
                        # Avoid recursive recompute.
                        joint.Placement = new_joint_placement
                    # For next link.
                    placement = (new_joint_placement
                                 * joint.Proxy.get_actuation_placement())

    def get_links(self) -> list[RosLink]:
        if ((not hasattr(self, 'robot')) or (not hasattr(self.robot, 'Group'))):
            return []
        return get_links(self.robot.Group)

    def get_joints(self) -> list[RosJoint]:
        if ((not hasattr(self, 'robot')) or (not hasattr(self.robot, 'Group'))):
            return []
        return get_joints(self.robot.Group)

    def get_link(self, name: str) -> Optional[RosLink]:
        if not name:
            # Shortcut.
            return
        for link in self.get_links():
            if ros_name(link) == name:
                return link

    def get_joint(self, name: str) -> Optional[RosJoint]:
        if not name:
            # Shortcut.
            return
        for joint in self.get_joints():
            if ros_name(joint) == name:
                return joint

    def get_chains(self) -> list[DOList]:
        if not hasattr(self, 'robot'):
            return []
        if not is_robot(self.robot):
            warn(f'{label_or(self.robot)} is not a ROS::Robot', True)
            return []
        links = self.get_links()
        joints = self.get_joints()
        return get_chains(links, joints)

    def reset_group(self) -> None:
        if ((not hasattr(self, 'robot'))
                or (not hasattr(self.robot, 'ViewObject'))
                or (not hasattr(self.robot.ViewObject, 'ShowReal'))
                or (not hasattr(self.robot.ViewObject, 'ShowVisual'))
                or (not hasattr(self.robot.ViewObject, 'ShowCollision'))):
            return

        links = self.get_links()  # ROS links.

        # Add objects from selected components.
        all_linked_objects: list[AppLink] = []
        if self.robot.ViewObject.ShowReal:
            for link in links:
                all_linked_objects += _add_links_lod(link, link.Real, 'real')

        if self.robot.ViewObject.ShowVisual:
            for link in links:
                all_linked_objects += _add_links_lod(link, link.Visual, 'visual')

        if self.robot.ViewObject.ShowCollision:
            for link in links:
                all_linked_objects += _add_links_lod(link, link.Collision, 'collision')

        # List of linked objects from all Ros::Link in robot.Group.
        current_linked_objects: list[AppLink] = []
        for link in links:
            for o in link.Group:
                current_linked_objects.append(o)

        # Remove objects that do not belong to `all_linked_objects`.
        objects_to_remove = set(current_linked_objects) - set(all_linked_objects)
        for o in objects_to_remove:
            self.robot.Document.removeObject(o.Name)
        # TODO?: doc.recompute() if objects_to_remove or (set(current_linked_objects) != set(all_linked_objects))

    def cleanup_group(self) -> DO:
        """Remove the last object not supported by ROS::Robot.

        Recursion provoked by modifying `Group` will take care of removing
        the remaining unsupported objects.

        """
        if ((not hasattr(self, 'robot'))
                or (not is_robot(self.robot))):
            return
        for o in self.robot.Group[::-1]:
            if is_link(o) or is_joint(o):
                # Supported.
                continue
            warn_unsupported(o, by='ROS::Robot', gui=True)
            return self.robot.removeObject(o)

    def set_joint_enum(self) -> None:
        """Set the enum for Child and Parent of all joints."""
        # We add the empty string to show that the child or parent
        # was not set yet.
        links: list[str] = ['']
        for link in self.get_links():
            links.append(ros_name(link))
        for joint in self.get_joints():
            # Implementation note: setting to a list sets the enumeration.
            if joint.getEnumerationsOfProperty('Child') != links:
                # Avoid recursive recompute.
                # Doesn't change the value if in the new enum.
                joint.Child = links
            if joint.getEnumerationsOfProperty('Parent') != links:
                # Avoid recursive recompute.
                # Doesn't change the value if in the new enum.
                joint.Parent = links

    def add_joint_variables(self) -> list[str]:
        """Add a property for each actuated joint."""
        variables: list[str] = []
        try:
            # Get all old variables.
            old_vars: set[str] = set(get_properties_of_category(
                self.robot,
                self._category_of_joint_values))
            # Add a variable for each actuated (supported) joint.
            for joint in self.get_joints():
                var = _add_joint_variable(self.robot, joint,
                                          self._category_of_joint_values)
                if var:
                    variables.append(var)
            # Remove obsoleted variables.
            for p in old_vars - set(variables):
                self.robot.removeProperty(p)
        except AttributeError:
            pass
        return variables

    def export_urdf(self) -> Optional[et.Element]:
        """Export the robot as URDF, writing files."""
        if ((not hasattr(self, 'robot'))
                or (not hasattr(self.robot, 'OutputPath'))):
            return
        if not self.robot.OutputPath:
            warn('Property `OutputPath` cannot be empty', True)
            return
        if not wb_globals.g_ros_workspace.name:
            ws = get_ros_workspace()
            wb_globals.g_ros_workspace = ws
            p = without_ros_workspace(self.robot.OutputPath)
            if p != self.robot.OutputPath:
                self.robot.OutputPath = p
        output_path = (wb_globals.g_ros_workspace.expanduser()
                       / 'src' / self.robot.OutputPath)
        package_parent, package_name = split_package_path(output_path)
        # TODO: warn if package name doesn't end with `_description`.
        xml = et.fromstring('<robot/>')
        xml.attrib['name'] = get_valid_urdf_name(self.robot.Label)
        xml.append(et.Comment('Generated by the ROS Workbench for'
                              ' FreeCAD (https://github.com/galou/'
                              'freecad.cross)'))
        for link in self.get_links():
            if not hasattr(link, 'Proxy'):
                error(f"Internal error with '{link.Label}', has no 'Proxy' attribute",
                      True)
                return
            xml.append(link.Proxy.export_urdf(package_parent, package_name))
        for joint in self.get_joints():
            if not joint.Parent:
                error(f"Joint '{joint.Label}' has no parent link", True)
                continue
            if not joint.Child:
                error(f"Joint '{joint.Label}' has no child link", True)
                continue
            if hasattr(joint, 'Proxy') and joint.Proxy:
                xml.append(joint.Proxy.export_urdf())
            else:
                error(f"Internal error with joint '{joint.Label}'"
                      ", has no 'Proxy' attribute", True)
        # Save the xml into a file.
        output_path.mkdir(parents=True, exist_ok=True)
        file_base = get_valid_filename(ros_name(self.robot))
        urdf_file = f'{file_base}.urdf'
        urdf_path = output_path / f'urdf/{urdf_file}'
        save_xml(xml, urdf_path)
        template_files = [
            'package.xml',
            'CMakeLists.txt',
            'launch/display.launch.py',
            'rviz/robot_description.rviz',
            ]
        export_templates(template_files,
                         package_parent,
                         package_name=package_name,
                         urdf_file=urdf_file)
        return xml

    def _remove_ros_workspace(self, obj) -> None:
        """Modify `obj.OutputPath` to remove $ROS_WORKSPACE/src."""
        rel_path = without_ros_workspace(obj.OutputPath)
        if wb_globals.g_ros_workspace.samefile(Path()):
            # g_ros_workspace was not defined yet.
            ws = get_ros_workspace_from_file(
                    obj.OutputPath)
            if not ws.samefile(Path()):
                # A workspace was found.
                wb_globals.g_ros_workspace = ws
                message('ROS workspace was set to'
                        f' {wb_globals.g_ros_workspace},'
                        ' change if not correct.'
                        ' Note that packages in this workspace will NOT be'
                        ' found, though, but only by launching FreeCAD from a'
                        ' sourced workspace',
                        True)
                rel_path = without_ros_workspace(obj.OutputPath)
        if rel_path != obj.OutputPath:
            obj.OutputPath = rel_path

    def _fix_lost_fc_links(self) -> None:
        """Fix linked objects in ROS links lost on restore.

        Probably because these elements are restored before the ROS links.

        """
        if not hasattr(self, 'robot'):
            return
        links = self.get_links()
        for obj in self.robot.Document.Objects:
            if (not hasattr(obj, 'InList')) or (len(obj.InList) != 1):
                continue
            link = obj.InList[0]
            if ((link not in links)
                    or (obj in link.Group)
                    or (obj in link.Real)
                    or (obj in link.Visual)
                    or (obj in link.Collision)):
                continue
            link.addObject(obj)


class _ViewProviderRobot:
    """A view provider for the Robot container object """

    def __init__(self, vobj: VPDO):
        vobj.Proxy = self

    def getIcon(self):
        # Implementation note: "return 'robot.svg'" works only after
        # workbench activation in GUI.
        return str(ICON_PATH / 'robot.svg')

    def attach(self, vobj: VPDO):
        self.ViewObject = vobj
        self.robot = vobj.Object

        # Level of detail.
        add_property(vobj, 'App::PropertyBool', 'ShowReal', 'ROS Display Options',
                     'Whether to show the real parts')
        add_property(vobj, 'App::PropertyBool', 'ShowVisual', 'ROS Display Options',
                     'Whether to show the parts for URDF visual')
        add_property(vobj, 'App::PropertyBool', 'ShowCollision', 'ROS Display Options',
                     'Whether to show the parts for URDF collision')

        # Joint display options.
        add_property(vobj, 'App::PropertyBool', 'ShowJointAxes',
                     'ROS Display Options',
                     'Toggle the display of the Z-axis for all child joints',
                     True)
        add_property(vobj, 'App::PropertyLength', 'JointAxisLength',
                     'ROS Display Options',
                     "Length of the arrow for the joints axes",
                     500.0)

    def updateData(self, obj: RosRobot, prop: str):
        return

    def onChanged(self, vobj: VPDO, prop: str):
        if prop == 'ShowJointAxes':
            if is_robot(vobj.Object):
                for j in get_joints(vobj.Object.Group):
                    if hasattr(j.ViewObject, 'ShowAxis'):
                        j.ViewObject.ShowAxis = vobj.ShowJointAxes
        if prop == 'JointAxisLength':
            if is_robot(vobj.Object):
                for j in get_joints(vobj.Object.Group):
                    if hasattr(j.ViewObject, 'AxisLength'):
                        j.ViewObject.AxisLength = vobj.JointAxisLength
        if prop in ['ShowReal', 'ShowVisual', 'ShowCollision']:
            robot: RosRobot = vobj.Object
            robot.Proxy.execute(robot)

    def doubleClicked(self, vobj: VPDO):
        gui_doc = vobj.Document
        if not gui_doc.getInEdit():
            gui_doc.setEdit(vobj.Object.Name)
        else:
            error('Task dialog already active')
        return True

    def setEdit(self, vobj: VPDO, mode):
        return False

    def unsetEdit(self, vobj: VPDO, mode):
        import FreeCADGui as fcgui
        fcgui.Control.closeDialog()

    def __getstate__(self):
        return

    def __setstate__(self, state):
        return


def make_robot(name, doc: Optional[fc.Document] = None) -> RosRobot:
    """Add a Ros::Robot to the current document."""
    if doc is None:
        doc = fc.activeDocument()
    if doc is None:
        warn('No active document, doing nothing', False)
        return
    obj = doc.addObject('App::DocumentObjectGroupPython', name)
    Robot(obj)

    if hasattr(fc, 'GuiUp') and fc.GuiUp:
        _ViewProviderRobot(obj.ViewObject)
        obj.ViewObject.ShowReal = True
        obj.ViewObject.ShowVisual = False
        obj.ViewObject.ShowCollision = False

    doc.recompute()
    return obj
