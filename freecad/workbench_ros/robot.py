from __future__ import annotations

from math import radians
from pathlib import Path
from typing import Iterable, Optional
import xml.etree.ElementTree as et

import FreeCAD as fc

from .utils import ICON_PATH
from .utils import add_property
from .utils import error
from .utils import get_chains
from .utils import get_joints
from .utils import get_links
from .utils import get_properties_of_category
from .utils import get_valid_property_name
from .utils import get_valid_urdf_name
from .utils import grouper
from .utils import is_joint
from .utils import is_link
from .utils import is_robot
from .utils import label_or
from .utils import save_xml
from .utils import split_package_path
from .utils import warn

# Typing hints.
DO = fc.DocumentObject
DOList = Iterable[DO]
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
        if len(o.Parents) != 1:
            warn(f'Wrong object type. {o.Name}.Parents'
                 ' has no or more than one entries')
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
    """Add a property to `robot` and return its name."""
    if not is_joint(joint):
        warn(f'Wrong object type. {joint.Name} ({joint.Label})'
             ' is not a ROS::Joint')
        return
    if joint.Type == 'prismatic':
        unit = 'mm'
    elif joint.Type in ['revolute', 'continuous']:
        unit = 'deg'
    else:
        # Non-simple joints not supported yet.
        return ''
    # e.g. name_candidate = "q0_deg" or "q0".
    name_candidate = f'{joint.Label}{f"_{unit}" if unit else ""}'
    var_name = get_valid_property_name(name_candidate)
    # e.g. help_txt = "q0 in deg" or "q0".
    help_txt = f'{joint.Label}{f" in {unit}" if unit else ""}'
    _, used_var_name = add_property(robot,
                                    'App::PropertyFloat', var_name,
                                    category, help_txt)
    value: Optional[float] = None
    if joint.Type == 'prismatic':
        value = robot.getPropertyByName(var_name) * 0.001
    elif joint.Type == 'revolute':
        value = radians(robot.getPropertyByName(var_name))
    if value is not None:
        if joint.Position != value:
            # Avoid recursive recompute.
            joint.Position = value
    return used_var_name


class Robot:
    """The Robot group."""

    # The member is often used in workbenches, particularly in the Draft
    # workbench, to identify the object type.
    Type = 'Ros::Robot'

    # Name of the category (or group) for the joint values, which are saved as
    # properties of `self.robot`.
    _category_of_joint_values = 'JointValues'

    def __init__(self, obj):
        obj.Proxy = self
        self.robot = obj
        self.Type = 'Ros::Robot'
        self.previous_link_count = 0

        self.init_properties(obj)

    def init_properties(self, obj):
        add_property(obj, 'App::PropertyString', '_Type', 'Internal',
                     'The type')
        obj.setPropertyStatus('_Type', ['Hidden', 'ReadOnly'])
        obj._Type = self.Type

        # Managed in self.reset_group().
        obj.setPropertyStatus('Group', 'ReadOnly')

        add_property(obj, 'App::PropertyBool', 'ShowReal', 'Components',
                     'Whether to show the real parts')
        obj.ShowReal = True
        add_property(obj, 'App::PropertyBool', 'ShowVisual', 'Components',
                     'Whether to show the parts for URDF visual')
        obj.ShowVisual = False
        add_property(obj, 'App::PropertyBool', 'ShowCollision', 'Components',
                     'Whether to show the parts for URDF collision')
        obj.ShowCollision = False

        add_property(obj, 'App::PropertyPath', 'OutputPath', 'Export',
                     'The path to the ROS package to export files to')

    def execute(self, obj: DO) -> None:
        self.add_joint_variables()
        self.compute_poses()
        self.reset_group()

    def onChanged(self, obj: DO, prop: str) -> None:
        if not hasattr(self, 'robot'):
            # Implementation note: happens because __init__ is not called on
            # restore.
            return
        if prop in ['Group', 'ShowReal', 'ShowVisual', 'ShowCollision']:
            self.execute(obj)

    def onDocumentRestored(self, obj):
        """Restore attributes because __init__ is not called on restore."""
        obj.Proxy = self
        self.robot = obj
        self.previous_link_count = len(get_links(self.robot.Group))
        self.init_properties(obj)

    def __getstate__(self):
        return (self.Type, self.previous_link_count)

    def __setstate__(self, state):
        if state:
            self.Type, self.previous_link_count = state

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
                new_link_placement = placement * link.MountedPlacement
                if link.Placement != new_link_placement:
                    # Avoid recursive recompute.
                    link.Placement = new_link_placement
                if joint:
                    new_joint_placement = placement * joint.Origin
                    if joint.Placement != new_joint_placement:
                        # Avoid recursive recompute.
                        joint.Placement = new_joint_placement
                    # For next link.
                    placement = new_joint_placement * joint.Proxy.get_actuation_placement()

    def get_chains(self) -> list[DOList]:
        if not hasattr(self, 'robot') or not is_robot(self.robot):
            warn(f'{label_or(self.robot)} is not a ROS::Robot', True)
            return []
        links = get_links(self.robot.Group)
        joints = get_joints(self.robot.Group)
        return get_chains(links, joints)

    def reset_group(self) -> None:
        if ((not hasattr(self.robot, 'ShowReal'))
                or (not hasattr(self.robot, 'ShowVisual'))
                or (not hasattr(self.robot, 'ShowCollision'))):
            return

        links = get_links(self.robot.Group)  # ROS links.

        # List of linked objects from all Ros::Link in robot.Group.
        current_linked_objects: list[AppLink] = []
        for link in links:
            for o in link.Group:
                current_linked_objects.append(o)

        # Add objects from selected components.
        all_linked_objects: list[AppLink] = []
        if self.robot.ShowReal:
            for link in links:
                all_linked_objects += _add_links_lod(link, link.Real, 'real')

        if self.robot.ShowVisual:
            for link in links:
                all_linked_objects += _add_links_lod(link, link.Visual, 'visual')

        if self.robot.ShowCollision:
            for link in links:
                all_linked_objects += _add_links_lod(link, link.Collision, 'collision')

        # Remove objects that do not belong to `all_linked_objects`.
        objects_to_remove = set(current_linked_objects) - set(all_linked_objects)
        for o in objects_to_remove:
            # print(f'Removing {o.Name}')
            self.robot.Document.removeObject(o.Name)
        # TODO?: doc.recompute() if objects_to_remove or (set(current_linked_objects) != set(all_linked_objects))

    def add_joint_variables(self) -> list[str]:
        """Add a property for each actuated joint."""
        variables: list[str] = []
        try:
            # Remove all old variables.
            for p in get_properties_of_category(
                    self.robot,
                    self._category_of_joint_values):
                self.robot.removeProperty(p)
            # Add a variable for each actuated (supported) joint.
            for joint in get_joints(self.robot.Group):
                var = _add_joint_variable(self.robot, joint,
                                          self._category_of_joint_values)
                if var:
                    variables.append(var)
        except AttributeError:
            pass
        return variables

    def _get_link_placement(self,
                            link: fc.DocumentObject,
                            ) -> Optional[fc.Placement]:
        """Return the placement of the link relative to the joint it's child of."""
        if not is_link(link):
            return
        parent_joint = link.Proxy.get_ref_joint()
        if parent_joint is None:
            # We have a root link (or an error).
            # TODO: return the root placement.
            return fc.Placement()
        return (parent_joint.Proxy.get_placement().inverse()
                * link.MountedPlacement)

    def export_urdf(self) -> et.ElementTree:
        if not hasattr(self, 'robot'):
            return et.ElementTree()
        if not hasattr(self.robot, 'OutputPath'):
            return et.ElementTree()
        output_path = Path(self.robot.OutputPath)
        package_parent, package_name = split_package_path(output_path)
        output_path.mkdir(parents=True, exist_ok=True)
        # TODO: warn if package name doesn't end with `_description`.
        xml = et.fromstring('<robot/>')
        xml.attrib['name'] = get_valid_urdf_name(self.robot.Label)
        xml.insert(et.Comment('Generated by the ROS Workbench for'
                              ' FreeCAD (https://github.com/galou/'
                              'freecad.workbench_ros)'))
        for link in get_links(self.robot.Group):
            if not link.Real:
                error(f"Link '{link.Label}' has no link in 'Real'", True)
                continue
            if not hasattr(link, 'Proxy'):
                error(f"Internal error with '{link.Label}', has no 'Proxy' attribute",
                      True)
                return
            link_placement = self._get_link_placement(link)
            xml.append(link.Proxy.export_urdf(package_parent, package_name,
                                              link_placement))
        for joint in get_joints(self.robot.Group):
            if not joint.Parent:
                error(f"Joint '{joint.Label}' has no parent link", True)
                continue
            if not joint.Child:
                error(f"Joint '{joint.Label}' has no child link", True)
                continue
            if hasattr(joint, 'Proxy'):
                xml.append(joint.Proxy.export_urdf())
            else: # DEBUG
                error(f"Internal error with joint '{joint.Label}'", True) # DEBUG
        # Save the xml into a file.
        urdf_path = output_path / 'urdf/robot.urdf'
        save_xml(xml, urdf_path)
        return xml


class _ViewProviderRobot:
    """A view provider for the Robot container object """

    def __init__(self, vobj):
        vobj.Proxy = self

    def getIcon(self):
        # TODO: Solve why this doesn't work.
        # return 'robot.svg'
        return str(ICON_PATH.joinpath('robot.svg'))

    def attach(self, vobj):
        self.ViewObject = vobj
        self.robot = vobj.Object

    def updateData(self, obj, prop):
        return

    def onChanged(self, vobj, prop):
        return

    def doubleClicked(self, vobj):
        gui_doc = vobj.Document
        if not gui_doc.getInEdit():
            gui_doc.setEdit(vobj.Object.Name)
        else:
            error('Task dialog already active')
        return True

    def setEdit(self, vobj, mode):
        import FreeCADGui as fcgui
        from .task_panel_robot import TaskPanelRobot
        task_panel = TaskPanelRobot(self.robot)
        fcgui.Control.showDialog(task_panel)
        return True

    def unsetEdit(self, vobj, mode):
        import FreeCADGui as fcgui
        fcgui.Control.closeDialog()
        return

    def __getstate__(self):
        return None

    def __setstate__(self, state):
        return None


def make_robot(name, doc: Optional[fc.Document] = None) -> DO:
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

    doc.recompute()
    return obj
