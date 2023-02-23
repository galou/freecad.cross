from __future__ import annotations

from pathlib import Path
from typing import Optional
import xml.etree.ElementTree as et

import FreeCAD as fc

from .utils import ICON_PATH
from .utils import add_property
from .utils import error
from .utils import get_joints
from .utils import get_links
from .utils import is_link
from .utils import save_xml
from .utils import split_package_path
from .utils import valid_urdf_name
from .utils import warn

# Typing hint aliases
DO = fc.DocumentObject


def _existing_link(link: DO, obj: DO, lod: str) -> Optional[DO]:
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
        link: DO,
        objects: list[DO],
        lod: str,
        ) -> list[DO]:
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
    old_and_new_objects: list[DO] = []
    for i, o in enumerate(objects):
        link_to_o = _existing_link(link, o, lod)
        if link_to_o is not None:
            # print(f' {o.Name} is already linked')
            old_and_new_objects.append(link_to_o)
            continue
        name = f'{lod}_{link.Name}_'
        lod_link = doc.addObject('App::Link', name)
        lod_link.Label = name
        # print(f'Adding link {lod_link.Name} to {o.Name} into {link.Name}')
        if len(o.Parents) != 1:
            warn(f'Wrong object type. {o.Name}.Parents has no or more than one entries')
        link_placement = link.Proxy.get_link_placement(lod, i) or fc.Placement()
        lod_link.LinkPlacement = link_placement
        lod_link.setLink(o)
        lod_link.adjustRelativeLinks(link)
        link.addObject(lod_link)
        old_and_new_objects.append(lod_link)
        print(f'Appending {lod_link.Name}')
    return old_and_new_objects


class Robot:
    """The Robot group."""

    type = 'Ros::Robot'

    def __init__(self, obj):
        obj.Proxy = self
        self.robot = obj
        self.type = 'Ros::Robot'
        self.previous_link_count = 0

        self.init_properties(obj)

    def init_properties(self, obj):
        add_property(obj, 'App::PropertyString', '_Type', 'Internal',
                     'The type')._Type = self.type
        obj.setPropertyStatus('_Type', ['Hidden', 'ReadOnly'])

        obj.setPropertyStatus('Group', 'ReadOnly')  # Managed in self.reset_group().

        add_property(obj, 'App::PropertyLink', 'Assembly', 'Components',
                    'The part object this robot is built upon')
        add_property(obj, 'App::PropertyBool', 'ShowReal', 'Components',
                    'Whether to show the real parts').ShowReal = True
        add_property(obj, 'App::PropertyBool', 'ShowVisual', 'Components',
                    'Whether to show the parts for URDF visual').ShowVisual = False
        add_property(obj, 'App::PropertyBool', 'ShowCollision', 'Components',
                    'Whether to show the parts for URDF collision').ShowCollision = False

        add_property(obj, 'App::PropertyPath', 'OutputPath', 'Export', 'The path to the ROS package to export files to')

    def execute(self, obj):
        self.reset_group()

    def onChanged(self, feature: DO, prop: str) -> None:
        # print(f'Robot::onChanged({feature.Name}, {prop})') # DEBUG
        if not hasattr(self, 'robot'):
            # Implementation note: happens but how is it possible?
            return
        if prop in ['Group', 'ShowReal', 'ShowVisual', 'ShowCollision']:
            self.reset_group()

    def onDocumentRestored(self, obj):
        """Restore attributes because __init__ is not called on restore."""
        print('Robot::onDocumenRestored')
        obj.Proxy = self
        self.robot = obj
        self.previous_link_count = len(get_links(self.robot.Group))
        self.init_properties(obj)

    def __getstate__(self):
        return (self.type, self.previous_link_count)

    def __setstate__(self, state):
        if state:
            self.type, self.previous_link_count = state

    def reset_group(self):
        # print('Robot::reset_group()') # DEBUG

        if ((not hasattr(self.robot, 'ShowReal'))
                or (not hasattr(self.robot, 'ShowVisual'))
                or (not hasattr(self.robot, 'ShowCollision'))):
            return

        links = get_links(self.robot.Group)  # ROS links.

        # List of linked objects from all Ros::Link in robot.Group.
        current_linked_objects: list[DO] = []
        for link in links:
            for o in link.Group:
                current_linked_objects.append(o)
                # print(f'  current_linked_objects; {o.Name}: {hash(current_linked_objects[-1])}') # DEBUG

        # Add objects from selected components.
        all_linked_objects: list[DO] = []
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

    def _get_link_placement(self, link: fc.DocumentObject) -> Optional[fc.Placement]:
        """Return the placement of the link relative to the joint it's child of."""
        if not is_link(link):
            return
        parent_joint = link.Proxy.get_ref_joint()
        if parent_joint is None:
            # We have a root link (or an error).
            return fc.Placement()
        return (parent_joint.Proxy.get_placement().inverse()
                * link.CachedPlacement)

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
        xml.attrib['name'] = valid_urdf_name(self.robot.Label)
        xml.insert(et.Comment('Generated by the ROS Workbench for FreeCAD (https://github.com/galou/freecad.workbench_ros)'))
        for link in get_links(self.robot.Group):
            if not link.Real:
                error(f"Link '{link.Label}' has no link in 'Real'", True)
                continue
            if not hasattr(link, 'Proxy'):
                error(f"Internal error with '{link.Label}', has no 'Proxy' attribute", True)
                return
            link_placement = self._get_link_placement(link)
            print(f'{link.Label}.link_placement = {link_placement}') # DEBUG
            xml.append(link.Proxy.export_urdf(package_parent, package_name, link_placement))
        for joint in get_joints(self.robot.Group):
            warn(f'About to export {joint.Label}') # DEBUG
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
        # return 'ros_9dotslogo_color.svg'
        return str(ICON_PATH.joinpath('ros_9dotslogo_color.svg'))

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


def makeRobot(name):
    """Add a Ros::Robot to the current document."""
    doc = fc.activeDocument()
    if doc is None:
        return
    obj = doc.addObject('App::DocumentObjectGroupPython', name)
    Robot(obj)

    if fc.GuiUp:
        _ViewProviderRobot(obj.ViewObject)

    doc.recompute()
    return obj
