"""Proxy for a RosWorkcell, i.e. a combination of RosXacroObject and RosJoint

A RosWorkcell allows to combine existing URDF and xacro file to generate a
single robot description (or more generally a workcell description).
Joints must be defined between the included RosXacroObject.

"""

from __future__ import annotations

from pathlib import Path
from typing import Iterable, Optional
import xml.etree.ElementTree as et

import FreeCAD as fc

from .freecad_utils import add_property
from .freecad_utils import warn
from .freecad_utils import is_same_placement
from .ros_utils import split_package_path
from .urdf_utils import urdf_origin_from_placement
from .utils import get_valid_filename
from .utils import hasallattr
from .utils import save_xml
from .wb_utils import export_templates
from .wb_utils import get_joints
from .wb_utils import get_valid_urdf_name
from .wb_utils import get_xacro_chains
from .wb_utils import get_xacro_object_attachments
from .wb_utils import get_xacro_objects
from .wb_utils import ros_name

# Typing hints.
DO = fc.DocumentObject
VPDO = 'FreeCADGui.ViewProviderDocumentObject'
DOList = Iterable[DO]
RosJoint = DO  # A Ros::Joint, i.e. a DocumentObject with Proxy "Joint".
RosWorkcell = DO  # A Ros::Workcell, i.e. a DocumentObject with Proxy "Workcell".
RosXacroObject = DO  # A Ros::XacroObject, i.e. a DocumentObject with Proxy "XacroObject".


class Workcell:
    """Proxy for ROS workcells."""

    # The member is often used in workbenches, particularly in the Draft
    # workbench, to identify the object type.
    Type = 'Ros::Workcell'

    def __init__(self, obj: RosWorkcell):
        obj.Proxy = self
        self.workcell = obj
        self.init_properties(obj)

    def init_properties(self, obj: RosWorkcell) -> None:
        add_property(obj, 'App::PropertyString', '_Type', 'Internal',
                     'The type')
        obj.setPropertyStatus('_Type', ['Hidden', 'ReadOnly'])
        obj._Type = self.Type

        add_property(obj, 'App::PropertyPath', 'OutputPath', 'Export',
                     'The path to the ROS package to export files to')

        add_property(obj, 'App::PropertyString', 'RootLink', 'Elements',
                     'The root link of the workcell', 'world')

    def execute(self, obj: RosWorkcell) -> None:
        self.set_joint_enum()
        self.place_xacro_objects()

    def onBeforeChange(self, obj: RosWorkcell, prop: str) -> None:
        # TODO: save the old ros_name and update all joints that used it.
        pass

    def onChanged(self, obj: RosWorkcell, prop: str) -> None:
        pass

    def onDocumentRestored(self, obj: RosWorkcell) -> None:
        self.__init__(obj)

    def __getstate__(self):
        return self.Type,

    def __setstate__(self, state):
        if state:
            self.Type, = state

    def get_xacro_objects(self) -> list[RosXacroObject]:
        if ((not hasattr(self, 'workcell'))
                or (not hasattr(self.workcell, 'Group'))):
            return []
        return get_xacro_objects(self.workcell.Group)

    def get_joints(self) -> list[RosJoint]:
        if ((not hasattr(self, 'workcell')) or (not hasattr(self.workcell, 'Group'))):
            return []
        return get_joints(self.workcell.Group)

    def set_joint_enum(self) -> None:
        """Set the enum for Child and Parent of all joints."""
        if ((not hasattr(self, 'workcell'))
                or (not hasattr(self.workcell, 'RootLink'))):
            return
        # We add the empty string to show that the child or parent
        # was not set yet.
        child_links: list[str] = ['']
        parent_links: list[str] = ['', self.workcell.RootLink]
        for xacro_object in self.get_xacro_objects():
            child_links.append(xacro_object.Proxy.root_link)
            parent_links += xacro_object.Proxy.get_link_names()
        for joint in self.get_joints():
            # Implementation note: setting to a list sets the enumeration.
            if joint.getEnumerationsOfProperty('Child') != child_links:
                # Avoid recursive recompute.
                # Doesn't change the value if old value in the new enum.
                joint.Child = child_links
            if joint.getEnumerationsOfProperty('Parent') != parent_links:
                # Avoid recursive recompute.
                # Doesn't change the value if old value in the new enum.
                joint.Parent = parent_links

    def get_xacro_object_with_link(self, link_name: str) -> Optional[RosXacroObject]:
        """Return the xacro object containing a given link."""
        for xacro_object in self.get_xacro_objects():
            if xacro_object.has_link(link_name):
                return xacro_object

    def place_xacro_objects(self) -> None:
        """Set the `Placement` of all xacro objects."""
        for chain in get_xacro_chains(self.get_xacro_objects(), self.get_joints()):
            placement = fc.Placement()
            for attachment in chain:
                xo = attachment.xacro_object
                joint = attachment.attached_by
                link = attachment.attached_to
                if joint:
                    placement = joint.Origin * placement
                if link:
                    placement = link.Placement * placement
                if not is_same_placement(xo.Placement, placement):
                    xo.Placement = placement

    def export_urdf(self) -> Optional[et.Element]:
        if not hasattr(self, 'workcell'):
            return
        obj: RosWorkcell = self.workcell
        if not hasallattr(obj, ['OutputPath', 'RootLink']):
            return
        if not obj.OutputPath:
            # TODO: ask the user for OutputPath.
            warn('Property `OutputPath` cannot be empty', True)
            return

        robot_et = et.fromstring('<robot/>')
        robot_et.attrib['name'] = get_valid_urdf_name(ros_name(obj))
        robot_et.append(et.Comment('Generated by the ROS Workbench for'
                                   ' FreeCAD (https://github.com/galou/'
                                   'freecad.workbench_ros)'))

        # Add the root link.
        world_link_et = et.fromstring('<link/>')
        world_link_et.attrib['name'] = obj.RootLink
        robot_et.append(world_link_et)

        xacro_objects = self.get_xacro_objects()
        joints = self.get_joints()
        attachments = get_xacro_object_attachments(xacro_objects, joints)
        includes: list[et.Element] = []
        for attachment in attachments:
            xacro_object = attachment.xacro_object
            joint = attachment.attached_by

            # Add the xacro's children.
            xacro_et = xacro_object.Proxy.export_urdf()
            for child_et in xacro_et:
                if ((child_et.tag == 'xmlns:include')
                        and (child_et.attrib['filename'] not in includes)):
                    includes.append(child_et.attrib['filename'])
                robot_et.append(child_et)

            # Add the joint attaching the xacro.
            joint_et = et.fromstring('<joint/>')
            if joint:
                parent_link = f'{joint.Parent}'
                child_link = f'{joint.Child}'
                origin = joint.Origin
            else:
                parent_link = world_link_et.attrib['name']
                root_link = xacro_object.Proxy.root_link
                child_link = root_link
                origin = xacro_object.Placement
            joint_et.attrib['name'] = f'{parent_link}_to_{child_link}'
            joint_et.attrib['type'] = 'fixed'
            parent_et = et.fromstring('<parent/>')
            parent_et.attrib['link'] = parent_link
            joint_et.append(parent_et)
            child_et = et.fromstring('<child/>')
            child_et.attrib['link'] = child_link
            joint_et.append(child_et)
            origin_et = urdf_origin_from_placement(origin)
            joint_et.append(origin_et)
            robot_et.append(joint_et)

        # Write out files.
        output_path = Path(obj.OutputPath).expanduser()
        package_parent, package_name = split_package_path(output_path)
        output_path.mkdir(parents=True, exist_ok=True)
        robot_name = ros_name(self.workcell)
        file_base = get_valid_filename(robot_name)
        urdf_file = f'{file_base}.urdf.xacro'
        urdf_path = output_path / f'urdf/{urdf_file}'
        save_xml(robot_et, urdf_path)
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


class _ViewProviderWorkcell:
    """A view provider for the Link container object """

    def __init__(self, vobj: VPDO):
        vobj.Proxy = self

    def getIcon(self):
        return 'workcell.svg'

    def attach(self, vobj: VPDO):
        self.ViewObject = vobj

    def updateData(self, obj: VPDO, prop):
        return

    def onChanged(self, vobj: VPDO, prop: str):
        return

    def __getstate__(self):
        return None

    def __setstate__(self, state):
        return None


def make_workcell(name, doc: Optional[fc.Document] = None) -> RosWorkcell:
    """Add a Ros::Workcell to the current document."""
    if doc is None:
        doc = fc.activeDocument()
    if doc is None:
        warn('No active document, doing nothing', False)
        return
    obj = doc.addObject('App::DocumentObjectGroupPython', name)
    Workcell(obj)

    if hasattr(fc, 'GuiUp') and fc.GuiUp:
        _ViewProviderWorkcell(obj.ViewObject)

    return obj
