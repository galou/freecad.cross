"""Proxy for a XacroObject that reads a xacro file and outputs a xacro file
including it.

Can read:
- xacro file containing only macro definitions
- xacro file generating a full-feature URDF
- URDF file

Generates a xacro file that includes the input xacro, add a transform from
"world" to the root link of the robot (if any), and use the macro. Whether a
robot, a workcell, a static object is generated from the xacro depends on the
input xacro and the chosen macro from this xacro file.

The object type is called XacroObject to avoid confusion with the ROS package xacro
(and Python module xacro).

"""

from __future__ import annotations

from pathlib import Path
from typing import Any, Optional
import xml.etree.ElementTree as et

import FreeCAD as fc

from .freecad_utils import add_property
from .freecad_utils import warn
from .ros_utils import split_package_path
from .urdf_utils import urdf_origin_from_placement
from .utils import get_joints
from .utils import get_links
from .utils import get_valid_filename
from .utils import save_xml
from .wb_utils import export_templates
from .wb_utils import ros_name
try:
    from .robot_from_urdf import robot_from_urdf
    from .urdf_loader import UrdfLoader
    from .utils import hasallattr
    from .utils import is_robot
    from .xacro_loader import XacroLoader
    from urdf_parser_py.urdf import Robot as UrdfRobot
except ImportError as e:
    # TODO: Warn the user more nicely.
    # TODO: provide alternates on import error
    warn(str(e))
    UrdfRobot = Any

# Typing hints.
DO = fc.DocumentObject
VPDO = 'FreeCADGui.ViewProviderDocumentObject'
RosXacroObject = DO  # A Ros::XacroObject, i.e. a DocumentObject with Proxy "Xacro".
RosRobot = DO  # A Ros::Robot, i.e. a DocumentObject with Proxy "Robot".


def _clear_robot(obj: RosRobot) -> None:
    """Delete all joints and links from a robot."""
    doc = obj.Document
    # Joints first.
    for joint in get_joints(obj.Group):
        doc.removeObject(joint.Name)
    for link in get_links(obj.Group):
        for fc_link in link.Group:
            doc.removeObject(fc_link.Name)
        doc.removeObject(link.Name)


class XacroObject:
    """The XacroObject proxy."""

    # The member is often used in workbenches, particularly in the Draft
    # workbench, to identify the object type.
    Type = 'Ros::XacroObject'

    def __init__(self, obj: RosXacroObject):
        obj.Proxy = self
        self.xacro_object = obj

        # List of macro parameters.
        self.param_properties: list[str] = []

        # Keep track of the old generated xacro, to avoid rebuilding the robot.
        if not hasattr(self, '_old_xacro_file_content'):
            self._old_xacro_file_content = ''

        self.init_properties(obj)
        self.init_extensions(obj)

    def init_properties(self, obj: RosXacroObject):
        add_property(obj, 'App::PropertyString', '_Type', 'Internal',
                     'The type')
        obj.setPropertyStatus('_Type', ['Hidden', 'ReadOnly'])
        obj._Type = self.Type

        add_property(obj, 'App::PropertyFile', 'InputFile', 'Input',
                     'The source xacro or URDF file')
        add_property(obj, 'App::PropertyEnumeration', 'MainMacro', 'Input',
                     'The macro to use')

        add_property(obj, 'App::PropertyPath', 'OutputPath', 'Export',
                     'The path to the ROS package to export files to')

        add_property(obj, 'App::PropertyPlacement', 'Placement',
                     'Base', 'Placement')

        self._toggle_editor_mode(obj)

    def init_extensions(self, obj: RosXacroObject):
        # Needed to make this object able to attach parameterically to other objects.
        obj.addExtension('Part::AttachExtensionPython')
        # Need a group to put the generated robot in.
        obj.addExtension('App::GroupExtensionPython')

        # Managed in self.reset_group().
        obj.setPropertyStatus('Group', ['ReadOnly', 'Hidden'])

    def execute(self, obj: RosXacroObject) -> None:
        """Update the embedded robot.

        Called on recompute(), this method is mandatory for scripted objects.

        """
        obj.positionBySupport()
        if (not hasallattr(obj, ['InputFile', 'MainMacro'])):
            return
        if not obj.InputFile:
            return
        self.xacro = XacroLoader.load_from_file(obj.InputFile)
        macro_names = self.xacro.get_macro_names()
        if obj.getEnumerationsOfProperty('MainMacro') != macro_names:
            obj.MainMacro = macro_names
        self._toggle_editor_mode(obj)
        self.set_param_properties(obj)
        self.reset_group(obj)

    def onChanged(self, obj: RosXacroObject, prop: str) -> None:
        if prop in ['InputFile', 'Label', 'Label2', 'MainMacro']:
            self.execute(obj)

    def onDocumentRestored(self, obj: RosXacroObject):
        """Restore attributes because __init__ is not called on restore."""
        self.__init__(obj)

    def __getstate__(self):
        return self.Type,

    def __setstate__(self, state):
        if state:
            self.Type, = state

    def set_param_properties(self, obj: RosXacroObject) -> None:
        if not hasattr(self, 'xacro_object'):
            # Implementation note: xacro_object is defined with other required members.
            return
        # Clear old parameters.
        for name in self.param_properties:
            obj.removeProperty(name)
        self.param_properties.clear()
        if (not hasallattr(obj, ['InputFile', 'MainMacro'])):
            return
        if self.xacro:
            macro = self.xacro.macros[obj.MainMacro]
            for name in macro.params:
                add_property(obj, 'App::PropertyString', name, 'Input',
                             f'Macro parameter "{name}"')
                self.param_properties.append(name)
                if name in macro.defaultmap:
                    # Implementation note: cannot set the property value
                    # knowing the property name, __setattr__ doesn't work.
                    value = (macro.defaultmap[name][1]
                             .replace('{', '')
                             .replace('}', ''))
                    obj.setExpression(name, f'<<{value}>>')
                    # It looks that it's better to keep the expression rather
                    # than clearing it may be interpreted as float.
                    # obj.clearExpression(name)

    def reset_group(self, obj: RosXacroObject):
        if not hasallattr(obj, ['Group', 'InputFile', 'MainMacro']):
            return
        new_robot = self._generate_robot(obj)
        if new_robot and obj.Group:
            old_robot = obj.Group[0]
            # Empty the group.
            obj.Group = []
            if is_robot(old_robot):
                # Delete the old robot.
                _clear_robot(old_robot)
                obj.Document.removeObject(old_robot.Name)
        if new_robot:
            # Add the new robot.
            obj.addObject(new_robot)

    def export_urdf(self) -> Optional[et.Element]:
        """Export the xacro object as URDF, writing files."""
        if ((not hasattr(self, 'xacro'))
                or (not hasattr(self.xacro_object, 'OutputPath'))):
            return
        obj: RosXacroObject = self.xacro_object
        if not obj.OutputPath:
            warn('Property `OutputPath` cannot be empty', True)
            return

        # Generate the URDF to obtain the root link.
        robot_name = ros_name(obj)
        params = {name: obj.getPropertyByName(name) for name in self.param_properties}
        urdf_robot = self._generate_urdf(robot_name, obj.MainMacro, params)
        root_link = urdf_robot.get_root()

        # Generate the xacro xml document.
        xacro_xml = self.xacro.to_xml(robot_name, obj.MainMacro, params)

        # Add the link "world" and a transform from it to the root link.
        robot = xacro_xml.firstChild
        world_link = robot.appendChild(xacro_xml.createElement('link'))
        world_link.setAttribute('name', 'world')
        joint = robot.appendChild(xacro_xml.createElement('joint'))
        joint.setAttribute('name', f'world_to_{root_link}')
        joint.setAttribute('type', 'fixed')
        child = joint.appendChild(xacro_xml.createElement('child'))
        child.setAttribute('link', world_link.attributes['name'].value)
        parent = joint.appendChild(xacro_xml.createElement('parent'))
        parent.setAttribute('link', root_link)
        origin = joint.appendChild(xacro_xml.createElement('origin'))
        # TODO: solve global placement.
        origin_et = urdf_origin_from_placement(obj.Placement)
        origin.setAttribute('xyz', origin_et.attrib['xyz'])
        origin.setAttribute('rpy', origin_et.attrib['rpy'])

        # Write out files.
        output_path = Path(obj.OutputPath).expanduser()
        package_parent, package_name = split_package_path(output_path)
        output_path.mkdir(parents=True, exist_ok=True)
        file_base = get_valid_filename(robot_name)
        urdf_file = f'{file_base}.urdf.xacro'
        urdf_path = output_path / f'urdf/{urdf_file}'
        xacro_et = et.fromstring(xacro_xml.toxml())
        save_xml(xacro_et, urdf_path)
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

        return xacro_et

    def _toggle_editor_mode(self, obj: RosXacroObject) -> None:
        """Show/hide properties."""
        # Hide until an input file is given and macros are defined.
        if (hasattr(obj, 'InputFile')
                and obj.InputFile
                and hasattr(self, 'xacro')
                and self.xacro
                and self.xacro.get_macro_names()):
            # Show property `MainMacro`.
            obj.setEditorMode('MainMacro', [])
        else:
            obj.setPropertyStatus('MainMacro', ['Hidden'])

    def _generate_robot(self, obj: RosXacroObject) -> Optional[RosRobot]:
        if not hasattr(self, 'xacro_object'):
            # Implementation note: xacro_object is defined with other required members.
            return
        params = {name: obj.getPropertyByName(name) for name in self.param_properties}
        xacro_txt = self.xacro.to_string(ros_name(obj), obj.MainMacro, params)
        if xacro_txt == self._old_xacro_file_content:
            return
        self._old_xacro_file_content = xacro_txt
        urdf_robot = self._generate_urdf(ros_name(obj), obj.MainMacro, params)
        robot = robot_from_urdf(obj.Document, urdf_robot)
        return robot

    def _generate_urdf(self,
                       robot_name: str,
                       macro: str = '',
                       params: dict[str, str] = None,
                       ) -> UrdfRobot:
        params = {} if params is None else params
        urdf_txt = self.xacro.to_urdf_string(robot_name, macro, params)
        urdf_robot = UrdfLoader.load_from_string(urdf_txt)
        return urdf_robot


class _ViewProviderXacroObject:
    """A view provider for the Xacro container object """

    def __init__(self, vobj: VPDO):
        vobj.Proxy = self

    def getIcon(self):
        return 'xacro.svg'

    def attach(self, vobj: VPDO):
        self.ViewObject = vobj
        vobj.addExtension('Gui::ViewProviderGroupExtensionPython')

    def updateData(self, obj: RosXacroObject, prop: str):
        return

    def onChanged(self, vobj: VPDO, prop: str):
        pass

    def doubleClicked(self, vobj: VPDO):
        return True

    def setEdit(self, vobj: VPDO, mode):
        return False

    def unsetEdit(self, vobj: VPDO, mode):
        return

    def __getstate__(self):
        return None

    def __setstate__(self, state):
        return None


def make_xacro_object(name, doc: Optional[fc.Document] = None) -> RosXacroObject:
    """Add a Ros::XacroObject to the current document."""
    if doc is None:
        doc = fc.activeDocument()
    if doc is None:
        warn('No active document, doing nothing', False)
        return
    obj = doc.addObject('App::FeaturePython', name)
    XacroObject(obj)

    if hasattr(fc, 'GuiUp') and fc.GuiUp:
        _ViewProviderXacroObject(obj.ViewObject)

    doc.recompute()
    return obj
