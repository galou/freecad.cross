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

from copy import copy
from typing import Any, Optional
import xml.etree.ElementTree as et

import FreeCAD as fc

from .freecad_utils import ProxyBase
from .freecad_utils import add_property
from .freecad_utils import warn
from .utils import hasallattr
from .wb_utils import ICON_PATH
from .wb_utils import is_robot
from .wb_utils import is_workcell
from .wb_utils import ros_name
try:
    from .robot_from_urdf import robot_from_urdf
    from .urdf_loader import UrdfLoader
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
RosXacroObject = DO  # A Ros::XacroObject, i.e. a DocumentObject with Proxy "XacroObject".
RosLink = DO  # A Ros::Link, i.e. a DocumentObject with Proxy "Link".
RosRobot = DO  # A Ros::Robot, i.e. a DocumentObject with Proxy "Robot".


def _clear_robot(obj: RosRobot) -> None:
    """Delete all joints and links from a robot."""
    doc = obj.Document
    # Let robot clean its generated objects.
    if obj.ViewObject:
        obj.ViewObject.ShowReal = False
        obj.ViewObject.ShowVisual = False
        obj.ViewObject.ShowCollision = False
    # Joints first.
    for joint in obj.Proxy.get_joints():
        doc.removeObject(joint.Name)
    for link in obj.Proxy.get_links():
        # If without GUI. Group should be empty with GUI.
        for fc_link in link.Group:
            doc.removeObject(fc_link.Name)
        doc.removeObject(link.Name)


class XacroObject(ProxyBase):
    """The XacroObject proxy."""

    # The member is often used in workbenches, particularly in the Draft
    # workbench, to identify the object type.
    Type = 'Ros::XacroObject'

    def __init__(self, obj: RosXacroObject):
        super().__init__('xacro_object', [
            'InputFile',
            'MainMacro',
            'Placement',
            '_Type',
            ])
        obj.Proxy = self
        self.xacro_object = obj

        # List of macro parameters.
        self.param_properties: list[str] = []

        # Keep track of the old generated xacro, to avoid rebuilding the robot.
        if not hasattr(self, '_old_xacro_file_content'):
            self._old_xacro_file_content = ''
            self._urdf_robot: Optional[UrdfRobot] = None

        # The root link of the generated URDF.
        if not hasattr(self, '_root_link'):
            self._root_link = ''

        self.init_properties(obj)
        self.init_extensions(obj)

    @property
    def root_link(self) -> str:
        return str(self._root_link)

    def has_link(self, link_name: str) -> bool:
        """Return True if the link belongs to the xacro object.

        The link_name is the full name after macro expansion.

        """
        if not self._urdf_robot:
            return ''
        return link_name in self._urdf_robot.link_map

    def get_link(self, link_name: str) -> Optional[RosLink]:
        """Return True if the link belongs to the xacro object.

        The link_name is the full name after macro expansion.

        """
        for link in self.get_links():
            if ros_name(link) == link_name:
                return link

    def get_links(self) -> list[RosLink]:
        if ((not hasattr(self, 'xacro_object')
             or (not hasattr(self.xacro_object, 'Group'))
             or (not self.xacro_object.Group))):
            return []
        robot = self._get_robot()
        if not robot:
            return []
        if not hasattr(robot, 'Proxy'):
            return []
        return robot.Proxy.get_links()

    def get_link_placement(self, link_name: str) -> Optional[fc.Placement]:
        """Return the link placement relative to the URDF root."""
        if not self.has_link(link_name):
            return
        for link in self.get_links():
            if ros_name(link) == link_name:
                return link.Placement

    def init_properties(self, obj: RosXacroObject):
        add_property(obj, 'App::PropertyString', '_Type', 'Internal',
                     'The type')
        obj.setPropertyStatus('_Type', ['Hidden', 'ReadOnly'])
        obj._Type = self.Type

        add_property(obj, 'App::PropertyFile', 'InputFile', 'Input',
                     'The source xacro or URDF file')
        add_property(obj, 'App::PropertyEnumeration', 'MainMacro', 'Input',
                     'The macro to use')

        # The computed placement (or the placement defined by the user if no
        # attachment mode is defined).
        add_property(obj, 'App::PropertyPlacement', 'Placement',
                     'Base', 'Placement')

        self._toggle_editor_mode(obj)

    def init_extensions(self, obj: RosXacroObject):
        # Needed to make this object able to attach parameterically to other objects.
        # obj.addExtension('Part::AttachExtensionPython')
        # Need a group to put the generated robot in.
        # obj.addExtension('App::GroupExtensionPython')
        obj.addExtension('App::GeoFeatureGroupExtensionPython')

        # Managed in self.reset_group().
        obj.setPropertyStatus('Group', ['ReadOnly', 'Hidden'])

    def execute(self, obj: RosXacroObject) -> None:
        """Update the embedded robot.

        Called on recompute(), this method is mandatory for scripted objects.

        """
        # obj.positionBySupport()
        if not hasallattr(obj, ['InputFile', 'MainMacro']):
            return
        if not obj.InputFile:
            self._root_link = ''
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
            # Implementation note: xacro_object is defined with other
            # required members.
            return

        # Get old parameters to clear old parameters later on.
        old_param_properties: list[str] = copy(self.param_properties)

        self.param_properties.clear()
        if not hasallattr(obj, ['InputFile', 'MainMacro']):
            return
        if not self.xacro:
            return
        macro = self.xacro.macros[obj.MainMacro]
        for name in macro.params:
            self.param_properties.append(name)
            if name in macro.defaultmap:
                # Default given.
                add_property(obj, 'App::PropertyString', name, 'Input',
                             f'Macro parameter "{name}"',
                             macro.defaultmap[name][1])
            else:
                # No default.
                add_property(obj, 'App::PropertyString', name, 'Input',
                             f'Macro parameter "{name}"')

        # Clear old parameters.
        for name in old_param_properties:
            if name not in self.param_properties:
                obj.removeProperty(name)

    def reset_group(self, obj: RosXacroObject):
        if not hasallattr(obj, ['Group', 'InputFile', 'MainMacro']):
            return
        new_robot = self._generate_robot(obj)
        if new_robot and obj.Group:
            old_robot = self._get_robot()
            if old_robot:
                # Delete the old robot.
                _clear_robot(old_robot)
                # Necessary to free the label for the new object object because
                # the object is removed later.
                old_robot.Label = 'will_be_removed'
                obj.Document.removeObject(old_robot.Name)
        if new_robot:
            new_robot.Label = f'Robot_of_{obj.Label}'
            # Add the new robot.
            # We add the robot by setting `Group` because of the bug described
            # below. When solved, use next line instead.
            # obj.addObject(new_robot)
            # Clean-up other objects.
            # The xacro objects contains other objects for some reason, remove
            # them from the group.
            # TODO: not nice, try to solve out why this is necessary.
            obj.Group = [new_robot]

    def export_urdf(self) -> Optional[et.Element]:
        """Export the xacro object as URDF, writing files."""
        if not hasattr(self, 'xacro_object'):
            return
        obj: RosXacroObject = self.xacro_object

        # Generate the URDF to obtain the root link.
        robot_name = ros_name(obj)
        params = {name: obj.getPropertyByName(name) for name in self.param_properties}

        # Generate the xacro xml document.
        xacro_xml = self.xacro.to_xml(robot_name, obj.MainMacro, params)

        return et.fromstring(xacro_xml.toxml())

    def get_link_names(self) -> list[str]:
        if not self._urdf_robot:
            return []
        return [link.name for link in self._urdf_robot.links]

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
        self._urdf_robot = self._generate_urdf(ros_name(obj), obj.MainMacro, params)
        self._root_link = self._urdf_robot.get_root()
        robot = robot_from_urdf(obj.Document, self._urdf_robot)
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

    def _get_robot(self) -> Optional[RosRobot]:
        if ((not hasattr(self, 'xacro_object'))
                or (not hasattr(self.xacro_object, 'Group'))):
            return
        for obj in self.xacro_object.Group:
            if is_robot(obj):
                return obj


class _ViewProviderXacroObject(ProxyBase):
    """A view provider for the ROS XacroObject object."""

    def __init__(self, vobj: VPDO):
        super().__init__('view_object', [
            'Visibility',
            ])
        vobj.Proxy = self

    def getIcon(self):
        # Implementation note: "return 'xacro.svg'" works only after
        # workbench activation in GUI.
        return str(ICON_PATH / 'xacro.svg')

    def attach(self, vobj: VPDO):
        self.ViewObject = vobj
        # vobj.addExtension('Gui::ViewProviderGroupExtensionPython')
        vobj.addExtension('Gui::ViewProviderGeoFeatureGroupExtensionPython')

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
    # obj = doc.addObject('App::FeaturePython', name)
    obj = doc.addObject('Part::FeaturePython', name)
    XacroObject(obj)

    if hasattr(fc, 'GuiUp') and fc.GuiUp:
        import FreeCADGui as fcgui

        _ViewProviderXacroObject(obj.ViewObject)

        # Make `obj` part of the selected `Ros::Workcell`.
        sel = fcgui.Selection.getSelection()
        if sel:
            candidate = sel[0]
            if (is_robot(candidate)
                    or is_workcell(candidate)):
                obj.adjustRelativeLinks(candidate)
                candidate.addObject(obj)

    doc.recompute()
    return obj
