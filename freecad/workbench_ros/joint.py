from __future__ import annotations

from math import degrees, radians
from typing import Iterable, Optional
import xml.etree.ElementTree as et

import FreeCAD as fc

from .freecad_utils import add_property
from .freecad_utils import error
from .freecad_utils import label_or
from .freecad_utils import warn
from .urdf_utils import urdf_origin_from_placement
from .wb_utils import get_valid_urdf_name
from .wb_utils import is_link
from .wb_utils import is_robot
from .wb_utils import is_workcell
from .wb_utils import ros_name

# Typing hints.
DO = fc.DocumentObject
DOList = Iterable[DO]
VPDO = 'FreeCADGui.ViewProviderDocumentObject'
RosJoint = DO  # A Ros::Joint, i.e. a DocumentObject with Proxy "Joint".
RosRobot = DO  # A Ros::Robot, i.e. a DocumentObject with Proxy "Robot".


class Joint:
    """The Ros::Joint object."""

    # The member is often used in workbenches, particularly in the Draft
    # workbench, to identify the object type.
    Type = 'Ros::Joint'

    # The names cannot be changed because they are used as-is in the generated
    # URDF. The order can be changed and influences the order in the GUI.
    # The first element is the default.
    type_enum = ['fixed', 'prismatic', 'revolute', 'continuous', 'planar', 'floating']

    def __init__(self, obj: RosJoint):
        obj.Proxy = self
        self.joint = obj
        self.init_properties(obj)

    def init_properties(self, obj: RosJoint):
        add_property(obj, 'App::PropertyString', '_Type', 'Internal',
                     'The type')
        obj.setPropertyStatus('_Type', ['Hidden', 'ReadOnly'])
        obj._Type = self.Type

        add_property(obj, 'App::PropertyEnumeration', 'Type', 'Elements',
                     'The kinematical type of the joint')
        obj.Type = Joint.type_enum
        add_property(obj, 'App::PropertyEnumeration', 'Parent', 'Elements',
                     'Parent link (from the ROS Workbench)')
        add_property(obj, 'App::PropertyEnumeration', 'Child', 'Elements',
                     'Child link (from the ROS Workbench)')
        add_property(obj, 'App::PropertyPlacement', 'Origin', 'Elements',
                     'Joint origin relative to the parent link')
        add_property(obj, 'App::PropertyFloat', 'LowerLimit', 'Limits',
                     'Lower position limit (m or rad)')
        add_property(obj, 'App::PropertyFloat', 'UpperLimit', 'Limits',
                     'Upper position limit (m or rad)')
        add_property(obj, 'App::PropertyFloat', 'Effort', 'Limits',
                     'Maximal effort (N)')
        add_property(obj, 'App::PropertyFloat', 'Velocity', 'Limits',
                     'Maximal velocity (m/s or rad/s)')
        add_property(obj, 'App::PropertyFloat', 'Position', 'Value',
                     'Joint position (m or rad)')
        obj.setEditorMode('Position', ['ReadOnly'])

        # Mimic joint.
        add_property(obj, 'App::PropertyBool', 'Mimic', 'Mimic',
                     'Whether this joint mimics another one')
        add_property(obj, 'App::PropertyLink', 'MimickedJoint', 'Mimic',
                     'Joint to mimic')
        add_property(obj, 'App::PropertyFloat', 'Multiplier', 'Mimic',
                     'value = Multiplier * other_joint_value + Offset', 1.0)
        add_property(obj, 'App::PropertyFloat', 'Offset', 'Mimic',
                     'value = Multiplier * other_joint_value + Offset, in mm or deg')
        self._toggle_editor_mode()

        add_property(obj, 'App::PropertyPlacement', 'Placement', 'Internal',
                     'Placement of the joint in the robot frame')
        obj.setEditorMode('Placement', ['ReadOnly'])

    def onChanged(self, obj: RosJoint, prop: str) -> None:
        if prop == 'Mimic':
            self._toggle_editor_mode()
        if prop == 'MimickedJoint':
            if obj.MimickedJoint is not None:
                if obj.Type != obj.MimickedJoint.Type:
                    warn('Mimicked joint must have the same type'
                         f' but "{obj.Label}"\'s type is {obj.Type} and'
                         f' "{obj.MimickedJoint}"\'s is {obj.MimickedJoint.Type}',
                         True)
                    obj.MimickedJoint = None
        if prop in ('Label', 'Label2'):
            robot = self.get_robot()
            if robot and hasattr(robot, 'Proxy'):
                robot.Proxy.add_joint_variables()

    def onDocumentRestored(self, obj: RosJoint):
        obj.Proxy = self
        self.joint = obj
        self.init_properties(obj)

    def __getstate__(self):
        return self.Type,

    def __setstate__(self, state):
        if state:
            self.Type, = state

    def get_actuation_placement(self, joint_value=None) -> fc.Placement:
        """Return the transform due to actuation.

        Parameters
        ----------

        - joint_value: joint value in mm or deg.

        """
        # Only actuation around/about z supported.
        if self.joint.Mimic and self.joint.MimickedJoint:
            mult = self.joint.Multiplier
            if self.joint.Type == 'prismatic':
                # User value in mm.
                off = self.joint.Offset / 1000.0
            elif self.joint.Type == 'revolute':
                # User value in deg.
                off = radians(self.joint.Offset)
            else:
                warn('Mimicking joint must be prismatic or revolute', True)
                mult = 0.0
                off = 0.0
            p = self.joint.MimickedJoint.Position
            pos = mult * p + off
            if self.joint.Position != pos:
                # Implementation note: avoid recursion.
                self.joint.Position = pos
        if self.joint.Type == 'prismatic':
            if joint_value is None:
                joint_value = self.joint.Position * 1000.0
            return fc.Placement(fc.Vector(0.0, 0.0, joint_value), fc.Rotation())
        if self.joint.Type in ['revolute', 'continuous']:
            if joint_value is None:
                joint_value = degrees(self.joint.Position)
            return fc.Placement(fc.Vector(),
                                fc.Rotation(fc.Vector(0.0, 0.0, 1.0),
                                            joint_value))
        return fc.Placement()

    def get_robot(self) -> Optional[RosRobot]:
        """Return the Ros::Robot this joint belongs to."""
        if not hasattr(self, 'joint'):
            return
        for o in self.joint.InList:
            if is_robot(o):
                return o

    def get_predecessor(self) -> Optional[RosJoint]:
        """Return the predecessing joint."""
        robot = self.get_robot()
        if robot is None:
            return
        for candidate_joint in robot.Proxy.get_joints():
            child_of_candidate = robot.Proxy.get_link(candidate_joint.Child)
            parent_of_self = robot.Proxy.get_link(self.joint.Parent)
            if child_of_candidate is parent_of_self:
                return candidate_joint

    def export_urdf(self) -> et.ElementTree:
        joint = self.joint
        joint_xml = et.fromstring('<joint/>')
        joint_xml.attrib['name'] = get_valid_urdf_name(ros_name(joint))
        joint_xml.attrib['type'] = joint.Type
        if joint.Parent:
            joint_xml.append(et.fromstring(f'<parent link="{get_valid_urdf_name(joint.Parent)}"/>'))
        else:
            joint_xml.append(et.fromstring('<parent link="NO_PARENT_DEFINED"/>'))
        if joint.Child:
            joint_xml.append(et.fromstring(f'<child link="{get_valid_urdf_name(joint.Child)}"/>'))
        else:
            joint_xml.append(et.fromstring('<child link="NO_CHILD_DEFINED"/>'))
        joint_xml.append(urdf_origin_from_placement(joint.Origin))
        joint_xml.append(et.fromstring('<axis xyz="0 0 1" />'))
        limit_xml = et.fromstring('<limit/>')
        limit_xml.attrib['lower'] = str(joint.LowerLimit)
        limit_xml.attrib['upper'] = str(joint.UpperLimit)
        limit_xml.attrib['effort'] = str(joint.Effort)
        limit_xml.attrib['velocity'] = str(joint.Velocity)
        joint_xml.append(limit_xml)
        if joint.Mimic:
            mimic_xml = et.fromstring('<mimic/>')
            mimic_joint = label_or(joint.MimickedJoint, 'no_joint_defined')
            mimic_xml.attrib['joint'] = get_valid_urdf_name(mimic_joint)
            mimic_xml.attrib['multiplier'] = str(joint.Multiplier)
            if joint.Type == 'prismatic':
                # Millimeters (FreeCAD) to meters (URDF).
                urdf_offset = joint.Offset / 1000.0
            else:
                # Should be only 'revolute' or 'continuous'.
                # Degrees (FreeCAD) to meters (URDF).
                urdf_offset = radians(joint.Offset)
            mimic_xml.attrib['offset'] = str(urdf_offset)
            joint_xml.append(mimic_xml)
        return joint_xml

    def _toggle_editor_mode(self):
        joint = self.joint
        if joint.Mimic:
            editor_mode = []
        else:
            editor_mode = ['Hidden', 'ReadOnly']
        if hasattr(joint, 'MimickedJoint'):
            joint.setEditorMode('MimickedJoint', editor_mode)
        if hasattr(joint, 'Multiplier'):
            joint.setEditorMode('Multiplier', editor_mode)
        if hasattr(joint, 'Offset'):
            joint.setEditorMode('Offset', editor_mode)


class _ViewProviderJoint:
    """A view provider for the Joint container object """

    def __init__(self, vobj: VPDO):
        vobj.Proxy = self
        self.set_properties(vobj)

    def set_properties(self, vobj: VPDO):
        """Set properties of the view provider."""
        add_property(vobj, 'App::PropertyBool', 'ShowAxis',
                     'ROS Display Options',
                     "Toggle the display of the joint's Z-axis",
                     True)
        add_property(vobj, 'App::PropertyLength', 'AxisLength',
                     'ROS Display Options',
                     "Length of the arrow for the joint's axis",
                     500.0)

    def getIcon(self):
        return 'joint.svg'

    def attach(self, vobj):
        """Setup the scene sub-graph of the view provider."""
        pass

    def updateData(self,
                   obj: RosJoint,
                   prop: str):
        vobj = obj.ViewObject
        if not hasattr(vobj, 'Visibility'):
            return
        if not vobj.Visibility or not hasattr(vobj, 'ShowAxis'):
            # root_node.removeAllChildren() # This segfaults when loading the document.
            return
        if prop in ['Placement']:
            self.draw(vobj, vobj.Visibility and vobj.ShowAxis)
        # Implementation note: no need to react on prop == 'Origin' because
        # this triggers a change in 'Placement'.

    def onChanged(self, vobj: VPDO, prop: str):
        if prop in ('ShowAxis', 'AxisLength'):
            self.draw(vobj, vobj.ShowAxis)

    def draw(self, vobj: VPDO, visible: bool):
        from .coin_utils import arrow_group

        if not hasattr(vobj, 'RootNode'):
            return
        root_node = vobj.RootNode
        root_node.removeAllChildren()
        if not visible:
            return
        if not hasattr(vobj.Object, 'Placement'):
            return
        placement = vobj.Object.Placement
        if placement is None:
            return
        if vobj.Object.Type == 'fixed':
            color = (0.0, 0.0, 0.7)
        else:
            color = (0.0, 0.0, 1.0)
        if hasattr(vobj, 'AxisLength'):
            length = vobj.AxisLength.Value
        else:
            length = 1000.0
        p0 = placement.Base
        pz = placement * fc.Vector(0.0, 0.0, length)
        arrow = arrow_group([p0, pz], scale=0.2, color=color)
        root_node.addChild(arrow)
        px = placement * fc.Vector(length / 2.0, 0.0, 0.0)
        arrow = arrow_group([p0, px], scale=0.2, color=(1.0, 0.0, 0.0))
        root_node.addChild(arrow)
        py = placement * fc.Vector(0.0, length / 2.0, 0.0)
        arrow = arrow_group([p0, py], scale=0.2, color=(0.0, 1.0, 0.0))
        root_node.addChild(arrow)

    def doubleClicked(self, vobj):
        gui_doc = vobj.Document
        if not gui_doc.getInEdit():
            gui_doc.setEdit(vobj.Object.Name)
        else:
            error('Task dialog already active')
        return True

    def setEdit(self, vobj, mode):
        return False

    def unsetEdit(self, vobj, mode):
        import FreeCADGui as fcgui
        fcgui.Control.closeDialog()

    def __getstate__(self):
        return

    def __setstate__(self, state):
        return


def make_joint(name, doc: Optional[fc.Document] = None) -> RosJoint:
    """Add a Ros::Joint to the current document."""
    if doc is None:
        doc = fc.activeDocument()
    if doc is None:
        warn('No active document, doing nothing', False)
        return
    obj = doc.addObject('App::FeaturePython', name)
    Joint(obj)
    # Default to type "fixed".
    obj.Type = 'fixed'

    if hasattr(fc, 'GuiUp') and fc.GuiUp:
        import FreeCADGui as fcgui

        _ViewProviderJoint(obj.ViewObject)

        # Make `obj` part of the selected `Ros::Robot`.
        sel = fcgui.Selection.getSelection()
        if sel:
            candidate = sel[0]
            if (is_robot(candidate)
                    or is_workcell(candidate)):
                obj.adjustRelativeLinks(candidate)
                candidate.addObject(obj)
            elif is_link(candidate):
                robot = candidate.Proxy.get_robot()
                if robot:
                    obj.adjustRelativeLinks(robot)
                    robot.addObject(obj)
                    obj.Parent = ros_name(candidate)
    return obj
