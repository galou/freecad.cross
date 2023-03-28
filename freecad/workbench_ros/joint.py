from __future__ import annotations

from math import degrees
from typing import Iterable, Optional
import xml.etree.ElementTree as et

import FreeCAD as fc

from .freecad_utils import add_property
from .freecad_utils import error
from .freecad_utils import warn
from .urdf_utils import urdf_origin_from_placement
from .utils import get_joints
from .utils import get_valid_urdf_name
from .utils import is_joint
from .utils import is_link
from .utils import is_robot
from .utils import warn_unsupported

# Typing hints.
DO = fc.DocumentObject
DOList = Iterable[DO]
VPDO = 'FreeCADGui.ViewProviderDocumentObject'
RosJoint = DO  # A Ros::Joint, i.e. a DocumentObject with Proxy "Joint".


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
        add_property(obj, 'App::PropertyLink', 'Parent', 'Elements',
                     'Parent link (from the ROS Workbench)')
        add_property(obj, 'App::PropertyLink', 'Child', 'Elements',
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

        add_property(obj, 'App::PropertyPlacement', 'Placement', 'Internal',
                     'Placement of the joint in the robot frame')
        obj.setEditorMode('Placement', ['ReadOnly'])

    def onChanged(self, obj: RosJoint, prop: str) -> None:
        if prop in ['Child', 'Parent']:
            self.cleanup_children()

    def cleanup_children(self) -> DOList:
        """Remove and return all objects not supported by ROS::Joint."""
        if ((not hasattr(self, 'joint'))
                or not is_joint(self.joint)):
            return

        try:
            if self.joint.Child and (not is_link(self.joint.Child)):
                warn_unsupported(self.joint.Child, by='ROS::Joint', gui=True)
                # Reject the current child.
                self.joint.Child = None
        except AttributeError:
            pass

        try:
            if self.joint.Parent and (not is_link(self.joint.Parent)):
                warn_unsupported(self.joint.Parent, by='ROS::Joint', gui=True)
                # Reject the current parent.
                self.joint.Parent = None
        except AttributeError:
            pass

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
        """Return the transform due to actuation."""
        # Only actuation around/about z supported.
        if self.joint.Type == 'prismatic':
            if joint_value is None:
                joint_value = self.joint.Position * 1000.0
            return fc.Placement(fc.Vector(0.0, 0.0, joint_value), fc.Rotation())
        if self.joint.Type == 'revolute':
            if joint_value is None:
                joint_value = self.joint.Position
            return fc.Placement(fc.Vector(),
                                fc.Rotation(fc.Vector(0.0, 0.0, 1.0),
                                            degrees(joint_value)))
        return fc.Placement()

    def get_robot(self) -> DO:
        """Return the Ros::Robot this joint belongs to."""
        if not hasattr(self, 'joint'):
            return
        for o in self.joint.InList:
            if is_robot(o):
                return o

    def get_predecessor(self) -> DO:
        """Return the predecessing joint."""
        robot = self.get_robot()
        if robot is None:
            return
        for candidate_joint in get_joints(robot.Group):
            if ((candidate_joint.Child is not None)
               and (candidate_joint.Child is self.joint.Parent)):
                return candidate_joint

    def export_urdf(self) -> et.ElementTree:
        joint = self.joint
        joint_xml = et.fromstring('<joint/>')
        joint_xml.attrib['name'] = get_valid_urdf_name(joint.Label)
        joint_xml.attrib['type'] = joint.Type
        if joint.Parent:
            joint_xml.append(et.fromstring(f'<parent link="{get_valid_urdf_name(joint.Parent.Label)}"/>'))
        else:
            joint_xml.append(et.fromstring('<parent link="NO_PARENT_DEFINED"/>'))
        if joint.Child:
            joint_xml.append(et.fromstring(f'<child link="{get_valid_urdf_name(joint.Child.Label)}"/>'))
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
        return joint_xml


class _ViewProviderJoint:
    """A view provider for the Joint container object """

    def __init__(self, vobj: VPDO):
        vobj.Proxy = self
        self.set_properties(vobj)

    def set_properties(self, vobj: VPDO):
        """Set properties of the view provider."""
        add_property(vobj, 'App::PropertyBool', 'ShowJointAxis',
                     'Display Options',
                     "Toggle the display of the joint's Z-axis")

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
        if not vobj.Visibility or not hasattr(vobj, 'ShowJointAxis'):
            # root_node.removeAllChildren() # This segfaults when loading the document.
            return
        if prop in ['Placement']:
            self.draw_arrow(vobj, vobj.Visibility and vobj.ShowJointAxis)
        # Implementation note: no need to react on prop == 'Origin' because
        # this triggers a change in 'Placement'.

    def onChanged(self, vobj: VPDO, prop: str):
        if prop == 'ShowJointAxis':
            self.draw_arrow(vobj, vobj.ShowJointAxis)
        return

    def draw_arrow(self, vobj: VPDO, visible: bool):
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
        p0 = placement.Base
        p1 = placement * fc.Vector(0.0, 0.0, 1000.0)
        arrow = arrow_group([p0, p1], scale=0.2, color=color)
        root_node.addChild(arrow)

    def doubleClicked(self, vobj):
        gui_doc = vobj.Document
        if not gui_doc.getInEdit():
            gui_doc.setEdit(vobj.Object.Name)
        else:
            error('Task dialog already active')
        return True

    def setEdit(self, vobj, mode):
        # import FreeCADGui as fcgui
        # from .task_panel_joint import TaskPanelJoint
        # task_panel = TaskPanelJoint(self.joint)
        # fcgui.Control.showDialog(task_panel)
        return True

    def unsetEdit(self, vobj, mode):
        import FreeCADGui as fcgui
        fcgui.Control.closeDialog()
        return

    def __getstate__(self):
        return None

    def __setstate__(self, state):
        return None


def make_joint(name, doc: Optional[fc.Document] = None) -> DO:
    """Add a Ros::Joint to the current document."""
    if doc is None:
        doc = fc.activeDocument()
    if doc is None:
        warn('No active document, doing nothing', False)
        return
    obj = doc.addObject('Part::FeaturePython', name)
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
            if is_robot(candidate):
                obj.adjustRelativeLinks(candidate)
                candidate.addObject(obj)

    return obj
