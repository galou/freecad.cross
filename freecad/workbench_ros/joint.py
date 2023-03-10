from typing import Optional
import xml.etree.ElementTree as et

import FreeCAD as fc

from .utils import ICON_PATH
from .utils import add_property
from .utils import error
from .utils import get_joints
from .utils import is_robot
from .utils import get_valid_urdf_name
from .utils import warn
from .export_urdf import urdf_origin_from_placement


class Joint:
    """The Ros::Joint object."""

    type = 'Ros::Joint'

    # The names cannot be changed because they are used as-is in the generated
    # URDF. The order can be changed an influence the order in the GUI.
    type_enum = ['revolute', 'continuous', 'prismatic', 'fixed', 'floating', 'planar']

    def __init__(self, obj):
        obj.Proxy = self
        self.joint = obj
        self.init_properties(obj)

    def init_properties(self, obj):
        add_property(obj, 'App::PropertyString', '_Type', 'Internal',
                     'The type')
        obj.setPropertyStatus('_Type', ['Hidden', 'ReadOnly'])
        obj._Type = self.type

        add_property(obj, 'App::PropertyEnumeration', 'Type', 'Elements', 'The kinematical type of the joint')
        obj.Type = Joint.type_enum
        add_property(obj, 'App::PropertyLink', 'Parent', 'Elements', 'Parent link (from the ROS Workbench)')
        add_property(obj, 'App::PropertyLink', 'Child', 'Elements', 'Child link (from the ROS Workbench)')
        add_property(obj, 'App::PropertyPlacement', 'Origin', 'Elements', 'Joint origin relative to the parent link')
        add_property(obj, 'App::PropertyFloat', 'LowerLimit', 'Limits', 'Lower position limit (m or rad)')
        add_property(obj, 'App::PropertyFloat', 'UpperLimit', 'Limits', 'Upper position limit (m or rad)')
        add_property(obj, 'App::PropertyFloat', 'Effort', 'Limits', 'Maximal effort (N)')
        add_property(obj, 'App::PropertyFloat', 'Velocity', 'Limits', 'Maximal velocity (m/s or rad/s)')

    def onChanged(self, feature: fc.DocumentObjectGroup, prop: str) -> None:
        # print(f'Joint::onChanged({feature.Name}, {prop})') # DEBUG
        pass

    def onDocumentRestored(self, obj):
        obj.Proxy = self
        self.joint = obj
        self.init_properties(obj)

    def __getstate__(self):
        return None

    def __setstate__(self, state):
        return None

    def get_placement(self) -> Optional[fc.Placement]:
        """Return the absolute joint placement."""
        predecessor = self.get_predecessor()
        if predecessor is None:
            # First joint in the chain.
            try:
                return self.joint.Parent.MountedPlacement * self.joint.Origin
            except (AttributeError, NotImplementedError):
                error('Joint.get_placement(), ERROR')
                return
        if predecessor is self.joint:
            error(f'Joint.get_placement(), ERROR, `{self.joint.Name}` is predecessor of itself')
            return
        pred_placement = predecessor.Proxy.get_placement()
        warn(f'{self.joint.Name}: {pred_placement=}') # DEBUG
        if pred_placement is None:
            return
        return pred_placement * self.joint.Origin

    def get_robot(self) -> fc.DocumentObject:
        """Return the Ros::Robot this joint belongs to."""
        if not hasattr(self, 'joint'):
            return
        for o in self.joint.InList:
            if is_robot(o):
                return o

    def get_predecessor(self) -> fc.DocumentObject:
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

    def __init__(self, vobj):
        vobj.Proxy = self

    def getIcon(self):
        # TODO: Solve why this doesn't work.
        # return 'ros_9dotslogo_color.svg'
        return str(ICON_PATH.joinpath('ros_9dotslogo_color.svg'))

    def attach(self, vobj):
        """Setup the scene sub-graph of the view provider."""
        pass

    def updateData(self,
            obj: 'FeaturePython',
            prop: str):
        from .coin_utils import arrow_group

        vobj = obj.ViewObject
        root_node = vobj.RootNode
        if not hasattr(vobj, 'Visibility'):
            return
        if not vobj.Visibility:
            # root_node.removeAllChildren() # This segfaults when loading the document.
            return
        if prop == 'Origin':
            placement = obj.Proxy.get_placement()
            if placement is None:
                return
            p0 = placement.Base
            p1 = placement * fc.Vector(0.0, 0.0, 1000.0)
            arrow = arrow_group([p0, p1], scale=0.2)
            root_node.removeAllChildren()
            root_node.addChild(arrow)

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


def makeJoint(name):
    """Add a Ros::Joint to the current document."""
    doc = fc.activeDocument()
    if doc is None:
        return
    obj = doc.addObject('Part::FeaturePython', name)
    Joint(obj)

    if fc.GuiUp:
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
