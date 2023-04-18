"""Proxy for a RosWorkcell, i.e. a combination of RosXacroObject and RosJoint

A RosWorkcell allows to combine existing URDF and xacro file to generate a
single robot description (or more generally a workcell description).
Joints must be defined between the included RosXacroObject.

"""

from typing import Iterable, Optional

import FreeCAD as fc

from .freecad_utils import add_property
from .freecad_utils import warn

# Typing hints.
DO = fc.DocumentObject
VPDO = 'FreeCADGui.ViewProviderDocumentObject'
DOList = Iterable[DO]
RosJoint = DO  # A Ros::Joint, i.e. a DocumentObject with Proxy "Joint".
RosWorkcell = DO  # A Ros::Workcell, i.e. a DocumentObject with Proxy "Workcell".


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

    def execute(self, obj: RosWorkcell) -> None:
        pass

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
