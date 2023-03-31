from __future__ import annotations

from typing import Optional

import FreeCAD as fc

from .freecad_utils import add_property
from .freecad_utils import warn

# Typing hints.
DO = fc.DocumentObject
VPDO = 'FreeCADGui.ViewProviderDocumentObject'
RosXacroObject = DO  # A Ros::XacroObject, i.e. a DocumentObject with Proxy "Xacro".


class XacroObject:
    """The XacroObject proxy."""

    # The member is often used in workbenches, particularly in the Draft
    # workbench, to identify the object type.
    Type = 'Ros::XacroObject'

    def __init__(self, obj: RosXacroObject):
        obj.Proxy = self
        self.xacro = obj

        self.init_properties(obj)
        self.init_extensions(obj)

    def init_properties(self, obj: RosXacroObject):
        add_property(obj, 'App::PropertyString', '_Type', 'Internal',
                     'The type')
        obj.setPropertyStatus('_Type', ['Hidden', 'ReadOnly'])
        obj._Type = self.Type

        # Managed in self.reset_group().
        obj.setPropertyStatus('Group', 'ReadOnly')

        add_property(obj, 'App::PropertyFile', 'Input', 'InputFile',
                     'The source xacro or URDF file')
        add_property(obj, 'App::PropertyFile', 'Input', 'MainMacro',
                     'The macro to use')

    def init_extensions(self, obj: RosXacroObject):
        # Needed to make this object able to attach parameterically to other objects.
        obj.addExtension('Part::AttachExtensionPython', obj)
        # Need a group to put the generated robot in.
        obj.addExtension('Part::AttachExtensionPython', obj)

    def execute(self, obj: RosXacroObject) -> None:
        obj.positionBySupport()
        self.reset_group()
        pass

    def onChanged(self, obj: RosXacroObject, prop: str) -> None:
        if prop in ['InputFile']:
            self.execute(obj)

    def onDocumentRestored(self, obj):
        """Restore attributes because __init__ is not called on restore."""
        self.__init__(obj)

    def __getstate__(self):
        return self.Type,

    def __setstate__(self, state):
        if state:
            self.Type, = state

    def reset_group(self):
        if not hasattr(self, 'xacro'):
            return


class _ViewProviderXacroObject:
    """A view provider for the Xacro container object """

    def __init__(self, vobj: VPDO):
        vobj.Proxy = self

    def getIcon(self):
        return 'xacro.svg'

    def attach(self, vobj: VPDO):
        self.ViewObject = vobj

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


def make_xacro(name, doc: Optional[fc.Document] = None) -> RosXacroObject:
    """Add a Ros::Robot to the current document."""
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
