
import FreeCAD as fc

from .utils import ICON_PATH
from .utils import add_property
from .utils import error


class Link:
    """The Link group."""
    def __init__(self, obj):
        obj.Proxy = self
        self.link = obj
        self.type = 'Ros::Link'
        self.init_properties(obj)

    def init_properties(self, obj):
        add_property(obj, 'App::PropertyString', 'Type', 'Internal',
                    'The type').Type = self.type
        obj.setEditorMode('Type', 3)  # Make read-only and hidden.

        add_property(obj, 'App::PropertyLinkList', 'Real', 'Elements',
                    'The real part objects of this link, optional')
        add_property(obj, 'App::PropertyLinkList', 'Visual', 'Elements',
                    'The part objects this link that consistute the URDF visual elements, optional')
        add_property(obj, 'App::PropertyLinkList', 'Collision', 'Elements',
                    'The part objects this link that consistute the URDF collision elements, optional')

    def execute(self, obj):
        pass

    def onChanged(self, feature: fc.DocumentObjectGroup, prop: str) -> None:
        print(f'Link::onChanged({feature.Name}, {prop})')

    def onDocumentRestored(self, obj):
        obj.Proxy = self
        self.link = obj
        self.type = 'Ros::Link'
        self.init_properties(obj)

    def __getstate__(self):
        return None

    def __setstate__(self, state):
        return None


class _ViewProviderLink:
    """A view provider for the Link container object """
    def __init__(self, vobj):
        vobj.Proxy = self

    def getIcon(self):
        # TODO: Solve why this doesn't work.
        # return 'ros_9dotslogo_color.svg'
        return str(ICON_PATH.joinpath('ros_9dotslogo_color.svg'))

    def attach(self, vobj):
        self.ViewObject = vobj
        self.link = vobj.Object

    def updateData(self, obj, prop):
        return

    def onChanged(self, vobj, prop):
        return

    def doubleClicked(self, vobj):
        import FreeCADGui as fcgui
        gui_doc = vobj.Document
        if not gui_doc.getInEdit():
            gui_doc.setEdit(vobj.Object.Name)
        else:
            error('Task dialog already active')
        return True

    def setEdit(self, vobj, mode):
        import FreeCADGui as fcgui
        from .task_panel_link import TaskPanelLink
        task_panel = TaskPanelLink(self.link)
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


def makeLink(name):
    """Add a Ros::Link to the current document."""
    doc = fc.activeDocument()
    if not doc:
        return
    obj = doc.addObject('App::DocumentObjectGroupPython', name)
    Link(obj)

    if fc.GuiUp:
        import FreeCADGui as fcgui

        _ViewProviderLink(obj.ViewObject)

        # Make `obj` part of the selected `Ros::Robot`.
        sel = fcgui.Selection.getSelection()
        if sel:
            candidate = sel[0]
            if hasattr(candidate, 'Type') and candidate.Type == 'Ros::Robot':
                obj.adjustRelativeLinks(candidate)
                candidate.addObject(obj)

    return obj

