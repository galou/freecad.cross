
import FreeCAD as fc

from .utils import ICONPATH


class Robot:
    """The Robot group."""
    def __init__(self, obj):
        obj.Proxy = self
        obj.Type = 'Ros::Robot'
        self.initProperties(obj)

    def initProperties(self, obj):
        obj.addProperty('App::PropertyPath', 'OutputPath', 'Export')

    def onDocumenRestored(self, obj):
        self.initProperties(obj)


class _ViewProviderRobot:
    """A view provider for the Robot container object """
    def __init__(self, vobj):
        vobj.Proxy = self

    def getIcon(self):
        return str(ICONPATH.joinpath('ros_9dotslogo_color.svg'))

    def attach(self, vobj):
        self.ViewObject = vobj
        self.Object = vobj.Object
        self.bubbles = None

    def updateData(self, obj, prop):
        return

    def onChanged(self, vobj, prop):
        return

    def doubleClicked(self, vobj):
        return True

    def __getstate__(self):
        return None

    def __setstate__(self, state):
        return None


def makeRobot(name):
    """Create a robot."""
    doc = fc.activeDocument()
    if not doc:
        return
    obj = doc.addObject('App::DocumentObjectGroupPython', name)
    Robot(obj)

    if fc.GuiUp:
        _ViewProviderRobot(obj.ViewObject)
    return obj

