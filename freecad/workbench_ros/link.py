from pathlib import Path
from typing import Optional
import xml.etree.ElementTree as et

import FreeCAD as fc

from .export_urdf import urdf_collision_from_object
from .export_urdf import urdf_visual_from_object
from .utils import ICON_PATH
from .utils import add_property
from .utils import error
from .utils import get_joints
from .utils import is_primitive
from .utils import is_robot
from .utils import save_mesh_dae
from .utils import get_valid_filename
from .utils import get_valid_urdf_name
from .utils import warn

# Typing hints.
DO = fc.DocumentObject
DOG = fc.DocumentObjectGroup
VPDO = 'FreeCADGui.ViewProviderDocumentObject'


class Link:
    """The Link group."""

    # The member is often used in workbenches, particularly in the Draft
    # workbench, to identify the object type.
    Type = 'Ros::Link'

    def __init__(self, obj: DOG):
        obj.Proxy = self
        self.link = obj
        self.init_properties(obj)

    def init_properties(self, obj: DOG):
        add_property(obj, 'App::PropertyString', '_Type', 'Internal',
                     'The type')
        obj.setPropertyStatus('_Type', ['Hidden', 'ReadOnly'])
        obj._Type = self.Type

        add_property(obj, 'App::PropertyLinkList', 'Real', 'Elements',
                     'The real part objects of this link, optional')
        add_property(obj, 'App::PropertyLinkList', 'Visual', 'Elements',
                     'The part objects this link that constitutes the URDF'
                     ' visual elements, optional')
        add_property(obj, 'App::PropertyLinkList', 'Collision', 'Elements',
                     'The part objects this link that constitutes the URDF'
                     ' collision elements, optional')

        add_property(obj, 'App::PropertyPlacement', 'Placement', 'Internal',
                     'Placement of elements in the robot frame')
        obj.setEditorMode('Placement', ['ReadOnly'])

        # Used when adding a link which shape in located at the origin but
        # looks correctly placed. For example, when opening a STEP file or a
        # mesh with all links at the mounted position.
        # This placement is the transform from origin to the location of the
        # joint that is parent of this link.
        add_property(obj, 'App::PropertyPlacement', 'MountedPlacement',
                     'Internal', 'Placement when building')

    def execute(self, obj: DOG):
        pass

    def onBeforeChange(self, obj: DOG, prop: str) -> None:
        if not hasattr(self, 'link'):
            return
        if prop == 'Real':
            self.real_was_empty = hasattr(self.link, 'Real') and not(self.link.Real)

    def onChanged(self, obj: DOG, prop: str) -> None:
        if not hasattr(self, 'link'):
            return

    def onDocumentRestored(self, obj: DOG):
        # obj.Proxy = self
        # self.link = obj
        # self.init_properties(obj)
        self.__init__(obj)

    def __getstate__(self):
        return self.Type

    def __setstate__(self, state):
        if state:
            self.Type, = state

    def get_robot(self) -> Optional[DO]:
        """Return the Ros::Robot this link belongs to."""
        if not hasattr(self, 'link'):
            return
        for o in self.link.InList:
            if is_robot(o):
                return o

    def get_ref_joint(self) -> Optional[DO]:
        """Return the joint this link is the child of."""
        robot = self.get_robot()
        if robot is None:
            return
        joints = get_joints(robot.Group)
        for joint in joints:
            if joint.Child is self.link:
                # Parallel mechanisms are not supported, there should be only
                # one joint that has `link` as child.
                return joint

    def may_be_base_link(self) -> bool:
        """Return True if the link is child of no joint."""
        return self.get_ref_joint() is None

    def is_tip_link(self) -> bool:
        """Return True if the link is parent of no joint."""
        robot = self.get_robot()
        if robot is None:
            return True
        joints = get_joints(robot.Group)
        for joint in joints:
            if joint.Parent is self.link:
                return False
        return True

    def export_urdf(self,
                    package_path: Path,
                    package_name: Path,
                    placement: fc.Placement = fc.Placement(),
                    ) -> et.ElementTree:
        """Return the xml for this link.

        Parameters
        ----------
        - package_path: the parent directory of the package.
        - package_name: the name of the package (also the name of the
                        directory).
        - placement: the placement of the link relative to the joint of the
                     previous link.

        """

        def get_xml(obj, urdf_function):
            mesh_name = get_valid_filename(obj.Label) + '.dae'
            visual_xml = urdf_function(
                obj,
                mesh_name=mesh_name,
                package_name=str(package_name),
                placement=placement,
                )
            if not is_primitive(obj):
                mesh_path = package_path / package_name / 'meshes' / mesh_name
                save_mesh_dae(obj.LinkedObject, mesh_path)
            return visual_xml

        link_xml = et.fromstring(
            f'<link name="{get_valid_urdf_name(self.link.Label)}" />')
        for link in self.link.Visual:
            link_xml.append(get_xml(link, urdf_visual_from_object))
        for link in self.link.Collision:
            link_xml.append(get_xml(link, urdf_collision_from_object))
        return link_xml


class _ViewProviderLink:
    """A view provider for the Link container object """

    def __init__(self, vobj: VPDO):
        vobj.Proxy = self

    def getIcon(self):
        # TODO: Solve why this doesn't work.
        # return 'link.svg'
        return str(ICON_PATH / 'link.svg')

    def attach(self, vobj: VPDO):
        self.ViewObject = vobj
        self.link = vobj.Object

    def updateData(self, obj: VPDO, prop):
        return

    def onChanged(self, vobj: VPDO, prop: str):
        return

    def doubleClicked(self, vobj: VPDO):
        gui_doc = vobj.Document
        if not gui_doc.getInEdit():
            gui_doc.setEdit(vobj.Object.Name)
        else:
            error('Task dialog already active')
        return True

    def setEdit(self, vobj: VPDO, mode):
        import FreeCADGui as fcgui
        from .task_panel_link import TaskPanelLink
        task_panel = TaskPanelLink(self.link)
        fcgui.Control.showDialog(task_panel)
        return True

    def unsetEdit(self, vobj: VPDO, mode):
        import FreeCADGui as fcgui
        fcgui.Control.closeDialog()
        return

    def __getstate__(self):
        return None

    def __setstate__(self, state):
        return None


def make_link(name, doc: Optional[fc.Document] = None) -> DO:
    """Add a Ros::Link to the current document."""
    if doc is None:
        doc = fc.activeDocument()
    if doc is None:
        warn('No active document, doing nothing', False)
        return
    obj = doc.addObject('App::DocumentObjectGroupPython', name)
    Link(obj)

    if hasattr(fc, 'GuiUp') and fc.GuiUp:
        import FreeCADGui as fcgui

        _ViewProviderLink(obj.ViewObject)

        # Make `obj` part of the selected `Ros::Robot`.
        sel = fcgui.Selection.getSelection()
        if sel:
            candidate = sel[0]
            if is_robot(candidate):
                obj.adjustRelativeLinks(candidate)
                candidate.addObject(obj)

    return obj
