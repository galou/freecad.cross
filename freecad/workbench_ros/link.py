from __future__ import annotations

from pathlib import Path
from typing import Iterable, Optional
import xml.etree.ElementTree as et

import FreeCAD as fc

from .freecad_utils import add_property
from .freecad_utils import error
from .freecad_utils import is_link as is_freecad_link
from .freecad_utils import warn
from .mesh_utils import save_mesh_dae
from .urdf_utils import urdf_collision_from_object
from .urdf_utils import urdf_visual_from_object
from .urdf_utils import XmlForExport
from .utils import ICON_PATH
from .utils import get_joints
from .utils import get_valid_urdf_name
from .utils import is_joint
from .utils import is_link
from .utils import is_primitive
from .utils import is_robot
from .utils import warn_unsupported

# Typing hints.
DO = fc.DocumentObject
DOList = Iterable[DO]
VPDO = 'FreeCADGui.ViewProviderDocumentObject'
RosLink = DO  # A Ros::Link, i.e. a DocumentObject with Proxy "Link".
RosJoint = DO  # A Ros::Joint, i.e. a DocumentObject with Proxy "Joint".
RosRobot = DO  # A Ros::Robot, i.e. a DocumentObject with Proxy "Robot".


def _skim_links_joints_from(group) -> tuple[DOList, DOList]:
    """Remove all Ros::Link and Ros::Joint from the list.

    `group` is a property that looks like a list but behaves differently
    (behaves like a tuple and is a copy of the original property content,
     so cannot be set here).

    Return (kept_objects, removed_objects).

    """
    removed_objects: DOList = []
    kept_objects: DOList = list(group)
    # Implementation note: reverse order required.
    for i, o in reversed(list(enumerate(kept_objects))):
        if is_link(o) or is_joint(o):
            warn_unsupported(o, by='ROS::Link', gui=True)
            # Implementation note: cannot use `kept_objects.remove`, this would
            # lose the object.
            removed_objects.append(kept_objects.pop(i))
    return kept_objects, removed_objects


def _get_xmls_and_export_meshes(obj,
                                urdf_function,
                                placement,
                                package_parent: [Path | str] = Path(),
                                package_name: str = '',
                                ) -> list[et.Element]:
    export_data: XmlForExport = urdf_function(
        obj,
        package_name=str(package_name),
        placement=placement,
        )
    visual_xmls: list[et.Element] = []
    for export_datum in export_data:
        if not is_primitive(export_datum.object):
            mesh_path = (package_parent / package_name
                         / 'meshes' / export_datum.mesh_filename)
            save_mesh_dae(export_datum.object, mesh_path)
        visual_xmls.append(export_datum.xml)
    return visual_xmls


class Link:
    """The Link group."""

    # The member is often used in workbenches, particularly in the Draft
    # workbench, to identify the object type.
    Type = 'Ros::Link'

    def __init__(self, obj: RosLink):
        obj.Proxy = self
        self.link = obj
        self.init_properties(obj)

    def init_properties(self, obj: RosLink):
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

    def execute(self, obj: RosLink):
        pass

    def onBeforeChange(self, obj: RosLink, prop: str) -> None:
        pass

    def onChanged(self, obj: RosLink, prop: str) -> None:
        if prop in ['Group', 'Real', 'Visual', 'Collision']:
            self.cleanup_children()

    def onDocumentRestored(self, obj: RosLink):
        self.__init__(obj)

    def __getstate__(self):
        return self.Type,

    def __setstate__(self, state):
        if state:
            self.Type, = state

    def cleanup_children(self) -> DOList:
        """Remove and return all objects not supported by ROS::Link."""

        if ((not hasattr(self, 'link'))
                or (not is_link(self.link))):
            return
        removed_objects: set[DO] = set()
        # Group is managed by us and the containing robot.
        for o in self.link.Group:
            if is_freecad_link(o):
                # Supported, and managed by us.
                continue
            warn_unsupported(o, by='ROS::Link', gui=True)
            # implementation note: removeobject doesn't raise any exception
            # and `o` exists even if already removed from the group.
            removed_objects.update(self.link.removeObject(o))

        # Clean-up `Visual`, `Real`, `Collision`.
        try:
            kept, removed = _skim_links_joints_from(self.link.Real)
            if self.link.Real != kept:
                self.link.Real = kept
            warn_unsupported(removed, by='ROS::Link', gui=True)
            removed_objects.update(removed)
        except AttributeError:
            pass
        try:
            kept, removed = _skim_links_joints_from(self.link.Visual)
            if self.link.Visual != kept:
                self.link.Visual = kept
            warn_unsupported(removed, by='ROS::Link', gui=True)
            removed_objects.update(removed)
        except AttributeError:
            pass
        try:
            kept, removed = _skim_links_joints_from(self.link.Collision)
            if self.link.Collision != kept:
                self.link.Collision = kept
            warn_unsupported(removed, by='ROS::Link', gui=True)
            removed_objects.update(removed)
        except AttributeError:
            pass
        return list(removed_objects)

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
                    package_parent: Path,
                    package_name: [Path | str],
                    ) -> et.ElementTree:
        """Return the xml for this link.

        Parameters
        ----------
        - package_parent: the parent directory of the package.
        - package_name: the name of the package (also the name of the
                        directory).

        """

        link_xml = et.fromstring(
            f'<link name="{get_valid_urdf_name(self.link.Label)}" />')
        for obj in self.link.Visual:
            for xml in _get_xmls_and_export_meshes(
                    obj,
                    urdf_visual_from_object,
                    self.link.MountedPlacement,
                    package_parent,
                    package_name):
                link_xml.append(xml)
        for obj in self.link.Collision:
            for xml in _get_xmls_and_export_meshes(
                    obj,
                    urdf_collision_from_object,
                    self.link.MountedPlacement,
                    package_parent,
                    package_name):
                link_xml.append(xml)
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
        #import FreeCADGui as fcgui
        #from .task_panel_link import TaskPanelLink
        #task_panel = TaskPanelLink(self.link)
        #fcgui.Control.showDialog(task_panel)
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
