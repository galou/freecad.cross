from pathlib import Path
from typing import List
import xml.etree.ElementTree as et

import FreeCAD as fc

from .export_urdf import urdf_collision_from_object
from .export_urdf import urdf_visual_from_object
from .utils import ICON_PATH
from .utils import add_property
from .utils import error
from .utils import get_placement
from .utils import hasallattr
from .utils import is_primitive
from .utils import save_mesh
from .utils import valid_filename
from .utils import valid_urdf_name


def _get_placement_from(
        moving_object: fc.DocumentObject,
        ref_objects: List[fc.DocumentObject]) -> fc.Placement:
    """Return the placement to put moving_object to ref_objects.

    Parameters
    ----------
    - moving_object: object to be placed. Link.Visual or Link.Collision
      expected.
    - ref_objects: Link.Real or Link.Visual. Only the first item is used.

    """
    if (not ref_objects):
        # Empty list.
        return None
    first_link_ref = ref_objects[0]
    link_placement = first_link_ref.LinkPlacement
    print(f'{link_placement = }') # DEBUG
    own_placement = get_placement(moving_object)
    print(f'{own_placement = }') # DEBUG
    print(f'{link_placement * own_placement = }')
    return link_placement * own_placement


class Link:
    """The Link group."""

    type = 'Ros::Link'

    # The names can be changed but not the order. Names can be added.
    collision_placement_enum = ['Own', 'From Real', 'From Visual']
    visual_placement_enum = ['Own', 'From Real']

    def __init__(self, obj):
        obj.Proxy = self
        self.link = obj
        self.init_properties(obj)

    def init_properties(self, obj):
        add_property(obj, 'App::PropertyString', '_Type', 'Internal',
                     'The type')._Type = self.type
        obj.setEditorMode('_Type', 3)  # Make read-only and hidden.

        add_property(obj, 'App::PropertyLinkList', 'Real', 'Elements',
                     'The real part objects of this link, optional')
        add_property(obj, 'App::PropertyLinkList', 'Visual', 'Elements',
                     'The part objects this link that consistute the URDF visual elements, optional')
        add_property(obj, 'App::PropertyLinkList', 'Collision', 'Elements',
                     'The part objects this link that consistute the URDF collision elements, optional')
        add_property(obj, 'App::PropertyEnumeration', 'CollisionPlacement', 'Elements', 'Placement of Collision')
        obj.CollisionPlacement = Link.collision_placement_enum
        add_property(obj, 'App::PropertyEnumeration', 'VisualPlacement', 'Elements', 'Placement of Visual')
        obj.VisualPlacement = Link.visual_placement_enum

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

    def get_link_placement(self, lod: str, index: int):
        """Return the placement of a linked object.

        Parameters
        ----------
        - lod: level of detail ∈ {'real', 'visual', 'collision'}.
        - index: index of the object in self.{Real,Visual,Collision}.

        """
        map = {
            'real': self.get_real_placement,
            'visual': self.get_visual_placement,
            'collision': self.get_collision_placement,
        }
        return map[lod.lower()](index)

    def get_real_placement(self, index: int):
        """Return the placement of a linked real object.

        Parameters
        ----------
        - index: index of the object in self.Real.

        """
        if not hasattr(self, 'link'):
            return None
        if not hasattr(self.link, 'Real'):
            return None
        if not self.link.Real:
            # Empty list.
            return None
        return get_placement(self.link.Real[index])

    def get_visual_placement(self, index: int):
        """Return the placement of a linked visual object.

        Parameters
        ----------
        - index: index of the object in self.Visual.

        """
        if not hasattr(self, 'link'):
            return None
        if not hasallattr(self.link, ['Visual', 'VisualPlacement']):
            return None
        if not self.link.Visual:
            # Empty list.
            return None
        if self.link.VisualPlacement == Link.visual_placement_enum[0]:
            # Own placement
            return get_placement(self.link.Visual[index])
        elif self.link.VisualPlacement == Link.visual_placement_enum[1]:
            # From `Real`.
            if not hasattr(self.link, 'Real'):
                return None
            return _get_placement_from(self.link.Visual[index], self.link.Real)

    def get_collision_placement(self, index: int):
        """Return the placement of a linked collision object.

        Parameters
        ----------
        - index: index of the object in self.Collision.

        """
        if not hasattr(self, 'link'):
            return None
        if not hasallattr(self.link, ['Collision', 'CollisionPlacement']):
            return None
        if not self.link.Collision:
            # Empty list.
            print('get_collision_placement(), not self.link.Collision') # DEBUG
            return None
        if self.link.CollisionPlacement == Link.collision_placement_enum[0]:
            # Own placement
            return get_placement(self.link.Collision[index])
        elif self.link.CollisionPlacement == Link.collision_placement_enum[1]:
            # From `Real`.
            if not hasattr(self.link, 'Real'):
                print('get_collision_placement(), no self.link.Real') # DEBUG
                return None
            return _get_placement_from(self.link.Collision[index], self.link.Real)
        elif self.link.CollisionPlacement == Link.collision_placement_enum[2]:
            # From `Visual`.
            if not hasattr(self.link, 'Visual'):
                print('get_collision_placement(), no self.link.Visual') # DEBUG
                return None
            return _get_placement_from(self.link.Collision[index], self.link.Visual)

    def export_urdf(self,
            package_path: Path,
            package_name: Path,
            placement: fc.Placement) -> et.ElementTree:
        """Return the xml for this link."""

        def get_xml(obj, urdf_function):
            # We export to STL because the DAE export (`importDAE.export()`)
            # doesn't support `App::Part` as of 2022-04-07.
            mesh_name = valid_filename(obj.Label) + '.stl'
            visual_xml = urdf_function(
                    obj,
                    mesh_name=mesh_name,
                    package_name=str(package_name),
                    placement=placement,
                    )
            if not is_primitive(obj):
                mesh_path = package_path / package_name / 'meshes' / mesh_name
                save_mesh(obj, mesh_path)
            return visual_xml

        link_xml = et.fromstring(
                f'<link name="{valid_urdf_name(self.link.Label)}" />')
        for l in self.link.Visual:
            print(f'About to export visual for {l.Label}')
            link_xml.append(get_xml(l, urdf_visual_from_object))
        for l in self.link.Collision:
            print(f'About to export collision for {l.Label}')
            link_xml.append(get_xml(l, urdf_collision_from_object))
        return link_xml


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
            if hasattr(candidate, '_Type') and candidate._Type == 'Ros::Robot':
                obj.adjustRelativeLinks(candidate)
                candidate.addObject(obj)

    return obj
