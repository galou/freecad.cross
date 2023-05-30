import FreeCAD as fc

import FreeCADGui as fcgui

from ..freecad_utils import message
from ..freecad_utils import validate_types
from ..freecadgui_utils import get_subobjects_and_placements
from ..gui_utils import tr


# Typing hints.
DO = fc.DocumentObject
CrossLink = DO  # A Cross::Link, i.e. a DocumentObject with Proxy "Link".
CrossJoint = DO  # A Cross::Joint, i.e. a DocumentObject with Proxy "Joint".
LCS = DO  # Local coordinate systen, TypeId == "PartDesign::CoordinateSystem"


def get_link_mounted_placement(
        cross_link: CrossLink,
        lcs: LCS,
        obj: DO,
        ) -> fc.Placement:
    resolve_mode_resolve = 1
    selection = fcgui.Selection.getSelectionEx('', resolve_mode_resolve)
    objects_placements = get_subobjects_and_placements(selection)
    objects, placements = zip(*objects_placements)
    lcs_placement = placements[objects.index(lcs)]
    obj_placement = placements[objects.index(obj)]
    cross_link_placement = lcs_placement.inverse() * obj_placement
    return cross_link_placement


def get_joint_origin(
        cross_joint: CrossJoint,
        lcs: LCS,
        obj: DO,
        ) -> fc.Placement:
    resolve_mode_resolve = 1
    selection = fcgui.Selection.getSelectionEx('', resolve_mode_resolve)
    objects_placements = get_subobjects_and_placements(selection)
    objects, placements = zip(*objects_placements)
    for o, p in zip(objects, placements):
        print(f'{o.Label}: App.Placement(App.Vector({p.Base}), App.Rotation({p.Rotation.Q}') # DEBUG
    lcs_placement = placements[objects.index(lcs)]
    obj_placement = placements[objects.index(obj)]
    cross_joint_placement = lcs_placement.inverse() * obj_placement
    return cross_joint_placement


def get_joint_origin2(
        cross_joint: CrossJoint,
        lcs: LCS,
        cross_link: CrossLink,
        ) -> fc.Placement:
    resolve_mode_resolve = 1
    selection = fcgui.Selection.getSelectionEx('', resolve_mode_resolve)
    objects_placements = get_subobjects_and_placements(selection)
    objects, placements = zip(*objects_placements)
    for o, p in zip(objects, placements):
        print(f'{o.Label}: App.Placement(App.Vector({p.Base}), App.Rotation({p.Rotation.Q}') # DEBUG
    lcs_placement = placements[objects.index(lcs)]
    cross_link_placement = placements[objects.index(cross_link)]
    cross_joint_placement = lcs_placement.inverse() * cross_link_placement
    return cross_joint_placement


class _SetCROSSPlacementCommand:
    """Command to set the mounted placement of a Link."""

    def GetResources(self):
        return {'Pixmap': 'set_link_mounted_placement.svg',
                'MenuText': tr('Set the mounted placement of a link'),
                'Accel': 'N, L',
                'ToolTip': tr('Set the mounted placement of a link.')}

    def IsActive(self):
        return True

    def Activated(self):
        doc = fc.activeDocument()
        selection_ok = False
        selection_link = False
        selection_joint = False
        selection_joint2 = False
        error = None
        try:
            cross_link, lcs, obj = validate_types(
                fcgui.Selection.getSelection(),
                ['Cross::Link', 'PartDesign::CoordinateSystem', 'Any'])
            selection_ok = True
            selection_link = True
        except RuntimeError as e:
            error = e

        if not selection_ok:
            try:
                cross_joint, lcs, obj = validate_types(
                    fcgui.Selection.getSelection(),
                    ['Cross::Joint', 'PartDesign::CoordinateSystem', 'Any'])
                selection_ok = True
                selection_joint = True
            except RuntimeError as e:
                error = e

        if not selection_ok:
            try:
                cross_joint, lcs, cross_link = validate_types(
                    fcgui.Selection.getSelection(),
                    ['Cross::Joint', 'PartDesign::CoordinateSystem', 'Cross::Link'])
                selection_ok = True
                selection_joint2 = True
            except RuntimeError as e:
                error = e

        if not selection_ok:
            message(f'{error}. Select either a) a CROSS::Link, a LCS, and'
                    ' something or b) a CROSS::Joint, a LCS, and something.',
                    gui=True)
            return

        if selection_link:
            placement = get_link_mounted_placement(cross_link, lcs, obj)
            doc.openTransaction(tr("Set link's mounted placement"))
            cross_link.MountedPlacement = placement
            doc.commitTransaction()
        elif selection_joint:
            print(f'{cross_joint.Label} {lcs.Label} {obj.Label}') # DEBUG
            placement = get_joint_origin(cross_joint, lcs, obj)
            doc.openTransaction(tr("Set link's mounted placement"))
            cross_joint.Origin = placement
            doc.commitTransaction()
        elif selection_joint2:
            print(f'{cross_joint.Label} {lcs.Label}') # DEBUG
            placement = get_joint_origin2(cross_joint, lcs, cross_link)
            doc.openTransaction(tr("Set link's mounted placement"))
            cross_joint.Origin = placement
            doc.commitTransaction()
        doc.recompute()


fcgui.addCommand('SetCROSSPlacement', _SetCROSSPlacementCommand())
