from __future__ import annotations

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


def get_relative_placement(
        cross_object: [CrossJoint, CrossLink],
        lcs: LCS,
        obj: DO,
        ) -> fc.Placement:
    """Set the placement of `cross_object` as `lcs` relative to `obj`."""
    resolve_mode_resolve = 1
    selection = fcgui.Selection.getSelectionEx('', resolve_mode_resolve)
    objects_placements = get_subobjects_and_placements(selection)
    objects, placements = zip(*objects_placements)
    lcs_placement = placements[objects.index(lcs)]
    obj_placement = placements[objects.index(obj)]
    cross_object_placement = lcs_placement.inverse() * obj_placement
    return cross_object_placement


class _SetCROSSPlacementCommand:
    """Command to set the placement of a Link or a Joint.

    Command to set the mounted placement of a Link or the Origin of a Joint.

    """

    def GetResources(self):
        return {'Pixmap': 'set_cross_placement.svg',
                'MenuText': tr('Set placement'),
                'Accel': 'N, L',
                'ToolTip': tr('Set the mounted placement of a link or the origin of a joint.')}

    def IsActive(self):
        return True

    def Activated(self):
        doc = fc.activeDocument()
        selection_ok = False
        selection_link = False
        selection_joint = False
        try:
            cross_link, lcs, obj = validate_types(
                fcgui.Selection.getSelection(),
                ['Cross::Link', 'PartDesign::CoordinateSystem', 'Any'])
            selection_ok = True
            selection_link = True
        except RuntimeError:
            pass

        if not selection_ok:
            try:
                cross_joint, lcs_child, lcs_parent = validate_types(
                    fcgui.Selection.getSelection(),
                    ['Cross::Joint', 'PartDesign::CoordinateSystem', 'PartDesign::CoordinateSystem'])
                selection_ok = True
                selection_joint = True
            except RuntimeError:
                pass

        if not selection_ok:
            message('Select either\n'
                    '  a) a CROSS::Link, a LCS, and something or\n'
                    '  b) a CROSS::Joint, the child LCS, and the parent LCS.',
                    gui=True)
            return

        if selection_link:
            placement = get_relative_placement(cross_link, lcs, obj)
            doc.openTransaction(tr("Set link's mounted placement"))
            cross_link.MountedPlacement = placement
            doc.commitTransaction()
        elif selection_joint:
            placement = get_relative_placement(cross_joint, lcs_child, lcs_parent)
            doc.openTransaction(tr("Set joint's origin"))
            cross_joint.Origin = placement
            doc.commitTransaction()
        doc.recompute()


fcgui.addCommand('SetCROSSPlacement', _SetCROSSPlacementCommand())
