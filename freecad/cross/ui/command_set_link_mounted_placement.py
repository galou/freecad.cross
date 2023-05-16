import FreeCAD as fc

import FreeCADGui as fcgui

from ..freecad_utils import message
from ..freecad_utils import validate_types
from ..freecadgui_utils import get_subobjects_and_placements
from ..gui_utils import tr


class _SetLinkMountedPlacementCommand:
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
        try:
            ros_link, lcs, obj = validate_types(
                fcgui.Selection.getSelection(),
                ['Cross::Link', 'PartDesign::CoordinateSystem', 'Any'])
        except RuntimeError as e:
            message(f'{e}. Select a ROS::Link, a LCS, and something.', gui=True)
            return
        resolve_mode_resolve = 1
        selection = fcgui.Selection.getSelectionEx('', resolve_mode_resolve)
        objects_placements = get_subobjects_and_placements(selection)
        objects, placements = zip(*objects_placements)
        lcs_placement = placements[objects.index(lcs)]
        obj_placement = placements[objects.index(obj)]
        doc.openTransaction(tr("Set link's mounted placement"))
        ros_link.MountedPlacement = lcs_placement.inverse() * obj_placement
        doc.commitTransaction()
        doc.recompute()


fcgui.addCommand('SetLinkMountedPlacement', _SetLinkMountedPlacementCommand())
