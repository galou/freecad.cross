import FreeCAD as fc
import FreeCADGui as fcgui

from ..freecad_utils import label_or
from ..utils import error
from ..gui_utils import tr


class BoxFromBoundingBoxCommand:
    def GetResources(self):
        return {'Pixmap': 'box_from_bbox.svg',
                'MenuText': tr('Box from bounding box'),
                'ToolTip': tr('Add a Part::Cube corresponding to the'
                              ' bounding box of the selected objects'),
                }

    def Activated(self):
        is_one_object_compatible = False
        for obj in fcgui.Selection.getSelection():
            # Cf. https://github.com/pboechat/pyobb for oriented bounding-box.
            has_bbox = False
            # TODO: simplify with obj.getPropertyOfGeometry()?
            try:
                bbox = obj.Shape.BoundBox
            except AttributeError:
                has_bbox = True
            try:
                bbox = obj.Mesh.BoundBox
            except AttributeError:
                has_bbox = True
            if not has_bbox:
                continue
            is_one_object_compatible = True
            box_name = label_or(obj, 'urdf') + '_bbox'
            doc = fc.activeDocument()
            doc.openTransaction(tr('Box from bounding box'))
            box = doc.addObject('Part::Box', box_name)
            box.Length = bbox.XMax - bbox.XMin
            box.Width = bbox.YMax - bbox.YMin
            box.Height = bbox.ZMax - bbox.ZMin
            box.Placement.Base.x = bbox.XMin
            box.Placement.Base.y = bbox.YMin
            box.Placement.Base.z = bbox.ZMin
            doc.commitTransaction()
            doc.recompute()
        if not is_one_object_compatible:
            error(tr('No compatible object selected'), gui=True)

    def IsActive(self):
        return bool(fcgui.Selection.getSelection())


fcgui.addCommand('BoxFromBoundingBox', BoxFromBoundingBoxCommand())
