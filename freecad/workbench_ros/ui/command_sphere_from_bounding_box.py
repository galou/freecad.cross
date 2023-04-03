import FreeCAD as fc
import FreeCADGui as fcgui

from ..freecad_utils import label_or
from ..utils import error
from ..gui_utils import tr


class SphereFromBoundingBoxCommand:
    def GetResources(self):
        return {'Pixmap': 'sphere_from_bbox.svg',
                'MenuText': tr('Sphere from bounding box'),
                'ToolTip': tr('Add a Part::Cube corresponding to the'
                              ' bounding box of the selected objects'),
                }

    def Activated(self):
        is_one_object_compatible = False
        for obj in fcgui.Selection.getSelection():
            has_bbox = False
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
            sphere_name = label_or(obj, 'urdf') + '_bbox'
            doc = fc.activeDocument()
            sphere = doc.addObject('Part::Sphere', sphere_name)
            sphere.Radius = bbox.DiagonalLength / 2
            sphere.Placement.Base = bbox.Center
            doc.commitTransaction()
            doc.recompute()
        if not is_one_object_compatible:
            error(tr('No compatible object selected'), gui=True)

    def IsActive(self):
        return bool(fcgui.Selection.getSelection())


fcgui.addCommand('SphereFromBoundingBox', SphereFromBoundingBoxCommand())
