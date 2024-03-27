import FreeCAD as fc
import FreeCADGui as fcgui

from ..freecad_utils import error, warn
from ..freecad_utils import get_material, get_inertia, get_volume, get_center_of_gravity
from ..gui_utils import tr
from ..wb_utils import is_robot_selected, is_link


class CalculateMassAndInertiaCommand:
    def GetResources(self):
        return {'Pixmap': 'sphere_from_bbox.svg',
                'MenuText': tr('Calculate mass and inertia'),
                'ToolTip': tr('Select robot and press this button. It will calculate mass and inertia based on density and fills links data. If link does not have material default material will be taken from robot element. Link will skipped if property of link - "MaterialNotCalculate" is true'),
                }
    
    def Activated(self):
        doc = fc.activeDocument()
        doc.openTransaction(tr('Calculate mass and inertia'))
        objs = fcgui.Selection.getSelection()
        #obj = FreeCADGui.Selection.getSelection()[0]
        #FreeCAD.ActiveDocument.ActiveObject.Shape
        #FreeCAD.ActiveDocument.ActiveObject.Group[0].Real
        
        robot = objs[0]
        
        defaultMaterial = get_material(robot.MaterialCardPath)

        for elem in robot.Group:
            if is_link(elem):
                print('Start process inertia and mass of elem - Label: ', elem.Label, ' Label2: ', elem.Label2)
                if elem.MaterialNotCalculate == False:
                    
                    if  len(elem.Real) > 0:

                        elemMatrixOfInertia = get_inertia(elem.Real[0])
                        elemVolume = get_volume(elem.Real[0])
                        elemCenterOfGravity = get_center_of_gravity(elem.Real[0])
                        elemMaterial = get_material(elem.MaterialCardPath)

                        # print('   len(elem.Real) > 0: ', len(elem.Real) > 0)
                        # print('   elem.Real: ', elem.Real)
                        # print('   elem.Real[0]: ', elem.Real[0])
                        # print('   elem.MaterialNotCalculate == False: ', elem.MaterialNotCalculate == False)
                        # print('   elemMatrixOfInertia: ', elemMatrixOfInertia)
                        # print('   elemVolume: ', elemVolume)
                        # print('   elemCenterOfGravity: ', elemCenterOfGravity)                
                        # print('   elem.MaterialCardPath: ', elem.MaterialCardPath)
                        # print('   elemMaterial: ', elemMaterial)
                        # print("   elemMaterial['density']: ", elemMaterial['density'])

                        if elemMaterial['card_name'] != False or defaultMaterial['card_name'] != False:
                            
                            if elemVolume != False:

                                if elemCenterOfGravity != False:

                                    if elemMatrixOfInertia != False:
                                        
                                        if elemMaterial['card_name'] == False:
                                            material = defaultMaterial
                                            print('   No material specified for Link. Used default material of robot element')
                                        else:
                                            material = elemMaterial

                                        if material['density'] > 0:
                                            elemVolumeM3 = elemVolume / 1000000000 # convert mm3 to m3
                                            elem.Mass = elemVolumeM3 * material['density']
                                        
                                        elem.CenterOfMass = fc.Placement(fc.Vector(elemCenterOfGravity), fc.Rotation(), fc.Vector()) 
                                        elem.Ixx = elemMatrixOfInertia.A11
                                        elem.Ixy = elemMatrixOfInertia.A12
                                        elem.Ixz = elemMatrixOfInertia.A13
                                        elem.Iyy = elemMatrixOfInertia.A22
                                        elem.Iyz = elemMatrixOfInertia.A23
                                        elem.Izz = elemMatrixOfInertia.A33

                                        # print('   elem.Ixx: ', elemMatrixOfInertia.A11)
                                        # print('   elem.Ixy: ', elemMatrixOfInertia.A12)
                                        # print('   elem.Ixz: ', elemMatrixOfInertia.A13)
                                        # print('   elem.Iyy: ', elemMatrixOfInertia.A22)
                                        # print('   elem.Iyz: ', elemMatrixOfInertia.A23)
                                        # print('   elem.Izz: ', elemMatrixOfInertia.A33)

                                    else:
                                        error('   Link skipped. Can not get MatrixOfInertia of binded Real element.')                                        
                                else:
                                    error('   Link skipped. Can not get CenterOfGravity of binded Real element.')
                            else:
                                error('   Link skipped. Can not get volume of binded Real element.')
                        else:
                            error('   Link skipped. No material specified for Link and no default material specified for robot element.')
                    else:
                        error('   Link skipped. No binded Real element for Link.')
                else:
                    print('   Link skipped. Option "MaterialNotCalculate" is True.')

                print('Finish process inertia and mass of elem - Label: ', elem.Label, ' Label2: ', elem.Label2)

        doc.recompute()
        doc.commitTransaction()

    def IsActive(self):
        return is_robot_selected()


fcgui.addCommand('CalculateMassAndInertia', CalculateMassAndInertiaCommand())
