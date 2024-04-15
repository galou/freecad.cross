import FreeCAD as fc
import FreeCADGui as fcgui

from ..freecad_utils import center_of_gravity_mm
from ..freecad_utils import correct_matrix_of_inertia
from ..freecad_utils import error
from ..freecad_utils import first_object_with_volume
from ..freecad_utils import get_linked_obj
from ..freecad_utils import material
from ..freecad_utils import matrix_of_inertia
from ..freecad_utils import volume_mm3
from ..gui_utils import tr
from ..wb_utils import is_link
from ..wb_utils import is_robot_selected


class _CalculateMassAndInertiaCommand:
    def GetResources(self) -> dict:
        return {'Pixmap': 'calculate_mass_and_inertia.svg',
                'MenuText': tr('Calculate mass and inertia'),
                'ToolTip': tr('Select robot and press this button.'
                              ' It will calculate mass and inertia based on'
                              ' density and fills links data. If link does not'
                              ' have material, default material will be taken'
                              ' from robot element. Link will skipped if'
                              ' property of link - "MaterialNotCalculate" is'
                              ' true. You can visually check inertia placement'
                              ' in Gazebo. Turn on display of inertia in Gazebo'
                              ' and check what generated inertia blocks'
                              ' approximately same size and same'
                              ' position/orientation as their links. Inertia'
                              ' block orientation tilt to towards the mass'
                              ' displacement is ok for unsymmetrical bodies.'),
                }

    def Activated(self) -> None:
        doc = fc.activeDocument()
        objs = fcgui.Selection.getSelection()

        if not objs:
            return

        robot = objs[0]

        defaultMaterial = material(robot.MaterialCardPath)

        doc.openTransaction(tr('Calculate mass and inertia'))
        for elem in robot.Group:
            if is_link(elem):
                # print('Start process inertia and mass of elem - Label: ', elem.Label, ' Label2: ', elem.Label2) # DEBUG
                if not elem.MaterialNotCalculate:
                    if elem.Real:
                        if elem.Collision:
                            real = elem.Real[0]

                            first_body = first_object_with_volume(real)
                            elem_center_of_gravity = center_of_gravity_mm(first_body)
                            elem_matrix_of_inertia = matrix_of_inertia(first_body)
                            elem_volume_mm3 = volume_mm3(first_body)
                            elem_material = material(elem.MaterialCardPath)

                            if (elem_material['card_name'] is not None) or (defaultMaterial['card_name'] is not None):

                                if elem_volume_mm3 is not None:

                                    if elem_center_of_gravity is not None:

                                        if elem_matrix_of_inertia is not None:

                                            if elem_material['card_name'] is not None:
                                                material = defaultMaterial
                                                print('   No material specified for Link. Used default material of robot element')
                                            else:
                                                material = elem_material

                                            if ((material['density'] is not None)
                                                    and (material['density'] > 0.0)):
                                                elem_volume_m3 = elem_volume_mm3 / 1e9
                                                elem.Mass = elem_volume_m3 * material['density']
                                                elem_matrix_of_inertia = correct_matrix_of_inertia(elem_matrix_of_inertia, elem_volume_mm3, elem.Mass)

                                                basic_obj = get_linked_obj(real)
                                                # correction if basic obj has not zero placement
                                                elem_center_of_gravity = elem_center_of_gravity - basic_obj.Placement.Base
                                                elem.CenterOfMass = fc.Placement(elem.MountedPlacement * elem_center_of_gravity, elem.MountedPlacement.Rotation, fc.Vector())

                                                elem.Ixx = elem_matrix_of_inertia.A11
                                                elem.Ixy = elem_matrix_of_inertia.A12
                                                elem.Ixz = elem_matrix_of_inertia.A13
                                                elem.Iyy = elem_matrix_of_inertia.A22
                                                elem.Iyz = elem_matrix_of_inertia.A23
                                                elem.Izz = elem_matrix_of_inertia.A33
                                            else:
                                                error('   Link skipped. Material density less or equal zero.')
                                        else:
                                            error('   Link skipped. Can not get MatrixOfInertia of bound Real element.')
                                    else:
                                        error('   Link skipped. Can not get CenterOfGravity of bound Real element.')
                                else:
                                    error('   Link skipped. Can not get volume of bound Real element.')
                            else:
                                error('   Link skipped. No material specified for Link and no default material specified for robot element.')
                        else:
                            error('   Link skipped. No bound Collision element for Link.')
                    else:
                        error('   Link skipped. No bound Real element for Link.')
                else:
                    print('   Link skipped. Option "MaterialNotCalculate" is True.')

                print('Finish process inertia and mass of elem - Label: ', elem.Label, ' Label2: ', elem.Label2)

        doc.recompute()
        doc.commitTransaction()

    def IsActive(self):
        return is_robot_selected()


fcgui.addCommand('CalculateMassAndInertia', _CalculateMassAndInertiaCommand())
