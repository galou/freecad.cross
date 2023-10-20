"""Proxy for Cross::Robot FreeCAD objects

A robot is a combination of Cross::Link and Cross::Joint objects that can be
exported as URDF file.

"""

from __future__ import annotations

from math import radians
from typing import Iterable, Optional
import xml.etree.ElementTree as et

import FreeCAD as fc

from .freecad_utils import ProxyBase
from .freecad_utils import add_property
from .freecad_utils import error
from .freecad_utils import get_properties_of_category
from .freecad_utils import get_valid_property_name
from .freecad_utils import is_origin
from .freecad_utils import label_or
from .freecad_utils import warn
from .gui_utils import tr
from .ros_utils import split_package_path
from .ui.file_overwrite_confirmation_dialog import FileOverwriteConfirmationDialog
from .utils import get_valid_filename
from .utils import grouper
from .utils import save_xml
from .utils import warn_unsupported
from .wb_utils import ICON_PATH
from .wb_utils import export_templates
from .wb_utils import get_chains
from .wb_utils import get_joints
from .wb_utils import get_links
from .wb_utils import get_rel_and_abs_path
from .wb_utils import get_valid_urdf_name
from .wb_utils import is_joint
from .wb_utils import is_link
from .wb_utils import is_robot
from .wb_utils import remove_ros_workspace
from .wb_utils import ros_name

# Typing hints.
DO = fc.DocumentObject
DOList = Iterable[DO]
VPDO = 'FreeCADGui.ViewProviderDocumentObject'
AppLink = DO  # TypeId == 'App::Link'.
CrossLink = DO  # A Cross::Link, i.e. a DocumentObject with Proxy "Link".
CrossJoint = DO  # A Cross::Joint, i.e. a DocumentObject with Proxy "Joint".
CrossRobot = DO  # A Cross::Robot, i.e. a DocumentObject with Proxy "Robot".


def _add_joint_variable(
        robot: CrossRobot,
        joint: CrossJoint,
        category: str,
        ) -> str:
    """Add a property for the actuator value to `robot` and return its name.

    Add a property starting with "joint.Label" to represent the actuation
    value of a joint. Supported types are 'prismatic', 'revolute', and
    'continuous'. There is no actuation value for mimicking joints.

    """
    if not is_joint(joint):
        warn(f'Wrong object type. {joint.Name} ({joint.Label})'
             ' is not a Cross::Joint')
        return
    if joint.Mimic:
        # No acuator for mimic joints.
        return ''
    if joint.Type == 'prismatic':
        unit = 'mm'
    elif joint.Type in ['revolute', 'continuous']:
        unit = 'deg'
    else:
        # Non-simple joints not supported yet.
        return ''
    rname = ros_name(joint)
    # e.g. name_candidate = "q0_deg" or "q0".
    name_candidate = f'{rname}{f"_{unit}" if unit else ""}'
    var_name = get_valid_property_name(name_candidate)
    # e.g. help_txt = "q0 in deg" or "q0".
    label = joint.Label
    id_ = rname if rname == label else f'{rname} ({label})'
    help_txt = f'{id_}{f" in {unit}" if unit else ""}'
    if joint.Type in ['prismatic', 'revolute']:
        prop_type = 'App::PropertyFloatConstraint'
    else:
        prop_type = 'App::PropertyFloat'
    _, used_var_name = add_property(robot,
                                    prop_type, var_name,
                                    category, help_txt)
    if joint.Type in ['prismatic', 'revolute']:
        # Set the default value to the current value to set min/max.
        value = robot.getPropertyByName(used_var_name)
        if (joint.LowerLimit == 0.0) and (joint.UpperLimit == 0.0):
            # Properties are not set.
            min_, max_ = -1e999, 1e999
        else:
            min_, max_ = joint.LowerLimit, joint.UpperLimit
        setattr(robot, used_var_name, (value, min_, max_, 1.0))
    value: Optional[float] = None
    if joint.Type == 'prismatic':
        value = robot.getPropertyByName(used_var_name) * 0.001
    elif joint.Type in ['revolute', 'continuous']:
        value = radians(robot.getPropertyByName(used_var_name))
    if ((value is not None)
            and (joint.Position != value)
            and (not joint.Mimic)):
        # Avoid recursive recompute.
        joint.Position = value
    return used_var_name


def _dispatch_to_joint_view_objects(
        robot: CrossRobot,
        prop: str,
        joint_prop: str,
        ) -> None:
    """Dispatch a property to the view objects of the joints of `robot`."""
    if not is_robot(robot):
        return
    if (not hasattr(robot, 'Proxy')) or (not robot.Proxy.is_execute_ready()):
        return
    prop_value = getattr(robot.ViewObject, prop)
    for joint in get_joints(robot.Group):
        if hasattr(joint.ViewObject, joint_prop):
            setattr(joint.ViewObject, joint_prop, prop_value)


def _dispatch_to_link_view_objects(
        robot: CrossRobot,
        prop: str,
        link_prop: str,
        ) -> None:
    if not is_robot(robot):
        return
    if (not hasattr(robot, 'Proxy')) or (not robot.Proxy.is_execute_ready()):
        return
    prop_value = getattr(robot.ViewObject, prop)
    for link in get_links(robot.Group):
        if hasattr(link.ViewObject, link_prop):
            setattr(link.ViewObject, link_prop, prop_value)


class Robot(ProxyBase):
    """The Robot proxy."""

    # The member is often used in workbenches, particularly in the Draft
    # workbench, to identify the object type.
    Type = 'Cross::Robot'

    # Name of the category (or group) for the joint values, which are saved as
    # properties of `self.robot`.
    _category_of_joint_values = 'JointValues'

    def __init__(self, obj: CrossRobot):
        # Implementation note: 'Group' is not required because
        # DocumentObjectGroupPython.
        super().__init__('robot', [
            'OutputPath',
            '_Type',
            ])
        obj.Proxy = self
        self.robot = obj

        # List of objects created for the robot.
        # Used for example by `robot_from_urdf` to keep track of imported
        # meshes.
        # This class doesn't add any object to this list itself.
        self._created_objects: DOList = []

        self.init_properties(obj)

    @property
    def created_objects(self) -> DOList:
        """Return the list of objects created for the robot."""
        return self._created_objects

    def init_properties(self, obj: CrossRobot):
        add_property(obj, 'App::PropertyString', '_Type', 'Internal',
                     'The type')
        obj.setPropertyStatus('_Type', ['Hidden', 'ReadOnly'])
        obj._Type = self.Type

        # Managed in self.reset_group().
        obj.setPropertyStatus('Group', 'ReadOnly')

        add_property(obj, 'App::PropertyPath', 'OutputPath', 'Export',
                     'The path to the ROS package to export files to,'
                     ' relative to $ROS_WORKSPACE/src')

        # The `Placement` is not used directly by the robot but it is used to
        # transform the pose of its links.
        add_property(obj, 'App::PropertyPlacement', 'Placement',
                     'Base', 'Placement')

    def execute(self, obj: CrossRobot) -> None:
        self.cleanup_group()
        self.set_joint_enum()
        self.add_joint_variables()
        self.compute_poses()
        # self.reset_group()

    def onChanged(self, obj: CrossRobot, prop: str) -> None:
        if prop in ['Group']:
            self.execute(obj)
        if prop == 'OutputPath':
            rel_path = remove_ros_workspace(obj.OutputPath)
            if rel_path != obj.OutputPath:
                obj.OutputPath = rel_path

    def onDocumentRestored(self, obj):
        """Restore attributes because __init__ is not called on restore."""
        self.__init__(obj)

    def __getstate__(self):
        return self.Type,

    def __setstate__(self, state):
        if state:
            self.Type, = state

    def compute_poses(self) -> None:
        """Set `Placement` of all joints and links.

        Compute and set the pose of all joints and links relative the the robot
        root link.

        """
        chains = self.get_chains()
        for chain in chains:
            placement = self.robot.Placement  # A copy.
            for link, joint in grouper(chain, 2):
                # TODO: some links and joints are already placed, re-use.
                if hasattr(link, 'MountedPlacement'):
                    new_link_placement = placement * link.MountedPlacement
                else:
                    # TODO: find out why `MountedPlacement` is not set.
                    new_link_placement = link.Placement
                if link.Placement != new_link_placement:
                    # Avoid recursive recompute.
                    link.Placement = new_link_placement
                if joint:
                    new_joint_placement = placement * joint.Origin
                    if joint.Placement != new_joint_placement:
                        # Avoid recursive recompute.
                        joint.Placement = new_joint_placement
                    # For next link.
                    placement = (new_joint_placement
                                 * joint.Proxy.get_actuation_placement())

    def get_links(self) -> list[CrossLink]:
        """Return the list of CROSS links in the order of creation."""
        if not self.is_execute_ready():
            return []
        return get_links(self.robot.Group)

    def get_joints(self) -> list[CrossJoint]:
        """Return the list of CROSS joints in the order of creation."""
        if not self.is_execute_ready():
            return []
        return get_joints(self.robot.Group)

    def get_link(self, name: str) -> Optional[CrossLink]:
        if not name:
            # Shortcut.
            return
        for link in self.get_links():
            if ros_name(link) == name:
                return link

    def get_joint(self, name: str) -> Optional[CrossJoint]:
        if not name:
            # Shortcut.
            return
        for joint in self.get_joints():
            if ros_name(joint) == name:
                return joint

    def get_chains(self) -> list[DOList]:
        """Return the list of chains.

        A chain starts at the root link, alternates links and joints, and ends
        at the last joint of the chain.

        If the last element of a chain would be a joint, that chain is not
        considered.

        """
        if not self.is_execute_ready():
            return []
        if not is_robot(self.robot):
            warn(f'{label_or(self.robot)} is not a CROSS::Robot', True)
            return []
        links = self.get_links()
        joints = self.get_joints()
        return get_chains(links, joints)

    def reset_group(self) -> None:
        """Add FreeCAD links in CrossLinks for Real, Visual, and Collision."""
        if ((not self.is_execute_ready())
                or (not hasattr(self.robot.ViewObject, 'Proxy'))
                or (not self.robot.ViewObject.Proxy.is_execute_ready())):
            return

        links: list[CrossLink] = self.get_links()
        for link in links:
            link.Proxy.update_fc_links()

    def cleanup_group(self) -> DO:
        """Remove the objects not supported by CROSS::Robot.

        Recursion provoked by modifying `Group` will take care of removing
        the remaining unsupported objects.

        """
        if not self.is_execute_ready():
            return
        for o in self.robot.Group[::-1]:
            if is_link(o) or is_joint(o):
                # Supported.
                continue
            warn_unsupported(o, by='CROSS::Robot', gui=True)
            return self.robot.removeObject(o)

    def is_exclusive_to_robot(self, obj: DO) -> bool:
        """Return True if `obj` was created for the Robot."""
        if not self.is_execute_ready():
            return False

        # Special case for Origin objects that are auto-generated by FreeCAD.
        if is_origin(obj):
            return self.is_exclusive_to_robot(obj.InList[0])

        show_objects: DOList = []
        for link in self.get_links():
            if not link.Proxy.is_execute_ready():
                continue
            for o in link.Group:
                show_objects.append(o)
        robot_objects = (self.get_links()
                         + self.get_joints()
                         + self.created_objects
                         + show_objects)
        return (obj in robot_objects) and len(set(obj.InList) - set(robot_objects)) == 0

    def delete_created_objects(self) -> None:
        """Delete all objects created for the Robot object.

        Objects that are used somewhere else should not be deleted but they are
        removed from `created_objects`.

        Calling this method may create broken links in FreeCAD links added for
        the Real, Visual, and Collision objects. Setting Robot.ViewObject.ShowReal
        and similars to False fixes the issue (deletes the objects) and should
        ideally be done before calling this method.

        This methods does not use any FreeCAD transaction.

        """
        for obj in self.created_objects[::-1]:
            try:
                name = obj.Name
            except RuntimeError:
                # Already deleted.
                continue
            if (self.is_exclusive_to_robot(obj)
                    and all([self.is_exclusive_to_robot(o) for o in obj.OutList])):
                # Object without "external" parent in the dependency graph.
                # "External" means not in self.created_objects.
                self.robot.Document.removeObject(name)
        self.created_objects.clear()

    def set_joint_enum(self) -> None:
        """Set the enum for Child and Parent of all joints."""
        def get_possible_parent_links(joint: CrossJoint) -> list[str]:
            links: list[str] = []
            for link in self.get_links():
                link_name = ros_name(link)
                if ((joint.Parent == link_name)
                    or (hasattr(link, 'Proxy')
                        and link.Proxy.is_execute_ready())):
                    links.append(link_name)
            return links

        def get_possible_child_links(joint: CrossJoint) -> list[str]:
            links: list[str] = []
            for link in self.get_links():
                link_name = ros_name(link)
                if ((joint.Child == link_name)
                    or (hasattr(link, 'Proxy')
                        and link.Proxy.is_execute_ready()
                        and link.Proxy.may_be_base_link()
                        and (not link.Proxy.is_in_chain_to_joint(joint))
                        and (joint.Parent != link_name))):
                    links.append(link_name)
            return links

        for joint in self.get_joints():
            # We add the empty string to show that the child or parent
            # was not set yet.
            parent_links: list[str] = ['']
            parent_links += get_possible_parent_links(joint)
            child_links: list[str] = ['']
            child_links += get_possible_child_links(joint)
            # Implementation note: setting to a list sets the enumeration.
            if joint.getEnumerationsOfProperty('Parent') != parent_links:
                # Avoid recursive recompute.
                # Doesn't change the value if in the new enum.
                joint.Parent = parent_links
            if joint.getEnumerationsOfProperty('Child') != child_links:
                # Avoid recursive recompute.
                # Doesn't change the value if in the new enum.
                joint.Child = child_links

    def add_joint_variables(self) -> list[str]:
        """Add a property for each actuated joint."""
        if not self.is_execute_ready():
            return []
        variables: list[str] = []
        # Get all old variables.
        old_vars: set[str] = set(get_properties_of_category(
            self.robot,
            self._category_of_joint_values))
        # Add a variable for each actuated (supported) joint.
        for joint in self.get_joints():
            var = _add_joint_variable(self.robot, joint,
                                      self._category_of_joint_values)
            if var:
                variables.append(var)
        # Remove obsoleted variables.
        for p in old_vars - set(variables):
            self.robot.removeProperty(p)
        return variables

    def export_urdf(self, interactive: bool = False) -> Optional[et.Element]:
        """Export the robot as URDF, writing files."""
        if not self.is_execute_ready():
            return
        if not self.robot.OutputPath:
            # TODO: ask the user for OutputPath.
            warn('Property `OutputPath` cannot be empty', True)
            return
        # TODO: also accept OutputPath as package name in $ROS_WORKSPACE/src.
        p, output_path = get_rel_and_abs_path(self.robot.OutputPath)
        if p != self.robot.OutputPath:
            self.robot.OutputPath = p

        template_files = [
            'package.xml',
            'CMakeLists.txt',
            'launch/display.launch.py',
            'rviz/robot_description.rviz',
            ]

        write_files = template_files + [
                'meshes/',
                'urdf/',
                ]

        if interactive and fc.GuiUp:
            diag = FileOverwriteConfirmationDialog(
                    output_path, write_files)
            ignore, write, overwrite = diag.exec_()
            diag.close()
        if set(ignore) == set(write_files):
            # No files to write.
            return
        elif set(write + overwrite) != set(write_files):
            warn(tr('Partial selection of files not supported yet'), True)
            return
        package_parent, package_name = split_package_path(output_path)
        # TODO: warn if package name doesn't end with `_description`.
        xml = et.fromstring('<robot/>')
        xml.attrib['name'] = get_valid_urdf_name(self.robot.Label)
        xml.append(et.Comment('Generated by CROSS, a ROS Workbench for'
                              ' FreeCAD ('
                              'https://github.com/galou/freecad.cross)'))
        for link in self.get_links():
            if not hasattr(link, 'Proxy'):
                error(f"Internal error with '{link.Label}', has no 'Proxy' attribute",
                      True)
                return
            xml.append(link.Proxy.export_urdf(package_parent, package_name))
        for joint in self.get_joints():
            if not joint.Parent:
                error(f"Joint '{joint.Label}' has no parent link", True)
                continue
            if not joint.Child:
                error(f"Joint '{joint.Label}' has no child link", True)
                continue
            if hasattr(joint, 'Proxy') and joint.Proxy:
                xml.append(joint.Proxy.export_urdf())
            else:
                error(f"Internal error with joint '{joint.Label}'"
                      ", has no 'Proxy' attribute", True)
        # Save the xml into a file.
        output_path.mkdir(parents=True, exist_ok=True)
        file_base = get_valid_filename(ros_name(self.robot))
        urdf_file = f'{file_base}.urdf'
        urdf_path = output_path / f'urdf/{urdf_file}'
        save_xml(xml, urdf_path)
        export_templates(template_files,
                         package_parent,
                         package_name=package_name,
                         urdf_file=urdf_file)
        return xml


class _ViewProviderRobot(ProxyBase):
    """A view provider for the Robot container object """

    def __init__(self, vobj: VPDO):
        super().__init__('view_object', [
            'JointAxisLength',
            'ShowCollision',
            'ShowJointAxes',
            'ShowReal',
            'ShowVisual',
            'Visibility',
            ])
        vobj.Proxy = self

    def getIcon(self):
        # Implementation note: "return 'robot.svg'" works only after
        # workbench activation in GUI.
        return str(ICON_PATH / 'robot.svg')

    def attach(self, vobj: VPDO):
        self.view_object = vobj
        self.robot = vobj.Object

        # vobj.addExtension('Gui::ViewProviderGeoFeatureGroupExtensionPython')

        # Level of detail.
        add_property(vobj, 'App::PropertyBool', 'ShowReal', 'ROS Display Options',
                     'Whether to show the real parts')
        add_property(vobj, 'App::PropertyBool', 'ShowVisual', 'ROS Display Options',
                     'Whether to show the parts for URDF visual')
        add_property(vobj, 'App::PropertyBool', 'ShowCollision', 'ROS Display Options',
                     'Whether to show the parts for URDF collision')

        # Joint display options.
        add_property(vobj, 'App::PropertyBool', 'ShowJointAxes',
                     'ROS Display Options',
                     'Toggle the display of the Z-axis for all child joints',
                     True)
        add_property(vobj, 'App::PropertyLength', 'JointAxisLength',
                     'ROS Display Options',
                     "Length of the arrow for the joints axes",
                     500.0)

    def updateData(self, obj: CrossRobot, prop: str):
        return

    def onChanged(self, vobj: VPDO, prop: str):
        robot: CrossRobot = vobj.Object

        if prop == 'ShowJointAxes':
            _dispatch_to_joint_view_objects(robot, prop, 'ShowAxis')
        if prop == 'JointAxisLength':
            _dispatch_to_joint_view_objects(robot, prop, 'AxisLength')
        if prop in ['ShowReal', 'ShowVisual', 'ShowCollision']:
            _dispatch_to_link_view_objects(robot, prop, prop)
            # robot.Proxy.execute(robot)

    def doubleClicked(self, vobj: VPDO):
        gui_doc = vobj.Document
        if not gui_doc.getInEdit():
            gui_doc.setEdit(vobj.Object.Name)
        else:
            error('Task dialog already active')
        return True

    def setEdit(self, vobj: VPDO, mode):
        return False

    def unsetEdit(self, vobj: VPDO, mode):
        import FreeCADGui as fcgui
        fcgui.Control.closeDialog()

    def __getstate__(self):
        return

    def __setstate__(self, state):
        return


def make_robot(name, doc: Optional[fc.Document] = None) -> CrossRobot:
    """Add a Cross::Robot to the current document."""
    if doc is None:
        doc = fc.activeDocument()
    if doc is None:
        warn('No active document, doing nothing', False)
        return
    obj = doc.addObject('App::DocumentObjectGroupPython', name)
    # obj = doc.addObject('Part::FeaturePython', name)
    Robot(obj)

    if hasattr(fc, 'GuiUp') and fc.GuiUp:
        _ViewProviderRobot(obj.ViewObject)
        obj.ViewObject.ShowReal = True
        obj.ViewObject.ShowVisual = False
        obj.ViewObject.ShowCollision = False

    doc.recompute()
    return obj
