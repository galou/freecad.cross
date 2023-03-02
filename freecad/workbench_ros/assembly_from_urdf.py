# The Assembly4 workbench mentioned here is
# https://github.com/Zolko-123/FreeCAD_Assembly4.

from __future__ import annotations

from typing import Any, Iterable, Tuple

import FreeCAD as fc

try:
    from urdf_parser_py import xml_reflection as xmlr
    from urdf_parser_py.urdf import Collision as UrdfCollision
    from urdf_parser_py.urdf import Joint as UrdfJoint
    from urdf_parser_py.urdf import Robot as UrdfRobot
    from urdf_parser_py.urdf import Visual as UrdfVisual

    from .urdf_parser_utils import obj_from_geometry
    from .urdf_parser_utils import placement_from_origin
except ModuleNotFoundError:
    UrdfJoint = Any
    UrdfRobot = Any
    UrdfVisual = Any
    UrdfCollision = Any

from .assembly4_utils import add_asm4_properties
from .assembly4_utils import new_variable_container
from .assembly4_utils import update_placement_expression
from .utils import add_property
from .utils import is_freecad_link
from .utils import is_group
from .utils import is_lcs
from .utils import is_part
from .utils import make_group
from .export_urdf import rotation_from_rpy

# Typing hints.
DO = fc.DocumentObject
# List of UrdfVisual or List of UrdfCollision.
VisualList = Iterable[UrdfVisual]
CollisionList = Iterable[UrdfCollision]
# A pair of `App::CoordinateSystem`, [parent, child].
LcsPair = Tuple[DO, DO]


def assembly_from_urdf(
        doc: fc.Document,
        robot: UrdfRobot,
        ) -> DO:
    assembly, parts_group, var_container = _make_assembly_container(doc, robot.name)
    link_map: dict[str, DO] = {}
    lcs_map: dict[str, DO] = {}
    for link in robot.links:
        fc_link, lcs = _add_link(assembly, parts_group, link.name)
        link_map[link.name] = fc_link
        lcs_map[link.name] = lcs
    joint_map: dict[str, LcsPair] = {}
    for joint in robot.joints:
        joint_map[joint.name] = _add_joint_lcs(link_map, joint)
        _make_structure(joint_map[joint.name], joint, link_map)
    # Rewrite the placement expression for the root link.
    # TODO: get the real LCS name.
    if robot.links:
        root_link = link_map[robot.get_root()]
        lcs_link = lcs_map[robot.get_root()]
        update_placement_expression(root_link,
            f'LCS_Origin.Placement * AttachmentOffset * {lcs_link.Name}.Placement ^ -1')
    _add_visual(robot, link_map)
    _add_joint_variables(var_container, robot, joint_map)

    # Make the assembly for collision.
    _make_collision_assembly_from_visual(doc, robot, link_map, joint_map,
                                         var_container)


def _make_assembly_container(
        doc: fc.Document,
        emulate_assembly4: bool = True,
        name: str = 'robot',
        ) -> tuple[DO, DO, DO]:
    """Create an App::Part.

    Return (assembly object, parts group, variable container).

    If `emulate_assembly4` is True, the `name` will be ignored and the
    App::Part object created will be named "Assembly" in order to emulate an
    Assembly4 assembly.

    The group called 'Parts' is potentially created and returned. If the object
    'Parts' is not a group, a different name will be given.

    Note that you can make an Assembly4 assembly from a App::Part with
    assembly.Label = 'Assembly'
    assembly.Type = 'Assembly'
    assembly.addProperty('App::PropertyString', 'SolverId', 'Assembly')
    assembly.SolverId = 'Asm4EE'

    """
    # Adapted from Assembly4/newAssemblyCmd.py (v0.12.5).
    # The difference is that a normal Part is created if a special Assembly4
    # Part already exists.
    # An Assembly4 Part is a Part with .Type = 'Assembly' and
    # .Label = 'Assembly'.
    candidate_assembly = doc.getObject('Assembly')
    if ((candidate_assembly is None)
            and emulate_assembly4):
        assembly = doc.addObject('App::Part', 'Assembly')
        # set the type as a "proof" that it's an Assembly
        assembly.Type = 'Assembly'
        assembly.addProperty('App::PropertyString', 'SolverId', 'Assembly')
        assembly.SolverId = 'Asm4EE'
    else:
        # Create a normal part.
        assembly = doc.addObject('App::Part', name)

    # Create a group 'Parts' to hold all parts in the assembly document.
    parts_group = make_group(doc, 'Parts', visible=False)

    # Add an LCS at the root of the Model, and attach it to the 'Origin'.
    lcs = assembly.newObject('PartDesign::CoordinateSystem', 'LCS_Origin')
    lcs.Support = [(assembly.Origin.OriginFeatures[0], '')]  # The X axis.
    lcs.MapMode = 'ObjectXY'
    lcs.MapReversed = False
    # Create an object Variables to hold variables to be used in this document.
    var_container = new_variable_container(assembly)
    assembly.addObject(var_container)
    # Create a group Constraints to store future solver constraints there.
    assembly.newObject('App::DocumentObjectGroup', 'Constraints')
    # Create a group Configurations to store future solver constraints there
    assembly.newObject('App::DocumentObjectGroup', 'Configurations')

    return assembly, parts_group, var_container


def _make_collision_assembly_from_visual(
        doc: fc.Document,
        robot: UrdfRobot,
        visual_link_map: dict[str, DO],
        visual_joint_map: dict[str, LcsPair],
        variable_container: DO,
        ) -> DO:
    assembly, _ = _make_part(doc, 'Collision Assembly')
    assembly.Label = 'Collision Assembly'
    parts_group = make_group(doc, 'Collision Parts', visible=False)
    collision_link_map: dict[str, DO] = {}
    for link_name, visual_link_obj in visual_link_map.items():
        fc_link, _ = _add_link(assembly, parts_group, link_name)
        collision_link_map[link_name] = fc_link
    collision_joint_map: dict[str, LcsPair] = {}
    for joint in robot.joints:
        joint_obj = _add_joint_lcs(collision_link_map, joint)
        _make_structure(joint_obj, joint, collision_link_map)
        collision_joint_map[joint.name] = joint_obj
    _add_collision(robot, collision_link_map)
    _add_joint_variables(variable_container, robot, collision_joint_map)


def _make_part(
        doc: fc.Document,
        name: str = 'Part',
        ) -> tuple[DO, DO]:
    """Create an App::Part.

    Emulates an Assembly4 Part by adding a local coordinate system.

    """
    part = doc.addObject('App::Part', name)
    # Add an LCS at the root of the Part, and attach it to the 'Origin'.
    lcs = part.newObject('PartDesign::CoordinateSystem', 'LCS_0')
    lcs.Support = [(part.Origin.OriginFeatures[0], '')]  # The X axis.
    lcs.MapMode = 'ObjectXY'
    lcs.MapReversed = False
    return part, lcs


def _add_link(
        assembly: DO,
        parts_group: DO,
        name: str,
        ) -> Tuple[DO, DO]:
    """Add an App::Part to the group and a link to it to the assembly.

    Return the link and its first LCS.

    """
    if not is_part(assembly):
        raise RuntimeError(
                'First argument must be an `App::Part` FreeCAD object')
    if not is_group(parts_group):
        raise RuntimeError(
                'First argument must be an `App::DocumentObjectGroup`'
                ' FreeCAD object')
    # We use an underscore to let the label free for the link in `assembly`.
    part, lcs = _make_part(assembly.Document, name + '_')
    parts_group.addObject(part)
    # Make a link
    link = assembly.newObject('App::Link', name)
    link.setLink(part)
    add_asm4_properties(link)
    # Attach to the parent assembly by default.
    # This will be changed by _make_structure later on.
    # TODO: get the real LCS name.
    link.AttachedTo = 'Parent Assembly#LCS_Origin'
    return link, lcs


def _lcs_name(
        joint_name: str,
        part_is_parent: bool,
        ) -> str:
    if part_is_parent:
        suffix = 'parent'
    else:
        suffix = 'child'
    return f'LCS_{joint_name}_{suffix}'


def _add_lcs(
        part_or_link: DO,
        joint_name: str,
        part_is_parent: bool,
        ) -> DO:
    """Add a coordinate system to a part.

    A link to a part can be provided and the coordinate system will be added to
    the part.

    """
    is_link = is_freecad_link(part_or_link)
    if (not (is_part(part_or_link) or is_link)):
        raise RuntimeError(
                'First argument must be an `App::Part`'
                ' or `App::Link` FreeCAD object')
    if is_link:
        part = part_or_link.LinkedObject
        if not is_part(part):
            raise RuntimeError('First argument must link to an `App::Part`')
    else:
        part = part_or_link

    lcs = part.newObject('PartDesign::CoordinateSystem',
                         _lcs_name(joint_name, part_is_parent))
    lcs.Support = [(part.Origin.OriginFeatures[0], '')]  # The X axis.
    lcs.MapMode = 'ObjectXY'
    lcs.MapReversed = False
    return lcs


def _add_joint_lcs(
        link_map: dict[str, DO],
        joint: UrdfJoint,
        ) -> LcsPair:
    """Add a LCS to the parent and child parts.

    Parameters
    ----------
    - link_map: the `App::Link` object associated with each URDF link,
          identified by its name.

    """
    if joint.parent not in link_map:
        raise RuntimeError(f'Parent `{joint.parent}` is not a known link')
    if joint.child not in link_map:
        raise RuntimeError(f'Child `{joint.child}` is not a known link')
    parent = link_map[joint.parent]
    child = link_map[joint.child]
    lcs_parent = _add_lcs(parent, joint.name, part_is_parent=True)
    lcs_child = _add_lcs(child, joint.name, part_is_parent=False)
    _place_parent_lcs(lcs_parent, joint)
    return lcs_parent, lcs_child


def _place_parent_lcs(
        lcs: DO,
        joint: UrdfJoint,
        ) -> None:
    """Place a coordinate system to correspond to the joint position.

    In an assembly generated from URDF, a joint is represented by 2 local
    coordinate systems (LCS) that are coincident when the joint value is 0.
    One LCS is in the parent link (i.e. FreeCAD part), one in the child link.

    The placement of the LCS in the parent part must correspond to the joint
    position in the URDF, i.e. `origin`, and this is what this function is for.

    """
    if not is_lcs(lcs):
        raise RuntimeError('First argument must be an'
                           ' `PartDesign::CoordinateSystem`')
    # Convert from meters to millimeters.
    lcs.AttachmentOffset.Base = fc.Vector(joint.origin.position) * 1000.0
    lcs.AttachmentOffset.Rotation = rotation_from_rpy(joint.origin.rpy)


def _get_parent_link(
        joint: UrdfJoint,
        link_map: dict[str, DO],
        ) -> DO:
    """Return the FreeCAD link associated with the parent link of the joint."""
    for link_name, link_object in link_map.items():
        if link_name == joint.parent:
            return link_object


def _get_child_link(
        joint: UrdfJoint,
        link_map: dict[str, DO],
        ) -> DO:
    """Return the FreeCAD link associated with the child link of the joint."""
    for link_name, link_object in link_map.items():
        if link_name == joint.child:
            return link_object


def _make_structure(
        lcs_pair: LcsPair,
        joint: UrdfJoint,
        link_map: dict[str, DO],
        ) -> None:
    """Create the parent-child inheritence."""
    if not (
            isinstance(lcs_pair, tuple)
            and len(lcs_pair) == 2
            and is_lcs(lcs_pair[0])
            and is_lcs(lcs_pair[1])):
        raise RuntimeError('First argument must be a list of 2'
                           ' `PartDesign::CoordinateSystem`')
    parent_lcs = lcs_pair[0]
    child_lcs = lcs_pair[1]
    parent_link = _get_parent_link(joint, link_map)
    if not parent_link:
        raise RuntimeError('The parent LCS is not associated to any'
                           ' `App::Part`')
    child_link = _get_child_link(joint, link_map)
    if not child_link:
        raise RuntimeError('The child LCS is not associated to any'
                           ' `App::Part`')
    child_link.AttachedBy = f'#{child_lcs.Name}'
    child_link.AttachedTo = f'{parent_link.Name}#{parent_lcs.Name}'

    expr = f'{parent_link.Name}.Placement * {parent_lcs.Name}.Placement * AttachmentOffset * {child_lcs.Name}.Placement ^ -1'
    update_placement_expression(child_link, expr)


def _add_joint_variables(
        variable_container: DO,
        robot: UrdfRobot,
        joint_map: dict[str, LcsPair],
        ) -> dict[str, str]:
    var_map: dict[str, str] = {}
    for joint_name, joint_obj in joint_map.items():
        _, child_lcs = joint_map[joint_name]
        var_name = _add_joint_variable(variable_container, robot, joint_name,
                                       joint_obj, child_lcs)
        var_map[joint_name] = var_name
    return var_map


def _add_joint_variable(
        variable_container: DO,
        robot: UrdfRobot,
        joint_name: str,
        joint: DO,
        child_lcs: DO,
        ) -> str:
    urdf_joint = robot.joint_map[joint_name]
    if urdf_joint.joint_type == 'prismatic':
        unit = 'mm'
    elif urdf_joint.joint_type in ['revolute', 'continuous']:
        unit = 'deg'
    else:
        return ''
    # `joint_name` may not be a valid property name in FreeCAD but the LCS was
    # already created based on `joint_name`. Just try this first.
    if child_lcs.Name.startswith('LCS_') and child_lcs.Name.endswith('_child'):
        # Remove the `LCS_` prefix and `_child` suffix.
        lcs_name = child_lcs.Name[len('LCS_'):-len('_child')]
        var_name = f'{lcs_name}{f"_{unit}" if unit else ""}'
    else:
        # TODO: replace forbidden characters with `_`.
        var_name = f'{joint_name}{f"_{unit}" if unit else ""}'
    help_txt = f'{joint_name}{f" in {unit}" if unit else ""}'
    _, used_var_name = add_property(variable_container,
                                    'App::PropertyFloat', var_name,
                                    'Variables', help_txt)
    if urdf_joint.joint_type == 'prismatic':
        _make_prismatic_lcs(urdf_joint.axis, variable_container.Name,
                            used_var_name, child_lcs)
    elif urdf_joint.joint_type in ['revolute', 'continuous']:
        _make_revolute_lcs(urdf_joint.axis, variable_container.Name,
                           used_var_name, child_lcs)
    return used_var_name


def _multiplier_for_expression(factor: float) -> str:
    if factor == 1.0:
        return ''
    elif factor == -1.0:
        return '-'
    else:
        return f'{factor} * '


def _make_prismatic_lcs(
        axis: list[float],
        var_container_name: str,
        var_name: str,
        child_lcs: DO,
        ) -> None:
    # We need to invert the direction.
    v = -fc.Vector(axis)
    v.normalize()
    m = _multiplier_for_expression
    if v.x != 0.0:
        expr = f'{m(v.x)}{var_container_name}.{var_name}'
        child_lcs.setExpression('.AttachmentOffset.Base.x', expr)
    if v.y != 0.0:
        expr = f'{m(v.y)}{var_container_name}.{var_name}'
        child_lcs.setExpression('.AttachmentOffset.Base.y', expr)
    if v.z != 0.0:
        expr = f'{m(v.z)}{var_container_name}.{var_name}'
        child_lcs.setExpression('.AttachmentOffset.Base.z', expr)


def _make_revolute_lcs(
        axis: list[float],
        var_container_name: str,
        var_name: str,
        child_lcs: DO,
        ) -> None:
    # We need to invert the direction.
    v = -fc.Vector(axis)
    v.normalize()
    child_lcs.AttachmentOffset.Rotation.Axis = v
    expr = f'{var_container_name}.{var_name}'
    child_lcs.setExpression('.AttachmentOffset.Rotation.Angle', expr)


def _add_visual(
        robot: UrdfRobot,
        link_map: dict[str, DO],
        ) -> None:
    for link_name, link_obj in link_map.items():
        _add_visual_geometries(link_obj, link_name,
                               robot.link_map[link_name].visuals)


def _add_collision(
        robot: UrdfRobot,
        link_map: dict[str, DO],
        ) -> None:
    for link_name, link_obj in link_map.items():
        _add_collision_geometries(link_obj, link_name,
                                  robot.link_map[link_name].collisions)


def _add_visual_geometries(
        link: DO,
        link_name: str,
        geometries: [VisualList | CollisionList],
        ) -> tuple[list[DO], list[DO]]:
    """Add the primitive shapes and meshes to a link."""
    visual_group = make_group(link.Document, 'Visuals', visible=False)
    group = make_group(visual_group, f'{link_name} Visuals')
    name_linked_geom = f'{link_name}_visual'
    return _add_geometries(group, geometries, link, name_linked_geom)


def _add_collision_geometries(
        link: DO,
        link_name: str,
        geometries: [VisualList | CollisionList],
        ) -> tuple[list[DO], list[DO]]:
    """Add the primitive shapes and meshes to a link."""
    collision_group = make_group(link.Document, 'Collisions', visible=False)
    group = make_group(collision_group, f'{link_name} Collisions')
    name_linked_geom = f'{link_name}_collision'
    return _add_geometries(group, geometries, link, name_linked_geom)


def _add_geometries(
        group: DO,
        geometries: [VisualList | CollisionList],
        link: DO = None,
        name_linked_geom: str = '',
        ) -> tuple[list[DO], list[DO]]:
    """Add the geometries from URDF into `group` and a FC link to it into `link`.

    `geometries` is either `visuals` or `collisions` and the geometry itself is
    `geometries[?].geometry`.
    If `name_linked_geom` is empty, not FC link is created in `link`.

    Return the list of objects reprensenting the geometries and the list of
    FreeCAD links.

    """
    geom_objs: list[DO] = []
    fc_links: list[DO] = []
    for geometry in geometries:
        # Make the FC object in the group.
        try:
            geom_obj, _ = obj_from_geometry(geometry.geometry, group)
        except NotImplementedError:
            continue
        geom_objs.append(geom_obj)
        geom_obj.Placement = (placement_from_origin(geometry.origin)
                              * geom_obj.Placement)
        if link is not None:
            # Make a FC link into `link`.
            original_part = link.LinkedObject
            link_to_geom = original_part.newObject('App::Link',
                                                   name_linked_geom)
            link_to_geom.setLink(geom_obj)
            link_to_geom.Placement = geom_obj.Placement
            fc_links.append(link_to_geom)
    return geom_objs, fc_links
