# Assembly4 mentioned here is https://github.com/Zolko-123/FreeCAD_Assembly4.

from __future__ import annotations

from typing import Tuple

import FreeCAD as fc

from urdf_parser_py.urdf import Robot as UrdfRobot
from urdf_parser_py.urdf import Joint as UrdfJoint

from .utils import add_property
from .utils import is_freecad_link
from .utils import is_group
from .utils import is_lcs
from .utils import is_part
from .export_urdf import rotation_from_rpy

# Typing hints.
DO = fc.DocumentObject
# A pair of `App::CoordinateSystem`, [parent, child].
LcsPair = Tuple[DO, DO]


def assembly_from_urdf(
        robot: UrdfRobot,
        doc: fc.Document,
        ) -> DO:
    assembly, parts_group = _make_assembly(doc, robot.name)
    link_map: dict[str, DO] = {}
    for link in robot.links:
        link_map[link.name] = _add_link(assembly, parts_group, link.name)
    joint_map: dict[str, LcsPair] = {}
    for joint in robot.joints:
        joint_map[joint.name] = _add_joint_lcs(link_map, joint)
        _make_structure(joint_map[joint.name])


# TODO: Use `from Asm4_libs import createVariables as _new_variable_container`
#       (but fallback if not importable)
# From Asm4_libs
def _new_variable_container(
        assembly: DO,
        ) -> DO:
    """Return a variable container for Assembly4."""
    if ((not hasattr(assembly, 'TypeId'))
            or (assembly.TypeId != 'App::Part')):
        raise RuntimeError(
                'First argument must be an `App::Part` FreeCAD object')
    # There is no object "Variables", so we create it.
    variables = assembly.Document.addObject('App::FeaturePython', 'Variables')
    if hasattr(variables, 'ViewObject') and variables.ViewObject:
        # TODO
        # variables.ViewObject.Proxy = ViewProviderCustomIcon(obj,
        #                                            path + "FreeCADIco.png")
        # variables.ViewObject.Proxy = setCustomIcon(object,
        #                                    'Asm4_Variables.svg')
        pass
    # Signature of a PropertyContainer.
    variables.addProperty('App::PropertyString', 'Type')
    variables.Type = 'App::PropertyContainer'
    assembly.addObject(variables)
    return variables


def _make_assembly(
        doc: fc.Document,
        emulate_assembly4: bool = True,
        name: str = 'robot',
        ) -> tuple[DO, DO]:
    """Create an App::Part.

    Return (assembly object, parts group).

    If `emulate_assembly4` is True, the `name` will be ignored and the
    App::Part object created will be named "Assembly" in order to emulate an
    Assembly4 assembly.

    The group called 'Parts' is potentially created and returned. If the object
    'Parts' is not a group, a different name will be given.

    Note that you can make an Assembly4 assembly from a App::Part with
    assembly.Label = 'Assembly'
    assembly.Type = 'Assembly'
    assembly.addProperty('App::PropertyString', 'AssemblyType', 'Assembly')
    assembly.AssemblyType = 'Part::Link'

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
        assembly.addProperty('App::PropertyString', 'AssemblyType', 'Assembly')
        assembly.AssemblyType = 'Part::Link'
    else:
        # Create a normal part.
        assembly = doc.addObject('App::Part', name)

    # Create a group 'Parts' to hold all parts in the assembly document.
    parts_group = doc.getObject('Parts')
    if parts_group is None:
        parts_group = doc.addObject('App::DocumentObjectGroup', 'Parts')

    # Add an LCS at the root of the Model, and attach it to the 'Origin'.
    lcs = assembly.newObject('PartDesign::CoordinateSystem', 'LCS_Origin')
    lcs.Support = [(assembly.Origin.OriginFeatures[0], '')]  # The X axis.
    lcs.MapMode = 'ObjectXY'
    lcs.MapReversed = False
    # Create an object Variables to hold variables to be used in this document.
    assembly.addObject(_new_variable_container(assembly))
    # Create a group Constraints to store future solver constraints there.
    assembly.newObject('App::DocumentObjectGroup', 'Constraints')
    # Create a group Configurations to store future solver constraints there
    assembly.newObject('App::DocumentObjectGroup', 'Configurations')

    return assembly, parts_group


def _make_part(
        doc: fc.Document,
        name: str = 'Part',
        ) -> DO:
    """Create an App::Part.

    Emulates an Assembly4 Part by adding a local coordinate system.

    """
    part = doc.addObject('App::Part', name)
    # Add an LCS at the root of the Part, and attach it to the 'Origin'.
    lcs = part.newObject('PartDesign::CoordinateSystem', 'LCS_0')
    lcs.Support = [(part.Origin.OriginFeatures[0], '')]  # The X axis.
    lcs.MapMode = 'ObjectXY'
    lcs.MapReversed = False
    return part


def _add_asm4_properties(obj: DO):
    """Render `obj` compatible with Assembly4."""
    # Adapted from Asm4_libs.makeAsmProperties.
    add_property(obj, 'App::PropertyString', 'AssemblyType', 'Assembly', '')
    add_property(obj, 'App::PropertyString', 'AttachedBy', 'Assembly', '')
    add_property(obj, 'App::PropertyString', 'AttachedTo', 'Assembly', '')
    add_property(obj, 'App::PropertyPlacement', 'AttachmentOffset', 'Assembly',
                 '')
    add_property(obj, 'App::PropertyString', 'SolverId', 'Assembly', '')
    obj.AssemblyType = 'Part::Link'
    obj.SolverId = 'Asm4EE'


def _add_link(
        assembly: DO,
        parts_group: DO,
        name: str,
        ) -> DO:
    """Add an App::Part to the group and a link to it to the assembly."""
    if not is_part(assembly):
        raise RuntimeError(
                'First argument must be an `App::Part` FreeCAD object')
    if not is_group(parts_group):
        raise RuntimeError(
                'First argument must be an `App::DocumentObjectGroup`'
                ' FreeCAD object')
    # We use an underscore to let the label free for the link in `assembly`.
    part = _make_part(assembly.Document, name + '_')
    parts_group.addObject(part)
    # Make a link
    link = assembly.newObject('App::Link', name)
    link.setLink(part)
    _add_asm4_properties(link)
    # Attach to the parent assembly by default.
    # This will be changed by _make_structure later on.
    # TODO: get the real LCS name.
    link.AttachedTo = 'Parent Assembly#LCS_Origin'
    return link


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


def _make_structure(
        lcs_pair: LcsPair,
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
    parent_part = parent_lcs.getParentGeoFeatureGroup()
    if not parent_part:
        raise RuntimeError('The parent LCS is not associated to any'
                           ' `App::Part`')
    child_part = child_lcs.getParentGeoFeatureGroup()
    if not child_part:
        raise RuntimeError('The child LCS is not associated to any'
                           ' `App::Part`')
    child_part.AttachedBy = f'#{child_lcs.Name}'
    child_part.AttachedTo = f'{parent_part.Name}#{parent_lcs.Name}'
