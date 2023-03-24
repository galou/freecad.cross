# This module should not be imported before the GUI is up otherwise, it'll be
# imported without `tr` and later import attempts will be ignored because this
# is own Python works.
# TODO: solve the import mess.

from __future__ import annotations

from itertools import zip_longest
import os
from pathlib import Path
import string
from typing import Any, Iterable, Optional
import xml.etree.ElementTree as et
from xml.dom import minidom

import FreeCAD as fc

import Mesh  # FreeCAD

from .import_dae import read as read_dae
from .import_dae import export as export_dae

if hasattr(fc, 'GuiUp') and fc.GuiUp:
    import FreeCADGui as fcgui

    from PySide import QtCore  # FreeCAD's PySide!
    from PySide import QtGui  # FreeCAD's PySide!

    def tr(text: str) -> str:
        return QtGui.QApplication.translate('workbench_ros', text)
else:
    def tr(text: str) -> str:
        return text

# Typing hints.
DO = fc.DocumentObject
DOList = Iterable[DO]
RosLink = DO  # A Ros::Link, i.e. a DocumentObject with Proxy "Link".
RosJoint = DO  # A Ros::Joint, i.e. a DocumentObject with Proxy "Joint".
RosRobot = DO  # A Ros::Robot, i.e. a DocumentObject with Proxy "Robot".

# MOD_PATH = Path(os.path.join(fc.getResourceDir(), 'Mod', 'workbench_ros'))
MOD_PATH = Path(os.path.dirname(__file__)).joinpath('../..').resolve()  # For development
RESOURCES_PATH = MOD_PATH.joinpath('resources')
UI_PATH = RESOURCES_PATH.joinpath('ui')
ICON_PATH = RESOURCES_PATH.joinpath('icons')


def with_fc_gui() -> bool:
    return hasattr(fc, 'GuiUp') and fc.GuiUp


def get_valid_filename(text: str) -> str:
    """Return a string that is a valid file name."""
    valids = string.ascii_letters + string.digits + '_-.'
    return ''.join(c if c in valids else '_' for c in text)


# Adapted from https://github.com/FreeCAD/FreeCAD/blob
#   /fe9ebfc4c5ea5cd26786627434b4158171a80a29/src/Base/Tools.cpp#L155
# Function getIdentifier in Tools.cpp.
# The difference is that an underscore is added if `text` starts with a number,
# whereas getIdentifier replaces the first character.
def get_valid_property_name(text: str) -> str:
    """Return a valid property from any string."""
    # Check for first character whether it's a digit.
    if not text:
        return '_'
    if text[0] in string.digits:
        text = '_' + text
    # Strip illegal chars.
    valids = string.ascii_letters + string.digits
    return ''.join(c if c in valids else '_' for c in text)


def xml_comment(comment: str) -> str:
    """Returns the string without '--'."""
    return f'{comment.replace("--", "⸗⸗")}'


def get_valid_urdf_name(name: str) -> str:
    if not name:
        return 'no_label'
    return name.replace(' ', '_')


def label_or(
        obj: DO,
        alternative: str = 'no label') -> str:
    """Return the `Label` or the alternative."""
    return obj.Label if hasattr(obj, 'Label') else alternative


def warn(text: str, gui: bool = False) -> None:
    """Warn the user."""
    fc.Console.PrintWarning(text + '\n')
    if gui and fc.GuiUp:
        diag = QtGui.QMessageBox(QtGui.QMessageBox.Warning,
                                 'ROS Workbench', text)
        diag.setWindowModality(QtCore.Qt.ApplicationModal)
        diag.exec_()


def error(text: str, gui: bool = False) -> None:
    """Log an error to the user."""
    fc.Console.PrintError(text + '\n')
    if gui and fc.GuiUp:
        diag = QtGui.QMessageBox(QtGui.QMessageBox.Critical,
                                 'FreeCAD - ROS Workbench', text)
        diag.setWindowModality(QtCore.Qt.ApplicationModal)
        diag.exec_()


def warn_unsupported(objects: [DO, DOList],
                     by: str = '',
                     gui: bool = False,
                     ) -> None:
    """Warn the user of an unsupported object type."""
    if not isinstance(objects, list):
        objects = list(objects)
    for o in objects:
        by_txt = f' by {by}' if by else ''
        try:
            label = o.Label
        except AttributeError:
            label = 'not_a_FreeCAD_object'
        try:
            warn(f'Object "{label}" of type {o.TypeId}'
                 f' not supported{by_txt}\n',
                 gui=gui)
        except AttributeError:
            warn(f'Object "{label}" not supported{by_txt}\n', gui=gui)


def strip_subelement(sub_fullpath: str) -> str:
    """Return sub_fullpath without the last sub-element.

    A sub-element is a face, edge or vertex.
    Examples:

    - 'Face6' -> ''
    - 'Body.Box001.' -> 'Body.Box001'
    - 'Body.Box001.Face6' -> 'Body.Box001'

    Parameters
    ----------
    - subobject_fullpath: SelectionObject.SubElementNames[i], where
        SelectionObject is obtained with gui.Selection.getSelectionEx('', 0)
        (i.e. not gui.Selection.getSelectionEx()).
        Examples:
        - 'Face6' if you select the top face of a cube solid made in Part.
        - 'Body.Box001.' if you select the tip of a Part->Body->"additive
            primitve" in PartDesign.
        - 'Body.Box001.Face6' if you select the top face of a Part->Body->
            "additive primitve" in PartDesign.

    """
    if (not sub_fullpath) or ('.' not in sub_fullpath):
        return ''
    return sub_fullpath.rsplit('.', maxsplit=1)[0]


def get_subobject_by_name(
        object_: DO,
        subobject_name: str,
        ) -> Optional[DO]:
    """Return the appropriate object from object_.OutListRecursive."""
    for o in object_.OutListRecursive:
        if o.Name == subobject_name:
            return o


def get_subobjects_by_full_name(
        root_object: DO,
        subobject_fullpath: str,
        ) -> DOList:
    """Return the list of objects after root_object to the named object.

    The last part of ``subobject_fullpath`` is then a specific vertex, edge or
    face and is ignored.
    So, subobject_fullpath has the form 'name0.name1.Edge001', for example; in
    this case, the returned objects are
    [object_named_name0, object_named_name1].

    Parameters
    ----------
    - root_object: SelectionObject.Object, where SelectionObject is obtained
        with gui.Selection.getSelectionEx('', 0)
        (i.e. not gui.Selection.getSelectionEx()).
    - subobject_fullpath: SelectionObject.SubElementNames[i].
        Examples:
        - 'Face6' if you select the top face of a cube solid made in Part.
        - 'Body.Box001.' if you select the tip of a Part->Body->"additive
            primitve" in PartDesign.
        - 'Body.Box001.Face6' if you select the top face of a Part->Body->
            "additive primitve" in PartDesign.

    """
    objects = []
    names = strip_subelement(subobject_fullpath).split('.')
    subobject = root_object
    for name in names:
        subobject = get_subobject_by_name(subobject, name)
        if subobject is None:
            # This should not append.
            return []
        objects.append(subobject)
    return objects


def add_property(
        obj: DO,
        type_: str,
        name: str,
        category: str,
        help_: str,
        ) -> tuple[DO, str]:
    """Add a dynamic property to the object.

    Return the `App::FeaturePython` object containing the property and the
    real property name.

    """
    name = get_valid_property_name(name)

    if name not in obj.PropertiesList:
        return obj.addProperty(type_, name, category, tr(help_)), name

    return obj, name


def get_properties_of_category(
        obj: DO,
        category: str,
        ) -> list[str]:
    """Return the list of properties belonging to the category."""
    properties: list[str] = []
    try:
        for p in obj.PropertyList:
            if obj.getGroupOfProperty(p) == category:
                properties.append(p)
    except AttributeError:
        return []
    return properties


def _has_ros_type(obj: DO, type_: str) -> bool:
    """Return True if the object is an object from this workbench."""
    if not isinstance(obj, DO):
        return False
    return hasattr(obj, '_Type') and (obj._Type == type_)


def is_robot(obj: DO) -> bool:
    """Return True if the object is a Ros::Robot."""
    return _has_ros_type(obj, 'Ros::Robot')


def is_link(obj: DO) -> bool:
    """Return True if the object is a Ros::Link."""
    return _has_ros_type(obj, 'Ros::Link')


def is_joint(obj: DO) -> bool:
    """Return True if the object is a Ros::Link."""
    return _has_ros_type(obj, 'Ros::Joint')


def is_simple_joint(obj: DO) -> bool:
    """Return True if prismatic, revolute, or continuous."""
    return (is_joint(obj)
            and (obj.Type in ['prismatic', 'revolute', 'continuous']))


def _is_derived_from(obj: DO, typeid: str) -> bool:
    """Return True if the object is a object of the given type."""
    if not isinstance(obj, DO):
        return False
    return hasattr(obj, 'isDerivedFrom') and obj.isDerivedFrom(typeid)


def is_box(obj: DO) -> bool:
    """Return True if the object is a 'Part::Box'."""
    return _is_derived_from(obj, 'Part::Box')


def is_sphere(obj: DO) -> bool:
    """Return True if the object is a 'Part::Sphere'."""
    return _is_derived_from(obj, 'Part::Sphere')


def is_cylinder(obj: DO) -> bool:
    """Return True if the object is a 'Part::Cylinder'."""
    return _is_derived_from(obj, 'Part::Cylinder')


def is_primitive(obj: DO) -> bool:
    """Return True if the object is a 'Part::{Box,Cylinder,Sphere}'."""
    return is_box(obj) or is_sphere(obj) or is_cylinder(obj)


def is_mesh(obj: DO) -> bool:
    """Return True if the object is a 'Mesh::Feature'."""
    return _is_derived_from(obj, 'Mesh::Feature')


def is_part(obj: DO) -> bool:
    """Return True if the object is a 'App::Part'."""
    return _is_derived_from(obj, 'App::Part')


def is_group(obj: DO) -> bool:
    """Return True if the object is a 'App::DocumentObjectGroup'."""
    return _is_derived_from(obj, 'App::DocumentObjectGroup')


def is_container(obj: DO) -> bool:
    """Return True if the object can contain other objects."""
    return is_part(obj) or is_group(obj)


def is_freecad_link(obj: DO) -> bool:
    """Return True if the object is a 'App::Link'."""
    return _is_derived_from(obj, 'App::Link')


def is_lcs(obj: DO) -> bool:
    """Return True if the object is a 'PartDesign::CoordinateSystem'."""
    return _is_derived_from(obj, 'PartDesign::CoordinateSystem')


def is_robot_selected() -> bool:
    """Return True if the first selected object is a Ros::Object."""
    if not fc.GuiUp:
        return False
    if fc.activeDocument() is None:
        return False
    sel = fcgui.Selection.getSelection()
    if not sel:
        return False
    return is_robot(sel[0])


def has_placement(obj: DO) -> bool:
    """Return True if obj has a Placement."""
    return (hasattr(obj, 'Placement')
            and isinstance(obj.Placement, fc.Placement))


def get_links(objs: DOList) -> list[RosLink]:
    """Return only the objects that are Ros::Link instances."""
    return [o for o in objs if is_link(o)]


def get_joints(objs: DOList) -> list[RosJoint]:
    """Return only the objects that are Ros::Joint instances."""
    return [o for o in objs if is_joint(o)]


def get_chains(
        links: list[RosLink],
        joints: list[RosJoint],
        ) -> list[DOList]:
    base_links: list[RosLink] = []
    tip_links: list[RosLink] = []
    for link in links:
        if link.Proxy.may_be_base_link():
            base_links.append(link)
        if link.Proxy.is_tip_link():
            tip_links.append(link)
    if len(base_links) > 1:
        warn('Two root links found, not supported', True)
        return []
    chains: list[DOList] = []
    for link in tip_links:
        chain = get_chain(link)
        if chain:
            chains.append(chain)
    return chains


def get_chain(link: RosLink) -> DOList:
    """Return the chain from base_link to link, excluded.

    The chain start with the base link, then alternates a joint and a link. The
    last item is the joint that has `link` as child.

    """
    chain: DOList = []
    ref_joint = link.Proxy.get_ref_joint()
    if not ref_joint:
        # A root link.
        return [link]
    if not ref_joint.Parent:
        warn(f'{label_or(ref_joint)} has no parent', True)
        # Return only ref_joint to indicate an error.
        return [ref_joint]
    subchain = get_chain(ref_joint.Parent)
    if subchain and is_joint(subchain[0]):
        # Propagate the error of missing joint.Parent.
        return subchain
    chain += subchain + [ref_joint] + [link]
    return chain


def is_subchain(subchain: DOList, chain: DOList) -> bool:
    """Return True if all items in `subchain` are in `chain`."""
    for link_or_joint in subchain:
        if link_or_joint not in chain:
            return False
    return True


def hasallattr(obj: Any, attrs: list[str]):
    """Return True if object has all attributes."""
    for attr in attrs:
        if not hasattr(obj, attr):
            return False
    return True


def get_placement(obj: DO) -> fc.Placement:
    """Return the object's placement."""
    if not isinstance(obj, DO):
        raise RuntimeError('Not a DocumentObject')
    if obj.TypeId == 'App::Link':
        if not obj.LinkedObject:
            # A link to nothing.
            return obj.LinkPlacement
        parent, subname = obj.Parents[0]
        return_type_placement = 3
        return parent.getSubObject(subname, retType=return_type_placement)
    else:
        if hasattr(obj, 'getGlobalPlacement'):
            return obj.getGlobalPlacement()
        elif hasattr(obj, 'Placement'):
            return obj.Placement


def split_package_path(package_path: [Path | str]) -> tuple[Path, Path]:
    """Return the package parent and the package name."""
    package_path = Path(package_path)
    if not package_path.is_dir():
        error('"OutputPath" must be a directory', True)
    path = package_path.resolve()
    parent = path.parent
    package_name = path.stem
    return parent, package_name


def read_mesh_dae(
        filename: [Path | str],
        ) -> Mesh.MeshObject:
    current_doc = fc.activeDocument()
    path = Path(filename)
    tmp_doc = fc.newDocument(hidden=True, temp=True)
    fc.setActiveDocument(tmp_doc.Name)
    read_dae(str(path))
    # A `Mesh::MeshObject`.
    merged_raw_mesh = Mesh.Mesh()
    for mesh_obj in tmp_doc.Objects:
        merged_raw_mesh.addMesh(mesh_obj.Mesh)
    fc.closeDocument(tmp_doc.Name)
    if current_doc:
        fc.setActiveDocument(current_doc.Name)
    return merged_raw_mesh


def save_mesh_dae(obj: DO,
                  filename: [Path | str],
                  ) -> None:
    """Save the mesh of a FreeCAD object into a Collada file."""
    Path(filename).parent.mkdir(parents=True, exist_ok=True)
    export_dae([obj], str(filename))


def save_mesh(obj: DO,
              filename: [Path | str],
              ) -> None:
    """Save the mesh of a FreeCAD object into a file.

    The type of the exported file is determined by the Mesh module.
    See the Mesh module for a list of supported formats.

    """
    Path(filename).parent.mkdir(parents=True, exist_ok=True)
    # TODO: scale to meters.
    Mesh.export([obj], str(filename))


def scale_mesh_object(obj: DO, scale_factor: [float | Iterable[float]]):
    """Scale a mesh object in place.

    Parameters
    ----------
    - obj: FreeCAD object of type `Mesh::Feature`.
    - scale_factor: a single float or a list of 3 floats.

    """
    if not is_mesh(obj):
        raise RuntimeError(
                'First argument must be `Mesh::Feature` FreeCAD object')
    if isinstance(scale_factor, float):
        scaling_vector = fc.Vector(scale_factor, scale_factor, scale_factor)
    else:
        try:
            scaling_vector = fc.Vector(scale_factor)
        except (IndexError, ValueError):
            raise RuntimeError('Scaling factor must be a float or a list'
                               f' of 3 floats, got {scale_factor}')
    scale_mat = fc.Matrix()
    scale_mat.scale(scaling_vector)
    mesh = obj.Mesh.copy()
    mesh.transform(scale_mat)
    obj.Mesh = mesh


def save_xml(
        xml: et.ElementTree,
        filename: [Path | str],
        ) -> None:
    """Save the xml element into a file."""
    file_path = Path(filename)
    file_path.parent.mkdir(parents=True, exist_ok=True)
    txt = minidom.parseString(et.tostring(xml)).toprettyxml(indent='  ')
    file_path.write_text(txt)


def make_group(
        doc_or_group: [fc.Document | DO],
        name: str,
        visible: bool = True,
        ) -> DO:
    """Create or retrieve a group."""
    if is_group(doc_or_group):
        doc = doc_or_group.Document
    else:
        doc = doc_or_group
    existing_group = doc.getObject(name)
    if existing_group and is_group(existing_group):
        return existing_group
    group = doc.addObject('App::DocumentObjectGroup', name)
    if is_group(doc_or_group):
        doc_or_group.addObject(group)
    if hasattr(group, 'Visibility'):
        group.Visibility = visible
    return group


def add_object(
        container: [fc.Document | DO],
        type_: str,
        name: str,
        ) -> DO:
    """Create a new object into the container.

    The object's label will be set `name` but, according to your settings in
    FreeCAD, the label will not be set if there's already an object with that
    Label. The object's name might be also different if FreeCAD decides so.

    """
    if is_container(container):
        doc = container.Document
    else:
        doc = container
    if not isinstance(doc, fc.Document):
        raise RuntimeError('First argument is not a Document, a Group, or a Part')
    obj = doc.addObject(type_, name)
    obj.Label = name
    if is_container(container):
        container.addObject(obj)
    return obj


def grouper(iterable, n, fillvalue=None):
    """Collect data into fixed-length chunks or blocks."""
    # grouper('ABCDEFG', 3, 'x') --> ABC DEF Gxx"
    # From https://docs.python.org/3.8/library/itertools.html.
    args = [iter(iterable)] * n
    return zip_longest(*args, fillvalue=fillvalue)
