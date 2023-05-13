"""Functions that could have belonged to FreeCAD."""

from __future__ import annotations

from abc import ABC
from copy import copy
import string
from typing import Any, Iterable, Optional

import FreeCAD as fc

if hasattr(fc, 'GuiUp') and fc.GuiUp:
    from PySide import QtCore  # FreeCAD's PySide!
    from PySide import QtGui  # FreeCAD's PySide!

    def tr(text: str) -> str:
        return QtGui.QApplication.translate('cross', text)
else:
    def tr(text: str) -> str:
        return text

# Typing hints.
DO = fc.DocumentObject
DOList = Iterable[DO]


def with_fc_gui() -> bool:
    return hasattr(fc, 'GuiUp') and fc.GuiUp


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


def label_or(
        obj: DO,
        alternative: str = 'no label') -> str:
    """Return the `Label` or the alternative."""
    if not hasattr(obj, 'Label'):
        return 'not_a_FreeCAD_object'
    return obj.Label if hasattr(obj, 'Label') else alternative


def message(text: str, gui: bool = False) -> None:
    """Inform the user."""
    fc.Console.PrintMessage(text + '\n')
    if gui and fc.GuiUp:
        diag = QtGui.QMessageBox(QtGui.QMessageBox.Information,
                                 'ROS Workbench', text)
        diag.setWindowModality(QtCore.Qt.ApplicationModal)
        diag.exec_()


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
        default: Any = None,
        ) -> tuple[DO, str]:
    """Add a dynamic property to the object.

    Return the `App::FeaturePython` object containing the property and the
    real property name.

    """
    name = get_valid_property_name(name)

    if name not in obj.PropertiesList:
        obj.addProperty(type_, name, category, tr(help_))
        if default is not None:
            setattr(obj, name, default)

    return obj, name


def get_properties_of_category(
        obj: DO,
        category: str,
        ) -> list[str]:
    """Return the list of properties belonging to the category."""
    properties: list[str] = []
    try:
        for p in obj.PropertiesList:
            if obj.getGroupOfProperty(p) == category:
                properties.append(p)
    except AttributeError:
        return []
    return properties


def is_derived_from(obj: DO, typeid: str) -> bool:
    """Return True if the object is a object of the given type."""
    if not isinstance(obj, DO):
        return False
    return hasattr(obj, 'isDerivedFrom') and obj.isDerivedFrom(typeid)


def has_type(obj: DO, typeid: str) -> bool:
    """Return True if the object has the given type, evaluating also its Proxy.

    Return True if the object is derived from the given type or obj.Proxy.Type
    is the given type.

    """
    return (is_derived_from(obj, typeid)
            or (hasattr(obj, 'Proxy')
                and hasattr(obj.Proxy, 'Type')
                and obj.Proxy.Type == typeid))


def is_body(obj: DO) -> bool:
    """Return True if the object is a 'Part::Box'."""
    return is_derived_from(obj, 'PartDesign::Body')


def is_box(obj: DO) -> bool:
    """Return True if the object is a 'Part::Box'."""
    return is_derived_from(obj, 'Part::Box')


def is_sphere(obj: DO) -> bool:
    """Return True if the object is a 'Part::Sphere'."""
    return is_derived_from(obj, 'Part::Sphere')


def is_cylinder(obj: DO) -> bool:
    """Return True if the object is a 'Part::Cylinder'."""
    return is_derived_from(obj, 'Part::Cylinder')


def is_mesh(obj: DO) -> bool:
    """Return True if the object is a 'Mesh::Feature'."""
    return is_derived_from(obj, 'Mesh::Feature')


def is_part(obj: DO) -> bool:
    """Return True if the object is a 'App::Part'."""
    return is_derived_from(obj, 'App::Part')


def is_origin(obj: DO) -> bool:
    """Return True if the object is a 'App::Origin'."""
    return is_derived_from(obj, 'App::Origin')


def is_group(obj: DO) -> bool:
    """Return True if the object is a 'App::DocumentObjectGroup'."""
    return is_derived_from(obj, 'App::DocumentObjectGroup')


def is_container(obj: DO) -> bool:
    """Return True if the object can contain other objects."""
    return is_part(obj) or is_group(obj)


def is_link(obj: DO) -> bool:
    """Return True if the object is a 'App::Link'."""
    return is_derived_from(obj, 'App::Link')


def is_lcs(obj: DO) -> bool:
    """Return True if the object is a 'PartDesign::CoordinateSystem'."""
    return is_derived_from(obj, 'PartDesign::CoordinateSystem')


def has_placement(obj: DO) -> bool:
    """Return True if obj has a Placement."""
    return (hasattr(obj, 'Placement')
            and isinstance(obj.Placement, fc.Placement))


def is_same_placement(
        p1: fc.Placement,
        p2: fc.Placement,
        trans_tol: float = 1e-6,
        rot_tol: float = 1e-7) -> bool:
    """Return True if both placements represent the same transform."""
    return (((p2.Base - p1.Base).Length < trans_tol)
            and p2.Rotation.isSame(p1.Rotation, rot_tol))


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
    candidates = doc.getObjectsByLabel(name)
    existing_group = candidates[0] if candidates else None
    if existing_group and is_group(existing_group):
        return existing_group
    group = doc.addObject('App::DocumentObjectGroup', name)
    group.Label = name
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


def get_leafs_and_subnames(obj: DO) -> list[tuple[DO, str]]:
    """Return all leaf subobjects and their path.

    Return a list of (object, path), where path (also called subname) can be
    used to retrieve the physical placement of the object, for example with
    `obj.getSubObject(path)`.

    Parameters
    ----------
    - obj: a FreeCAD object that has the attribute `getSubObjects()`.
           If the object doesn't have `getSubObjects()`, it's considered a leaf
           and (obj, '') is returned.

    """
    def get_subobjects_recursive(
            obj: DO,
            subname: str,
            ) -> list[tuple[DO, str]]:
        if (not hasattr(obj, 'getSubObjects')) or (not obj.getSubObjects()):
            # A leaf node.
            return [(obj, subname)]
        outlist: list[tuple[DO, str]] = []
        subnames = obj.getSubObjects()
        for name in subnames:
            o = obj.getSubObjectList(name)[-1]
            outlist += get_subobjects_recursive(o, f'{subname}{name}')

        return outlist

    return get_subobjects_recursive(obj, '')


def validate_types(objects: DOList, types: list[str]) -> DOList:
    """Sort objects by required types.

    Raises a RuntimeError if a listed type has no appropriate object in the input
    list.

    """
    if len(objects) < len(types):
        raise RuntimeError('Less types required that the number of objects')

    copy_of_objects = copy(objects)
    copy_of_types = copy(types)
    objects_of_precise_type: DOList = []
    indexes_of_any: list[int] = []
    for i, type_ in enumerate(types):
        object_found = False
        for o in copy_of_objects:
            if object_found:
                # Indirectly, next `type_`.
                continue
            if type_ in [None, 'any', 'Any']:
                indexes_of_any.append(i)
                copy_of_types.pop(0)
                object_found = True
                # Indirectly, next `type_`.
                continue
            if has_type(o, type_):
                objects_of_precise_type.append(o)
                copy_of_objects.remove(o)
                copy_of_types.pop(0)
                object_found = True
                # Indirectly, next `type_`.
                continue
        if not object_found:
            raise RuntimeError(f'No object of type "{type_}"')
    outlist: DOList = []
    for i in range(len(types)):
        if i in indexes_of_any:
            outlist.append(copy_of_objects.pop(0))
        else:
            outlist.append(objects_of_precise_type.pop(0))
    return outlist


class ProxyBase(ABC):
    """A base class for proxies of dynamic (scripted) objects in FreeCAD."""

    def __init__(self, object_name: str, properties: list[str]):
        self._object_name: str = object_name
        self._properties: list[str] = properties

    def is_ready(self) -> bool:
        """Return True if the object and all properties are defined.

        Return True if `self` has the attribute `self._object_name` and
        `self._object_name` has all attributes given in `self._properties`.

        """
        try:
            obj = getattr(self, self._object_name)
        except AttributeError:
            return False
        for p in self._properties:
            if not hasattr(obj, p):
                return False
        return True
