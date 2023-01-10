from __future__ import annotations

import FreeCAD as fc


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


def get_subobject_by_name(object_: fc.DocumentObject,
                          subobject_name: str,
                          ) -> fc.DocumentObject:
    """Return the appropriate object from object_.OutListRecursive."""
    for o in object_.OutListRecursive:
        if o.Name == subobject_name:
            return o


def get_subobjects_by_full_name(
        root_object: fc.DocumentObject,
        subobject_fullpath: str,
        ) -> list[fc.DocumentObject]:
    """Return the list of objects from root_object excluded to the named object.

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


def get_global_placement(root_object: fc.DocumentObject,
                         subobject_fullpath: str,
                         ) -> fc.Placement:
    """Return the global placement by recursively going through parents.

    ``object.getGlobalPlacement()`` returns the placement of the original
    object, not the linked one. This function recursively goes through the
    hierarchy, including links to retrieve the linked object placement.

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
    objects = get_subobjects_by_full_name(root_object, subobject_fullpath)
    placement = root_object.Placement
    for o in objects:
        placement *= o.Placement
    return placement
