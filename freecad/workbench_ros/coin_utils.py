from __future__ import annotations

import FreeCAD as fc

from pivy import coin


def transform_from_placement(
        placement: [fc.Placement | fc.Vector | fc.Rotation],
        ) -> coin.SoTransform:
    """Return the SoTransform equivalent to the placement.

    # Should show a cone entering a cylinder.
    >>> placement = App.Placement(App.Vector(10, 20, 30), App.Rotation(10, 20, 30))
    >>> cyl_to_y = App.Placement(App.Rotation(App.Vector(1, 0, 0), -90).toMatrix())
    >>> cyl = App.ActiveDocument.addObject('Part::Cylinder', 'Cylinder')
    >>> cyl.Placement = placement * cyl_to_y
    >>> cone = coin.SoCone()  # Centered, along y.
    >>> sg = Gui.ActiveDocument.ActiveView.getSceneGraph()
    >>> sep = coin.SoSeparator()
    >>> sep.addChild(transform_from_placement(placement))
    >>> sep.addChild(cone)
    >>> sg.addChild(sep)
    >>> App.ActiveDocument.recompute()
    """
    if (not (isinstance(placement, fc.Placement)
             or isinstance(placement, fc.Vector)
             or isinstance(placement, fc.Rotation))):
        raise RuntimeError('Argument must be a FreeCAD.{Placement,Vector,Rotation}')
    if isinstance(placement, fc.Vector):
        placement = fc.Placement(placement, fc.Rotation())
    elif isinstance(placement, fc.Rotation):
        placement = fc.Placement(placement.toMatrix())
    transform = coin.SoTransform()
    transform.translation = placement.Base
    transform.rotation = placement.Rotation.Q
    return transform


def arrow_group(
        points: list[fc.Vector],
        color: tuple[float] = (0.0, 0.0, 1.0),
        scale: float = 1.0,
        ) -> coin.SoSeparator:
    """Return the SoSeparator of an arrow between two points.

    """
    if len(points) < 2:
        raise RuntimeError('At least 2 points expected')
    if len(color) != 3:
        raise RuntimeError('color must have 3 values')

    # Ratio cone_height / length_between_points.
    head_height_ratio = 0.3

    # Ratio cone_base_radius / cylinder_radius.
    head_radius_ratio = 2.0

    p0, p1 = points[:2]
    if ((not isinstance(p0, fc.Vector))
            or (not isinstance(p1, fc.Vector))):
        raise RuntimeError('The list of points must contains FreeCAD.Vector instances')

    v = p1 - p0
    # Rotation to bring the x-axis to v.
    rotation = fc.Rotation(fc.Vector(1.0, 0.0, 0.0), v)

    # An SoCylinder is centered, along y.
    cylinder = coin.SoCylinder()
    cylinder.height = v.Length * (1.0 - head_height_ratio)
    cylinder.radius = v.Length / 20.0 * scale

    # Transform to bring the cylinder with origin at bottom, along x.
    along_x = coin.SoTransform()
    # Translation after the rotation.
    along_x.translation = cylinder.height.getValue() / 2.0, 0.0, 0.0
    along_x.rotation = fc.Rotation(fc.Vector(0.0, 0.0, 1.0), -90).Q

    cylinder_sep = coin.SoSeparator()
    cylinder_sep.addChild(along_x)
    cylinder_sep.addChild(cylinder)

    # An SoCone is centered, along y+.
    cone = coin.SoCone()
    cone.bottomRadius = cylinder.radius.getValue() * head_radius_ratio
    cone.height = v.Length * head_height_ratio

    # Transform to bring the top of the cone to p1.
    cone_trans = coin.SoTransform()
    # Translation after the rotation.
    cone_trans.translation = v.Length - (cone.height.getValue() / 2.0), 0.0, 0.0
    cone_trans.rotation = fc.Rotation(fc.Vector(0.0, 0.0, 1.0), -90).Q

    cone_sep = coin.SoSeparator()
    cone_sep.addChild(cone_trans)  # Translation will be done first.
    cone_sep.addChild(cone)

    # Transform to the first point (with orientation).
    transform = coin.SoTransform()
    transform.translation = p0
    transform.rotation = rotation.Q

    coin_color = coin.SoBaseColor()
    coin_color.rgb = color

    group = coin.SoSeparator()
    group.addChild(transform)
    group.addChild(coin_color)
    group.addChild(cylinder_sep)
    group.addChild(cone_sep)
    return group


def face_group(
        points: list[fc.Vector],
        color: tuple[float] = (0.0, 0.0, 1.0),
        ) -> coin.SoSeparator:
    if len(points) < 3:
        raise RuntimeError('At least 3 points expected')
    if len(color) != 3:
        raise RuntimeError('color must have 3 values')

    # Create a material to fill the square.
    material = coin.SoMaterial()
    material.diffuseColor = color

    # Create the vertices of the face.
    vertices = coin.SoVertexProperty()
    n = len(points)
    vertices.vertex.setValues(0, n, points)

    # Create the face.
    indices = coin.SoIndexedFaceSet()
    indices.coordIndex.setValues(0, n + 1, list(range(n)) + [-1])

    # Set the material for the face.
    # indices.addChild(material)

    # Create a group and add the vertices and indices.
    group = coin.SoSeparator()
    group.addChild(material)
    group.addChild(vertices)
    group.addChild(indices)
    return group
