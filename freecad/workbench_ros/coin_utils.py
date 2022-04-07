from typing import List, Tuple

import FreeCAD as fc

from pivy import coin


def arrow_group(
        points: List[fc.Vector],
        color: Tuple[float] = (0.0, 0.0, 1.0),
        scale: float = 1.0):
    """Return the SoSeparator of an arrow between two points.

    """
    if len(points) < 2:
        raise RuntimeError('Not enough points')
    if len(color) != 3:
        raise RuntimeError('color must have three values')

    # Ratio cone_height / length_between_points.
    head_height_ratio = 0.3

    # Ratio cone_base_radius / cylinder_radius.
    head_radius_ratio = 2.0

    p0, p1 = points[:2]
    if ((not isinstance(p0, fc.Vector))
            or (not isinstance(p1, fc.Vector))):
        raise RuntimeError('The list must contains FreeCAD.Vector instances')

    v = p1 - p0
    # Rotation to bring the x-axis to v.
    rotation = fc.Rotation(fc.Vector(1.0, 0.0, 0.0), v)

    # An SoCylinder is centered, along y.
    cylinder = coin.SoCylinder()
    cylinder.height = v.Length * (1.0 - head_height_ratio)
    cylinder.radius = v.Length / 20.0 * scale

    # Transform to bring the cylinder with origin at bottom, along x.
    along_x = coin.SoTransform()
    along_x.translation = cylinder.height.getValue() / 2.0, 0.0, 0.0  # After the rotation.
    along_x.rotation = 0.0, 0.0, 0.7071067811865476, -0.7071067811865475  # Rotation about z, -90°.

    cylinder_sep = coin.SoSeparator()
    cylinder_sep.addChild(along_x)
    cylinder_sep.addChild(cylinder)

    # An SoCone is centered, along y+.
    cone = coin.SoCone()
    cone.bottomRadius = cylinder.radius.getValue() * head_radius_ratio
    cone.height = v.Length * head_height_ratio

    # Transform to bring the top of the cone to p1.
    cone_trans = coin.SoTransform()
    cone_trans.translation = v.Length - (cone.height.getValue() / 2.0), 0.0, 0.0  # After the rotation.
    cone_trans.rotation = 0.0, 0.0, 0.7071067811865476, -0.7071067811865475  # Rotation about z, -90°.

    cone_sep = coin.SoSeparator()
    cone_sep.addChild(cone_trans) # Translation will be done first.
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
