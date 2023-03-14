# Generates a ROS::Robot from a URDF robot.

from __future__ import annotations

from typing import Any, Iterable, Tuple

import FreeCAD as fc

try:
    from urdf_parser_py.urdf import Collision as UrdfCollision
    from urdf_parser_py.urdf import Joint as UrdfJoint
    from urdf_parser_py.urdf import Link as UrdfLink
    from urdf_parser_py.urdf import Robot as UrdfRobot
    from urdf_parser_py.urdf import Visual as UrdfVisual

    from .urdf_parser_utils import obj_from_geometry
    from .urdf_parser_utils import placement_from_origin
except ModuleNotFoundError:
    UrdfCollision = Any
    UrdfJoint = Any
    UrdfLink = Any
    UrdfRobot = Any
    UrdfVisual = Any

from .link import make_link
from .robot import make_robot
from .utils import add_object
from .utils import make_group

# Typing hints.
DO = fc.DocumentObject
DOList = Iterable[DO]
DOG = fc.DocumentObjectGroup
AppLink = DO  # TypeId == 'App::Link'
AppPart = DO  # TypeId == 'App::Part'
RosLink = DO  # A Ros::Link, i.e. a DocumentObject with Proxy "Link".
RosJoint = DO  # A Ros::Joint, i.e. a DocumentObject with Proxy "Joint".
RosRobot = DO  # A Ros::Robot, i.e. a DocumentObject with Proxy "Robot".
# List of UrdfVisual or List of UrdfCollision.
VisualList = Iterable[UrdfVisual]
CollisionList = Iterable[UrdfCollision]


def robot_from_urdf(
        doc: fc.Document,
        urdf_robot: UrdfRobot,
        ) -> DO:
    """Creates a ROS::Robot from URDF."""

    robot, parts_group = _make_robot(doc, urdf_robot.name)
    # link_map: dict[str, RosLink] = {}
    # visual_map: dict[str, AppPart] = {}
    # collision_map: dict[str, AppPart] = {}
    for urdf_link in urdf_robot.links:
        ros_link, visual_part, collision_part = _add_ros_link(
            robot, parts_group, urdf_link.name)
        # link_map[urdf_link.name] = ros_link
        # visual_map[urdf_link.name] = visual_part
        # collision_map[urdf_link.name] = collision_part
        _add_visual(urdf_link, parts_group, ros_link, visual_part)
        _add_collision(urdf_link, parts_group, ros_link, collision_part)
    # joint_map: dict[str, RosJoint] = {}


def _make_robot(
        doc: fc.Document,
        name: str = 'robot',
        ) -> tuple[DO, DO]:
    """Create a ROS::Robot

    Return (robot object, parts group).

    The group called 'Parts' is potentially created and returned. If the object
    'Parts' is not a group, a different name will be given.

    """

    robot = make_robot(name, doc)
    # Create a group 'Parts' to hold all parts in the assembly document.
    parts_group = make_group(doc, 'URDF Parts', visible=False)

    return robot, parts_group


def _add_ros_link(
        robot: RosRobot,
        parts_group: DOG,
        name: str,
        ) -> Tuple[DO, DO, DO]:
    """Add two App::Part to the group and links to them to the robot.

    - Add an "App::Part" for the visual of each link.
    - Add an "App::Part for the collision of each link.
    - Add a "Ros::Link" with references to them.

    Return ???.

    Parameters
    ----------
    - robot: robot to add the Ros::Link to.
    - part_groups: group for geometries and "App::Part" objects.
    - name: name of the link if the URDF description.

    """
    doc = robot.Document
    visual_part = add_object(parts_group, 'App::Part', f'visual_{name}_')
    collision_part = add_object(parts_group, 'App::Part', f'collision_{name}_')
    ros_link = make_link(name, doc)
    ros_link.adjustRelativeLinks(robot)
    robot.addObject(ros_link)
    # Implementation note: ros_link.Visual.append() doesn't work because ros_link.Visual
    # is a new object on each evoking.
    # TODO: make a function utils.make_link.
    container = doc  # Should be `parts_group` but because of bug
        # 'Object can only be in a single Group' must be added to `doc`.
        # doc.recompute() doesn't help.
    link_to_visual_part = add_object(container, 'App::Link',
                                     f'visual_{name}')
    link_to_visual_part.setLink(visual_part)
    ros_link.Visual = [link_to_visual_part]

    link_to_collision_part = add_object(container, 'App::Link',
                                        f'collision_{name}')
    link_to_collision_part.setLink(collision_part)
    ros_link.Collision = [link_to_collision_part]
    return ros_link, visual_part, collision_part


def _add_visual(
        urdf_link: UrdfLink,
        parts_group: DOG,
        ros_link: RosLink,
        visual_part: AppPart,
        ) -> None:
    """Add the visual geometries to an assembly.

    Parameters
    ==========

    - robot: an UrdfRobot.
    - link_map: a dict[urdf_link_name: "Ros::Link"].

    """
    name_linked_geom = f'{urdf_link.name}_visual'
    return _add_geometries(parts_group, ros_link, visual_part, urdf_link.visuals, name_linked_geom)


def _add_collision(
        urdf_link: UrdfLink,
        parts_group: DOG,
        ros_link: RosLink,
        collision_part: AppPart,
        ) -> None:
    """Add the visual geometries to an assembly.

    Parameters
    ==========

    - robot: an UrdfRobot.
    - link_map: a dict[urdf_link_name: "Ros::Link"].

    """
    name_linked_geom = f'{urdf_link.name}_collision'
    return _add_geometries(parts_group, ros_link, collision_part, urdf_link.collisions, name_linked_geom)


def _add_geometries(
        parts_group: DOG,
        ros_link: DOG,
        part: AppPart,
        geometries: [VisualList | CollisionList],
        name_linked_geom: str,
        ) -> tuple[list[DO], list[DO]]:
    """Add the geometries from URDF into `group` and an App::Link to it into `link`.

    `geometries` is either `visuals` or `collisions` and the geometry itself is
    `geometries[?].geometry`.
    If `name_linked_geom` is empty, not FC link is created in `link`.

    Return the list of objects reprensenting the geometries and the list of
    FreeCAD links.

    Parameters
    ==========

    - group: an "App::DocumentObjectGroup" where the generated FreeCAD objects
             will be placed.
    - geometries: list of URDF geometries.
    - link: an "App::Link" object linking to an "App::Part" object representing
            the visual- or collision geometries of a URDF link.
    - name_linked_geom: base pattern for the generated "App::Link" objects
                        generated. The final name may then be
                        `name_linked_geom`, `name_linked_geom`001, ...

    """
    geom_objs: DOList = []
    for geometry in geometries:
        # Make the FC object in the group.
        try:
            geom_obj, _ = obj_from_geometry(geometry.geometry, parts_group)
        except NotImplementedError:
            continue
        geom_objs.append(geom_obj)
        if hasattr(geometry, 'origin'):
            geom_obj.Placement = (placement_from_origin(geometry.origin)
                                  * geom_obj.Placement)
        # Add a reference to geom_obj to `ros_link.Visual` or
        # `ros_link.Collision`.
        link_to_geom = add_object(part, 'App::Link', name_linked_geom)
        link_to_geom.setLink(geom_obj)
        link_to_geom.Placement = geom_obj.Placement
    return geom_objs
