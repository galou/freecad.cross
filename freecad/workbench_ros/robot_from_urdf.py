# Generates a ROS::Robot from a URDF robot.

from __future__ import annotations

from typing import Any, Iterable, Tuple

import FreeCAD as fc

try:
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
    link_map: dict[str, RosLink] = {}
    visual_map: dict[str, AppPart] = {}
    collision_map: dict[str, AppPart] = {}
    for link in robot.links:
        ros_link, visual_part, collision_part = _add_ros_link(
                robot, parts_group, link.name)
        link_map[link.name] = ros_link
        visual_map[link.name] = visual_part
        collision_map[link.name] = collision_part
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
    visual_part = doc.addObject('App::Part', f'visual_{name}')
    collision_part = doc.addObject('App::Part', f'collision_{name}')
    ros_link = make_link(name, doc)
    ros_link.adjustRelativeLinks(robot)
    robot.addObject(ros_link)
    return ros_link, visual_part, collision_part


def _add_visual(
        robot: UrdfRobot,
        parts_group: DOG,
        link_map: dict[str, RosLink],
        ) -> None:
    """Add the visual geometries to an assembly.

    Parameters
    ==========

    - robot: an UrdfRobot.
    - link_map: a dict[urdf_link_name: "Ros::Link"].

    """
    for link_name, ros_link in link_map.items():
        _add_visual_geometries(parts_group, ros_link, link_name,
                               robot.link_map[link_name].visuals)


def _add_visual_geometries(
        parts_group: DOG,
        ros_link: RosLink,
        link_name: str,
        geometries: VisualList,
        ) -> tuple[list[DO], list[DO]]:
    """Add the primitive shapes and meshes to a Ros::Link.

    Parameters
    ==========

    - ros_link: a "RosLink" object.
    - link_name: name of the URDF link.
    - geometries: list of URDF geometries.

    """
    name_linked_geom = f'{link_name}_visual'
    return _add_geometries(parts_group, ros_link, geometries, 'visual', name_linked_geom)


def _add_geometries(
        parts_group: DOG,
        ros_link: DOG,
        geometries: [VisualList | CollisionList],
        lod: str,
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
        link_to_geom = add_object(parts_group, 'App::Link',
                                               name_linked_geom)
        link_to_geom.setLink(geom_obj)
        link_to_geom.Placement = geom_obj.Placement
        if lod == 'visual':
            ros_link.Visual.append(link_to_geom)
        if lod == 'collision':
            ros_link.Collision.append(link_to_geom)
    return geom_objs
