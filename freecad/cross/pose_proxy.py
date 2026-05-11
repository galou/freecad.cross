from __future__ import annotations

from typing import Optional

import FreeCAD as fc

from .freecad_utils import warn
from .wb_utils import ICON_PATH
from .wb_utils import is_link
from .wb_utils import is_robot
from .wb_utils import ros_name
from freecad.cross.vendor.fcapi import fpo  # Cf. https://github.com/mnesarco/fcapi

# Stubs and type hints.
from .link import Link as CrossLink  # A Cross::Link, i.e. a DocumentObject with Proxy "Link". # noqa: E501
from .pose import Pose as CrossPose  # A Cross::Pose, i.e. a DocumentObject with Proxy "Pose". # noqa: E501
from .pose import ViewProviderPose as VP


def _get_potential_end_effectors(pose: CrossPose) -> list[str]:
    """Return the list of potential end-effectors."""
    if ((not hasattr(pose, 'Proxy'))
            or (pose.Proxy is None)
            or (not hasattr(pose, 'Robot'))):
        return []
    return pose.Proxy.get_potential_end_effectors()


@fpo.view_proxy(
        icon=str(ICON_PATH / 'pose.svg'),
)
class _ViewProviderPose:
    """The view provider for CROSS::Pose objects."""

    show_end_effector = fpo.PropertyBool(
            name='ShowEndEffector',
            section='ROS Display Options',
            description=(
                'Whether to show the end-effector link or a symbolic'
                'representation of the pose'
                ),
            default=True,
     )

    axis_length = fpo.PropertyLength(
            name='AxisLength',
            section='ROS Display Options',
            description="Length of the rods for the joint's axes",
            default=500.0,  # mm.
    )

    shaded_display_mode = fpo.DisplayMode(name='Shaded', is_default=True)
    wireframe_display_mode = fpo.DisplayMode(name='Wireframe')

    def on_object_change(self) -> None:
        """Callback when the data object changes."""
        self.draw()

    @show_end_effector.observer
    def _show_end_effector_onchange(self) -> None:
        """Callback when the `ShowEndEffector` property changed."""
        self.draw()

    @axis_length.observer
    def _axis_length_onchange(self) -> None:
        """Callback when the `AxisLength` property changed."""
        self.draw()

    def on_change(self, event: fpo.events.PropertyChangedEvent) -> None:
        """Callback when a property changed.

        We need this for properties defined by mother classes.
        """
        if event.property_name == 'Visibility':
            self.draw()

    def draw(self) -> None:
        from pivy import coin
        from .coin_utils import tcp_group
        from .coin_utils import transform_from_placement

        vobj: VP = self.ViewObject
        obj: CrossPose = vobj.Object

        if not hasattr(vobj, 'RootNode'):
            return
        root = vobj.RootNode
        root.removeAllChildren()

        shaded = coin.SoSeparator()
        root.addChild(shaded)
        style = coin.SoDrawStyle()
        style.style = coin.SoDrawStyle.LINES
        wireframe = coin.SoSeparator()
        wireframe.addChild(style)
        root.addChild(wireframe)

        if not vobj.Visibility:
            return
        sep = coin.SoSeparator()
        sep.addChild(transform_from_placement(obj.Placement))
        sep.addChild(
            tcp_group(
            tcp_length_mm=0.66 * vobj.AxisLength,
            tcp_diameter_ratio_to_length=0.17,
            tcp_color=(0.7, 0.7, 0.7),
            axis_length_mm=vobj.AxisLength,
            axis_diameter_ratio_to_length=0.03,
            ),
        )

        if obj.Robot and self.show_end_effector:
            robot_proxy = obj.Robot.Proxy
            link = robot_proxy.get_link(obj.EndEffector)
            if link is not None:
                # TODO: only works if robot at null joint-space pose.
                # TODO: get the correct transform.
                link_sep = coin.SoSeparator()
                if link.Proxy.is_execute_ready():
                    robot = link.Proxy.get_robot()
                    if robot is None:
                        # A link outside of a robot (though it should not
                        # happen).
                        link_sep.addChild(_coin_separator_from_link(link))
                    else:
                        for fixed_with_link in robot.Proxy.get_links_fixed_with(
                                ros_name(link),
                        ):
                            ln_sep = coin.SoSeparator()
                            fixed_transform = robot.Proxy.get_transform(
                                ros_name(link),
                                ros_name(fixed_with_link),
                            )
                            ln_sep.addChild(
                                transform_from_placement(fixed_transform),
                            )
                            ln_sep.addChild(
                                _coin_separator_from_link(fixed_with_link),
                            )
                            link_sep.addChild(ln_sep)
                sep.addChild(link_sep)

        shaded.addChild(sep)
        wireframe.addChild(sep)


@fpo.proxy(
        object_type='App::GeometryPython',
        subtype='Cross::Pose',
        view_proxy=_ViewProviderPose,
)
class PoseProxy:
    """The proxy for CROSS::Pose objects."""

    robot = fpo.PropertyLink(
            name='Robot',
            section='Robot',
            description='The associated robot',
    )

    end_effector, end_effector_meta = fpo.PropertyOptions(
            name='EndEffector',
            section='Robot',
            description='End-effector link (from CROSS) to bring to the pose',
            meta=True,
            # Implementation note: no `self` available here.
            options_provider=_get_potential_end_effectors,
    )

    allow_non_leaf_link = fpo.PropertyBool(
         name='AllowNonLeafLink',
         section='Robot',
         description='Whether to list non-leaf links in `EndEffector`',
         default=False,
     )

    def on_attach(self) -> None:
        """Callback when the proxy is attached to the object."""
        self._end_effectors: list[str] = []

    @robot.observer
    def _robot_onchange(self) -> None:
        """Callback when the `Robot` property changed."""
        self._update_end_effector_list()

    @allow_non_leaf_link.observer
    def _allow_non_leaf_onchange(self) -> None:
        """Callback when the `AllowNonLeafLink` property changed."""
        self._update_end_effector_list()

    def _update_end_effector_list(self) -> None:
        """Update the list of potential end-effectors."""
        pose_obj = self.Object
        if not pose_obj.Robot:
            pose_obj.EndEffector = []
            return
        robot = pose_obj.Robot
        if ((not hasattr(robot, 'Proxy'))
                or (robot.Proxy is None)
                or (not robot.Proxy.is_execute_ready())):
            return
        chains = robot.Proxy.get_chains()
        links: set[CrossLink] = set()
        for chain in chains:
            if pose_obj.AllowNonLeafLink:
                links.update([ln for ln in chain if is_link(ln)])
            else:
                links.add(chain[-1])
        # Update the list of choices of the enumeration.
        # If the current value is in the new list, it'll be kept.
        # Otherwise, the new value will be set to the first entry.
        self.end_effector_meta.options = sorted([ros_name(ln) for ln in links])

    def get_potential_end_effectors(self) -> list[str]:
        self. _update_end_effector_list()
        return self._end_effectors


def make_pose(
        name: str,
        doc: Optional[fc.Document] = None,
) -> CrossPose:
    """Add a Cross::Pose to the current document."""
    if doc is None:
        doc = fc.activeDocument()
    if doc is None:
        warn('No active document, doing nothing', False)
        return
    obj: CrossPose = PoseProxy.create(name=name, doc=doc)
    obj.Label2 = name

    if hasattr(fc, 'GuiUp') and fc.GuiUp:
        import FreeCADGui as fcgui

        _ViewProviderPose(obj.ViewObject)

        # Set `obj.Robot` and possibly `obj.EndEffector` if the selected
        # object is a robot or a link.
        sel = fcgui.Selection.getSelection()
        if sel:
            candidate = sel[0]
            if is_robot(candidate):
                obj.Robot = candidate
            elif is_link(candidate) and (hasattr(candidate, 'Proxy')):
                robot = candidate.Proxy.get_robot()
                if robot:
                    obj.Robot = robot
                    link_name = ros_name(candidate)
                    if link_name not in obj.getEnumerationsOfProperty('EndEffector'):
                        obj.AllowNonLeafLink = True
                    obj.EndEffector = ros_name(candidate)
                    obj.Placement = candidate.Placement
    return obj


def _coin_separator_from_object(obj: 'fc.DocumentObject') -> 'coin.SoSeparator':
    """Return an independent SoSeparator for the visual representation of obj.

    Uses writeInventor() on the Shape or Mesh attribute to create independent
    coin nodes, entirely unrelated to obj's own view provider scene graph.
    This ensures that visibility or display-mode changes on obj (or any robot
    that contains it) do not affect the returned separator.

    The object's Placement is applied as an SoTransform so that the geometry
    appears at the same position as in obj's own view.

    Returns an empty separator when obj has neither a usable Shape nor a Mesh.

    """
    from pivy import coin
    from .coin_utils import transform_from_placement
    from .freecad_utils import is_mesh

    sep = coin.SoSeparator()

    inventor_str = None
    if hasattr(obj, 'Shape') and not obj.Shape.isNull():
        inventor_str = obj.Shape.writeInventor()
    elif (
        is_mesh(obj)
        and hasattr(obj, 'Mesh')
        and obj.Mesh.CountFacets > 0
    ):
        inventor_str = obj.Mesh.writeInventor()

    if not inventor_str:
        return sep

    if hasattr(obj, 'Placement'):
        sep.addChild(transform_from_placement(obj.Placement))

    coin_input = coin.SoInput()
    coin_input.setBuffer(
        inventor_str.encode() if isinstance(inventor_str, str) else inventor_str,
    )
    node = coin.SoDB.readAll(coin_input)
    if node is not None:
        sep.addChild(node)
    return sep


def _coin_separator_from_link(link: CrossLink) -> 'coin.SoSeparator':
    """Return an independent SoSeparator for the visual elements of link.

    Applies link.MountedPlacement as the first transform, then calls
    _coin_separator_from_object() for each element in link.Group. The result
    shares no scene graph nodes with the Robot's representation, so hiding the
    Robot (or any of its links) does not affect the returned separator.

    """
    from pivy import coin
    from .coin_utils import transform_from_placement
    from .freecad_utils import get_linked_obj

    sep = coin.SoSeparator()
    sep.addChild(transform_from_placement(link.MountedPlacement))
    for fc_link in link.Group:
        linked = get_linked_obj(fc_link, recursive=True)
        if linked is None:
            continue
        sep.addChild(_coin_separator_from_object(linked))
    return sep
