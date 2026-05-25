"""Generate a URDF robot from different sources."""

from __future__ import annotations

from pathlib import Path
from typing import Any
import warnings

from xacro import process_file


YOURDFPY = 'yourdfpy'
URDF_PARSER_PY = 'urdf_parser_py'
SUPPORTED_PARSERS = (YOURDFPY, URDF_PARSER_PY)
# Prefer yourdfpy because it is more robust on real-world URDF inputs and is
# the parser backend this workbench now prioritizes for import workflows.
DEFAULT_PARSER = YOURDFPY


class ParsedUrdfRobot:
    """Normalized URDF robot interface for parser backends."""

    def __init__(self, robot: Any):
        self._robot = robot

    def __getattr__(self, name: str) -> Any:
        return getattr(self._robot, name)

    @property
    def name(self) -> str:
        return self._robot.name

    @property
    def links(self) -> list[Any]:
        if hasattr(self._robot, 'links'):
            return self._robot.links
        if hasattr(self._robot, 'robot'):
            return self._robot.robot.links
        return []

    @property
    def joints(self) -> list[Any]:
        if hasattr(self._robot, 'joints'):
            return self._robot.joints
        if hasattr(self._robot, 'robot'):
            return self._robot.robot.joints
        return []

    @property
    def materials(self) -> list[Any]:
        if hasattr(self._robot, 'materials'):
            return self._robot.materials
        if hasattr(self._robot, 'robot'):
            return self._robot.robot.materials
        return []

    @property
    def link_map(self) -> dict[str, Any]:
        if hasattr(self._robot, 'link_map'):
            return self._robot.link_map
        return {link.name: link for link in self.links}

    @property
    def joint_map(self) -> dict[str, Any]:
        if hasattr(self._robot, 'joint_map'):
            return self._robot.joint_map
        return {joint.name: joint for joint in self.joints}

    def get_root(self) -> str | None:
        if hasattr(self._robot, 'get_root'):
            return self._robot.get_root()
        if hasattr(self._robot, 'base_link'):
            return self._robot.base_link
        root_candidates = {link.name for link in self.links}
        for joint in self.joints:
            if hasattr(joint, 'child'):
                root_candidates.discard(joint.child)
        if not root_candidates:
            return None
        # Deterministic fallback if the parser backend does not provide root link
        # lookup.
        root_candidates = sorted(root_candidates)
        if len(root_candidates) > 1:
            warnings.warn(
                f'URDF has multiple root-link candidates, selecting '
                f'"{root_candidates[0]}"',
            )
        return root_candidates[0]


class UrdfLoader:
    """Generate a URDF robot from different sources."""

    def __init__(self):
        pass

    @classmethod
    def available_parsers(cls) -> dict[str, bool]:
        return {
            YOURDFPY: cls._is_parser_available(YOURDFPY),
            URDF_PARSER_PY: cls._is_parser_available(URDF_PARSER_PY),
        }

    @classmethod
    def parser_install_help(cls, parser: str) -> str:
        if parser == YOURDFPY:
            return 'Install with: pip install yourdfpy'
        if parser == URDF_PARSER_PY:
            return (
                'Install with: pip install urdf-parser-py'
                ' or apt install ros-$ROS_DISTRO-urdf-parser-py'
            )
        return ''

    @classmethod
    def parser_display_name(cls, parser: str) -> str:
        if parser == YOURDFPY:
            return 'yourdfpy (recommended)'
        if parser == URDF_PARSER_PY:
            return 'urdf_parser_py'
        return parser

    @classmethod
    def default_parser(cls) -> str:
        if cls._is_parser_available(DEFAULT_PARSER):
            return DEFAULT_PARSER
        if cls._is_parser_available(URDF_PARSER_PY):
            return URDF_PARSER_PY
        return DEFAULT_PARSER

    @classmethod
    def load_from_file(
            cls,
            filename: [str | Path],
            parser: str | None = None,
    ) -> ParsedUrdfRobot:
        """Load from a URDF file."""
        filename = Path(filename)
        parser = cls._resolve_parser(parser)
        if parser == YOURDFPY:
            from yourdfpy import URDF
            if filename.suffix == '.xacro':
                robot = URDF.load(
                    process_file(filename.expanduser()).toxml(),
                    build_scene_graph=False,
                    load_meshes=False,
                )
            else:
                robot = URDF.load(
                    filename,
                    build_scene_graph=False,
                    load_meshes=False,
                )
            return ParsedUrdfRobot(robot)

        from urdf_parser_py.urdf import Robot
        if filename.suffix == '.xacro':
            robot = Robot.from_xml_string(
                process_file(filename.expanduser()).toxml(),
            )
        else:
            robot = Robot.from_xml_file(filename)
        return ParsedUrdfRobot(robot)

    @classmethod
    def load_from_string(
            cls,
            description: [str | bytes],
            parser: str | None = None,
    ) -> ParsedUrdfRobot:
        """Load from an xml string."""
        parser = cls._resolve_parser(parser)
        if parser == YOURDFPY:
            from yourdfpy import URDF
            robot = URDF.load(
                description,
                build_scene_graph=False,
                load_meshes=False,
            )
            return ParsedUrdfRobot(robot)

        from urdf_parser_py.urdf import Robot
        return ParsedUrdfRobot(Robot.from_xml_string(description))

    @classmethod
    def load_from_parameter_server(
        cls,
        key: str = 'robot_description',
        parser: str | None = None,
    ) -> ParsedUrdfRobot:
        """Load from ROS parameter server."""
        parser = cls._resolve_parser(parser)
        if parser == URDF_PARSER_PY:
            from urdf_parser_py.urdf import Robot
            return ParsedUrdfRobot(Robot.from_parameter_server(key))
        raise NotImplementedError(
            'Loading from ROS parameter server is only supported with'
            ' urdf_parser_py',
        )

    @classmethod
    def _resolve_parser(cls, parser: str | None = None) -> str:
        parser = parser or cls.default_parser()
        if parser not in SUPPORTED_PARSERS:
            raise ValueError(f'Unsupported parser "{parser}"')
        if not cls._is_parser_available(parser):
            raise ImportError(
                f'Parser "{parser}" is not available.'
                f' {cls.parser_install_help(parser)}',
            )
        return parser

    @classmethod
    def _is_parser_available(cls, parser: str) -> bool:
        if parser == YOURDFPY:
            try:
                import yourdfpy  # noqa: F401
                return True
            except ImportError:
                return False
        if parser == URDF_PARSER_PY:
            try:
                import urdf_parser_py  # noqa: F401
                return True
            except ImportError:
                return False
        return False
