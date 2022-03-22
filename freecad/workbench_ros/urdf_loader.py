#!/usr/bin/env python3
"""Generate a URDF robot from different sources."""

from pathlib import Path
from typing import Union

from urdf_parser_py.urdf import Robot
from urdf_parser_py.urdf import URDF


class UrdfLoader:
    def __init__(self):
        pass

    @classmethod
    def load_from_file(filename: Union[str, Path]) -> Robot:
        """Load from a URDF file."""
        if isinstance(filename, str):
            with open(filename, 'r') as f:
                robot = URDF.from_xml_string(f.read())
        elif isinstance(filename, Path):
            robot = URDF.from_xml_string(filename.read_bytes())
        return robot

    @classmethod
    def load_from_string(description: Union[str, bytes]) -> Robot:
        """Load from an xml string."""
        return URDF.from_xml_string(description)

    @classmethod
    def load_from_parameter_server(key: str = 'robot_description') -> Robot:
        """Load from ROS parameter server."""
        return URDF.from_parameter_server(key)
