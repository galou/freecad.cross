# Utilities to read trajectories from ROS 2 bags.
#
# This file is part of the CROSS workbench for FreeCAD.
#
# This module does not depend on FreeCAD.

from __future__ import annotations

from pathlib import Path
from typing import Any
from typing import Union

try:
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    _g_imports_ok = True
except ImportError:
    _g_imports_ok = False


DISPLAY_TRAJECTORY_TYPE = 'moveit_msgs/msg/DisplayTrajectory'
JOINT_TRAJECTORY_TYPE = 'trajectory_msgs/msg/JointTrajectory'
JOINT_STATE_TYPE = 'sensor_msgs/msg/JointState'

COMPATIBLE_TYPES = (
    DISPLAY_TRAJECTORY_TYPE,
    JOINT_TRAJECTORY_TYPE,
    JOINT_STATE_TYPE,
)


def find_bag(path: Union[Path, str]) -> tuple[Path, dict[str, Any]]:
    """
    Return the bag directory and the content of its `metadata.yaml`.

    `path` can be the bag directory, the `metadata.yaml` file (or any YAML
    file holding the metadata), or any other file in the bag directory (e.g.
    a mcap file).

    Raise `RuntimeError` if the metadata cannot be found or read.

    """
    import yaml

    path = Path(path)
    if path.is_dir():
        metadata_file = path / 'metadata.yaml'
    elif path.suffix in ('.yaml', '.yml'):
        metadata_file = path
    else:
        metadata_file = path.parent / 'metadata.yaml'
    if not metadata_file.is_file():
        raise RuntimeError(f'No bag metadata found at "{metadata_file}"')
    try:
        metadata = yaml.safe_load(metadata_file.read_text())
        info = metadata['rosbag2_bagfile_information']
    except (yaml.YAMLError, TypeError, KeyError) as e:
        raise RuntimeError(f'Invalid bag metadata in "{metadata_file}": {e}')
    return metadata_file.parent, info


def compatible_topics(metadata: dict[str, Any]) -> list[tuple[str, str, int]]:
    """Return (name, type, message_count) of topics with trajectories."""
    topics: list[tuple[str, str, int]] = []
    for t in metadata.get('topics_with_message_count', []):
        topic_metadata = t['topic_metadata']
        if topic_metadata['type'] in COMPATIBLE_TYPES:
            topics.append((
                topic_metadata['name'],
                topic_metadata['type'],
                t.get('message_count', 0),
            ))
    return topics


def read_messages(
        bag_dir: Union[Path, str],
        metadata: dict[str, Any],
        topics: list[str],
) -> dict[str, tuple[str, list[Any]]]:
    """
    Return {topic: (type, [message, ...])} for the given topics.

    Messages are in bag order.

    """
    if not _g_imports_ok:
        raise RuntimeError('ROS 2 packages (rosbag2_py, rclpy) not available')

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(
            uri=str(bag_dir),
            storage_id=metadata.get('storage_identifier', ''),
        ),
        rosbag2_py.ConverterOptions(
            input_serialization_format='cdr',
            output_serialization_format='cdr',
        ),
    )
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    reader.set_filter(rosbag2_py.StorageFilter(topics=topics))

    msg_classes = {t: get_message(types[t]) for t in topics if t in types}
    messages: dict[str, tuple[str, list[Any]]] = {
        t: (types[t], []) for t in msg_classes
    }
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic not in msg_classes:
            continue
        messages[topic][1].append(deserialize_message(data, msg_classes[topic]))
    return messages
