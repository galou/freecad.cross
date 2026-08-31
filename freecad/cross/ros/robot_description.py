"""Read a robot description (URDF) from a ROS topic."""

from __future__ import annotations

import time
from typing import Callable, Optional

try:
    from rclpy.exceptions import InvalidTopicNameException
    from rclpy.qos import DurabilityPolicy
    from rclpy.qos import HistoryPolicy
    from rclpy.qos import QoSProfile
    from rclpy.qos import ReliabilityPolicy
    from rclpy.wait_for_message import wait_for_message
    from std_msgs.msg import String
    imports_ok = True
except ImportError:
    imports_ok = False

from .. import wb_globals
from ..freecad_utils import tr
from ..freecad_utils import warn

# The topic `robot_state_publisher` publishes the URDF on.
DEFAULT_TOPIC = '/robot_description'

# Default deadline to get the description, in seconds.
DEFAULT_TIMEOUT = 5.0

# Duration of a single call to `wait_for_message`, in seconds. Short enough
# to keep a `Cancel` click responsive, long enough not to churn through too
# many subscription create/destroy cycles.
_ATTEMPT_DURATION = 0.2

# A callback called once per attempt with the elapsed time in seconds and
# returning False to abort the wait.
TickCallback = Callable[[float], bool]


def normalize_topic_name(topic: str) -> str:
    """Return a topic name without surrounding blanks and with a leading '/'."""
    topic = topic.strip()
    if topic and (not topic.startswith('/')):
        topic = f'/{topic}'
    return topic


def get_robot_description(
        topic: str = DEFAULT_TOPIC,
        timeout_sec: float = DEFAULT_TIMEOUT,
        on_tick: Optional[TickCallback] = None,
) -> Optional[str]:
    """
    Return the robot description published on `topic`, read once.

    Subscribe to `topic` with the QoS profile used by `robot_state_publisher`
    (transient local, i.e. latched), return the first message received and
    close the subscription. Return None if no message could be read.

    Parameters
    ----------
    - topic: name of the `std_msgs/msg/String` topic to read from.
    - timeout_sec: deadline to get the message, in seconds.
    - on_tick: called once per attempt with the elapsed time in seconds,
               returning False to abort the wait. Typically used to keep a
               progress dialog alive; None when running without GUI.

    """
    if not imports_ok:
        warn(tr('ROS modules cannot be imported'), gui=True)
        return None

    node = wb_globals.g_ros_node
    if node is None:
        warn(tr('No ROS node available, cannot read the robot description'), gui=True)
        return None

    topic = normalize_topic_name(topic)
    if not topic:
        warn(tr('No topic given'), gui=True)
        return None

    qos_profile = _robot_description_qos()
    start_time = time.monotonic()
    while True:
        elapsed = time.monotonic() - start_time
        if (on_tick is not None) and (not on_tick(elapsed)):
            # Cancelled by the caller, no warning, this was deliberate.
            return None
        try:
            found, msg = wait_for_message(
                String,
                node,
                topic,
                qos_profile=qos_profile,
                time_to_wait=_ATTEMPT_DURATION,
            )
        except (InvalidTopicNameException, ValueError) as e:
            warn(tr(f'Cannot subscribe to "{topic}": {e}'), gui=True)
            return None
        if found:
            return msg.data
        if (time.monotonic() - start_time) >= timeout_sec:
            break

    _warn_no_message(topic, timeout_sec)
    return None


def _robot_description_qos() -> QoSProfile:
    """
    Return the QoS profile of the `/robot_description` topic.

    `robot_state_publisher` latches the description, a subscriber must
    therefore be transient local as well to receive it. All fields are set
    explicitly because the defaults of `QoSProfile` are not guaranteed to be
    stable across ROS distributions.

    """
    return QoSProfile(
        depth=1,
        history=HistoryPolicy.KEEP_LAST,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


def _warn_no_message(topic: str, timeout_sec: float) -> None:
    """Warn about a failed read, distinguishing the two likely causes."""
    node = wb_globals.g_ros_node
    try:
        publisher_count = node.count_publishers(topic)
    except Exception:
        publisher_count = 0
    if publisher_count == 0:
        warn(tr(f'No publisher found on "{topic}"'), gui=True)
    else:
        warn(
            tr(
                f'A publisher was found on "{topic}" but no message was'
                f' received within {timeout_sec} s, is it publishing with'
                ' the "transient local" durability?',
            ),
            gui=True,
        )
