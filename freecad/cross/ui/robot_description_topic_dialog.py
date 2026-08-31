from __future__ import annotations

from typing import List, Optional

import FreeCADGui as fcgui

from .. import wb_globals
from ..ros.robot_description import DEFAULT_TOPIC
from ..ros.robot_description import normalize_topic_name

# Type of the messages carrying a robot description.
_DESCRIPTION_MSG_TYPE = 'std_msgs/msg/String'


class RobotDescriptionTopicDialog:
    """
    A dialog to confirm the topic to read the robot description from.

    """

    def __init__(self, default_topic: str = DEFAULT_TOPIC):
        """Constructor with the topic proposed by default."""
        # Import late to avoid slowing down workbench start-up.
        from ..wb_utils import UI_PATH

        self.topic: Optional[str] = None

        self.form = fcgui.PySideUic.loadUi(
                str(UI_PATH / 'robot_description_topic_dialog.ui'),
                fcgui.getMainWindow(),
        )

        self.form.combo_topic.addItems(_candidate_topics(default_topic))
        self.form.combo_topic.setEditText(default_topic)

        self.form.button_box.accepted.connect(self._on_accept)
        self.form.button_box.rejected.connect(self._on_cancel)

    def exec(self) -> Optional[str]:
        """Return the chosen topic, None if the dialog was cancelled."""
        self.form.exec()
        return self.topic

    def close(self) -> None:
        self.form.close()

    def _on_accept(self) -> None:
        topic = normalize_topic_name(self.form.combo_topic.currentText())
        self.topic = topic if topic else None
        self.form.close()

    def _on_cancel(self) -> None:
        self.topic = None
        self.form.close()


def _candidate_topics(default_topic: str) -> List[str]:
    """
    Return the topics a robot description may be published on.

    Return the topics of type `std_msgs/msg/String`, `default_topic` first,
    even if it is not advertised (yet).

    """
    node = wb_globals.g_ros_node
    if node is None:
        return [default_topic]
    try:
        names_and_types = node.get_topic_names_and_types()
    except Exception:
        return [default_topic]
    topics = sorted(
        name
        for name, types in names_and_types
        if _DESCRIPTION_MSG_TYPE in types
    )
    if default_topic in topics:
        topics.remove(default_topic)
    return [default_topic] + topics
