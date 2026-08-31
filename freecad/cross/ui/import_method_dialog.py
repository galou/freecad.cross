from __future__ import annotations

from typing import Optional

import FreeCADGui as fcgui

from ..gui_utils import tr


class ImportMethodDialog:
    """
    A dialog to choose the source of a robot description.

    """

    FILE = 'file'
    TOPIC = 'topic'

    def __init__(self, topic_available: bool = True):
        """
        Constructor.

        Parameters
        ----------
        - topic_available: False to disable the import from a topic, e.g.
                           when no ROS node could be created.

        """
        # Import late to avoid slowing down workbench start-up.
        from ..wb_utils import UI_PATH

        self.method: Optional[str] = None

        self.form = fcgui.PySideUic.loadUi(
                str(UI_PATH / 'import_method_dialog.ui'),
                fcgui.getMainWindow(),
        )

        self.form.radio_file.setChecked(True)
        if not topic_available:
            self.form.radio_topic.setEnabled(False)
            self.form.radio_topic.setToolTip(
                tr('No ROS node available, cannot subscribe to a topic'),
            )

        self.form.button_box.accepted.connect(self._on_accept)
        self.form.button_box.rejected.connect(self._on_cancel)

    def exec(self) -> Optional[str]:
        """Return the chosen method, None if the dialog was cancelled."""
        self.form.exec()
        return self.method

    def close(self) -> None:
        self.form.close()

    def _on_accept(self) -> None:
        if self.form.radio_topic.isChecked():
            self.method = self.TOPIC
        else:
            self.method = self.FILE
        self.form.close()

    def _on_cancel(self) -> None:
        self.method = None
        self.form.close()
