from __future__ import annotations

from typing import Optional

from PySide import QtCore  # FreeCAD's PySide!
from PySide import QtWidgets  # FreeCAD's PySide!


class ChooseTopicsDialog(QtWidgets.QDialog):
    """A dialog to choose topics from a ROS bag."""

    def __init__(
            self,
            topics: list[tuple[str, str, int]],
            parent: Optional[QtWidgets.QWidget] = None,
    ):
        """Constructor from a list of (name, type, message_count)."""
        super().__init__(parent)
        self.setWindowTitle('Choose topics to load trajectories from')

        self._list = QtWidgets.QListWidget(self)
        for name, type_, count in topics:
            item = QtWidgets.QListWidgetItem(f'{name} [{type_}] ({count} msgs)')
            item.setData(QtCore.Qt.UserRole, name)
            item.setFlags(item.flags() | QtCore.Qt.ItemIsUserCheckable)
            item.setCheckState(QtCore.Qt.Checked)
            self._list.addItem(item)

        button_box = QtWidgets.QDialogButtonBox(
                QtWidgets.QDialogButtonBox.Ok | QtWidgets.QDialogButtonBox.Cancel,
                parent=self,
        )
        button_box.accepted.connect(self.accept)
        button_box.rejected.connect(self.reject)

        layout = QtWidgets.QVBoxLayout(self)
        layout.addWidget(self._list)
        layout.addWidget(button_box)

    def selected_topics(self) -> list[str]:
        """Show the dialog, return the checked topics, empty if cancelled."""
        if not self.exec_():
            return []
        return [
                self._list.item(i).data(QtCore.Qt.UserRole)
                for i in range(self._list.count())
                if self._list.item(i).checkState() == QtCore.Qt.Checked
        ]
