from __future__ import annotations

import FreeCAD as fc

import FreeCADGui as fcgui

from PySide import QtWidgets  # FreeCAD's PySide!
from PySide import QtCore  # FreeCAD's PySide!

from ..gui_utils import tr
from ..utils import values_from_string
from ..wb_utils import UI_PATH
from ..wb_utils import ros_name
from .set_joints_manual_input_table import SetJointsManualInputTable

DO = fc.DocumentObject  # A FreeCAD DocumentObject.
CrossRobot = DO  # A Cross::Robot, i.e. a DocumentObject with Proxy "Robot".
CrossJoint = DO  # A Cross::Joint, i.e. a DocumentObject with Proxy "Joint".


class SetJointsDialog:
    """A dialog to input DH and KK parameters.

    A dialog to set some of the joint values of the robot passed to the
    constuctor.

    """

    def __init__(self, robot: CrossRobot):
        """Constructor with a CROSS::Robot."""
        self.robot = robot

        self.form = fcgui.PySideUic.loadUi(str(UI_PATH / 'set_joints.ui'))
        self.dialog_confirmed = False
        self.table_manual_input = SetJointsManualInputTable(self.robot, self.form)
        self._set_up_gui()
        self._set_up_callback()

        # Joint values for the changed joints in meters and radians.
        self._joint_values: dict[CrossJoint, float] = {}

        # Deactivate the `tab_input_topic` tab until supported.
        # TODO: implement the `tab_input_topic` tab.
        self.form.tab_input_type.removeTab(1)

    @property
    def joint_values(self) -> dict[CrossJoint, float]:
        """Joint values for the changed joints in meters and radians."""
        return self._joint_values

    def _set_up_gui(self) -> None:
        """Set up the GUI."""
        # Remove the default focus on the OK button (and the Cancel button).
        self.form.button_box.setFocusPolicy(QtCore.Qt.NoFocus)
        # buttonBox->button(QDialogButtonBox::Save)->setDefault(false);
        # buttonBox->button(QDialogButtonBox::Cancel)->setDefault(false);
        self.form.table_container_widget.layout().addWidget(self.table_manual_input)
        self.table_manual_input.setToolTip(tr('Drag and drop to reorder'))

    def _set_up_callback(self) -> None:
        """Set up the callback to update the joint values."""
        self.form.button_box.accepted.connect(self._on_accept)
        self.form.button_box.rejected.connect(self._on_cancel)
        self.table_manual_input.itemChanged.connect(self._on_table_edited)
        self.form.line_edit_joint_values.textChanged.connect(
                self._on_line_edited) # DEBUG
        self.form.line_edit_joint_values.editingFinished.connect(
                self._on_line_edited)

        # TODO: Use only two dynamic buttons depending on the table content.
        self.form.push_button_xmm_to_xm.clicked.connect(
                lambda: self.table_manual_input.to_m_rad(False))
        self.form.push_button_xm_to_xmm.clicked.connect(
                lambda: self.table_manual_input.to_mm_deg(False))
        self.form.push_button_xmm_to_ym.clicked.connect(
                lambda: self.table_manual_input.to_m_rad(True))
        self.form.push_button_xm_to_ymm.clicked.connect(
                lambda: self.table_manual_input.to_mm_deg(True))

    def exec_(self) -> dict[CrossJoint, float]:
        self.form.exec_()
        if self.dialog_confirmed:
            return self.joint_values
        return {}

    def close(self) -> None:
        self.form.close()

    def _on_accept(self) -> None:
        self.dialog_confirmed = True
        self.form.close()

    def _on_cancel(self) -> None:
        self._joint_values.clear()
        self.dialog_confirmed = False
        self.form.close()

    def _on_line_edited(self) -> None:
        """Update the joint values from the line edit."""
        joint_name_column = 0
        value_column = 1
        unit_column = 2
        table = self.table_manual_input
        joint_values_str = self.form.line_edit_joint_values.text()
        joint_values = values_from_string(joint_values_str)
        joint_names = [item.text() for item in column_items(table, joint_name_column)]
        joints: list[CrossJoint] = self.robot.Proxy.joint_variables.keys()
        for i, (joint, value) in enumerate(zip(joints, joint_values)):
            joint_row = joint_names.index(ros_name(joint))
            unit = table.item(joint_row, unit_column).text()
            quantity = fc.Units.Quantity(f'{value} {unit}')
            if joint.Type == 'prismatic':
                # As of 2023-08-31 (0.21.1.33694) `Value` must be used as workaround
                # Cf. https://forum.freecad.org/viewtopic.php?t=82905.
                value = quantity.getValueAs('m').Value
            else:
                # joint.Type in ['revolute', 'continuous']
                # As of 2023-08-31 (0.21.1.33694) `Value` must be used as workaround
                # Cf. https://forum.freecad.org/viewtopic.php?t=82905.
                value = quantity.getValueAs('rad').Value
            # We set the joint value directly, without going through the
            # robot in order to avoid recomputing the robot at each joint value
            # change.
            # As of 2023-08-31 (0.21.1.33694) `Value` must be used as workaround
            # Cf. https://forum.freecad.org/viewtopic.php?t=82905.
            item = QtWidgets.QTableWidgetItem(f'{quantity.getValueAs(unit).Value}')
            table.setItem(i, value_column, item)

    def _on_table_edited(self) -> None:
        """Update the joint values from the table."""
        joint_name_column = 0
        value_column = 1
        unit_column = 2
        table = self.table_manual_input
        joint_names = [item.text() for item in column_items(table, joint_name_column)]
        joints: list[CrossJoint] = self.robot.Proxy.joint_variables.keys()
        for i, joint in enumerate(joints):
            joint_row = joint_names.index(ros_name(joint))
            unit = table.item(joint_row, unit_column).text()
            value = table.item(joint_row, value_column).text()
            quantity = fc.Units.Quantity(f'{value} {unit}')
            if joint.Type == 'prismatic':
                # As of 2023-08-31 (0.21.1.33694) `Value` must be used as workaround
                # Cf. https://forum.freecad.org/viewtopic.php?t=82905.
                value = quantity.getValueAs('m').Value
            else:
                # joint.Type in ['revolute', 'continuous']
                # As of 2023-08-31 (0.21.1.33694) `Value` must be used as workaround
                # Cf. https://forum.freecad.org/viewtopic.php?t=82905.
                value = quantity.getValueAs('rad').Value
            # We set the joint value directly, without going through the
            # robot in order to avoid recomputing the robot at each joint value
            # change.
            print(f'{ros_name(joint)}.Position = {value}')
            joint.Position = value
            self._joint_values.update({joint: value})
        # TODO: recompute has no effect.
        self.robot.Proxy.compute_poses()
        self.robot.Document.recompute()


def column_items(table: QtWidgets.QTableWidget,
                 column: int,
                 ) -> list[QtWidgets.QTableWidgetItem]:
    """Return the items in a column of a table."""
    row_count = table.rowCount()
    return [table.item(row, column) for row in range(row_count)]
