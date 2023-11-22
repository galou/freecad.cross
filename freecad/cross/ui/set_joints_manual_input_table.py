from __future__ import annotations

from math import degrees, radians

import FreeCAD as fc

from PySide import QtGui  # FreeCAD's PySide!
from PySide import QtWidgets  # FreeCAD's PySide!

from ..wb_utils import ros_name

DO = fc.DocumentObject  # A FreeCAD DocumentObject.
CrossRobot = DO  # A Cross::Robot, i.e. a DocumentObject with Proxy "Robot".
CrossJoint = DO  # A Cross::Joint, i.e. a DocumentObject with Proxy "Joint".


class SetJointsManualInputTable(QtWidgets.QTableWidget):
    def __init__(self, robot: CrossRobot, *args):
        super().__init__(*args)
        self.robot = robot

        self._activate_dnd()
        self._initialize_table()

    def _activate_dnd(self) -> None:
        """Activate drag and drop for the table."""
        self.setDragEnabled(True)
        self.setAcceptDrops(True)
        self.setDragDropMode(QtWidgets.QTableWidget.InternalMove)

    def dropEvent(self, event: QtGui.QDropEvent):
        joint_name_column = 0
        unit_column = 2
        dropped_item = self.selectedItems()[0]
        old_index = dropped_item.row()
        drop_row = self.indexAt(event.pos()).row()

        row_count = self.rowCount()
        texts = [self.item(row, joint_name_column).text() for row in range(row_count)]
        new_texts = dnd(texts, old_index, drop_row)
        units = [self.item(row, unit_column).text() for row in range(row_count)]
        new_units = dnd(units, old_index, drop_row)

        super().dropEvent(event)

        # Update the table with the new order, after super().dropEvent().
        for row, (text, unit) in enumerate(zip(new_texts, new_units)):
            name_item = QtWidgets.QTableWidgetItem(text)
            self.setItem(row, joint_name_column, name_item)
            unit_item = QtWidgets.QTableWidgetItem(unit)
            self.setItem(row, unit_column, unit_item)

    def _initialize_table(self) -> None:
        """Fill up the table with the current robot state."""
        joints: list[CrossJoint] = self.robot.Proxy.get_actuated_joints()
        self.setRowCount(len(joints))
        self.setColumnCount(3)

        # Populate the first column of the table with the joint names, the
        # second column with the joint values and the third column with the
        # joint units (mm and deg).
        for i, joint in enumerate(joints):
            name_item = QtWidgets.QTableWidgetItem(ros_name(joint))
            self.setItem(i, 0, name_item)
            if joint.Type == 'prismatic':
                unit = 'mm'
                value = joint.Position * 1000.0
            elif joint.Type in ['revolute', 'continuous']:
                unit = 'deg'
                value = degrees(joint.Position)
            else:
                # Other types are not supported.
                unit = ''
                value = ''
            value_item = QtWidgets.QTableWidgetItem(f'{value}')
            self.setItem(i, 1, value_item)
            unit_item = QtWidgets.QTableWidgetItem(unit)
            self.setItem(i, 2, unit_item)

    def to_m_rad(self, change_values: bool) -> None:
        """Convert the values in the table to meters and radians."""
        print(f'to_m_rad({change_values})') # DEBUG
        value_column = 1
        unit_column = 2
        for i in range(self.rowCount()):
            unit_item = self.item(i, unit_column)
            value_item = self.item(i, value_column)
            unit = unit_item.text()
            value = value_item.text()
            if unit == 'mm':
                if change_values:
                    value = f'{float(value) / 1000.0}'
                unit_item.setText('m')
                value_item.setText(value)
            elif unit == 'deg':
                if change_values:
                    value = f'{radians(float(value))}'
                unit_item.setText('rad')
                value_item.setText(value)

    def to_mm_deg(self, change_values: bool) -> None:
        """Convert the values in the table to millimeters and degrees."""
        print(f'to_mm_deg({change_values})') # DEBUG
        value_column = 1
        unit_column = 2
        for i in range(self.rowCount()):
            unit_item = self.item(i, unit_column)
            value_item = self.item(i, value_column)
            unit = unit_item.text()
            value = value_item.text()
            if unit == 'm':
                if change_values:
                    value = f'{float(value) * 1000.0}'
                unit_item.setText('mm')
                value_item.setText(value)
            elif unit == 'rad':
                if change_values:
                    value = f'{degrees(float(value))}'
                unit_item.setText('deg')
                value_item.setText(value)


def dnd(values, old_index, drop_index):
    """Reorder elements by drag and drop.

    "dnd" stands for "drag and drop".
    `drop_index` is the index where the element at `old_index`
    in `values` will be in the returned list.

    >>> dnd(['a', 'b', 'c', 'd', 'e'], 0, 2)
    ['b', 'c', 'a', 'd', 'e']
    >>> dnd(['a', 'b', 'c', 'd', 'e'], 1, 3)
    ['a', 'c', 'd', 'b', 'e']
    >>> dnd(['a', 'b', 'c', 'd', 'e'], 2, 4)
    ['a', 'b', 'd', 'e', 'c']
    >>> dnd(['a', 'b', 'c', 'd', 'e'], 4, 1)
    ['a', 'e', 'b', 'c', 'd']
    >>> dnd(['a', 'b', 'c', 'd', 'e'], 4, 0)
    ['e', 'a', 'b', 'c', 'd']
    >>> dnd(['a', 'b', 'c', 'd', 'e'], 2, 2)
    ['a', 'b', 'c', 'd', 'e']
    >>> dnd(['a', 'b', 'c', 'd', 'e'], 2, 5)
    ['a', 'b', 'c', 'd', 'e']
    >>> dnd(['a', 'b', 'c', 'd', 'e'], 5, 2)
    ['a', 'b', 'c', 'd', 'e']

    """

    if old_index == drop_index:
        return list(values)
    if (drop_index < 0) or (drop_index >= len(values)):
        return list(values)
    if (old_index < 0) or (old_index >= len(values)):
        return list(values)
    if old_index < drop_index:
        return (
                values[:old_index]
                + values[old_index + 1:drop_index + 1]
                + values[old_index: old_index + 1]
                + values[drop_index + 1:]
                )
    return (
            values[:drop_index]
            + values[old_index: old_index + 1]
            + values[drop_index: old_index]
            + values[old_index + 1:]
            )
