from __future__ import annotations

from collections import OrderedDict

import FreeCAD as fc

from PySide import QtCore  # FreeCAD's PySide!
from PySide import QtGui  # FreeCAD's PySide!
from PySide import QtWidgets  # FreeCAD's PySide!

from ..freecad_utils import convert_units
from ..freecad_utils import unit_type
from ..wb_utils import ros_name

# Stubs and type hints.
from ..joint import Joint
from ..robot import Robot
CrossJoint = Joint
CrossRobot = Robot


class SetJointsManualInputTable(QtWidgets.QTableWidget):

    # Cache the joint order and units, so that we can restore them when
    # called a second time.
    _cache: OrderedDict[CrossJoint, str] = OrderedDict()

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
            name_item.setFlags(name_item.flags() & ~QtCore.Qt.ItemIsEditable)
            self.setItem(row, joint_name_column, name_item)
            unit_item = QtWidgets.QTableWidgetItem(unit)
            self.setItem(row, unit_column, unit_item)
        self._update_cache()

    def _initialize_table(self) -> None:
        """Fill up the table with the current robot state."""
        joints: list[CrossJoint] = self.robot.Proxy.get_actuated_joints()
        self.setRowCount(len(joints))
        self.setColumnCount(3)

        # Complete (or populate) the cache.
        all_joints = self.robot.Proxy.get_actuated_joints()
        for joint in all_joints:
            if joint not in self._cache:
                self._cache[joint] = _get_joint_unit(joint)

        # Populate the first column of the table with the joint names, the
        # second column with the joint values and the third column with the
        # joint units (mm and deg).
        for i, joint in enumerate(self._cache.keys()):
            name_item = QtWidgets.QTableWidgetItem(ros_name(joint))
            name_item.setFlags(name_item.flags() & ~QtCore.Qt.ItemIsEditable)
            self.setItem(i, 0, name_item)
            unit = self._cache[joint]
            value = _get_joint_value(joint, unit)
            # TODO: Show a rounded value but use the exact value as output and
            # propose the original value when starting to edit.
            value_item = QtWidgets.QTableWidgetItem(f'{value}')
            self.setItem(i, 1, value_item)
            unit_item = QtWidgets.QTableWidgetItem(unit)
            self.setItem(i, 2, unit_item)

    def to_m_rad(self, change_values: bool) -> None:
        """Convert the values in the table to meters and radians."""
        target_units = {
                'Length': 'm',
                'Angle': 'rad',
                }
        self._to_new_unit(change_values, target_units)

    def to_mm_deg(self, change_values: bool) -> None:
        """Convert the values in the table to millimeters and degrees."""
        target_units = {
                'Length': 'mm',
                'Angle': 'deg',
                }
        self._to_new_unit(change_values, target_units)

    def _to_new_unit(self, change_values: bool,
                     target_units: dict[str, str]) -> None:
        """Convert the values in the table to meters and radians.

        The keys of `target_units` must be in the set {'Length', 'Angle'}.

        """
        value_column = 1
        unit_column = 2
        for i in range(self.rowCount()):
            unit_item = self.item(i, unit_column)
            value_item = self.item(i, value_column)
            source_unit = unit_item.text()
            target_unit = target_units[unit_type(source_unit)]
            value = value_item.text()
            if change_values:
                value = convert_units(value, source_unit, target_unit)
            unit_item.setText(target_unit)
            value_item.setText(value)
        self._update_cache()

    def _update_cache(self) -> None:
        """Update the cache with the current joint units."""
        self._cache.clear()
        joint_name_column = 0
        unit_column = 2
        for i in range(self.rowCount()):
            name_item = self.item(i, joint_name_column)
            unit_item = self.item(i, unit_column)
            joint = self.robot.Proxy.get_joint(name_item.text())
            self._cache[joint] = unit_item.text()


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


def _get_joint_unit(joint: CrossJoint) -> str:
    """Get the joint unit in FreeCAD's default unit system."""
    if joint.Type == 'prismatic':
        return 'mm'
    elif joint.Type in ['revolute', 'continuous']:
        return 'deg'
    else:
        # Other types are not supported.
        raise NotImplementedError()


def _get_joint_value(joint: CrossJoint,
                     unit: str,
                     ) -> float:
    """Get the joint value in the specified unit."""
    if joint.Type == 'prismatic':
        # TODO: add a check for unit compatibility.
        # As of 2023-08-31 (0.21.1.33694) `Value` must be used as workaround
        # Cf. https://forum.freecad.org/viewtopic.php?t=82905.
        return fc.Units.Quantity(
                f'{joint.Position} m').getValueAs(unit).Value
    elif joint.Type in ['revolute', 'continuous']:
        # TODO: add a check for unit compatibility.
        # As of 2023-08-31 (0.21.1.33694) `Value` must be used as workaround
        # Cf. https://forum.freecad.org/viewtopic.php?t=82905.
        return fc.Units.Quantity(
                f'{joint.Position} rad').getValueAs(unit).Value
    else:
        # Other types are not supported.
        raise NotImplementedError()
