from __future__ import annotations

from contextlib import suppress
from functools import partial

import FreeCAD as fc
import FreeCADGui as fcgui

from PySide import QtCore  # FreeCAD's PySide!
from PySide import QtGui  # FreeCAD's PySide!

from ..gui_utils import tr
from ..ik import ik
from ..wb_utils import UI_PATH
from ..wb_utils import is_link
from ..wb_utils import joint_values_si_units_from_freecad as wb_si_from_fc
from ..wb_utils import ros_name

from ..joint import Joint as CrossJoint  # noqa: F401
from ..robot import Robot as CrossRobot  # noqa: F401


class MoveCartesianDialog(QtGui.QDialog):
    """A dialog to move a robot end-effector with Cartesian steps."""

    CUSTOM_STEP_TEXT = 'Custom'
    LINEAR_STEP_VALUES = [0.1, 1.0, 10.0, 100.0]
    ANGULAR_STEP_VALUES = [0.1, 1.0, 5.0, 10.0, 45.0, 90.0]
    AXES = {
        'X': fc.Vector(1.0, 0.0, 0.0),
        'Y': fc.Vector(0.0, 1.0, 0.0),
        'Z': fc.Vector(0.0, 0.0, 1.0),
        'A': fc.Vector(1.0, 0.0, 0.0),
        'B': fc.Vector(0.0, 1.0, 0.0),
        'C': fc.Vector(0.0, 0.0, 1.0),
    }
    SHORTCUTS = {
        'X': ('Ctrl+1', 'Ctrl+Shift+1'),
        'Y': ('Ctrl+2', 'Ctrl+Shift+2'),
        'Z': ('Ctrl+3', 'Ctrl+Shift+3'),
        'A': ('Ctrl+4', 'Ctrl+Shift+4'),
        'B': ('Ctrl+5', 'Ctrl+Shift+5'),
        'C': ('Ctrl+6', 'Ctrl+Shift+6'),
    }

    def __init__(self, robot: CrossRobot, *args):
        """Constructor with a CROSS::Robot."""
        super().__init__(args[0] if args else None)
        self.robot = robot
        self.form = fcgui.PySideUic.loadUi(
            str(UI_PATH / 'move_cartesian.ui'),
            self,
        )
        self._set_up_gui()
        self._set_up_callbacks()
        self._set_up_shortcuts()
        self._sync_linear_step()
        self._sync_angular_step()
        self._update_buttons_state()

    def exec_(self) -> int:
        return super().exec_()

    def close(self) -> None:
        super().close()

    def _set_up_gui(self) -> None:
        self.form.button_box.setFocusPolicy(QtCore.Qt.NoFocus)

        leaf_links = sorted(self._get_leaf_links())
        self.form.end_effector_combo_box.clear()
        self.form.end_effector_combo_box.addItems(leaf_links)

        self._populate_step_combo_box(
            self.form.linear_step_combo_box,
            self.LINEAR_STEP_VALUES,
            10.0,
        )

        self._populate_step_combo_box(
            self.form.angular_step_combo_box,
            self.ANGULAR_STEP_VALUES,
            10.0,
        )

        if not leaf_links:
            self._set_result_text(tr('No leaf link is available on the selected robot.'))
        else:
            self._set_result_text(tr('Choose a direction to move the selected end-effector.'))

    def _set_up_callbacks(self) -> None:
        self.form.button_box.rejected.connect(self._on_close)
        self.form.button_box.accepted.connect(self._on_close)
        self.form.end_effector_combo_box.currentIndexChanged.connect(
            self._update_buttons_state,
        )
        self.form.linear_step_combo_box.currentIndexChanged.connect(
            self._sync_linear_step,
        )
        self.form.angular_step_combo_box.currentIndexChanged.connect(
            self._sync_angular_step,
        )
        self.form.linear_step_spin_box.valueChanged.connect(
            self._on_linear_step_spin_changed,
        )
        self.form.angular_step_spin_box.valueChanged.connect(
            self._on_angular_step_spin_changed,
        )

        for axis in ('X', 'Y', 'Z', 'A', 'B', 'C'):
            minus_button = getattr(self.form, f'{axis.lower()}_minus_push_button')
            plus_button = getattr(self.form, f'{axis.lower()}_plus_push_button')
            minus_button.clicked.connect(partial(self._move, axis, -1.0))
            plus_button.clicked.connect(partial(self._move, axis, 1.0))

    def _set_up_shortcuts(self) -> None:
        self._shortcut_actions: list[QtGui.QAction] = []
        for axis, (negative, positive) in self.SHORTCUTS.items():
            for shortcut, direction in ((negative, -1.0), (positive, 1.0)):
                action = QtGui.QAction(self)
                action.setShortcut(QtGui.QKeySequence(shortcut))
                action.setShortcutContext(QtCore.Qt.WidgetWithChildrenShortcut)
                action.triggered.connect(partial(self._move, axis, direction))
                self.addAction(action)
                self._shortcut_actions.append(action)

    def _on_close(self) -> None:
        self.close()

    def _get_leaf_links(self) -> list[str]:
        leaf_links: set[str] = set()
        for chain in self.robot.Proxy.get_chains():
            if chain and is_link(chain[-1]):
                leaf_links.add(ros_name(chain[-1]))
        return list(leaf_links)

    def _sync_linear_step(self) -> None:
        self._sync_step_spin_box(
            self.form.linear_step_combo_box,
            self.form.linear_step_spin_box,
            self.LINEAR_STEP_VALUES,
        )

    def _sync_angular_step(self) -> None:
        self._sync_step_spin_box(
            self.form.angular_step_combo_box,
            self.form.angular_step_spin_box,
            self.ANGULAR_STEP_VALUES,
        )

    def _update_buttons_state(self) -> None:
        enabled = bool(self.form.end_effector_combo_box.currentText())
        for axis in ('X', 'Y', 'Z', 'A', 'B', 'C'):
            getattr(self.form, f'{axis.lower()}_minus_push_button').setEnabled(enabled)
            getattr(self.form, f'{axis.lower()}_plus_push_button').setEnabled(enabled)
        for action in self._shortcut_actions:
            action.setEnabled(enabled)

    def _populate_step_combo_box(
        self,
        combo_box: QtGui.QComboBox,
        values: list[float],
        default: float,
    ) -> None:
        combo_box.clear()
        for value in values:
            combo_box.addItem(f'{value:g}')
        combo_box.addItem(tr(self.CUSTOM_STEP_TEXT))
        combo_box.setCurrentIndex(values.index(default))

    def _sync_step_spin_box(
        self,
        combo_box: QtGui.QComboBox,
        spin_box: QtGui.QDoubleSpinBox,
        values: list[float],
    ) -> None:
        current_index = combo_box.currentIndex()
        if current_index < 0 or current_index >= len(values):
            return
        spin_box.setValue(values[current_index])

    def _on_linear_step_spin_changed(self, value: float) -> None:
        self._sync_step_combo_box(
            self.form.linear_step_combo_box,
            value,
            self.LINEAR_STEP_VALUES,
        )

    def _on_angular_step_spin_changed(self, value: float) -> None:
        self._sync_step_combo_box(
            self.form.angular_step_combo_box,
            value,
            self.ANGULAR_STEP_VALUES,
        )

    def _sync_step_combo_box(
        self,
        combo_box: QtGui.QComboBox,
        value: float,
        values: list[float],
    ) -> None:
        for i, preset in enumerate(values):
            if abs(value - preset) < 1e-9:
                if combo_box.currentIndex() != i:
                    combo_box.blockSignals(True)
                    combo_box.setCurrentIndex(i)
                    combo_box.blockSignals(False)
                return
        custom_index = len(values)
        if combo_box.currentIndex() != custom_index:
            combo_box.blockSignals(True)
            combo_box.setCurrentIndex(custom_index)
            combo_box.blockSignals(False)

    def _move(self, axis_name: str, direction: float) -> None:
        end_effector = self.form.end_effector_combo_box.currentText()
        link = self.robot.Proxy.get_link(end_effector)
        if not link:
            self._set_result_text(tr('The selected end-effector is not valid.'))
            return

        target = self._target_placement(link.Placement, axis_name, direction)
        solution = self._solve_ik(target, end_effector)
        if not solution:
            return

        chain_joints, joint_values = solution
        applied_joint_values_fc = {
            joint: value for joint, value in zip(chain_joints, joint_values)
        }
        previous_joint_quantities: dict[CrossJoint, fc.Units.Quantity] = dict(
            self.robot.Proxy.get_joint_values(),
        )
        first_sol_si_dict = wb_si_from_fc(applied_joint_values_fc)
        doc = self.robot.Document
        doc.openTransaction(tr('Move Cartesian step'))
        try:
            self.robot.Proxy.set_joint_values(first_sol_si_dict)
            doc.recompute()
        except Exception as exc:
            doc.abortTransaction()
            with suppress(Exception):
                self.robot.Proxy.set_joint_values(previous_joint_quantities)
                doc.recompute()
            self._set_result_text(str(exc))
            return
        doc.commitTransaction()
        self._set_result_text(
            tr('IK solution (mm/deg):') + '\n'
            + '\n'.join(
                f'{ros_name(joint)} = {value:g}{self._joint_unit_suffix(joint)}'
                for joint, value in applied_joint_values_fc.items()
            ),
        )

    def _target_placement(
        self,
        current: fc.Placement,
        axis_name: str,
        direction: float,
    ) -> fc.Placement:
        axis = self.AXES[axis_name]
        if axis_name in ('X', 'Y', 'Z'):
            step = self.form.linear_step_spin_box.value() * direction
            delta = self._selected_frame_rotation(current).multVec(axis) * step
            return fc.Placement(current.Base + delta, current.Rotation)

        step = self.form.angular_step_spin_box.value() * direction
        frame_rotation = self._selected_frame_rotation(current)
        target_rotation = (
            frame_rotation
            * fc.Rotation(axis, step)
            * frame_rotation.inverted()
            * current.Rotation
        )
        return fc.Placement(current.Base, target_rotation)

    def _selected_frame_rotation(self, current: fc.Placement) -> fc.Rotation:
        if self.form.global_radio_button.isChecked():
            return fc.Rotation()
        if self.form.robot_radio_button.isChecked():
            return self.robot.Placement.Rotation
        return current.Rotation

    def _solve_ik(
        self,
        target: fc.Placement,
        end_effector: str,
    ) -> tuple[list[CrossJoint], list[float]] | None:
        root_link = self.robot.Proxy.get_root_link()
        if not root_link:
            self._set_result_text(tr('The robot has no root link.'))
            return None
        chain_joints = self.robot.Proxy.get_actuated_joints_to(end_effector)
        if not chain_joints:
            self._set_result_text(tr('No actuated joint chain was found for the end-effector.'))
            return None

        try:
            sols = ik(
                robot=self.robot,
                from_link=ros_name(root_link),
                to_link=end_effector,
                target=target,
            )
        except Exception as exc:
            self._set_result_text(str(exc))
            return None
        if not sols:
            self._set_result_text(tr('IK failed to find a solution.'))
            return None
        return chain_joints, sols[0]

    def _joint_unit_suffix(self, joint: CrossJoint) -> str:
        if joint.Type == 'prismatic':
            return ' mm'
        return ' °'

    def _set_result_text(self, text: str) -> None:
        self.form.ik_result_plain_text_edit.setPlainText(text)
