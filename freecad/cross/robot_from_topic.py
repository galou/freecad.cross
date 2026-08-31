"""Generate a CROSS::Robot from a robot description published on a ROS topic."""

from __future__ import annotations

from typing import Optional
from typing import TYPE_CHECKING

import FreeCAD as fc

from .freecad_utils import tr
from .freecad_utils import warn
from .robot_from_urdf import robot_from_urdf
from .ros.robot_description import DEFAULT_TIMEOUT
from .ros.robot_description import DEFAULT_TOPIC
from .ros.robot_description import get_robot_description

try:
    from .urdf_loader import UrdfLoader
    imports_ok = True
except ImportError as e:
    warn(str(e), gui=False)
    imports_ok = False

if TYPE_CHECKING:
    from .robot import Robot as CrossRobot  # A Cross::Robot. # noqa: E501

# Delay before the progress dialog appears, in seconds. A description that is
# already latched arrives faster than this, so the usual case shows no dialog.
_SHOW_PROGRESS_AFTER_S = 0.3


def robot_from_description_topic(
        topic: str = DEFAULT_TOPIC,
        timeout_sec: float = DEFAULT_TIMEOUT,
        doc: Optional[fc.Document] = None,
) -> Optional[CrossRobot]:
    """
    Create a CROSS::Robot from the robot description published on `topic`.

    Read the description once, close the subscriber and build the robot. The
    document is only created and modified once the description was received
    and successfully parsed, so that a failure leaves the session untouched.
    Return None on failure or if the user cancelled.

    Parameters
    ----------
    - topic: name of the `std_msgs/msg/String` topic to read from.
    - timeout_sec: deadline to get the message, in seconds.
    - doc: the document to create the robot in, the active document by
           default, a new one if there is none.

    """
    if not imports_ok:
        warn(tr('The URDF parser cannot be imported'), gui=True)
        return None

    description = _get_description_with_progress(topic, timeout_sec)
    if description is None:
        # Already reported, or cancelled by the user.
        return None
    if not description.strip():
        warn(tr(f'The description published on "{topic}" is empty'), gui=True)
        return None

    try:
        urdf_robot = UrdfLoader.load_from_string(description)
    except Exception as e:
        warn(tr(f'Cannot parse the robot description from "{topic}": {e}'), gui=True)
        return None

    if doc is None:
        doc = fc.activeDocument()
    if doc is None:
        doc = fc.newDocument()

    doc.openTransaction(tr('Robot from topic'))
    # `filename` is None: meshes must be given as `package://…` or as
    # absolute paths, relative mesh paths cannot be resolved.
    robot = robot_from_urdf(doc, urdf_robot)
    doc.commitTransaction()
    doc.recompute()
    if hasattr(fc, 'GuiUp') and fc.GuiUp:
        import FreeCADGui as fcgui
        fcgui.activeDocument().activeView().sendMessage('ViewFit')
    return robot


def _get_description_with_progress(
        topic: str,
        timeout_sec: float,
) -> Optional[str]:
    """
    Return the robot description, showing a cancellable progress dialog.

    Without GUI, wait without any dialog.

    """
    if not (hasattr(fc, 'GuiUp') and fc.GuiUp):
        return get_robot_description(topic, timeout_sec)

    import FreeCADGui as fcgui
    from PySide import QtCore  # FreeCAD's PySide!
    from PySide import QtGui  # FreeCAD's PySide!

    dialog = QtGui.QProgressDialog(
        tr(f'Waiting for a message on "{topic}"…'),
        tr('Cancel'),
        0,
        0,  # Same as the minimum: indeterminate progress.
        fcgui.getMainWindow(),
    )
    dialog.setWindowTitle(tr('FreeCAD - CROSS - Import from a Topic'))
    # Modal, so that processing the events below cannot let the user modify
    # the document while we wait.
    dialog.setWindowModality(QtCore.Qt.ApplicationModal)
    dialog.setMinimumDuration(int(_SHOW_PROGRESS_AFTER_S * 1000.0))
    dialog.setAutoClose(False)
    dialog.setAutoReset(False)

    def on_tick(elapsed: float) -> bool:
        if elapsed >= _SHOW_PROGRESS_AFTER_S:
            # `QProgressDialog` shows itself based on `setValue()` calls that
            # make no sense for an indeterminate bar, be explicit.
            dialog.show()
        QtGui.QApplication.processEvents()
        return not dialog.wasCanceled()

    try:
        return get_robot_description(topic, timeout_sec, on_tick=on_tick)
    finally:
        dialog.close()
        dialog.deleteLater()
