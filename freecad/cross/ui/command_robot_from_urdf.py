
import FreeCAD as fc
import FreeCADGui as fcgui

from PySide import QtGui  # FreeCAD's PySide!

from ..freecad_utils import warn
from ..gui_utils import tr
from ..robot_from_urdf import robot_from_urdf
from ..ros.utils import is_ros_found
try:
    from ..urdf_loader import SUPPORTED_PARSERS
    from ..urdf_loader import UrdfLoader
    imports_ok = True
except ImportError as e:
    # TODO: Warn the user more nicely.
    warn(str(e), gui=False)
    imports_ok = False


class _UrdfImportCommand:
    def GetResources(self):
        return {
            'Pixmap': 'robot_from_urdf.svg',
            'MenuText': tr('Import a URDF or xacro file'),
            'ToolTip': tr('Import a URDF or xacro file'),
        }

    def Activated(self):
        doc = fc.activeDocument()
        dialog = QtGui.QFileDialog(
            fcgui.getMainWindow(),
            'Select URDF/xacro file to import part from',
        )
        # set option "DontUseNativeDialog"=True, as native Filedialog shows
        # misbehavior on Ubuntu 18.04. It works case sensitively, what is not wanted...
        dialog.setNameFilter('Supported Formats *.urdf *.xacro;;All files (*.*)')
        if dialog.exec_():
            if not doc:
                doc = fc.newDocument()
            filename = str(dialog.selectedFiles()[0])
            parser = _choose_parsing_library()
            if parser is None:
                return
            urdf_robot = UrdfLoader.load_from_file(filename, parser=parser)
            doc.openTransaction(tr('Robot from URDF'))
            robot_from_urdf(doc, urdf_robot)
            doc.commitTransaction()
            doc.recompute()
            fcgui.SendMsgToActiveView('ViewFit')

    def IsActive(self):
        return is_ros_found() and imports_ok


def _choose_parsing_library() -> str | None:
    available = UrdfLoader.available_parsers()
    dialog = QtGui.QDialog(fcgui.getMainWindow())
    dialog.setWindowTitle(tr('Choose URDF parser'))

    layout = QtGui.QVBoxLayout(dialog)
    layout.addWidget(QtGui.QLabel(tr('Select the URDF parsing library:')))

    button_group = QtGui.QButtonGroup(dialog)
    parser_buttons: dict[str, QtGui.QRadioButton] = {}
    for parser in SUPPORTED_PARSERS:
        button = QtGui.QRadioButton(UrdfLoader.parser_display_name(parser))
        button.setToolTip(UrdfLoader.parser_install_help(parser))
        button.setEnabled(available.get(parser, False))
        layout.addWidget(button)
        button_group.addButton(button)
        parser_buttons[parser] = button

    default_parser = UrdfLoader.default_parser()
    if default_parser in parser_buttons and parser_buttons[default_parser].isEnabled():
        parser_buttons[default_parser].setChecked(True)
    else:
        for parser in SUPPORTED_PARSERS:
            if parser_buttons[parser].isEnabled():
                parser_buttons[parser].setChecked(True)
                break

    buttons = QtGui.QDialogButtonBox(
        QtGui.QDialogButtonBox.Ok | QtGui.QDialogButtonBox.Cancel,
    )
    buttons.accepted.connect(dialog.accept)
    buttons.rejected.connect(dialog.reject)
    layout.addWidget(buttons)

    if not any(available.values()):
        buttons.button(QtGui.QDialogButtonBox.Ok).setEnabled(False)
        warn(
            tr(
                'No URDF parser found. Install yourdfpy with'
                ' "pip install yourdfpy" or urdf_parser_py with'
                ' "pip install urdf-parser-py".',
            ),
            gui=True,
        )

    if not dialog.exec_():
        return None

    for parser, button in parser_buttons.items():
        if button.isChecked():
            return parser
    return None


fcgui.addCommand('UrdfImport', _UrdfImportCommand())
