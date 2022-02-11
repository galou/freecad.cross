from PySide import QtCore, QtGui  # FreeCAD's PySide!

import FreeCAD as fc


def error_dialog(msg: str, title: str = 'FreeCAD'):
    """Create a simple dialog QMessageBox.

    Also display the error in FreeCAD's console.

    """
    fc.Console.PrintError(msg + '\n')
    diag = QtGui.QMessageBox(QtGui.QMessageBox.Warning, title, msg)
    diag.setWindowModality(QtCore.Qt.ApplicationModal)
    diag.exec_()
