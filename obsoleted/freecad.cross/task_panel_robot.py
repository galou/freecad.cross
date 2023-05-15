import FreeCAD as fc

import FreeCADGui as fcgui

from PySide import QtCore  # FreeCAD's PySide!
from PySide import QtGui  # FreeCAD's PySide!

from .utils import UI_PATH

class TaskPanelRobot:
    """Dialog for the Task Panel to define a robot."""
    def __init__(self, robot: 'cross.robot.Robot'):
        self.robot = robot
        # The UI must be in self.form.
        self.form = fcgui.PySideUic.loadUi(str(UI_PATH.joinpath('task_panel_robot.ui')))

        self.form.add_button.clicked.connect(self.add_assembly)

    def accept(self):
        gui_doc = fcgui.getDocument(self.robot.Document)
        gui_doc.resetEdit()
        # #self.robot.Assembly = self.assembly
        #
        fc.getDocument(self.robot.Document.Name).recompute()
        return True

    def reject(self):
        self.form.close()
        fcgui.Selection.removeObserver(self)
        gui_doc = fcgui.getDocument(self.robot.Document)
        gui_doc.resetEdit()
        return True
    #
    # def clicked(self, index):
    #     pass
    #
    # def open(self):
    #     pass
    #
    # def needsFullSpace(self):
    #     return False
    #
    # def isAllowedAlterSelection(self):
    #     return True
    #
    # def isAllowedAlterView(self):
    #     return False
    #
    # def isAllowedAlterDocument(self):
    #     return True
    #
    def getStandardButtons(self):
        return int(QtGui.QDialogButtonBox.Cancel)
    #
    # def helpRequested(self):
    #     pass

    # def setupUi(self):
        # mw = self.getMainWindow()
        # form = mw.findChild(QtGui.QWidget, "TaskPanel")
        # form.pushButton = form.findChild(QtGui.QPushButton, "pushButton")
        # form.listWidget = form.findChild(QtGui.QListWidget, "listWidget")
        # self.form = form
        # #Connect Signals and Slots
        # QtCore.QObject.connect(form.pushButton, QtCore.SIGNAL("clicked()"), self.addElement)
        # pass

    def getMainWindow(self):
        "returns the main window"
        # using QtGui.QApplication.activeWindow() isn't very reliable because if another
        # widget than the mainwindow is active (e.g. a dialog) the wrong widget is
        # returned
        toplevel = QtGui.QApplication.topLevelWidgets()
        for i in toplevel:
            if i.metaObject().className() == "Gui::MainWindow":
                return i
        raise RuntimeError("No main window found")

    def add_assembly(self):
        pass
