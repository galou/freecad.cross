# Code Snippets
There is no way to add a button, menu entry from python to a workbench which is added with c++. So here is a comparison how to do that with python and with c++.

## Adding a command:
This can be done either with python or c++.

### 1. python

```python
import FreeCAD as App

class MyCommand(object):
    def IsActive(self):
        """
        availability of the command (eg.: check for existence of a document,...)
        if this function returns False, the menu/ buttons are ßdisabled (gray)
        """
        if App.ActiveDocument is None:
            return False
        else:
            return True

    def GetResources(self):
        """
        resources which are used by buttons and menu-items
        """
        return {'Pixmap': 'path_to_icon.svg', 'MenuText': 'my command', 'ToolTip': 'very short description'}

    def Activated(self):
        """
        the function to be handled, when a user starts the command
        """
```
To register the command in FreeCAD:

```python
import FreeCADGui as Gui
Gui.addCommand('MyCommand', MyCommand())
```

Adding a new toolbar/menu:
```python
from FreeCADGui import Workbench
class myWorkbench(Workbench):
    MenuText = "name_of_workbench"
    ToolTip = "short description of workbench"
    Icon = "path_to_icon.svg"

    def GetClassName(self):
        return "Gui::PythonWorkbench"

    def Initialize(self):
        self.appendToolbar("Gear", ["MyCommand"])
        self.appendMenu("Gear", ["MyCommand"])
```

### 2. C++

```c++

#include <App/Document.h>
#include <Gui/Command.h>
#include <Gui/Control.h>
#include <Gui/Document.h>

using namespace std;

DEF_STD_CMD_A(MyCommand)

MyCommand::MyCommand()
  : Command("MyCommand")
{
    sAppModule    = "module";
    sGroup        = QT_TR_NOOP("Mesh");
    sMenuText     = QT_TR_NOOP("my command");
    sToolTipText  = QT_TR_NOOP("very short description");
    sWhatsThis    = "MyCommand";
    sStatusTip    = sToolTipText;
}

void MyCommand::activated(int)
{
    // the function to be handled, when a user starts the command
}

bool MyCommand::isActive(void)
{
    // availability of the command (eg.: check for existence of a document,...)
    // if this function returns False, the menu/ buttons are ßdisabled (gray)
    return (hasActiveDocument() && !Gui::Control().activeDialog());
}
```
To register the command in FreeCAD:

```c++
#include <Gui/Command.h>

Gui::CommandManager &rcCmdMgr = Gui::Application::Instance->commandManager();
rcCmdMgr.addCommand(new MyCommand());
```
Adding a item to a menu/toolbar:

if your command is added with python you have to run this code:
in src/module/Gui/AppModuleGui.cpp add to PyMOD_INIT_FUNC:

```c++
// try to instantiate a python module
try{
    Base::Interpreter().runStringObject("import MyCommands");
} catch (Base::PyException &err){
    err.ReportException();
}
```

and add the name of the command to a tooltip/menu in src/module/Gui/Workbench.cpp Workbench::setupToolBars


```c++
Gui::ToolBarItem* Workbench::setupToolBars() const
{
    Gui::ToolBarItem* root = StdWorkbench::setupToolBars();
    Gui::ToolBarItem* myToolbar = new Gui::ToolBarItem(root);
    myToolbar->setCommand("my_commands");
    *myToolbar << "MyCommand";
    return root;
}
```
