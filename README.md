# ROS Workbench

This is a FreeCAD workbench to generate robot description packages (xacro or URDF) for the Robot Operating System, [ROS].

## Compatibility

Compatible with FreeCAD v0.19 (or earlier version with the local coordinate system feature) but tested on FreeCAD v0.21.
Compatible with ROS2 (for now).

## Features

- Export `Part::Box`, `Part::Sphere`, and `Part::Cylinder` as text to be included in a URDF file,
- Generate an enclosing box or sphere as collision object (only axis-aligned box for now),
- Build a robot from scratch and generate the URDF file for it,
- Set a value for each actuated joint of a robot and have the links move accordingly,
- Import URDF/xacro files,
- Import xacro files that only define some macros and ask the user to choose a macro and its parameters to generate a full-feature URDF,
- Export the xacro as xacro that includes (`xacro:include`) the original xacro file and uses the macro.
- Combine several xacros files (i.e. also URDF) into a workcell and export them as a xacro file.

## Installation

You need a recent version of FreeCAD v0.21 with the ability to configure custom repositories for the Addon Manager to install the workbench via the Addon Manager. On earlier version you're on your own, see instructions for local install below.

- In FreeCAD, menu "Edit / Preferences ..."
- Category "Addon Manager"
- Add an entry to "Custom repository" by clicking on the "+" sign.
- Repository URL: `https://github.com/galou/freecad.workbench_ros.git`, branch: `main`
- Click on "OK" to close the dialog "Preferences"
- Back to FreeCAD's main window, menu "Tools / Addon manager"
- Install the workbench via the [Addon Manager](https://wiki.freecad.org/Std_AddonMgr)


## Testing/developing the workbench

If you want to work on this workbench you have the following options:

- Clone the repository directory in FreeCAD's `Mod` directory: `cd ~/.local/share/FreeCAD/Mod && git clone https://github.com/galou/freecad.workbench_ros.git` on Linux
- Start FreeCAD from the root-directory of this repository in (by default `freecad.workbench_ros`)
- Clone this repository and create a symbolic link to the directory `freecad.workbench_ros` (or the directory containing this repository if you changed the name) to FreeCAD's `Mod` directory (`~/.local/share/FreeCAD/Mod` on Linux).
- `pip install -e .` adds the root-directory to `easy_install.path`.


### Install locally

To install your extension locally with pip, do the following from a cmd (windows) or terminal (unix):

```bash
cd <path_to_the_root_of_this_repo>
pip install .
```

### Uploading your package to pypi

Please have a look at this [pypi twine tutorial][twine].

Be careful with version-numbering. It seems pypi doesn't allow to upload a package with a version smaller than the biggest version of the package uploaded. This seems to be true also for deleted packages and deleted versions.

Once uploaded, the package can be installed with:

```bash
pip install <package-name>
```

## Additional Information

### Glossary

- **_module_** : a Python source file, which can expose classes, functions and global variables
- **_package_** : a directory containing Python modules.
- **_distribution_** : the artifacts which are created by running the setup.py. Can contain multiple packages.
- **_workbench_** : a _graphical space_ inside the FreeCAD-Gui which adds functionality related to a specific task
- **_namespace-package_** : a package which adds functionality to a specific namespace. For FreeCAD we are talking about packages which are importable with `from freecad import my_package`. (Sometimes also called new-style-module)
- **_namespace-workbench_**: a **_namespace-package_** containing the freecad-initialization files.
- **_extension-module_**: a library (`.so` or `.dll`) written in C/C++ which adds the possibility to import this library with python.

### Tip

Due to the fact we are now using the `pktutil-module` to find extensions of FreeCAD, it's possible to use standard-python-paths to place the extension. This is any location which is included in the `sys.path`.

To get a list of all the locations simple run this code in the FreeCAD-console:

```python
import sys
sys.path
```

--------------------------------------------------------------------------------

[ROS]: https://www.ros.org/
