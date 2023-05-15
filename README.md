# CROSS - CAD and ROS, Open-Source Synergy

CROSS is a FreeCAD workbench to generate robot description packages (xacro or URDF) for the Robot Operating System, [ROS].

## Compatibility

Compatible with FreeCAD v0.19 (or earlier version with the local coordinate system feature) but tested on FreeCAD v0.21.
Compatible with ROS2 (for now).

## Features

- Export `Part::Box`, `Part::Sphere`, and `Part::Cylinder` as text to be included in a URDF file,
- Generate an enclosing box or sphere as collision object (only axis-aligned box for now),
- Build a robot from scratch and generate the URDF file for it,
- Set a value for each actuated joint of a robot and have the links move accordingly,
- Import URDF/xacro files,
- Import xacro definitions, i.e. import xacro files that only define some macros and ask the user to choose a macro and its parameters to generate a full-feature URDF,
- Export the xacro as xacro that includes (`xacro:include`) the original xacro file and uses the macro.
- Combine several xacros files (i.e. also URDF) into a workcell and export them as a xacro file.

## Installation

You need a recent version of FreeCAD v0.21 with the ability to configure custom repositories for the Addon Manager to install the workbench via the Addon Manager. On earlier version you're on your own, see instructions for local install below.

- In FreeCAD, menu "Edit / Preferences ..."
- Category "Addon Manager"
- Add an entry to "Custom repository" by clicking on the "+" sign.
- Repository URL: `https://github.com/galou/freecad.cross.git`, branch: `main`
- Click on "OK" to close the dialog "Preferences"
- Back to FreeCAD's main window, menu "Tools / Addon manager"
- Search and install the workbench via the [Addon Manager](https://wiki.freecad.org/Std_AddonMgr)


## Testing/developing the workbench

If you want to work on this workbench you have the following options:

- Clone the repository directory in FreeCAD's `Mod` directory: `cd ~/.local/share/FreeCAD/Mod && git clone https://github.com/galou/freecad.cross.git` on Linux
- Start FreeCAD from the root-directory of this repository in (by default `freecad.cross`)
- Clone this repository and create a symbolic link to the directory `freecad.cross` (or the directory containing this repository if you changed the name) to FreeCAD's `Mod` directory (`~/.local/share/FreeCAD/Mod` on Linux).
- `pip install -e .` adds the root-directory to `easy_install.path`.


--------------------------------------------------------------------------------

[ROS]: https://www.ros.org/
