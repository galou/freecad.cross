# ROS Workbench

This is an early-stage FreeCAD workbench to deal with [ROS] (Robot Operating System).

## Compatibility

Compatible with FreeCAD v0.19 (earlier version with the local coordinate system feature).
Compatible with ROS2 (for now).

## Features

- Export `Part::Box`, `Part::Sphere`, and `Part::Cylinder` as text to be included in a URDF file.

## Installation

You're on your own for now, see instructions for local install below.


## Testing/developing the workbench

If you want to work on this workbench you have the following options:

- Start FreeCAD from the root-directory of this repository in (by default `freecad.workbench_ros`)
- Simply link the extension to a location where python can find it.
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

### Projects using this structure

- [pyrate][pyrate] - Optical raytracing based on Python
- [OpenGlider][OpenGlider] - Python library to build paragliders
- [FCGear][FCGear] - a gear module for FreeCAD
- [freecad_pipintegration][FC_pipintegration] - support pip installable freecad-packages

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
[twine]: https://pypi.python.org/pypi/twine
[pyrate]: https://github.com/mess42/pyrate
[OpenGlider]: https://github.com/booya-at/OpenGlider
[FCGear]: https://github.com/looooo/FCGear
[FC_pipintegration]: https://github.com/looooo/freecad_pipintegration
