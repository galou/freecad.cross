# ROS Workbench

This is an early-stage FreeCAD workbench to deal with ROS (Robot Operating System).

## Compatibility

Compatible with FreeCAD v0.19 (earlier version with the local coordinate system feature). Compatible with ROS2 (for now).


## Features

None yet.

## Installation

You're on your own for now.


## Testing your module/workbench

If you want to work on your extension you have the following options:

- Start FreeCAD from the root-directory you are working in (eg. freecad.workbench_ros)
- Simply link the extension to a location where python can find it.
- `pip install -e .` adds the root-directory to easy_install.path.

## Using pip (setuptool or distutlis)

Currently FreeCAD has several ways to install packages: 
1. [Freecad Addon Manager][AddonManager] 
2. [freecad-pluginloader][pluginloader]  

With `pip` and `pypi` a third option is introduced. In addition, utilizing `pip` also provides powerful possibilities to install third party dependencies. This option is not available yet.


### resources

In addition to the `setup.py` there is often the need for a [`MANIFEST.in`][MANIFEST] With this file it's possible to install data like icons, documentation files, ... (everything not directly connected to python).  
To tell `setuptools` to use the `MANIFEST.in` add this line to the setup function in the setup.py:

```python
setup(..., include_package_data=True)
```


### install local

To install your extension locally with pip, do the following from a cmd (windows) or terminal (unix):

```bash
cd <path_to_your_package>
pip install .
```

### uploading your package to pypi

Please have a look at this [pypi twine tutorial][twine].

Be careful with version-numbering. It seems pypi doesn't allow to upload a package with a version smaller then the biggest version of the package uploaded. This seems to be true also for deleted packages and deleted versions.

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

### Glossary terms used in this discussion (that may lead to confusion)

- **_freecad-module_**: It's anything available through FreeCAD's python interpreter and placed in FreeCADs directory structure.  
This can be a **module, package, workbench, namespace-package, extension-module**.
- **_new_style_module_**: This refers to **packages** which are added to FreeCAD as **namespace-packages**
- **_old_style_module_**: A **package** which is plugged into FreeCAD by adding it's base-directory to `sys.path` and uses `Init.py` and `InitGui.py` to get initialized by FreeCAD.

### Motivation for Namespace Workbenches
There are several reasons why you might consider using the namespace-workbenche proposed in this template, and upgrade existing workbenches using the "old style".

1. The ability to execute your module using a regular `python` interpreter and have it "just work" (See [related forum discussion](https://forum.freecadweb.org/viewtopic.php?f=8&t=40749#p346331)).

2. Name-spaced packages (namespace-workbenches) avoid namespace collisions and the need to have a common prefix on all classes and files to ensure uniqueness (See [related forum discussion](https://forum.freecadweb.org/viewtopic.php?f=23&t=38593&p=345439#p345437)).

3. Integrating with [PyPI](https://pypi.org/) / [pip](https://pip.pypa.io/en/stable/). The ability to `pip install freecad.myworkbench` (See [related forum discussion](https://forum.freecadweb.org/viewtopic.php?f=10&t=38476&p=326444#p326574)).

4. InitGui.py and Init.py (legacy-workbenches) do not behave like expected because these files are called with exec and are not properly imported. This leads to problems like:
   - `__file__` not useable to get the path to the python file
   - predefined variables

### Tip

Due to the fact we are now using the `pktutil-module` to find extensions of FreeCAD, it's possible to use standard-python-paths to place the extension. This is any location which is included in the `sys.path`.  

To get a list of all the locations simple run this code in the FreeCAD-console:

```python
import sys
sys.path
```

--------------------------------------------------------------------------------

[AddonManager]: https://github.com/FreeCAD/FreeCAD-addons
[pluginloader]: https://github.com/microelly2/freecad-pluginloader
[setuptools]: https://setuptools.readthedocs.io/en/latest/
[MANIFEST]: https://docs.python.org/2/distutils/sourcedist.html#commands
[twine]: https://pypi.python.org/pypi/twine
[pyrate]: https://github.com/mess42/pyrate
[OpenGlider]: https://github.com/booya-at/OpenGlider
[FCGear]: https://github.com/looooo/FCGear
[FC_pipintegration]: https://github.com/looooo/freecad_pipintegration
