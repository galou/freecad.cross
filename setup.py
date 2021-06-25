from setuptools import setup
import os
# from freecad.workbench_starterkit.version import __version__
# name: this is the name of the distribution.
# Packages using the same name here cannot be installed together

version_path = os.path.join(os.path.abspath(os.path.dirname(__file__)), 
                            "freecad", "workbench_starterkit", "version.py")
with open(version_path) as fp:
    exec(fp.read())

setup(name='freecad.workbench_starterkit',
      version=str(__version__),
      packages=['freecad',
                'freecad.workbench_starterkit'],
      maintainer="looooo",
      maintainer_email="sppedflyer@gmail.com",
      url="https://github.com/FreeCAD/Workbench-Starterkit",
      description="template for a freecad extensions, installable with pip",
      install_requires=['numpy'], # should be satisfied by FreeCAD's system dependencies already
      include_package_data=True)
