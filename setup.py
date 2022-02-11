import os

from setuptools import setup

# name: this is the name of the distribution.
# Packages using the same name here cannot be installed together

version_path = os.path.join(os.path.abspath(os.path.dirname(__file__)),
                            'freecad', 'workbench_ros', 'version.py')
with open(version_path) as fp:
    exec(fp.read())

setup(name='freecad.workbench_ros',
      version=str(__version__),
      packages=['freecad',
                'freecad.workbench_ros',
                ],
      maintainer='galou',
      maintainer_email='gael.ecorchard@cvut.cz',
      url='https://github.com/galou/freecad.workbench_ros',
      description='A workbench to work with ROS',
      install_requires=[],
      include_package_data=True)

# install_requires should be ['xacro'].
