"""Utility function specific to this workbench."""

from __future__ import annotations

from pathlib import Path

import FreeCAD as fc

from .utils import RESOURCES_PATH

# Typing hints.
DO = fc.DocumentObject


def ros_name(obj: DO):
    """Return in order obj.Label2, obj.Label, obj.Name."""
    if ((not hasattr(obj, 'isDerivedFrom')
         or (not obj.isDerivedFrom('App::DocumentObject')))):
        return 'not_a_FreeCAD_object'
    if obj.Label2:
        return obj.Label2
    if obj.Label:
        return obj.Label
    return obj.Name


def export_templates(
        template_files: list[str],
        package_parent: [Path | str],
        **keys: dict[str, str],
        ) -> None:
    """Export generated files.

    Parameters
    ----------

    - template_files: list of files to export, relative to
                      `RESOURCES_PATH/templates`, where RESOURCES_PATH is the
                      directory `resources` of this workbench.
    - package_name: the directory containing the directory called
                    `package_name`, usually "$ROS_WORKSPACE/src".
    - keys: dictionnary of replacement string in templates.
            - package_name (compulsary): name of the ROS package and its containing
                                         directory.
            - urdf_file: name of the URDF/xacro file without directory.
                         Used in `launch/display.launch.py`.

    """
    try:
        package_name: str = keys['package_name']
    except KeyError:
        raise RuntimeError('Parameter "package_name" must be given')

    package_parent = Path(package_parent)
    meshes_dir = ('meshes '
                  if _has_meshes_directory(package_parent, package_name)
                  else '')
    for f in template_files:
        template_file_path = RESOURCES_PATH / 'templates' / f
        template = template_file_path.read_text()
        txt = template.format(meshes_dir=meshes_dir, **keys)
        output_path = package_parent / package_name / f
        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_text(txt)


def _has_meshes_directory(
        package_parent: [Path | str],
        package_name: str,
        ) -> bool:
    """Return True if the directory "meshes" exists in the package."""
    meshes_directory = Path(package_parent) / package_name / 'meshes'
    return meshes_directory.exists()
