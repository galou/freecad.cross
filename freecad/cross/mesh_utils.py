"""Utility functions to work with meshes."""

from __future__ import annotations

from pathlib import Path
from typing import Iterable

import FreeCAD as fc

import Mesh  # FreeCAD

from .deep_copy import deep_copy_object
from .freecad_utils import is_mesh
from .import_dae import export as export_dae
from .import_dae import read as read_dae

# Typing hints.
DO = fc.DocumentObject
DOList = Iterable[DO]


def read_mesh_dae(
        filename: [Path | str],
        ) -> Mesh.MeshObject:
    current_doc = fc.activeDocument()
    path = Path(filename)
    # `read_dae` does not export its object, so we need to let it create an object in a
    # temporary document.
    tmp_doc = fc.newDocument(hidden=True, temp=True)
    fc.setActiveDocument(tmp_doc.Name)
    read_dae(str(path))
    tmp_doc.recompute()
    # A `Mesh::MeshObject`.
    merged_raw_mesh = Mesh.Mesh()
    for mesh_obj in tmp_doc.Objects:
        if hasattr(mesh_obj, 'Mesh'):
            merged_raw_mesh.addMesh(mesh_obj.Mesh)
    fc.closeDocument(tmp_doc.Name)
    if current_doc:
        fc.setActiveDocument(current_doc.Name)
    return merged_raw_mesh


def save_mesh_dae(obj: DO,
                  filename: [Path | str],
                  ) -> None:
    """Save the mesh of a FreeCAD object into a Collada file."""
    current_doc = fc.activeDocument()
    # `export_dae` doesn't support links. Deep copy the shape in a new
    # temporary document and export them.
    tmp_doc = fc.newDocument(hidden=True, temp=True)
    copies = deep_copy_object(obj, tmp_doc)
    Path(filename).parent.mkdir(parents=True, exist_ok=True)
    export_dae(copies, str(filename))
    if current_doc:
        fc.setActiveDocument(current_doc.Name)


def save_mesh(obj: DO,
              filename: [Path | str],
              ) -> None:
    """Save the mesh of a FreeCAD object into a file.

    The type of the exported file is determined by the Mesh module.
    See the Mesh module for a list of supported formats.

    """
    Path(filename).parent.mkdir(parents=True, exist_ok=True)
    # TODO: scale to meters.
    Mesh.export([obj], str(filename))


def scale_mesh_object(obj: DO, scale_factor: [float | Iterable[float]]) -> None:
    """Scale a mesh object in place.

    Parameters
    ----------
    - obj: FreeCAD object of type `Mesh::Feature`.
    - scale_factor: a single float or a list of 3 floats.

    """
    if not is_mesh(obj):
        raise RuntimeError(
            'First argument must be `Mesh::Feature` FreeCAD object')
    if isinstance(scale_factor, float):
        scaling_vector = fc.Vector(scale_factor, scale_factor, scale_factor)
    else:
        try:
            scaling_vector = fc.Vector(scale_factor)
        except (IndexError, ValueError):
            raise RuntimeError('Scaling factor must be a float or a list'
                               f' of 3 floats, got {scale_factor}')
    scale_mat = fc.Matrix()
    scale_mat.scale(scaling_vector)
    mesh = obj.Mesh.copy()
    mesh.transform(scale_mat)
    obj.Mesh = mesh
