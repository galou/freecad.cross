"""Utility classes and functions to export robots to glTF from FreeCAD.

The glTF (GL Transmission Format) is a JSON-based format for 3D scenes.
Reference: https://registry.khronos.org/glTF/specs/2.0/glTF-2.0.html

Coordinate systems
------------------
glTF uses a **Y-up, right-handed** coordinate system (X right, Y up, Z toward
the viewer).  FreeCAD and ROS/URDF use a **Z-up, right-handed** coordinate
system (X forward, Y left, Z up).

The conversion is a rotation of −90° about the X axis:
- FreeCAD X → glTF X  (unchanged)
- FreeCAD Y → glTF −Z
- FreeCAD Z → glTF Y  (up direction preserved)

"""

from __future__ import annotations

import base64
import json
import math
import struct
from pathlib import Path
from typing import Optional

import FreeCAD as fc

import numpy as np

# glTF component type constants (GLTF spec Table 3).
_GLTF_UNSIGNED_SHORT = 5123
_GLTF_UNSIGNED_INT = 5125
_GLTF_FLOAT = 5126

# glTF buffer target constants (GLTF spec §5.18).
_GLTF_ARRAY_BUFFER = 34962       # vertex data
_GLTF_ELEMENT_ARRAY_BUFFER = 34963  # index data

# glTF accessor type strings (GLTF spec §5.1).
_GLTF_SCALAR = 'SCALAR'
_GLTF_VEC3 = 'VEC3'

# Maximum value that fits in an UNSIGNED_SHORT index (2^16 - 1).
_GLTF_MAX_UNSIGNED_SHORT = 65535

_GLTF_GENERATOR = (
    'CROSS, a ROS Workbench for FreeCAD'
    ' (https://github.com/galou/freecad.cross)'
)

_SQRT2_OVER_2 = math.sqrt(2.0) / 2.0

# Quaternion [x, y, z, w] for a rotation of −90° about the X axis.
# Applying this rotation transforms a FreeCAD/ROS Z-up coordinate frame into
# glTF's Y-up coordinate frame:
#   FreeCAD X → glTF X
#   FreeCAD Y → glTF −Z
#   FreeCAD Z → glTF Y (up direction)
FC_TO_GLTF_ROTATION: list[float] = [-_SQRT2_OVER_2, 0.0, 0.0, _SQRT2_OVER_2]


def placement_to_gltf_trs(
    placement: fc.Placement,
) -> tuple[list[float], list[float]]:
    """Convert a FreeCAD placement to a glTF (translation, rotation) pair.

    Returns
    -------
    (translation, rotation) where:
    - translation is [x, y, z] in **meters** (FreeCAD uses mm internally).
    - rotation is a quaternion [x, y, z, w].

    """
    t = placement.Base
    translation = [t.x * 1e-3, t.y * 1e-3, t.z * 1e-3]
    q = placement.Rotation.Q  # FreeCAD returns (x, y, z, w).
    rotation = [q[0], q[1], q[2], q[3]]
    return translation, rotation


class GltfDocument:
    """Accumulates glTF data and serializes it to a ``.gltf`` file.

    Usage
    -----
    Build the scene graph by calling :meth:`add_mesh_from_fc_object` and
    :meth:`add_node`, then call :meth:`set_root_nodes` and :meth:`save`.

    """

    def __init__(self, name: str = 'robot') -> None:
        self.name = name

        # Ordered lists that map directly to glTF JSON arrays.
        self._nodes: list[dict] = []
        self._meshes: list[dict] = []
        self._accessors: list[dict] = []
        self._buffer_views: list[dict] = []

        # Raw binary buffer that backs all buffer views.
        self._buffer_data = bytearray()

        # Indices of top-level nodes that go into the single scene.
        self._root_node_indices: list[int] = []

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _pad_buffer(self) -> None:
        """Pad the binary buffer to the required 4-byte alignment."""
        rem = len(self._buffer_data) % 4
        if rem:
            self._buffer_data += b'\x00' * (4 - rem)

    def _add_buffer_view(
        self,
        data: bytes,
        target: Optional[int] = None,
    ) -> int:
        """Append *data* to the binary buffer and return a buffer-view index."""
        self._pad_buffer()
        byte_offset = len(self._buffer_data)
        self._buffer_data += data
        view: dict = {
            'buffer': 0,
            'byteOffset': byte_offset,
            'byteLength': len(data),
        }
        if target is not None:
            view['target'] = target
        self._buffer_views.append(view)
        return len(self._buffer_views) - 1

    def _add_accessor(
        self,
        buffer_view_idx: int,
        component_type: int,
        count: int,
        type_str: str,
        min_vals: Optional[list] = None,
        max_vals: Optional[list] = None,
    ) -> int:
        """Append an accessor and return its index."""
        accessor: dict = {
            'bufferView': buffer_view_idx,
            'byteOffset': 0,
            'componentType': component_type,
            'count': count,
            'type': type_str,
        }
        if min_vals is not None:
            accessor['min'] = min_vals
        if max_vals is not None:
            accessor['max'] = max_vals
        self._accessors.append(accessor)
        return len(self._accessors) - 1

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def add_mesh_from_fc_object(
        self,
        obj: fc.DocumentObject,
    ) -> Optional[int]:
        """Tessellate *obj* and add it as a glTF mesh.

        The object's ``Placement`` is applied to the tessellated vertices so
        that the returned mesh is expressed in the object's **parent** frame
        (i.e. the link's URDF frame when called via
        :meth:`LinkProxy.export_gltf`).

        Parameters
        ----------
        obj:
            A FreeCAD document object that has a ``Shape`` or ``Mesh``
            attribute.

        Returns
        -------
        The glTF mesh index, or ``None`` if the object cannot be tessellated.

        """
        try:
            import MeshPart  # FreeCAD module, available at runtime.
        except ImportError:
            return None

        placement: fc.Placement = getattr(obj, 'Placement', fc.Placement())

        try:
            if hasattr(obj, 'Shape'):
                raw_mesh = MeshPart.meshFromShape(
                    Shape=obj.Shape,
                    LinearDeflection=0.1,
                    AngularDeflection=0.1,
                    Relative=True,
                )
            elif hasattr(obj, 'Mesh'):
                raw_mesh = obj.Mesh.copy()
            else:
                return None
        except Exception:
            return None

        if raw_mesh.CountFacets == 0:
            return None

        # Build flat position and normal arrays from the tessellated facets.
        # Each facet contributes 3 vertices (no index sharing).
        positions: list[float] = []
        normals: list[float] = []
        indices: list[int] = []
        idx = 0
        for facet in raw_mesh.Facets:
            for pt in facet.Points:
                # Transform from the object's local frame to its parent frame
                # and convert from mm (FreeCAD) to metres (glTF).
                p = placement.multVec(fc.Vector(pt[0], pt[1], pt[2]))
                positions.extend([p.x * 1e-3, p.y * 1e-3, p.z * 1e-3])
                n = facet.Normal
                normals.extend([n.x, n.y, n.z])
            indices.extend([idx, idx + 1, idx + 2])
            idx += 3

        if not indices:
            return None

        # ---- Positions -------------------------------------------------------
        pos_bytes = struct.pack(f'{len(positions)}f', *positions)
        pos_view_idx = self._add_buffer_view(pos_bytes, _GLTF_ARRAY_BUFFER)
        pos_arr = np.array(positions, dtype=np.float32).reshape(-1, 3)
        pos_acc_idx = self._add_accessor(
            pos_view_idx,
            _GLTF_FLOAT,
            len(positions) // 3,
            _GLTF_VEC3,
            min_vals=pos_arr.min(axis=0).tolist(),
            max_vals=pos_arr.max(axis=0).tolist(),
        )

        # ---- Normals ---------------------------------------------------------
        norm_bytes = struct.pack(f'{len(normals)}f', *normals)
        norm_view_idx = self._add_buffer_view(norm_bytes, _GLTF_ARRAY_BUFFER)
        norm_acc_idx = self._add_accessor(
            norm_view_idx, _GLTF_FLOAT, len(normals) // 3, _GLTF_VEC3,
        )

        # ---- Indices ---------------------------------------------------------
        if len(indices) <= _GLTF_MAX_UNSIGNED_SHORT:
            ind_bytes = struct.pack(f'{len(indices)}H', *indices)
            ind_component_type = _GLTF_UNSIGNED_SHORT
        else:
            ind_bytes = struct.pack(f'{len(indices)}I', *indices)
            ind_component_type = _GLTF_UNSIGNED_INT
        ind_view_idx = self._add_buffer_view(
            ind_bytes, _GLTF_ELEMENT_ARRAY_BUFFER,
        )
        ind_acc_idx = self._add_accessor(
            ind_view_idx, ind_component_type, len(indices), _GLTF_SCALAR,
        )

        mesh_dict: dict = {
            'primitives': [{
                'attributes': {
                    'POSITION': pos_acc_idx,
                    'NORMAL': norm_acc_idx,
                },
                'indices': ind_acc_idx,
                'mode': 4,  # TRIANGLES
            }],
        }
        label = getattr(obj, 'Label', None)
        if label:
            mesh_dict['name'] = label
        self._meshes.append(mesh_dict)
        return len(self._meshes) - 1

    def add_node(
        self,
        name: str,
        children: Optional[list[int]] = None,
        mesh_idx: Optional[int] = None,
        translation: Optional[list[float]] = None,
        rotation: Optional[list[float]] = None,
    ) -> int:
        """Append a scene-graph node and return its index.

        Parameters
        ----------
        name:
            Human-readable node name (used by viewers and for debugging).
        children:
            Indices of child nodes.
        mesh_idx:
            Index of the glTF mesh attached to this node, if any.
        translation:
            ``[x, y, z]`` in metres.  Omitted when all components are zero.
        rotation:
            Quaternion ``[x, y, z, w]``.  Omitted when equal to identity.

        """
        node: dict = {'name': name}
        if children:
            node['children'] = list(children)
        if mesh_idx is not None:
            node['mesh'] = mesh_idx
        if translation is not None and any(abs(v) > 1e-12 for v in translation):
            node['translation'] = translation
        if rotation is not None:
            x, y, z, w = rotation
            if any(abs(v) > 1e-12 for v in [x, y, z]) or abs(w - 1.0) > 1e-12:
                node['rotation'] = rotation
        self._nodes.append(node)
        return len(self._nodes) - 1

    def append_child_to_node(self, parent_idx: int, child_idx: int) -> None:
        """Add *child_idx* to the children list of the node at *parent_idx*."""
        node = self._nodes[parent_idx]
        node.setdefault('children', []).append(child_idx)

    def set_node_transform(
        self,
        node_idx: int,
        translation: list[float],
        rotation: list[float],
    ) -> None:
        """Set the translation and rotation of an existing node.

        Parameters
        ----------
        node_idx:
            Index of the node to update.
        translation:
            ``[x, y, z]`` in metres.
        rotation:
            Quaternion ``[x, y, z, w]``.

        """
        node = self._nodes[node_idx]
        if any(abs(v) > 1e-12 for v in translation):
            node['translation'] = translation
        x, y, z, w = rotation
        if any(abs(v) > 1e-12 for v in [x, y, z]) or abs(w - 1.0) > 1e-12:
            node['rotation'] = rotation

    def set_root_nodes(self, indices: list[int]) -> None:
        """Set the indices of the top-level scene nodes."""
        self._root_node_indices = list(indices)

    def to_dict(self) -> dict:
        """Build and return the complete glTF JSON as a Python dict."""
        gltf: dict = {
            'asset': {
                'version': '2.0',
                'generator': _GLTF_GENERATOR,
            },
        }

        if self._root_node_indices:
            gltf['scene'] = 0
            gltf['scenes'] = [
                {'name': self.name, 'nodes': self._root_node_indices},
            ]

        if self._nodes:
            gltf['nodes'] = self._nodes
        if self._meshes:
            gltf['meshes'] = self._meshes
        if self._accessors:
            gltf['accessors'] = self._accessors
        if self._buffer_views:
            gltf['bufferViews'] = self._buffer_views

        if self._buffer_data:
            encoded = base64.b64encode(
                bytes(self._buffer_data),
            ).decode('ascii')
            gltf['buffers'] = [{
                'byteLength': len(self._buffer_data),
                'uri': 'data:application/octet-stream;base64,' + encoded,
            }]

        return gltf

    def save(self, output_path: Path) -> None:
        """Serialize and write the glTF document to *output_path*.

        The parent directory is created automatically if it does not exist.

        """
        output_path = Path(output_path)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        gltf_dict = self.to_dict()
        with open(output_path, 'w', encoding='utf-8') as f:
            json.dump(gltf_dict, f, indent=2)
