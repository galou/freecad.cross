# Stub for a CROSS::Link.

from __future__ import annotations

import FreeCAD as fc


class Link(fc.DocumentObject):
    Collision: list[fc.DocumentObject]
    Group: list[fc.DocumentObject]
    Mass: float
    MountedPlacement: fc.Placement
    Placement: fc.Placement
    Real: list[fc.DocumentObject]
    Visual: list[fc.DocumentObject]
    _Type: str
