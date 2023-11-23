# Stub for a CROSS::XacroObject.

from __future__ import annotations

from typing import ForwardRef

import FreeCAD as fc

# Implementation note: The following import is necessary to avoid a circular
# dependency.
LinkRef = ForwardRef('Link')
RobotRef = ForwardRef('Robot')

# Typing hints
DO = fc.DocumentObject
DOList = list[DO]


class XacroObject(fc.DocumentObject):
    Group: list[fc.DocumentObject]
    InputFile: str
    MainMacro: str
    Placement: fc.Placement
    _Type: str

    def addObject(self, object_to_add: fc.DocumentObject) -> None: ...
    def removeObject(self, object_to_remove: fc.DocumentObject) -> DOList: ...
