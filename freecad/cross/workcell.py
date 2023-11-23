# Stub for a CROSS::Robot.

from __future__ import annotations

from typing import ForwardRef, Union

import FreeCAD as fc

# Implementation note: The following import is necessary to avoid a circular
# dependency.
JointRef = ForwardRef('Joint')
XacroObjectRef = ForwardRef('XacroObject')

JointOrXacroObject = Union[JointRef, XacroObjectRef]


class Workcell(fc.DocumentObject):
    Group: list[JointOrXacroObject]
    OutputPath: str
    RootLink: str
    _Type: str

