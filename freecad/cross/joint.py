# Stub for a CROSS::Joint.

from __future__ import annotations

from typing import ForwardRef

import FreeCAD as fc

# Implementation note: The following import is necessary to avoid a circular
# dependency.
JointRef = ForwardRef('Joint')


class Joint(fc.DocumentObject):
    Child: str  # Must name a CROSS::Link by its ROS name.
    Effort: float
    LowerLimit: float
    Mimic: JointRef
    MimickedJoint: JointRef
    Multiplier: float
    Offset: float
    Origin: fc.Placement
    Parent: str  # Must name a CROSS::Link by its ROS name.
    Placement: fc.Placement
    Position: float
    Type: str
    UpperLimit: float
    Velocity: float
    _Type: str
