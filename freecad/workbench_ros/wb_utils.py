"""Utility function specific to this workbench."""

from __future__ import annotations

import FreeCAD as fc

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
