"""Utility functions to work with FreeCAD's Gui."""

from __future__ import annotations

import FreeCAD as fc

if hasattr(fc, 'GuiUp') and fc.GuiUp:
    from PySide import QtGui  # FreeCAD's PySide!

    def tr(text: str) -> str:
        return QtGui.QApplication.translate('cross', text)
else:
    def tr(text: str) -> str:
        return text
