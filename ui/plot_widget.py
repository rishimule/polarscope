"""LidarPlot: dark polar-style scatter widget."""
from __future__ import annotations

import numpy as np
import pyqtgraph as pg
from PySide6.QtCore import Slot
from PySide6.QtGui import QColor, QFont, QPen
from PySide6.QtWidgets import QGraphicsEllipseItem, QGraphicsLineItem

import theme

GRID_RADII = (2.0, 4.0, 8.0, 12.0)
SPOKE_STEP_DEG = 30


class LidarPlot(pg.PlotWidget):
    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self.setBackground(theme.BG)
        self.showAxis("left", False)
        self.showAxis("bottom", False)
        vb = self.getViewBox()
        vb.setAspectLocked(True)
        vb.setRange(xRange=(-12.5, 12.5), yRange=(-12.5, 12.5), padding=0)
        vb.setLimits(xMin=-15, xMax=15, yMin=-15, yMax=15, minXRange=0.5, maxXRange=30)
        vb.setMouseEnabled(x=True, y=True)
        vb.setMenuEnabled(False)

        self._build_grid()
        self._scatter = pg.ScatterPlotItem(
            size=4, brush=pg.mkBrush(theme.ACCENT), pen=None, pxMode=True
        )
        self.addItem(self._scatter)

    def _build_grid(self) -> None:
        grid_color = QColor(31, 111, 235, 120)
        pen = QPen(grid_color)
        pen.setWidthF(0.5)
        pen.setCosmetic(True)

        for r in GRID_RADII:
            circle = QGraphicsEllipseItem(-r, -r, 2 * r, 2 * r)
            circle.setPen(pen)
            self.addItem(circle)

        for deg in range(0, 360, SPOKE_STEP_DEG):
            rad = np.deg2rad(90 - deg)
            x = 12.0 * np.cos(rad)
            y = 12.0 * np.sin(rad)
            line = QGraphicsLineItem(0, 0, float(x), float(y))
            line.setPen(pen)
            self.addItem(line)

        # Distance labels along NE (45°) diagonal
        for r in GRID_RADII:
            x = float(r * np.cos(np.pi / 4))
            y = float(r * np.sin(np.pi / 4))
            text = pg.TextItem(f"{int(r)}m", color=theme.TEXT_MUTED, anchor=(0.5, 0.5))
            font = QFont()
            font.setPointSize(9)
            text.setFont(font)
            text.setPos(x, y)
            self.addItem(text)

        # Cardinal labels
        for label, (x, y) in (
            ("N", (0.0, 12.4)),
            ("E", (12.4, 0.0)),
            ("S", (0.0, -12.4)),
            ("W", (-12.4, 0.0)),
        ):
            t = pg.TextItem(label, color=theme.TEXT, anchor=(0.5, 0.5))
            font = QFont()
            font.setPointSize(10)
            font.setBold(True)
            t.setFont(font)
            t.setPos(x, y)
            self.addItem(t)

    @Slot(object)
    def update_points(self, xy: np.ndarray) -> None:
        if xy is None or len(xy) == 0:
            self._scatter.setData([], [])
            return
        self._scatter.setData(x=xy[:, 0], y=xy[:, 1])

    def clear_points(self) -> None:
        self._scatter.setData([], [])
