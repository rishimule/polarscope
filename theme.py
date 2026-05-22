"""Dark palette + pyqtgraph color config."""
from __future__ import annotations

import pyqtgraph as pg
from PySide6.QtGui import QColor, QPalette
from PySide6.QtWidgets import QApplication

BG = "#0d1117"
SURFACE = "#161b22"
BORDER = "#30363d"
TEXT = "#e6edf3"
TEXT_MUTED = "#7d8590"
ACCENT = "#39ff14"
GRID = "#1f6feb"
LED_CONNECTED = "#2ea043"
LED_SCANNING = "#39ff14"
LED_ERROR = "#f85149"


def apply_dark_theme(app: QApplication) -> None:
    app.setStyle("Fusion")
    palette = QPalette()
    palette.setColor(QPalette.Window, QColor(BG))
    palette.setColor(QPalette.WindowText, QColor(TEXT))
    palette.setColor(QPalette.Base, QColor(SURFACE))
    palette.setColor(QPalette.AlternateBase, QColor(BG))
    palette.setColor(QPalette.Text, QColor(TEXT))
    palette.setColor(QPalette.Button, QColor(SURFACE))
    palette.setColor(QPalette.ButtonText, QColor(TEXT))
    palette.setColor(QPalette.Highlight, QColor(ACCENT))
    palette.setColor(QPalette.HighlightedText, QColor(BG))
    palette.setColor(QPalette.PlaceholderText, QColor(TEXT_MUTED))
    app.setPalette(palette)
    app.setStyleSheet(
        f"""
        QWidget {{ background-color: {BG}; color: {TEXT}; }}
        QFrame#sidebar {{ background-color: {SURFACE}; border-right: 1px solid {BORDER}; }}
        QPushButton {{
            background-color: {SURFACE};
            border: 1px solid {BORDER};
            padding: 6px 12px;
            border-radius: 4px;
        }}
        QPushButton:hover {{ border-color: {ACCENT}; }}
        QPushButton:disabled {{ color: {TEXT_MUTED}; border-color: {BORDER}; }}
        QPushButton:checked {{ background-color: {ACCENT}; color: {BG}; }}
        QComboBox {{
            background-color: {SURFACE};
            border: 1px solid {BORDER};
            padding: 4px 8px;
            border-radius: 4px;
        }}
        QToolButton {{
            background-color: {SURFACE};
            border: 1px solid {BORDER};
            padding: 4px 8px;
            border-radius: 4px;
        }}
        QToolButton:hover {{ border-color: {ACCENT}; }}
        QStatusBar {{ background-color: {SURFACE}; }}
        QLabel#banner_error {{
            background-color: {LED_ERROR};
            color: {BG};
            padding: 8px;
            border-radius: 4px;
            font-weight: bold;
        }}
        QLabel#banner_info {{
            background-color: {GRID};
            color: {TEXT};
            padding: 8px;
            border-radius: 4px;
        }}
        """
    )
    pg.setConfigOption("background", BG)
    pg.setConfigOption("foreground", TEXT)
    pg.setConfigOption("antialias", True)
