"""Sidebar: port selection + connect/scan/snapshot/record controls."""
from __future__ import annotations

from PySide6.QtCore import Signal
from PySide6.QtGui import QFont
from PySide6.QtWidgets import (
    QComboBox,
    QFileDialog,
    QFrame,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QToolButton,
    QVBoxLayout,
)

import theme

VERSION = "0.1.0"


class _LedDot(QLabel):
    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self.setFixedSize(12, 12)
        self.set_state("disconnected")

    def set_state(self, state: str) -> None:
        color = {
            "connected": theme.LED_CONNECTED,
            "connected (idle)": theme.LED_CONNECTED,
            "scanning": theme.LED_SCANNING,
        }.get(state, theme.LED_ERROR)
        self.setStyleSheet(f"background-color: {color}; border-radius: 6px;")


class Sidebar(QFrame):
    connect_requested = Signal(str)
    disconnect_requested = Signal()
    start_requested = Signal()
    stop_requested = Signal()
    refresh_requested = Signal()
    snapshot_requested = Signal()
    record_started = Signal(str)
    record_stopped = Signal()

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self.setObjectName("sidebar")
        self.setFixedWidth(260)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(16, 16, 16, 16)
        layout.setSpacing(10)

        title = QLabel("RPLIDAR C1 Viewer")
        title_font = QFont()
        title_font.setPointSize(14)
        title_font.setBold(True)
        title.setFont(title_font)
        layout.addWidget(title)

        version_lbl = QLabel(f"v{VERSION}")
        version_lbl.setStyleSheet(f"color: {theme.TEXT_MUTED};")
        layout.addWidget(version_lbl)

        layout.addSpacing(12)

        layout.addWidget(QLabel("Serial port"))
        port_row = QHBoxLayout()
        self._port_combo = QComboBox()
        self._refresh_btn = QToolButton()
        self._refresh_btn.setText("↻")
        self._refresh_btn.setToolTip("Refresh port list")
        port_row.addWidget(self._port_combo, stretch=1)
        port_row.addWidget(self._refresh_btn)
        layout.addLayout(port_row)

        self._connect_btn = QPushButton("Connect")
        self._disconnect_btn = QPushButton("Disconnect")
        self._disconnect_btn.hide()
        layout.addWidget(self._connect_btn)
        layout.addWidget(self._disconnect_btn)

        self._start_btn = QPushButton("Start Scan")
        self._stop_btn = QPushButton("Stop Scan")
        self._stop_btn.hide()
        self._start_btn.setEnabled(False)
        layout.addWidget(self._start_btn)
        layout.addWidget(self._stop_btn)

        self._snapshot_btn = QPushButton("Save Snapshot")
        self._snapshot_btn.setEnabled(False)
        self._record_btn = QPushButton("Record CSV")
        self._record_btn.setCheckable(True)
        self._record_btn.setEnabled(False)
        layout.addWidget(self._snapshot_btn)
        layout.addWidget(self._record_btn)

        layout.addStretch(1)

        led_row = QHBoxLayout()
        self._led = _LedDot()
        self._state_label = QLabel("Disconnected")
        led_row.addWidget(self._led)
        led_row.addWidget(self._state_label)
        led_row.addStretch(1)
        layout.addLayout(led_row)

        self._refresh_btn.clicked.connect(self.refresh_requested.emit)
        self._connect_btn.clicked.connect(self._on_connect)
        self._disconnect_btn.clicked.connect(self.disconnect_requested.emit)
        self._start_btn.clicked.connect(self.start_requested.emit)
        self._stop_btn.clicked.connect(self.stop_requested.emit)
        self._snapshot_btn.clicked.connect(self.snapshot_requested.emit)
        self._record_btn.toggled.connect(self._on_record_toggled)

    # ----- public slots called by MainWindow -----

    def set_ports(self, ports: list[str]) -> None:
        self._port_combo.blockSignals(True)
        self._port_combo.clear()
        if not ports:
            self._port_combo.addItem("No devices found")
            self._port_combo.setEnabled(False)
            self._connect_btn.setEnabled(False)
        else:
            self._port_combo.addItems(ports)
            self._port_combo.setEnabled(True)
            self._connect_btn.setEnabled(True)
        self._port_combo.blockSignals(False)

    def set_state(self, state: str) -> None:
        self._state_label.setText(state.capitalize())
        self._led.set_state(state)
        connected = state in (
            "connected", "connected (idle)", "connected (warning)", "scanning"
        )
        scanning = state == "scanning"
        self._connect_btn.setVisible(not connected)
        self._connect_btn.setEnabled(self._port_combo.isEnabled() and not connected)
        self._disconnect_btn.setVisible(connected)
        self._start_btn.setVisible(not scanning)
        self._start_btn.setEnabled(connected and not scanning)
        self._stop_btn.setVisible(scanning)
        # Record only while actively scanning — prevents orphan empty CSVs.
        self._record_btn.setEnabled(scanning)
        if not scanning and self._record_btn.isChecked():
            self._record_btn.setChecked(False)

    def set_connect_busy(self, busy: bool) -> None:
        """Debounce: disable Connect while a connect attempt is in flight."""
        if busy:
            self._connect_btn.setEnabled(False)
        else:
            self._connect_btn.setEnabled(
                self._port_combo.isEnabled() and self._connect_btn.isVisible()
            )

    def enable_snapshot(self) -> None:
        self._snapshot_btn.setEnabled(True)

    def selected_port(self) -> str:
        return self._port_combo.currentText() if self._port_combo.isEnabled() else ""

    # ----- internal -----

    def _on_connect(self) -> None:
        port = self.selected_port()
        if port:
            self.connect_requested.emit(port)

    def _on_record_toggled(self, checked: bool) -> None:
        if checked:
            path, _ = QFileDialog.getSaveFileName(
                self, "Save scan to CSV", "scan.csv", "CSV files (*.csv)"
            )
            if not path:
                self._record_btn.blockSignals(True)
                self._record_btn.setChecked(False)
                self._record_btn.blockSignals(False)
                return
            self.record_started.emit(path)
        else:
            self.record_stopped.emit()
