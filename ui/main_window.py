"""MainWindow: owns QThread + LidarWorker, wires Sidebar <-> Worker <-> Plot."""
from __future__ import annotations

from datetime import datetime

from PySide6.QtCore import QMetaObject, QSize, Qt, QThread, Signal
from PySide6.QtGui import QCloseEvent, QPainter, QPixmap
from PySide6.QtWidgets import (
    QFileDialog,
    QHBoxLayout,
    QMainWindow,
    QVBoxLayout,
    QWidget,
)

from lidar.ports import list_serial_ports
from lidar.worker import LidarWorker
from ui.plot_widget import LidarPlot
from ui.sidebar import Sidebar
from ui.status_bar import TopBanner, attach_status_widgets


class MainWindow(QMainWindow):
    # Bridge signal to invoke worker.open_device(port) across thread boundary.
    _request_open = Signal(str)

    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("RPLIDAR C1 Viewer")
        self.resize(1200, 800)

        central = QWidget()
        root = QHBoxLayout(central)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(0)

        self._sidebar = Sidebar()
        root.addWidget(self._sidebar)

        plot_col = QVBoxLayout()
        plot_col.setContentsMargins(8, 8, 8, 8)
        self._banner = TopBanner()
        self._plot = LidarPlot()
        plot_col.addWidget(self._banner)
        plot_col.addWidget(self._plot, stretch=1)
        root.addLayout(plot_col, stretch=1)

        self.setCentralWidget(central)
        self._stats = attach_status_widgets(self.statusBar())

        # Worker on a QThread
        self._thread = QThread(self)
        self._worker = LidarWorker()
        self._worker.moveToThread(self._thread)
        self._thread.start()

        self._wire_signals()
        self._sidebar.set_ports(list_serial_ports())
        self._sidebar.set_state("disconnected")
        self._first_scan_seen = False

    def _wire_signals(self) -> None:
        s, w = self._sidebar, self._worker
        s.connect_requested.connect(self._on_connect_clicked)
        self._request_open.connect(w.open_device, Qt.QueuedConnection)
        s.disconnect_requested.connect(w.close_device, Qt.QueuedConnection)
        s.start_requested.connect(w.start_scan, Qt.QueuedConnection)
        s.stop_requested.connect(self._on_stop)
        s.refresh_requested.connect(self._on_refresh)
        s.snapshot_requested.connect(self._on_snapshot)
        s.record_started.connect(w.record_started, Qt.QueuedConnection)
        s.record_stopped.connect(w.record_stopped, Qt.QueuedConnection)

        w.scan_ready.connect(self._on_scan_ready)
        w.stats.connect(self._stats.update_stats)
        w.status_changed.connect(s.set_state)
        w.status_changed.connect(self._on_status)
        w.error_occurred.connect(self._on_error)

    # ----- handlers -----

    def _on_connect_clicked(self, port: str) -> None:
        # Debounce: disable connect immediately so rapid clicks can't enqueue
        # multiple open_device calls (each would leak a serial handle).
        self._sidebar.set_connect_busy(True)
        self._request_open.emit(port)

    def _on_stop(self) -> None:
        # Direct, NOT a Qt slot — must bypass queued dispatch because scan loop
        # blocks the worker's event loop.
        self._worker._stop_event.set()

    def _on_refresh(self) -> None:
        self._sidebar.set_ports(list_serial_ports())

    def _on_scan_ready(self, xy) -> None:
        self._plot.update_points(xy)
        if not self._first_scan_seen:
            self._first_scan_seen = True
            self._sidebar.enable_snapshot()

    def _on_status(self, state: str) -> None:
        self._sidebar.set_connect_busy(False)
        if state == "disconnected":
            self._plot.clear_points()
            self._first_scan_seen = False

    def _on_error(self, msg: str) -> None:
        self._banner.show_error(msg)
        self._sidebar.set_connect_busy(False)

    def _on_snapshot(self) -> None:
        default = f"scan_{datetime.now().strftime('%Y%m%d_%H%M%S')}.png"
        path, _ = QFileDialog.getSaveFileName(self, "Save snapshot", default, "PNG (*.png)")
        if not path:
            return
        # Render at device-pixel resolution so Retina captures are sharp.
        dpr = self.devicePixelRatioF()
        logical = self._plot.size()
        physical = QSize(int(logical.width() * dpr), int(logical.height() * dpr))
        pixmap = QPixmap(physical)
        pixmap.setDevicePixelRatio(dpr)
        pixmap.fill(Qt.transparent)
        painter = QPainter(pixmap)
        self._plot.render(painter)
        painter.end()
        pixmap.save(path, "PNG")
        self._banner.show_info(f"Saved {path}")

    # ----- shutdown -----

    def closeEvent(self, event: QCloseEvent) -> None:
        # 1. Signal scan loop to exit.
        self._worker._stop_event.set()
        # 2. Queue shutdown on the worker thread. The worker's event loop will
        #    dispatch close_device once start_scan returns (which happens once
        #    the stop_event is observed at the next iteration boundary).
        QMetaObject.invokeMethod(self._worker, "close_device", Qt.QueuedConnection)
        # 3. Ask the worker thread's event loop to quit after close_device runs.
        self._thread.quit()
        # 4. Wait for thread to finish. SERIAL_TIMEOUT_S=1 means worst case ~1s
        #    blocked in serial read + small overhead — give 3s budget.
        finished = self._thread.wait(3000)
        if not finished:
            # Last-resort: avoid "QThread destroyed while running" segfault.
            self._thread.terminate()
            self._thread.wait(500)
        event.accept()
