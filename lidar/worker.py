"""LidarWorker — QObject that drives pyrplidar and emits scan data."""
from __future__ import annotations

import threading
import time
from datetime import datetime, timezone
from typing import Optional

import numpy as np
from PySide6.QtCore import QObject, Signal, Slot

import serial
from pyrplidar import PyRPlidar

from .recorder import CsvRecorder
from .transform import filter_scan, polar_to_xy

C1_BAUDRATE = 460800
SERIAL_TIMEOUT_S = 1
SCAN_WATCHDOG_S = 2.0  # max time without seeing a start_flag before declaring stall
MOTOR_PWM = 660       # default motor PWM (A-series + C1)
MOTOR_SPINUP_S = 1.2  # let motor reach steady RPM before first scan


class LidarWorker(QObject):
    scan_ready = Signal(object)        # np.ndarray of shape (N, 2) float32
    stats = Signal(float, int)         # hz, n_points
    status_changed = Signal(str)       # "connected" | "connected (warning)" | "scanning" | "connected (idle)" | "disconnected"
    error_occurred = Signal(str)

    def __init__(self) -> None:
        super().__init__()
        self._lidar: Optional[PyRPlidar] = None
        self._stop_event = threading.Event()
        self._recorder: Optional[CsvRecorder] = None
        self._scan_idx = 0
        self._last_emit = 0.0
        self._hz_ema = 0.0

    # ----- lifecycle slots -----

    @Slot(str)
    def open_device(self, port: str) -> None:
        if self._lidar is not None:
            # Idempotent: ignore re-entrant open.
            return
        lidar = PyRPlidar()
        try:
            lidar.connect(port=port, baudrate=C1_BAUDRATE, timeout=SERIAL_TIMEOUT_S)
        except serial.SerialException as exc:
            if getattr(exc, "errno", None) == 13:
                self.error_occurred.emit("Permission denied — check cable / driver install")
            else:
                self.error_occurred.emit(f"Cannot open {port}")
            return
        except Exception as exc:
            self.error_occurred.emit(f"Cannot open {port}: {exc}")
            return

        self._lidar = lidar
        try:
            info = lidar.get_info()
        except Exception:
            self.error_occurred.emit(f"Device on {port} not RPLIDAR")
            self._shutdown_driver()
            return

        try:
            health = lidar.get_health()
        except Exception:
            health = None

        if health is not None and health.status == 2:
            self.error_occurred.emit("Device health ERROR — restart device")
            self._shutdown_driver()
            return

        if health is not None and health.status == 1:
            self.status_changed.emit("connected (warning)")
        else:
            self.status_changed.emit("connected")

    @Slot()
    def close_device(self) -> None:
        self._stop_event.set()
        self._shutdown_driver()
        self.status_changed.emit("disconnected")

    @Slot(str)
    def record_started(self, path: str) -> None:
        if self._recorder is not None:
            self._recorder.stop()
        self._recorder = CsvRecorder(path)
        self._recorder.start()

    @Slot()
    def record_stopped(self) -> None:
        if self._recorder is not None:
            self._recorder.stop()
            self._recorder = None

    @Slot()
    def start_scan(self) -> None:
        if self._lidar is None:
            self.error_occurred.emit("Not connected")
            return
        self._stop_event.clear()
        self._last_emit = time.perf_counter()
        self._hz_ema = 0.0
        self._scan_idx = 0

        # Spin up motor. RPLIDAR motor is controlled via DTR + optional PWM.
        # Without this, start_scan() returns a descriptor but no measurements
        # ever arrive — serial reads time out and parsing yields IndexError.
        try:
            self._lidar.set_motor_pwm(MOTOR_PWM)
        except Exception as exc:
            self.error_occurred.emit(f"Failed to start motor: {exc!r}")
            self._shutdown_driver()
            self.status_changed.emit("disconnected")
            return
        time.sleep(MOTOR_SPINUP_S)

        try:
            scan_gen = self._lidar.start_scan()
        except Exception as exc:
            self.error_occurred.emit(f"Failed to start scan: {exc!r}")
            self._safe_stop_motor()
            self._shutdown_driver()
            self.status_changed.emit("disconnected")
            return

        # Emit scanning only after device confirms start_scan accepted.
        self.status_changed.emit("scanning")

        try:
            self._run_scan_loop(scan_gen)
        except Exception as exc:
            self.error_occurred.emit(f"Lidar disconnected: {exc!r}")
            self._safe_stop_motor()
            self._shutdown_driver()
            self.status_changed.emit("disconnected")
            return
        finally:
            self._safe_stop()
            self._safe_stop_motor()

        if self._lidar is not None:
            self.status_changed.emit("connected (idle)")

    # ----- hot loop -----

    def _run_scan_loop(self, scan_gen) -> None:
        """Iterate per-measurement, accumulate into full scans, emit per rotation.

        Watchdog: if no `start_flag=True` arrives within SCAN_WATCHDOG_S, raise
        — covers the case where pyrplidar silently stalls on serial read after
        an unplug.
        """
        cur_a: list[float] = []
        cur_r: list[float] = []
        cur_q: list[int] = []
        last_boundary = time.perf_counter()

        for m in scan_gen():
            if self._stop_event.is_set():
                break
            now = time.perf_counter()
            if m.start_flag:
                if cur_a:
                    self._emit_scan(cur_a, cur_r, cur_q)
                    cur_a, cur_r, cur_q = [], [], []
                last_boundary = now
            elif now - last_boundary > SCAN_WATCHDOG_S:
                raise RuntimeError("Lidar stalled (no rotation boundary)")
            cur_a.append(m.angle)
            cur_r.append(m.distance)
            cur_q.append(m.quality)

        # Emit any buffered final scan on clean stop.
        if cur_a:
            self._emit_scan(cur_a, cur_r, cur_q)

    def _emit_scan(
        self, angles_deg: list[float], distances_mm: list[float], qualities: list[int]
    ) -> None:
        a_deg = np.asarray(angles_deg, dtype=np.float32)
        r_mm = np.asarray(distances_mm, dtype=np.float32)
        q = np.asarray(qualities, dtype=np.uint8)
        a_rad = np.deg2rad(a_deg)
        r_m = r_mm / 1000.0
        a_rad, r_m, q = filter_scan(a_rad, r_m, q)
        # Reconstruct filtered degree array directly (no rad→deg round trip).
        a_deg_kept = np.rad2deg(a_rad) if len(a_rad) else a_rad
        if len(a_rad) == 0:
            return
        x, y = polar_to_xy(a_rad, r_m)
        xy = np.column_stack([x, y]).astype(np.float32)
        self.scan_ready.emit(xy)

        now = time.perf_counter()
        dt = now - self._last_emit
        self._last_emit = now
        if dt > 0:
            self._hz_ema = 0.9 * self._hz_ema + 0.1 * (1.0 / dt)
        self.stats.emit(self._hz_ema, len(x))

        if self._recorder is not None:
            self._recorder.write(
                scan_index=self._scan_idx,
                timestamp_iso=datetime.now(timezone.utc).isoformat(),
                angles_deg=a_deg_kept,
                distances_m=r_m,
                qualities=q,
            )
        self._scan_idx += 1

    # ----- helpers -----

    def _safe_stop(self) -> None:
        if self._lidar is not None:
            try:
                self._lidar.stop()
            except Exception:
                pass

    def _safe_stop_motor(self) -> None:
        if self._lidar is not None:
            try:
                self._lidar.set_motor_pwm(0)
            except Exception:
                pass

    def _shutdown_driver(self) -> None:
        self._safe_stop()
        self._safe_stop_motor()
        if self._lidar is not None:
            try:
                self._lidar.disconnect()
            except Exception:
                pass
            self._lidar = None
        if self._recorder is not None:
            try:
                self._recorder.stop()
            except Exception:
                pass
            self._recorder = None
