"""LidarWorker — QObject that drives pyrplidar and emits scan data.

We use pyrplidar for connection / info / health / motor PWM, but bypass
its built-in scan_generator (which aborts on the first short serial read
— the C1 has ~200ms of startup lag after SCAN before measurements stream).
Instead we read raw 5-byte measurement frames directly off the underlying
pyserial.Serial and parse them per the Slamtec SCAN response layout.
"""
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
SERIAL_TIMEOUT_S = 0.5
SCAN_WATCHDOG_S = 4.0  # tolerance for first-data startup delay + transient hiccups
MOTOR_PWM = 660       # default motor PWM (A-series + C1)
MOTOR_SPINUP_S = 1.2  # let motor reach steady RPM before first scan


class LidarWorker(QObject):
    scan_ready = Signal(object)        # np.ndarray of shape (N, 2) float32
    stats = Signal(float, int)         # hz, n_points
    status_changed = Signal(str)       # "connected" | "connected (warning)" | "scanning" | "connected (idle)" | "disconnected"
    error_occurred = Signal(str)
    record_failed = Signal()           # CsvRecorder.start raised — UI should revert recording state

    def __init__(self) -> None:
        super().__init__()
        self._lidar: Optional[PyRPlidar] = None
        self._stop_event = threading.Event()
        self._recorder: Optional[CsvRecorder] = None
        # Guards _recorder against races between the UI thread (which installs /
        # removes the recorder mid-scan via record_started / record_stopped) and
        # the worker thread (which writes to it from inside _emit_scan).
        self._recorder_lock = threading.Lock()
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

        # pyrplidar opens the serial with dsrdtr=True, which engages hardware
        # flow control on DSR/DTR. That blocks the RPLIDAR's TX stream on C1.
        # Re-open without it so measurement bytes flow.
        try:
            lidar.lidar_serial._serial.close()
            lidar.lidar_serial._serial = serial.Serial(
                port=port,
                baudrate=C1_BAUDRATE,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=SERIAL_TIMEOUT_S,
                dsrdtr=False,
            )
        except Exception as exc:
            self.error_occurred.emit(f"Serial reopen failed: {exc}")
            try:
                lidar.disconnect()
            except Exception:
                pass
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
        # Called directly from the UI thread (DirectConnection). Cannot be
        # queued: the scan loop holds the worker's event loop while running,
        # so a QueuedConnection slot would never dispatch mid-scan and the
        # CSV would only contain its header.
        with self._recorder_lock:
            if self._recorder is not None:
                self._recorder.stop()
                self._recorder = None
            recorder = CsvRecorder(path)
            try:
                recorder.start()
            except OSError as exc:
                self.error_occurred.emit(f"Cannot record to {path}: {exc}")
                self.record_failed.emit()
                return
            self._recorder = recorder

    @Slot()
    def record_stopped(self) -> None:
        # Same threading rationale as record_started — DirectConnection from UI.
        with self._recorder_lock:
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

        # Settle the device + spin up motor + flush stale bytes in the OS
        # serial buffer (info/health responses, motor noise). Without the
        # final flush, start_scan()'s descriptor read picks up leftover
        # bytes and pyrplidar raises PyRPlidarProtocolError on sync mismatch.
        try:
            try:
                self._lidar.stop()
            except Exception:
                pass
            time.sleep(0.05)
            self._lidar.set_motor_pwm(MOTOR_PWM)
            time.sleep(MOTOR_SPINUP_S)
            self._flush_serial_input()
        except Exception as exc:
            self.error_occurred.emit(f"Failed to start motor: {exc!r}")
            self._shutdown_driver()
            self.status_changed.emit("disconnected")
            return

        # pyrplidar.start_scan() sends SCAN and reads the response descriptor.
        # We discard the returned generator and read measurement bytes ourselves.
        try:
            self._lidar.start_scan()
        except Exception as exc:
            self.error_occurred.emit(f"Failed to start scan: {exc!r}")
            self._safe_stop_motor()
            self._shutdown_driver()
            self.status_changed.emit("disconnected")
            return

        # Emit scanning only after device confirms start_scan accepted.
        self.status_changed.emit("scanning")

        try:
            ser = self._lidar.lidar_serial._serial
            self._raw_scan_loop(ser)
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

    def _raw_scan_loop(self, ser) -> None:
        """Read 5-byte measurement frames directly from pyserial.

        Slamtec standard SCAN response per measurement:
          byte0: bit0=S, bit1=!S, bits2-7=quality
          byte1: bit0=C (must be 1), bits1-7=angle_q6 low
          byte2: angle_q6 high
          byte3-4: distance_q2 (little-endian, /4 = mm)

        Tolerates partial reads by accumulating into a 5-byte buffer. On
        protocol error (S == !S or C != 1) drops one byte and re-syncs.
        """
        cur_a: list[float] = []
        cur_r: list[float] = []
        cur_q: list[int] = []
        buf = bytearray()
        last_data = time.perf_counter()

        while not self._stop_event.is_set():
            need = 5 - len(buf)
            chunk = ser.read(need)
            if chunk:
                buf.extend(chunk)
                last_data = time.perf_counter()
            else:
                if time.perf_counter() - last_data > SCAN_WATCHDOG_S:
                    raise RuntimeError("Lidar stalled (no data)")
                continue
            if len(buf) < 5:
                continue

            b0, b1 = buf[0], buf[1]
            start_flag = bool(b0 & 0x01)
            inv_start = bool((b0 >> 1) & 0x01)
            check_bit = b1 & 0x01
            if start_flag == inv_start or check_bit != 1:
                # Out of sync; drop one byte and try again.
                buf = buf[1:]
                continue

            quality = b0 >> 2
            angle = ((b1 >> 1) | (buf[2] << 7)) / 64.0
            distance = (buf[3] | (buf[4] << 8)) / 4.0
            buf.clear()

            if start_flag and cur_a:
                self._emit_scan(cur_a, cur_r, cur_q)
                cur_a, cur_r, cur_q = [], [], []
            cur_a.append(angle)
            cur_r.append(distance)
            cur_q.append(quality)

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

        with self._recorder_lock:
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

    def _flush_serial_input(self) -> None:
        """Drop bytes queued in OS serial buffer before issuing SCAN.

        pyrplidar exposes no flush API; reach in once to its underlying
        pyserial.Serial. Without this, leftover descriptor bytes from
        prior commands (info/health) cause PyRPlidarProtocolError on
        sync-byte mismatch.
        """
        if self._lidar is None:
            return
        try:
            self._lidar.lidar_serial._serial.reset_input_buffer()
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
        with self._recorder_lock:
            if self._recorder is not None:
                try:
                    self._recorder.stop()
                except Exception:
                    pass
                self._recorder = None
