"""Worker tests with mocked pyrplidar driver + fake pyserial for scan loop."""
from unittest.mock import MagicMock, patch

import numpy as np
import pytest
from PySide6.QtCore import QCoreApplication

from lidar.worker import LidarWorker


@pytest.fixture(scope="module")
def qapp():
    app = QCoreApplication.instance() or QCoreApplication([])
    yield app


def _encode_measurement(start_flag: bool, quality: int, angle_deg: float, distance_mm: float) -> bytes:
    s = 1 if start_flag else 0
    inv = 0 if start_flag else 1
    b0 = (quality << 2) | (inv << 1) | s
    angle_q6 = int(angle_deg * 64) & 0x7FFF
    b1 = ((angle_q6 & 0x7F) << 1) | 0x01
    b2 = (angle_q6 >> 7) & 0xFF
    dist_q2 = int(distance_mm * 4) & 0xFFFF
    b3 = dist_q2 & 0xFF
    b4 = (dist_q2 >> 8) & 0xFF
    return bytes([b0, b1, b2, b3, b4])


def _encode_stream(measurements: list[tuple[bool, int, float, float]]) -> bytes:
    return b"".join(_encode_measurement(*m) for m in measurements)


class _FakeSerial:
    """Minimal pyserial.Serial substitute."""

    def __init__(self, data: bytes = b"", stop_callback=None, stop_after_reads: int | None = None):
        self._data = data
        self._pos = 0
        self._reads = 0
        self._stop_callback = stop_callback
        self._stop_after_reads = stop_after_reads
        self.timeout = 0.5
        self.dsrdtr = False

    def read(self, n: int) -> bytes:
        self._reads += 1
        if self._stop_after_reads is not None and self._reads >= self._stop_after_reads:
            if self._stop_callback:
                self._stop_callback()
        chunk = self._data[self._pos:self._pos + n]
        self._pos += len(chunk)
        return chunk  # may be b"" on exhaustion → mimics timeout

    def close(self):
        pass

    def reset_input_buffer(self):
        pass


def _make_lidar(model=24, status=0) -> MagicMock:
    """Fake PyRPlidar that succeeds on connect/info/health/motor."""
    lidar = MagicMock()
    lidar.get_info.return_value = MagicMock(
        model=model, firmware_minor=0, firmware_major=1, hardware=1, serialnumber="ABC123"
    )
    lidar.get_health.return_value = MagicMock(status=status, error_code=0)
    lidar.lidar_serial = MagicMock()
    lidar.lidar_serial._serial = _FakeSerial()
    return lidar


@pytest.fixture
def serial_holder():
    """Container for the fake serial the test wants serial.Serial() to return.

    Each test sets `holder['serial']` before calling worker.open_device.
    """
    holder: dict[str, _FakeSerial] = {"serial": _FakeSerial()}
    with patch("lidar.worker.serial.Serial", side_effect=lambda *a, **kw: holder["serial"]):
        yield holder


def test_open_device_emits_connected(qapp, serial_holder):
    fake = _make_lidar()
    with patch("lidar.worker.PyRPlidar", return_value=fake):
        worker = LidarWorker()
        states = []
        worker.status_changed.connect(states.append)
        worker.open_device("/dev/cu.usbserial-FAKE")
        QCoreApplication.processEvents()
    assert "connected" in states
    fake.connect.assert_called_once()


def test_open_device_health_error_emits_error(qapp, serial_holder):
    fake = _make_lidar(status=2)
    with patch("lidar.worker.PyRPlidar", return_value=fake):
        worker = LidarWorker()
        errors = []
        worker.error_occurred.connect(errors.append)
        worker.open_device("/dev/cu.usbserial-FAKE")
        QCoreApplication.processEvents()
    assert any("health" in e.lower() for e in errors)


def test_scan_loop_emits_one_signal_per_full_rotation(qapp, serial_holder):
    meas = [
        (True, 40, 0.0, 1000.0),
        (False, 40, 90.0, 2000.0),
        (False, 40, 180.0, 3000.0),
        (True, 40, 0.0, 1100.0),
        (False, 40, 90.0, 2100.0),
    ]
    worker = LidarWorker()
    serial_holder["serial"] = _FakeSerial(
        _encode_stream(meas),
        stop_callback=worker._stop_event.set,
        stop_after_reads=len(meas) + 2,
    )
    fake = _make_lidar()
    with patch("lidar.worker.PyRPlidar", return_value=fake), \
         patch("lidar.worker.MOTOR_SPINUP_S", 0.0), \
         patch("lidar.worker.SCAN_WATCHDOG_S", 5.0):
        worker.open_device("/dev/cu.usbserial-FAKE")
        QCoreApplication.processEvents()
        received = []
        worker.scan_ready.connect(lambda xy: received.append(xy))
        worker.start_scan()
    assert len(received) >= 1
    xy = received[0]
    assert xy.shape == (3, 2)
    assert abs(xy[0, 0]) < 1e-2
    assert abs(xy[0, 1] - 1.0) < 1e-2


def test_unplug_during_scan_emits_disconnected(qapp, serial_holder):
    worker = LidarWorker()
    serial_holder["serial"] = _FakeSerial(b"")
    fake = _make_lidar()
    with patch("lidar.worker.PyRPlidar", return_value=fake), \
         patch("lidar.worker.MOTOR_SPINUP_S", 0.0), \
         patch("lidar.worker.SCAN_WATCHDOG_S", 0.05):
        worker.open_device("/dev/cu.usbserial-FAKE")
        QCoreApplication.processEvents()
        errors = []
        states = []
        worker.error_occurred.connect(errors.append)
        worker.status_changed.connect(states.append)
        worker.start_scan()
    assert any("disconnect" in e.lower() or "stall" in e.lower() for e in errors)
    assert "disconnected" in states


def test_close_device_disconnects(qapp, serial_holder):
    fake = _make_lidar()
    with patch("lidar.worker.PyRPlidar", return_value=fake):
        worker = LidarWorker()
        worker.open_device("/dev/cu.usbserial-FAKE")
        QCoreApplication.processEvents()
        worker.close_device()
    fake.disconnect.assert_called()


def test_open_device_is_idempotent(qapp, serial_holder):
    fake = _make_lidar()
    with patch("lidar.worker.PyRPlidar", return_value=fake):
        worker = LidarWorker()
        worker.open_device("/dev/cu.usbserial-FAKE")
        QCoreApplication.processEvents()
        worker.open_device("/dev/cu.usbserial-FAKE")
    assert fake.connect.call_count == 1


def test_final_partial_scan_emits_on_clean_stop(qapp, serial_holder):
    meas = [
        (True, 40, 0.0, 1000.0),
        (False, 40, 90.0, 2000.0),
        (False, 40, 180.0, 3000.0),
    ]
    worker = LidarWorker()
    serial_holder["serial"] = _FakeSerial(
        _encode_stream(meas),
        stop_callback=worker._stop_event.set,
        stop_after_reads=len(meas),
    )
    fake = _make_lidar()
    with patch("lidar.worker.PyRPlidar", return_value=fake), \
         patch("lidar.worker.MOTOR_SPINUP_S", 0.0), \
         patch("lidar.worker.SCAN_WATCHDOG_S", 5.0):
        worker.open_device("/dev/cu.usbserial-FAKE")
        QCoreApplication.processEvents()
        received = []
        worker.scan_ready.connect(lambda xy: received.append(xy))
        worker.start_scan()
    assert len(received) >= 1
    assert received[-1].shape == (3, 2)


def test_health_warning_emits_warning_state(qapp, serial_holder):
    fake = _make_lidar(status=1)
    with patch("lidar.worker.PyRPlidar", return_value=fake):
        worker = LidarWorker()
        states = []
        worker.status_changed.connect(states.append)
        worker.open_device("/dev/cu.usbserial-FAKE")
        QCoreApplication.processEvents()
    assert "connected (warning)" in states


def test_open_device_permission_denied_emits_specific_error(qapp, serial_holder):
    import serial
    fake = MagicMock()
    exc = serial.SerialException("permission denied")
    exc.errno = 13
    fake.connect.side_effect = exc
    with patch("lidar.worker.PyRPlidar", return_value=fake):
        worker = LidarWorker()
        errors = []
        worker.error_occurred.connect(errors.append)
        worker.open_device("/dev/cu.usbserial-FAKE")
    assert any("permission denied" in e.lower() for e in errors)


def test_open_device_serial_failure_emits_cannot_open(qapp, serial_holder):
    import serial
    fake = MagicMock()
    fake.connect.side_effect = serial.SerialException("device not present")
    with patch("lidar.worker.PyRPlidar", return_value=fake):
        worker = LidarWorker()
        errors = []
        worker.error_occurred.connect(errors.append)
        worker.open_device("/dev/cu.usbserial-MISSING")
    assert any("cannot open" in e.lower() for e in errors)


def test_record_started_unwritable_path_emits_record_failed(qapp, tmp_path):
    worker = LidarWorker()
    failures: list[None] = []
    errors: list[str] = []
    worker.record_failed.connect(lambda: failures.append(None))
    worker.error_occurred.connect(errors.append)
    bogus = str(tmp_path / "nonexistent_dir" / "scan.csv")
    worker.record_started(bogus)
    assert len(failures) == 1
    assert any("cannot record" in e.lower() for e in errors)
    assert worker._recorder is None


def test_recording_started_mid_scan_writes_rows(qapp, serial_holder, tmp_path):
    """Regression: record_started used to be a QueuedConnection slot that
    couldn't dispatch while the scan loop held the worker event loop, so the
    CSV ended up containing only its header. Worker must accept the recorder
    handoff synchronously and start writing rows on the next emitted scan."""
    meas = [
        (True, 40, 0.0, 1000.0),
        (False, 40, 90.0, 2000.0),
        (False, 40, 180.0, 3000.0),
        (True, 40, 0.0, 1100.0),  # boundary => emit first scan
        (False, 40, 90.0, 2100.0),
        (False, 40, 180.0, 3100.0),
        (True, 40, 0.0, 1200.0),  # boundary => emit second scan
        (False, 40, 90.0, 2200.0),
    ]
    worker = LidarWorker()
    csv_path = tmp_path / "midscan.csv"

    def install_recorder_after_first_read():
        # Mimics the UI thread calling record_started while the scan loop is
        # running. The worker_thread will be inside ser.read() / processing.
        worker.record_started(str(csv_path))

    serial_holder["serial"] = _FakeSerial(
        _encode_stream(meas),
        stop_callback=lambda: (install_recorder_after_first_read(), worker._stop_event.set()),
        stop_after_reads=len(meas) + 1,
    )
    fake = _make_lidar()
    with patch("lidar.worker.PyRPlidar", return_value=fake), \
         patch("lidar.worker.MOTOR_SPINUP_S", 0.0), \
         patch("lidar.worker.SCAN_WATCHDOG_S", 5.0):
        worker.open_device("/dev/cu.usbserial-FAKE")
        QCoreApplication.processEvents()
        worker.start_scan()
        # Flush + close the CSV (mimics user clicking Stop Recording after scan).
        worker.record_stopped()

    # CSV must contain header + at least one data row (final partial-scan flush
    # on clean stop, with recorder installed before that emit).
    lines = csv_path.read_text().strip().splitlines()
    assert lines[0].startswith("timestamp_iso,scan_index,angle_deg")
    assert len(lines) > 1, f"only header written; full file: {lines!r}"
