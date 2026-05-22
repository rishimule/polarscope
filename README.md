# PolarScope — RPLIDAR C1 Live Viewer

A dark-mode macOS desktop app for visualizing live 2D scans from a Slamtec RPLIDAR C1 over USB. Built with PySide6 + pyqtgraph.

![RPLIDAR C1 Viewer screenshot](Screenshot.png)

## Features

- Live polar plot of LIDAR returns at ~10 Hz with FPS and point-count readout
- One-click serial port discovery (filters to USB CDC/CP210x devices)
- Connect / Disconnect / Start / Stop scan lifecycle with status LED
- Save plot snapshot as PNG (Retina-aware, device-pixel resolution)
- Record raw scans to CSV (`timestamp_iso, scan_index, angle_deg, distance_m, quality`)
- Range and quality filtering (5 cm – 12 m, quality > 0)
- Background `QThread` worker — UI never blocks on serial I/O

## Hardware

- Slamtec RPLIDAR **C1** (460 800 baud, USB-C → CP210x UART)
- macOS 11+ ships an Apple-signed CP210x driver; no install needed. Remove any legacy SiLabs kext if present.
- Device enumerates as `/dev/cu.usbserial-*`.

## Setup

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

Requires Python 3.10+.

## Run

```bash
python main.py
```

1. Plug in the C1, click **↻** to refresh, pick the `usbserial` port.
2. **Connect** → **Start Scan**.
3. Use **Save Snapshot** for a PNG of the current frame, or **Record CSV** to log raw measurements.

## Project layout

```
lidar/        # serial worker, port discovery, polar→xy transform, CSV recorder
ui/           # MainWindow, sidebar, plot widget, status bar
theme.py      # dark-mode QPalette + stylesheet
main.py       # entry point
tests/        # pytest + pytest-qt suite
tools/probe.py  # standalone serial probe for debugging
```

## Implementation notes

- **Driver:** uses `pyrplidar` (0.1.2) for connect / info / health / motor PWM, but bypasses its `scan_generator` and reads raw 5-byte SCAN frames directly off `pyserial`. `pyrplidar`'s generator aborts on the first short read, and the C1 has ~200 ms of startup lag after `SCAN` before measurements stream.
- **Serial quirk:** `pyrplidar` opens the port with `dsrdtr=True`, which engages hardware flow control and blocks the C1's TX stream. The worker re-opens the underlying `pyserial.Serial` with `dsrdtr=False` immediately after connect.
- **Startup sequence:** `stop()` → `set_motor_pwm(660)` → 1.2 s spin-up → flush input buffer → `start_scan()`. Without the flush, stale descriptor bytes from prior `info`/`health` commands cause sync-byte mismatches.
- **Watchdog:** 4 s tolerance on first-data delay and transient stalls; raises and surfaces a `Lidar disconnected` error otherwise.
- **Stop path:** scan loop checks a `threading.Event` on every iteration; `closeEvent` drains the worker thread with a 3 s budget before falling back to `terminate()` to avoid `QThread destroyed while running`.

## Tests

```bash
pytest -v
```

`pytest-qt` is used for UI bits. The worker tests stub `pyrplidar` and `pyserial` and exercise the raw-frame parser, filtering, and recorder.

## License

MIT
