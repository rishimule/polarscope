# CLAUDE.md

Guidance for Claude Code (and other AI agents) working in this repository.

## Project

Dark-mode macOS desktop app for live visualization of Slamtec RPLIDAR C1 scans. PySide6 (Qt) + pyqtgraph + numpy. Python 3.10+.

Entry point: `main.py`. Run with `python main.py` from an activated `.venv`.

## Architecture

- `lidar/worker.py` — `LidarWorker` `QObject`. Owns the `pyrplidar.PyRPlidar` handle. Lives on a `QThread` started in `MainWindow`. Communicates with UI only via Qt signals/slots.
- `lidar/transform.py` — pure-numpy polar → cartesian + filtering. No Qt, no I/O.
- `lidar/recorder.py` — `CsvRecorder`, append-only per-scan CSV writer.
- `lidar/ports.py` — `pyserial.tools.list_ports`-based discovery, filtered to USB-CDC / CP210x.
- `ui/main_window.py` — `MainWindow`, owns the worker thread, wires sidebar ↔ worker ↔ plot.
- `ui/sidebar.py` — port/connect/scan/snapshot/record controls. Emits intent signals only; no I/O.
- `ui/plot_widget.py` — pyqtgraph polar plot.
- `ui/status_bar.py` — FPS + point-count readout, top banner for errors/info.
- `theme.py` — dark-mode `QPalette` + stylesheet.
- `tools/probe.py` — standalone serial probe for hardware debugging. Not imported by the app.

## Threading model

UI thread owns Qt widgets and the `QThread`. Worker thread owns the serial handle and runs the blocking scan loop. **All worker calls cross the thread boundary via `Qt.QueuedConnection`** — see `MainWindow._wire_signals`.

Exceptions — all rooted in the scan loop blocking the worker's event loop:

1. `_on_stop` and `_on_disconnect_clicked` directly set `worker._stop_event` from the UI thread.
2. `record_started` / `record_stopped` are connected with `Qt.DirectConnection` (not Queued). Cross-thread safety comes from `worker._recorder_lock`, which also wraps the recorder write in `_emit_scan` and the cleanup in `_shutdown_driver`. A QueuedConnection here is a load-bearing bug — the CSV would only ever contain its header because the slot couldn't dispatch mid-scan.
3. `closeEvent` sets `_stop_event`, queues `close_device`, then `thread.wait(3000)` with `terminate()` fallback.

When adding new worker methods that touch worker-owned state mid-scan, follow the recorder pattern: DirectConnection + a lock around the shared state. Otherwise, default to `@Slot(...)` + `QueuedConnection` / `QMetaObject.invokeMethod`.

## Hardware / protocol notes (load-bearing — don't "clean up")

- C1 baud is 460 800. Hard-coded.
- `pyrplidar` opens the serial with `dsrdtr=True`, which engages hardware flow control and **blocks the C1's TX stream**. `LidarWorker.open_device` re-opens the underlying `pyserial.Serial` with `dsrdtr=False`. Do not remove.
- `pyrplidar.scan_generator()` aborts on the first short serial read; the C1 has ~200 ms of startup lag after `SCAN` before measurements stream. The worker bypasses the generator and reads raw 5-byte SCAN frames directly off `lidar_serial._serial`. Do not "simplify" back to `scan_generator()`.
- Startup sequence: `stop()` → `set_motor_pwm(660)` → 1.2 s spin-up → `reset_input_buffer()` → `start_scan()`. The flush is required — leftover descriptor bytes from `info`/`health` cause `PyRPlidarProtocolError` on sync mismatch.
- Watchdog: 4 s on first-data + transient stalls. Lower values cause false disconnects on warm-restart.
- 5-byte frame layout (per Slamtec SCAN response):
  - `b0`: bit0=S, bit1=!S, bits 2–7 = quality
  - `b1`: bit0=C (must be 1), bits 1–7 = angle_q6 low
  - `b2`: angle_q6 high
  - `b3..b4`: distance_q2 little-endian, `/4 = mm`
  - On `S == !S` or `C != 1`: drop one byte, re-sync.

## Conventions

- `from __future__ import annotations` at top of every module.
- Type hints throughout. Prefer `np.ndarray` shape comments in docstrings.
- Coordinate convention: radar — `0 rad = +y` (up), `pi/2 rad = +x` (right). See `transform.polar_to_xy`.
- Range filter: 0.05 m – 12.0 m, quality > 0.
- No global state. No singletons. Everything constructed in `main()` or `MainWindow.__init__`.

## Tooling

- **Prefer `gh` over `git` for anything that touches GitHub.** PRs, issues, releases, repo creation, checks, reviews, comments, gists — use the GitHub CLI. It authenticates once, returns structured output (`--json`), and avoids the browser round-trip. Examples: `gh pr create`, `gh pr view <n> --comments`, `gh issue list`, `gh run watch`, `gh release create`.
- Use plain `git` for local-only operations: `commit`, `branch`, `log`, `diff`, `status`, `rebase`, `stash`, `worktree`. `gh` does not replace these.
- When a task could be done with either a `git` remote command or a `gh` subcommand (e.g., creating a PR vs. pushing a branch and opening one in the browser), pick `gh` — the result is identical but scriptable.

## Testing

```bash
pytest -v
```

- `pytest-qt` for widget tests.
- Worker tests stub `pyrplidar` and the underlying `pyserial.Serial`. Do not hit real hardware in tests.
- `tests/test_transform.py` is pure numpy — fast, deterministic.
- New code should ship with tests under `tests/`. Mirror the source layout.

## What NOT to do

- Do not commit `*.png` or `*.csv` artifacts — `.gitignore` excludes them. Exception: anything under `assets/` (whitelisted for the README demo gif, screenshot, raw recording).
- Do not add per-scan `fh.flush()` to `CsvRecorder.write` — under disk pressure it stalls the scan loop. Rely on `close()` to flush.
- Do not call `pyrplidar` methods from the UI thread.
- Do not catch + swallow exceptions in the scan loop without surfacing via `error_occurred`.
- Do not add a "reconnect on failure" path without an explicit user gesture — silent reconnect masks hardware issues.
- Do not bump the watchdog below 4 s.
- Do not add `Co-Authored-By: Claude ...` (or any AI-attribution) trailer to commit messages. GitHub maps that email to the `claude` user and surfaces it in the repo's Contributors list, which the maintainer does not want. Commit as the maintainer only.
