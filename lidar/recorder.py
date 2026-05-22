"""CSV recording for live lidar scans."""
from __future__ import annotations

import csv
from typing import Optional

import numpy as np

HEADER = ["timestamp_iso", "scan_index", "angle_deg", "distance_m", "quality"]


class CsvRecorder:
    def __init__(self, path: str) -> None:
        self._path = path
        self._fh = None
        self._writer: Optional["csv._writer"] = None

    def start(self) -> None:
        self._fh = open(self._path, "w", newline="", encoding="utf-8")
        self._writer = csv.writer(self._fh)
        self._writer.writerow(HEADER)
        self._fh.flush()

    def write(
        self,
        scan_index: int,
        timestamp_iso: str,
        angles_deg: np.ndarray,
        distances_m: np.ndarray,
        qualities: np.ndarray,
    ) -> None:
        if self._writer is None:
            raise RuntimeError("CsvRecorder.write called before start")
        rows = (
            (timestamp_iso, scan_index, float(a), float(d), int(q))
            for a, d, q in zip(angles_deg, distances_m, qualities)
        )
        self._writer.writerows(rows)
        # Note: deliberate no-flush. Per-scan flush blocked the scan loop under
        # disk pressure; rely on close() to flush. Crash-mid-record may lose
        # the trailing OS-buffered window, acceptable for visualization use.

    def stop(self) -> None:
        if self._fh is not None:
            try:
                self._fh.close()
            finally:
                self._fh = None
                self._writer = None
