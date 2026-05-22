"""Serial port enumeration with macOS CP210x filter."""
from __future__ import annotations

from serial.tools import list_ports

_ALLOWED_PREFIXES = (
    "/dev/cu.usbserial",
    "/dev/cu.SLAB_USBtoUART",
    "/dev/cu.usbmodem",
)


def list_serial_ports() -> list[str]:
    """Return sorted list of likely-RPLIDAR serial device paths on macOS."""
    return sorted(
        p.device
        for p in list_ports.comports()
        if p.device.startswith(_ALLOWED_PREFIXES)
    )
