from unittest.mock import patch, MagicMock
from lidar.ports import list_serial_ports


def _mock_port(device: str, description: str = "USB Serial"):
    m = MagicMock()
    m.device = device
    m.description = description
    return m


@patch("lidar.ports.list_ports.comports")
def test_includes_cp210x_usbserial(mock_comports):
    mock_comports.return_value = [_mock_port("/dev/cu.usbserial-A50285BI")]
    assert list_serial_ports() == ["/dev/cu.usbserial-A50285BI"]


@patch("lidar.ports.list_ports.comports")
def test_includes_legacy_slab(mock_comports):
    mock_comports.return_value = [_mock_port("/dev/cu.SLAB_USBtoUART")]
    assert list_serial_ports() == ["/dev/cu.SLAB_USBtoUART"]


@patch("lidar.ports.list_ports.comports")
def test_includes_usbmodem(mock_comports):
    mock_comports.return_value = [_mock_port("/dev/cu.usbmodem14201")]
    assert list_serial_ports() == ["/dev/cu.usbmodem14201"]


@patch("lidar.ports.list_ports.comports")
def test_excludes_bluetooth(mock_comports):
    mock_comports.return_value = [
        _mock_port("/dev/cu.Bluetooth-Incoming-Port"),
        _mock_port("/dev/cu.usbserial-A50285BI"),
    ]
    assert list_serial_ports() == ["/dev/cu.usbserial-A50285BI"]


@patch("lidar.ports.list_ports.comports")
def test_sorted(mock_comports):
    mock_comports.return_value = [
        _mock_port("/dev/cu.usbserial-B"),
        _mock_port("/dev/cu.usbserial-A"),
    ]
    assert list_serial_ports() == ["/dev/cu.usbserial-A", "/dev/cu.usbserial-B"]


@patch("lidar.ports.list_ports.comports")
def test_empty(mock_comports):
    mock_comports.return_value = []
    assert list_serial_ports() == []
