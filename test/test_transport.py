"""
Unit tests for the transport layer.

Covers the mock transport (used by the device tests and later by dry runs) and
the error wrapping of the real serial transport.  No device is needed.
"""

from ik_pressure_sensor import transport as t
import pytest


# --------------------------------------------------------------------------- #
# MockTransport
# --------------------------------------------------------------------------- #

def test_mock_transport_serves_scripted_responses_in_order():
    mock = t.MockTransport([b'\x01\x02', b'\x03'])
    mock.open_port()
    assert mock.is_open
    assert mock.read(2, 0.1) == b'\x01\x02'
    assert mock.read(2, 0.1) == b'\x03'
    assert mock.read(1, 0.1) == b''
    mock.close()
    assert not mock.is_open
    assert mock.opened == 1
    assert mock.closed == 1


def test_mock_transport_records_written_bytes():
    mock = t.MockTransport()
    mock.open_port()
    mock.write(b'\x01\x03')
    mock.write(b'\x02')
    assert mock.written == [b'\x01\x03', b'\x02']


def test_mock_transport_keeps_leftover_for_later_reads():
    mock = t.MockTransport([b'\xAA\xBB\xCC'])
    mock.open_port()
    assert mock.read(2, 0.1) == b'\xAA\xBB'
    assert mock.read(2, 0.1) == b'\xCC'


def test_mock_transport_reset_input_discards_leftover():
    mock = t.MockTransport([b'\xAA\xBB\xCC'])
    mock.open_port()
    assert mock.read(2, 0.1) == b'\xAA\xBB'
    mock.reset_input()
    assert mock.read(2, 0.1) == b''


def test_mock_transport_queue_response():
    mock = t.MockTransport()
    mock.open_port()
    assert mock.pending_responses == 0
    mock.queue_response(b'\x01')
    assert mock.pending_responses == 1
    assert mock.read(1, 0.1) == b'\x01'


def test_mock_transport_can_fail_writes():
    mock = t.MockTransport()
    mock.open_port()
    mock.write_error = t.TransportError('boom')
    with pytest.raises(t.TransportError):
        mock.write(b'\x00')


def test_mock_transport_zero_size_read():
    mock = t.MockTransport([b'\x01'])
    mock.open_port()
    assert mock.read(0, 0.1) == b''
    assert mock.read(-1, 0.1) == b''


# --------------------------------------------------------------------------- #
# SerialTransport
# --------------------------------------------------------------------------- #

def test_serial_transport_rejects_non_positive_timeout():
    with pytest.raises(ValueError):
        t.SerialTransport('COM1', 9600, timeout=0)


def test_serial_transport_reports_unopenable_port():
    pytest.importorskip('serial')
    transport = t.SerialTransport('COM_DOES_NOT_EXIST_42', 9600, timeout=0.05)
    assert not transport.is_open
    with pytest.raises(t.TransportError):
        transport.open_port()
    assert not transport.is_open


def test_serial_transport_read_write_require_open_port():
    pytest.importorskip('serial')
    transport = t.SerialTransport('COM_DOES_NOT_EXIST_42', 9600, timeout=0.05)
    with pytest.raises(t.TransportError):
        transport.write(b'\x00')
    with pytest.raises(t.TransportError):
        transport.read(1, 0.05)
    # Flushing a closed port is a no-op rather than an error.
    transport.reset_input()


def test_serial_transport_exposes_configuration():
    transport = t.SerialTransport('/dev/ttyUSB0', 19200, timeout=0.5)
    assert transport.port == '/dev/ttyUSB0'
    assert transport.baudrate == 19200
    assert not transport.is_open
