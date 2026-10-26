"""
Unit tests for the device layer.

Every scenario is driven through :class:`MockTransport`, so the retry policy,
the framing rules and the statistics are verified without a sensor on the bus.
"""

from ik_pressure_sensor import modbus_rtu as codec
from ik_pressure_sensor import transport as tr
from ik_pressure_sensor.device import (
    DeviceCommunicationError,
    DeviceTimeout,
    PressureSensor,
)
import pytest

GOOD = bytes.fromhex('01 03 04 26 F7 01 41 81 29')
GOOD_SINGLE = bytes.fromhex('01 03 02 26 F6 22 62')
IDENTITY = bytes.fromhex('01 03 04 27 1B 19 02 0A D1')

REQUEST = bytes.fromhex('01 03 01 00 00 02 C5 F7')
IDENTITY_REQUEST = bytes.fromhex('01 03 FE 00 00 02 F5 E3')


def _frame(pdu_hex: str) -> bytes:
    pdu = bytes.fromhex(pdu_hex)
    return pdu + codec.crc16_bytes(pdu)


def _sensor(responses=(), **kwargs):
    mock = tr.MockTransport(responses)
    mock.open()
    return PressureSensor(mock, **kwargs), mock


def _corrupt(frame: bytes) -> bytes:
    return frame[:-1] + bytes((frame[-1] ^ 0xFF,))


# --------------------------------------------------------------------------- #
# Happy path
# --------------------------------------------------------------------------- #

def test_read_returns_decoded_reading_and_sends_expected_request():
    sensor, mock = _sensor([GOOD])
    reading = sensor.read()

    assert reading.pressure_pa == pytest.approx(99750.0)
    assert reading.temperature_c == pytest.approx(32.1)
    assert mock.written == [REQUEST]
    assert sensor.stats.queries == 1
    assert sensor.stats.successes == 1
    assert sensor.stats.consecutive_failures == 0
    assert sensor.stats.last_error is None


def test_read_identity_reads_model_and_version():
    sensor, mock = _sensor([IDENTITY])
    model, version = sensor.read_identity()

    assert (model, version) == (0x271B, 0x1902)
    assert mock.written == [IDENTITY_REQUEST]


def test_two_reads_use_two_requests():
    sensor, mock = _sensor([GOOD, GOOD])
    sensor.read()
    sensor.read()
    assert mock.written == [REQUEST, REQUEST]
    assert sensor.stats.queries == 2
    assert sensor.stats.successes == 2


# --------------------------------------------------------------------------- #
# Retries
# --------------------------------------------------------------------------- #

def test_corrupt_crc_is_retried():
    sensor, mock = _sensor([_corrupt(GOOD), GOOD], retries=1)
    reading = sensor.read()

    assert reading.pressure_pa == pytest.approx(99750.0)
    assert mock.written == [REQUEST, REQUEST]
    assert sensor.stats.crc_errors == 1
    assert sensor.stats.successes == 1
    assert sensor.stats.consecutive_failures == 0


def test_truncated_frame_is_retried():
    # 4 bytes then silence: the frame never completes, so it times out.
    sensor, mock = _sensor([GOOD[:4], b'', GOOD], retries=1)
    reading = sensor.read()

    assert reading.pressure_pa == pytest.approx(99750.0)
    assert len(mock.written) == 2
    assert sensor.stats.timeouts == 1
    assert sensor.stats.successes == 1


def test_response_from_other_slave_is_retried():
    other_slave = _frame('02 03 04 26 F7 01 41')
    sensor, mock = _sensor([other_slave, GOOD], retries=1)
    reading = sensor.read()

    assert reading.temperature_c == pytest.approx(32.1)
    assert sensor.stats.frame_errors == 1
    assert len(mock.written) == 2


def test_timeout_exhausts_retries_and_reports_cause():
    sensor, mock = _sensor([], retries=2)
    with pytest.raises(DeviceCommunicationError) as excinfo:
        sensor.read()

    assert isinstance(excinfo.value.last_error, DeviceTimeout)
    assert len(mock.written) == 3          # initial attempt + 2 retries
    assert sensor.stats.queries == 1
    assert sensor.stats.timeouts == 3
    assert sensor.stats.consecutive_failures == 1
    assert sensor.stats.last_error is not None


def test_single_attempt_without_retries():
    sensor, mock = _sensor([], retries=0)
    with pytest.raises(DeviceCommunicationError):
        sensor.read()
    assert len(mock.written) == 1
    assert sensor.stats.timeouts == 1


# --------------------------------------------------------------------------- #
# Non retryable and transport level failures
# --------------------------------------------------------------------------- #

def test_device_exception_response_is_not_retried():
    exception_frame = _frame('01 83 02')
    sensor, mock = _sensor([exception_frame], retries=2)

    with pytest.raises(codec.ModbusExceptionResponse) as excinfo:
        sensor.read()

    assert excinfo.value.exception_code == 0x02
    assert len(mock.written) == 1
    assert sensor.stats.exception_responses == 1
    assert sensor.stats.consecutive_failures == 1


def test_transport_error_propagates_without_retry():
    sensor, mock = _sensor([], retries=2)
    mock.write_error = tr.TransportError('port vanished')

    with pytest.raises(tr.TransportError):
        sensor.read()

    assert mock.written == []
    assert sensor.stats.transport_errors == 1
    assert sensor.stats.consecutive_failures == 1


# --------------------------------------------------------------------------- #
# Framing hygiene
# --------------------------------------------------------------------------- #

def test_trailing_garbage_after_valid_frame_is_discarded():
    sensor, mock = _sensor([GOOD + b'\x00\x00', GOOD])

    assert sensor.read().pressure_raw == 9975
    assert sensor.read().pressure_raw == 9975
    assert sensor.stats.successes == 2
    assert sensor.stats.crc_errors == 0


def test_response_with_wrong_register_count_is_retried():
    # A 1-register reply to a 2-register request must be rejected, not decoded.
    sensor, mock = _sensor([GOOD_SINGLE, GOOD], retries=1)
    reading = sensor.read()
    assert reading.pressure_raw == 9975
    assert len(mock.written) == 2
    assert sensor.stats.frame_errors == 1


# --------------------------------------------------------------------------- #
# Recovery and statistics
# --------------------------------------------------------------------------- #

def test_consecutive_failures_reset_after_recovery():
    mock = tr.MockTransport()
    mock.open()
    sensor = PressureSensor(mock, retries=0)

    with pytest.raises(DeviceCommunicationError):
        sensor.read()
    assert sensor.consecutive_failures == 1

    mock.queue_response(GOOD)
    reading = sensor.read()
    assert reading.pressure_pa == pytest.approx(99750.0)
    assert sensor.consecutive_failures == 0
    assert sensor.stats.queries == 2
    assert sensor.stats.successes == 1


# --------------------------------------------------------------------------- #
# Construction
# --------------------------------------------------------------------------- #

@pytest.mark.parametrize('kwargs', [
    {'retries': -1},
    {'timeout': 0},
    {'timeout': -1},
])
def test_invalid_construction_arguments(kwargs):
    with pytest.raises(ValueError):
        PressureSensor(tr.MockTransport(), **kwargs)


def test_properties():
    mock = tr.MockTransport()
    sensor = PressureSensor(mock, slave_id=7, timeout=0.1, retries=1)
    assert sensor.transport is mock
    assert sensor.slave_id == 7
    assert sensor.timeout == 0.1
    assert sensor.retries == 1
