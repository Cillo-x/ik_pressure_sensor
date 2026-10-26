"""
Modbus-RTU codec for the IKMSIK RS485 barometric pressure transmitter.

Everything here is a pure function over ``bytes``: no serial port, no ROS,
no I/O of any kind.  That is deliberate.  It keeps the protocol layer
unit-testable on any machine (including a Windows development box with no
ROS installed) and lets the serial transport be replaced by a mock without
touching the device logic.

Device facts encoded below (protocol reference: the vendor manual V2.1 and
the frame captures shipped with it):

* Modbus-RTU, 8N1, default slave id 1, default baudrate 9600 bit/s.
* Function codes ``0x03`` (read holding registers) and ``0x06``
  (write single register).  Nothing else is supported by the device.
* Register ``0x0100``: pressure, uint16, 1 LSB = 10 Pa.
* Register ``0x0101``: temperature, int16, 1 LSB = 0.1 degC  (not documented
  by the vendor, confirmed only from frame captures).
* CRC-16/MODBUS (polynomial 0xA001 reflected, init 0xFFFF), transmitted with
  the **low byte first**.
"""

from __future__ import annotations

from dataclasses import dataclass
import struct
from typing import Tuple

__all__ = [
    'BAUDRATE_CODES',
    'BAUDRATE_FROM_CODE',
    'CrcError',
    'DEFAULT_BAUDRATE',
    'DEFAULT_SLAVE_ID',
    'EXCEPTION_BIT',
    'FC_READ_HOLDING',
    'FC_WRITE_SINGLE',
    'FrameError',
    'MAX_SLAVE_ID',
    'MIN_SLAVE_ID',
    'ModbusError',
    'ModbusExceptionResponse',
    'ModbusResponseError',
    'PRESSURE_LSB_PA',
    'PressureReading',
    'REG_BAUDRATE',
    'REG_DEVICE_ADDRESS',
    'REG_PRESSURE',
    'REG_PRESSURE_TEMPERATURE_COUNT',
    'REG_PRODUCT_MODEL',
    'REG_SOFTWARE_VERSION',
    'REG_TEMPERATURE',
    'SlaveMismatchError',
    'SUPPORTED_BAUDRATES',
    'TEMPERATURE_LSB_C',
    'build_read_pressure_temperature_request',
    'build_read_request',
    'build_set_baudrate_request',
    'build_set_slave_address_request',
    'build_write_request',
    'check_crc',
    'crc16',
    'crc16_bytes',
    'decode_pressure',
    'decode_temperature',
    'describe_exception_code',
    'expected_read_response_length',
    'parse_pressure_temperature',
    'parse_read_response',
    'to_signed16',
    'verify_write_response',
]

# --------------------------------------------------------------------------- #
# Protocol constants
# --------------------------------------------------------------------------- #

FC_READ_HOLDING = 0x03
FC_WRITE_SINGLE = 0x06
EXCEPTION_BIT = 0x80

REG_PRESSURE = 0x0100
REG_TEMPERATURE = 0x0101
REG_PRESSURE_TEMPERATURE_COUNT = 2

REG_DEVICE_ADDRESS = 0x07D0
REG_BAUDRATE = 0x07D1
REG_PRODUCT_MODEL = 0xFE00
REG_SOFTWARE_VERSION = 0xFE01

MIN_SLAVE_ID = 1
MAX_SLAVE_ID = 247
DEFAULT_SLAVE_ID = 1
DEFAULT_BAUDRATE = 9600

#: Values written to ``REG_BAUDRATE``.  Note that 1200 bit/s is code 7 and
#: 2400 bit/s is code 0 -- this is easy to get wrong.
BAUDRATE_CODES = {
    2400: 0,
    4800: 1,
    9600: 2,
    19200: 3,
    38400: 4,
    57600: 5,
    115200: 6,
    1200: 7,
}
BAUDRATE_FROM_CODE = {code: rate for rate, code in BAUDRATE_CODES.items()}
SUPPORTED_BAUDRATES = tuple(sorted(BAUDRATE_CODES))

#: One pressure LSB is 10 Pa, so ``raw * 10.0`` yields pascal.
PRESSURE_LSB_PA = 10.0
#: One temperature LSB is 0.1 degC.
TEMPERATURE_LSB_C = 0.1

_MAX_READ_REGISTERS = 125

_EXCEPTION_NAMES = {
    0x01: 'illegal function',
    0x02: 'illegal data address',
    0x03: 'illegal data value',
    0x04: 'slave device failure',
    0x05: 'acknowledge',
    0x06: 'slave device busy',
}


# --------------------------------------------------------------------------- #
# Exceptions
# --------------------------------------------------------------------------- #

class ModbusError(Exception):
    """Base class for every protocol level failure."""


class ModbusResponseError(ModbusError):
    """The bytes received do not form the response that was requested."""


class FrameError(ModbusResponseError):
    """Frame too short, wrong length, wrong byte count or wrong function code."""


class CrcError(ModbusResponseError):
    """The frame CRC does not match its payload."""


class SlaveMismatchError(ModbusResponseError):
    """The response carries a different slave address than requested."""


class ModbusExceptionResponse(ModbusResponseError):
    """The slave answered with an exception frame (function code OR 0x80)."""

    def __init__(  # noqa: B042 - keep str() readable in logs
        self,
        function_code: int,
        exception_code: int,
    ) -> None:
        self.function_code = function_code
        self.exception_code = exception_code
        super().__init__(
            f'exception 0x{exception_code:02X} for function 0x{function_code:02X}'
            f' ({describe_exception_code(exception_code)})'
        )


def describe_exception_code(exception_code: int) -> str:
    """Return a human readable name for a Modbus exception code."""
    return _EXCEPTION_NAMES.get(exception_code, 'unknown exception code')


# --------------------------------------------------------------------------- #
# CRC-16/MODBUS
# --------------------------------------------------------------------------- #

def crc16(data: bytes) -> int:
    """Return the CRC-16/MODBUS of ``data`` as an integer."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc


def crc16_bytes(data: bytes) -> bytes:
    """Return the CRC of ``data`` in wire order (low byte first)."""
    crc = crc16(data)
    return bytes((crc & 0xFF, (crc >> 8) & 0xFF))


def _append_crc(pdu: bytes) -> bytes:
    return pdu + crc16_bytes(pdu)


def check_crc(frame: bytes) -> None:
    """Raise :class:`CrcError` unless the trailing CRC matches ``frame``."""
    if len(frame) < 3:
        raise FrameError(f'frame too short for a CRC: {len(frame)} bytes')
    expected = crc16_bytes(frame[:-2])
    actual = bytes(frame[-2:])
    if expected != actual:
        raise CrcError(
            f'CRC mismatch: expected {expected.hex(" ").upper()},'
            f' got {actual.hex(" ").upper()} (frame {bytes(frame).hex(" ").upper()})'
        )


# --------------------------------------------------------------------------- #
# Argument validation
# --------------------------------------------------------------------------- #

def _check_slave_id(slave_id: int) -> None:
    if not MIN_SLAVE_ID <= slave_id <= MAX_SLAVE_ID:
        raise ValueError(
            f'slave id must be in [{MIN_SLAVE_ID}, {MAX_SLAVE_ID}], got {slave_id}'
        )


def _check_register(register: int) -> None:
    if not 0 <= register <= 0xFFFF:
        raise ValueError(f'register address must fit in uint16, got {register}')


def _check_register_value(value: int) -> None:
    if not 0 <= value <= 0xFFFF:
        raise ValueError(f'register value must fit in uint16, got {value}')


def _check_count(count: int) -> None:
    if not 1 <= count <= _MAX_READ_REGISTERS:
        raise ValueError(
            f'register count must be in [1, {_MAX_READ_REGISTERS}], got {count}'
        )


# --------------------------------------------------------------------------- #
# Request builders
# --------------------------------------------------------------------------- #

def build_read_request(
    slave_id: int = DEFAULT_SLAVE_ID,
    start_register: int = REG_PRESSURE,
    count: int = REG_PRESSURE_TEMPERATURE_COUNT,
) -> bytes:
    """Build a function 0x03 request reading ``count`` registers."""
    _check_slave_id(slave_id)
    _check_register(start_register)
    _check_count(count)
    if start_register + count - 1 > 0xFFFF:
        raise ValueError(
            f'register range {start_register:#06x}+{count} runs past 0xFFFF'
        )
    pdu = struct.pack('>BBHH', slave_id, FC_READ_HOLDING, start_register, count)
    return _append_crc(pdu)


def build_read_pressure_temperature_request(
    slave_id: int = DEFAULT_SLAVE_ID,
) -> bytes:
    """Build the recommended request: pressure + temperature in one round trip."""
    return build_read_request(slave_id, REG_PRESSURE, REG_PRESSURE_TEMPERATURE_COUNT)


def build_write_request(
    slave_id: int,
    register: int,
    value: int,
) -> bytes:
    """Build a function 0x06 request writing one holding register."""
    _check_slave_id(slave_id)
    _check_register(register)
    _check_register_value(value)
    pdu = struct.pack('>BBHH', slave_id, FC_WRITE_SINGLE, register, value)
    return _append_crc(pdu)


def build_set_slave_address_request(slave_id: int, new_address: int) -> bytes:
    """Build the request that changes the device's Modbus slave address."""
    _check_slave_id(new_address)
    return build_write_request(slave_id, REG_DEVICE_ADDRESS, new_address)


def build_set_baudrate_request(slave_id: int, baudrate: int) -> bytes:
    """Build the request that changes the device's baudrate."""
    try:
        code = BAUDRATE_CODES[baudrate]
    except KeyError:
        raise ValueError(
            f'unsupported baudrate {baudrate}, expected one of {SUPPORTED_BAUDRATES}'
        ) from None
    return build_write_request(slave_id, REG_BAUDRATE, code)


# --------------------------------------------------------------------------- #
# Response parsing
# --------------------------------------------------------------------------- #

def expected_read_response_length(count: int) -> int:
    """Total frame length of a function 0x03 response carrying ``count`` registers."""
    _check_count(count)
    return 5 + 2 * count


def parse_read_response(
    frame: bytes,
    slave_id: int = DEFAULT_SLAVE_ID,
    count: int = REG_PRESSURE_TEMPERATURE_COUNT,
) -> Tuple[int, ...]:
    """
    Validate a function 0x03 response and return its register values.

    Raises a subclass of :class:`ModbusError` when the frame is not exactly
    the response to a ``count``-register read from ``slave_id``.
    """
    frame = bytes(frame)
    if len(frame) < 5:
        raise FrameError(f'response too short: {len(frame)} bytes')

    if frame[0] != slave_id:
        raise SlaveMismatchError(
            f'expected slave id {slave_id}, got {frame[0]}'
        )

    function_code = frame[1]
    if function_code == (FC_READ_HOLDING | EXCEPTION_BIT):
        raise ModbusExceptionResponse(FC_READ_HOLDING, frame[2])
    if function_code != FC_READ_HOLDING:
        raise FrameError(
            f'expected function code 0x{FC_READ_HOLDING:02X},'
            f' got 0x{function_code:02X}'
        )

    check_crc(frame)

    byte_count = frame[2]
    if byte_count != 2 * count:
        raise FrameError(
            f'expected byte count {2 * count} for {count} registers,'
            f' got {byte_count}'
        )
    expected_length = expected_read_response_length(count)
    if len(frame) != expected_length:
        raise FrameError(
            f'expected a {expected_length} byte response, got {len(frame)}'
        )

    return struct.unpack(f'>{count}H', frame[3:3 + byte_count])


def verify_write_response(request: bytes, response: bytes) -> None:
    """Check that a function 0x06 response echoes the request (success)."""
    request = bytes(request)
    response = bytes(response)
    if len(request) != 8:
        raise FrameError(f'write request must be 8 bytes, got {len(request)}')
    if len(response) != 8:
        raise FrameError(f'write response must be 8 bytes, got {len(response)}')
    if response[0] != request[0]:
        raise SlaveMismatchError(
            f'expected slave id {request[0]}, got {response[0]}'
        )
    if response[1] == (request[1] | EXCEPTION_BIT):
        raise ModbusExceptionResponse(request[1], response[2])
    check_crc(response)
    if response[:6] != request[:6]:
        raise FrameError(
            f'write echo mismatch: sent {request[:6].hex(" ").upper()},'
            f' got {response[:6].hex(" ").upper()}'
        )


# --------------------------------------------------------------------------- #
# Value decoding
# --------------------------------------------------------------------------- #

def to_signed16(value: int) -> int:
    """Interpret ``value`` as a two's complement 16 bit signed integer."""
    value &= 0xFFFF
    return value - 0x10000 if value & 0x8000 else value


def decode_pressure(raw: int) -> float:
    """Convert a raw pressure register value to pascal."""
    return raw * PRESSURE_LSB_PA


def decode_temperature(raw: int) -> float:
    """Convert a raw temperature register value to degrees Celsius."""
    return to_signed16(raw) * TEMPERATURE_LSB_C


@dataclass(frozen=True)
class PressureReading:
    """One decoded pressure/temperature sample."""

    pressure_pa: float
    temperature_c: float
    pressure_raw: int
    temperature_raw: int


def parse_pressure_temperature(
    frame: bytes,
    slave_id: int = DEFAULT_SLAVE_ID,
) -> PressureReading:
    """Decode a 2-register response into a :class:`PressureReading`."""
    pressure_raw, temperature_raw = parse_read_response(
        frame, slave_id, REG_PRESSURE_TEMPERATURE_COUNT
    )
    return PressureReading(
        pressure_pa=decode_pressure(pressure_raw),
        temperature_c=decode_temperature(temperature_raw),
        pressure_raw=pressure_raw,
        temperature_raw=temperature_raw,
    )
