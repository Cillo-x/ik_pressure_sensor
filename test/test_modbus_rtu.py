"""
Unit tests for the pure Modbus-RTU codec.

These tests need no ROS installation, no serial port and no hardware, so they
run on any machine with Python and pytest:

    cd ros2_ws/src/ik_pressure_sensor
    python -m pytest test/test_modbus_rtu.py -v

The expected byte strings are copied verbatim from the vendor's serial capture
(``MODBUS通信.txt``), which makes this suite a regression test against the real
device rather than against our own assumptions.
"""

from ik_pressure_sensor import modbus_rtu as m
import pytest


# --------------------------------------------------------------------------- #
# Captured frames
# --------------------------------------------------------------------------- #

#: Every frame below appears verbatim in the vendor capture and has a valid CRC.
#: One line of that file -- the single-register software version read -- was
#: mis-transcribed by the vendor and is excluded; see
#: ``test_vendor_txt_software_version_value_is_mis_transcribed``.
VENDOR_FRAMES = [
    # read pressure + temperature (2 registers), slave 1
    '01 03 01 00 00 02 C5 F7',
    '01 03 04 26 F7 01 41 81 29',
    # read pressure only (1 register)
    '01 03 01 00 00 01 85 F6',
    '01 03 02 26 F6 22 62',
    # read temperature only
    '01 03 02 01 41 79 E4',
    # slave address change to 2, then reads from slave 2
    '01 06 07 D0 00 02 08 86',
    '02 03 01 00 00 02 C5 C4',
    '02 03 04 26 F5 01 41 13 E9',
    # slave address change to 3, then reads from slave 3
    '02 06 07 D0 00 03 C9 75',
    '03 03 01 00 00 02 C4 15',
    '03 03 04 26 F4 01 42 12 E8',
    # back to slave 1
    '03 06 07 D0 00 01 49 65',
    # baudrate changes
    '01 06 07 D1 00 03 98 86',
    '01 06 07 D1 00 02 59 46',
    # product model / software version
    '01 03 FE 00 00 01 B5 E2',
    '01 03 02 27 1B E3 BF',
    '01 03 FE 01 00 01 E4 22',
    '01 03 FE 00 00 02 F5 E3',
    '01 03 04 27 1B 19 02 0A D1',
    # the "PLC" capture that uses the 4xxxx address form directly
    '01 03 9D 41 00 02 BB B3',
    '01 03 04 26 F2 01 45 90 EB',
    '01 03 9D 41 00 01 FB B2',
    '01 03 02 26 F1 63 A0',
    '01 03 9D 42 00 01 0B B2',
    '01 03 02 01 45 78 27',
]


def _hex(text: str) -> bytes:
    return bytes.fromhex(text)


def _with_crc(pdu_hex: str) -> bytes:
    pdu = _hex(pdu_hex)
    return pdu + m.crc16_bytes(pdu)


# --------------------------------------------------------------------------- #
# CRC
# --------------------------------------------------------------------------- #

@pytest.mark.parametrize('frame_hex', VENDOR_FRAMES)
def test_crc_of_vendor_captures(frame_hex):
    frame = _hex(frame_hex)
    assert m.crc16_bytes(frame[:-2]) == frame[-2:]


def test_crc_is_transmitted_low_byte_first():
    body = _hex('01 03 01 00 00 02')
    assert m.crc16(body) == 0xF7C5
    assert m.crc16_bytes(body) == _hex('C5 F7')


def test_vendor_txt_software_version_value_is_mis_transcribed():
    """
    Show why the capture line cannot be taken literally.

    The vendor file contains ``01 03 02 19 04 32 15``.  That frame cannot
    exist: CRC ``32 15`` belongs to payload ``01 03 02 19 02``.  The
    two-register read in the same capture (valid CRC) also reports ``0x1902``,
    so the device software version is 0x1902 and the ``19 04`` data bytes --
    and the "V4.0" comment derived from them -- are transcription errors.
    """
    assert m.crc16_bytes(_hex('01 03 02 19 02')) == _hex('32 15')
    assert m.crc16_bytes(_hex('01 03 02 19 04')) == _hex('B2 17')


def test_check_crc_raises_on_corrupt_frame():
    frame = bytearray(_hex('01 03 04 26 F7 01 41 81 29'))
    frame[-1] ^= 0xFF
    with pytest.raises(m.CrcError):
        m.check_crc(bytes(frame))


# --------------------------------------------------------------------------- #
# Request builders
# --------------------------------------------------------------------------- #

def test_build_read_one_register():
    assert m.build_read_request(1, m.REG_PRESSURE, 1) == _hex('01 03 01 00 00 01 85 F6')


def test_build_read_two_registers():
    assert m.build_read_request(1, m.REG_PRESSURE, 2) == _hex('01 03 01 00 00 02 C5 F7')


def test_build_read_pressure_temperature_request():
    assert m.build_read_pressure_temperature_request() == _hex(
        '01 03 01 00 00 02 C5 F7'
    )


def test_build_read_request_for_other_slave():
    assert m.build_read_request(2, m.REG_PRESSURE, 2) == _hex('02 03 01 00 00 02 C5 C4')


def test_build_read_request_for_product_model():
    assert m.build_read_request(1, m.REG_PRODUCT_MODEL, 2) == _hex(
        '01 03 FE 00 00 02 F5 E3'
    )


def test_build_set_slave_address_request():
    assert m.build_set_slave_address_request(1, 2) == _hex('01 06 07 D0 00 02 08 86')


def test_build_set_baudrate_request():
    assert m.build_set_baudrate_request(1, 19200) == _hex('01 06 07 D1 00 03 98 86')
    assert m.build_set_baudrate_request(1, 9600) == _hex('01 06 07 D1 00 02 59 46')


@pytest.mark.parametrize(
    'kwargs',
    [
        {'slave_id': 0},
        {'slave_id': 248},
        {'count': 0},
        {'count': 126},
        {'start_register': -1},
        {'start_register': 0x10000},
        {'start_register': 0xFFFF, 'count': 2},
    ],
)
def test_build_read_request_rejects_invalid_arguments(kwargs):
    with pytest.raises(ValueError):
        m.build_read_request(**kwargs)


def test_build_set_baudrate_request_rejects_unsupported_rate():
    with pytest.raises(ValueError):
        m.build_set_baudrate_request(1, 1234)


def test_build_set_slave_address_request_rejects_out_of_range():
    with pytest.raises(ValueError):
        m.build_set_slave_address_request(1, 0)


# --------------------------------------------------------------------------- #
# Response parsing
# --------------------------------------------------------------------------- #

def test_parse_single_pressure_register():
    values = m.parse_read_response(_hex('01 03 02 26 F6 22 62'), slave_id=1, count=1)
    assert values == (9974,)


def test_parse_pressure_and_temperature():
    reading = m.parse_pressure_temperature(_hex('01 03 04 26 F7 01 41 81 29'))
    assert reading.pressure_raw == 9975
    assert reading.temperature_raw == 321
    assert reading.pressure_pa == pytest.approx(99750.0)
    assert reading.temperature_c == pytest.approx(32.1)


def test_parse_pressure_and_temperature_from_second_slave():
    reading = m.parse_pressure_temperature(
        _hex('02 03 04 26 F5 01 41 13 E9'), slave_id=2
    )
    assert reading.pressure_pa == pytest.approx(99730.0)
    assert reading.temperature_c == pytest.approx(32.1)


def test_parse_temperature_only():
    values = m.parse_read_response(_hex('01 03 02 01 41 79 E4'), slave_id=1, count=1)
    assert m.decode_temperature(values[0]) == pytest.approx(32.1)


def test_parse_product_model_and_software_version():
    values = m.parse_read_response(
        _hex('01 03 04 27 1B 19 02 0A D1'), slave_id=1, count=2
    )
    assert values == (0x271B, 0x1902)


def test_parse_response_from_wrong_slave_raises():
    with pytest.raises(m.SlaveMismatchError):
        m.parse_read_response(_hex('01 03 02 26 F6 22 62'), slave_id=2, count=1)


def test_parse_short_frame_raises():
    with pytest.raises(m.FrameError):
        m.parse_read_response(_hex('01 03'), count=1)


def test_parse_corrupt_crc_raises():
    frame = bytearray(_hex('01 03 04 26 F7 01 41 81 29'))
    frame[-1] ^= 0xFF
    with pytest.raises(m.CrcError):
        m.parse_pressure_temperature(bytes(frame))


def test_parse_wrong_byte_count_raises():
    # claims 2 data bytes but is a 4-byte payload: impossible for count=2
    frame = _with_crc('01 03 02 26 F7 01 41')
    with pytest.raises(m.FrameError):
        m.parse_read_response(frame, slave_id=1, count=2)


def test_parse_trailing_garbage_raises():
    frame = _hex('01 03 04 26 F7 01 41 81 29') + b'\x00'
    with pytest.raises(m.ModbusResponseError):
        m.parse_pressure_temperature(frame)


def test_parse_wrong_function_code_raises():
    frame = _with_crc('01 04 02 00 00')
    with pytest.raises(m.FrameError):
        m.parse_read_response(frame, slave_id=1, count=1)


def test_parse_exception_response_raises_with_code():
    frame = _with_crc('01 83 02')
    with pytest.raises(m.ModbusExceptionResponse) as excinfo:
        m.parse_read_response(frame, slave_id=1, count=2)
    assert excinfo.value.function_code == m.FC_READ_HOLDING
    assert excinfo.value.exception_code == 0x02
    assert 'illegal data address' in str(excinfo.value)


def test_expected_read_response_length():
    assert m.expected_read_response_length(1) == 7
    assert m.expected_read_response_length(2) == 9


# --------------------------------------------------------------------------- #
# Write response echo
# --------------------------------------------------------------------------- #

def test_verify_write_response_accepts_echo():
    request = m.build_set_slave_address_request(1, 2)
    m.verify_write_response(request, request)


def test_verify_write_response_rejects_wrong_echo():
    request = m.build_set_slave_address_request(1, 2)
    other = m.build_set_slave_address_request(1, 3)
    with pytest.raises(m.FrameError):
        m.verify_write_response(request, other)


def test_verify_write_response_rejects_corrupt_crc():
    request = m.build_set_slave_address_request(1, 2)
    corrupt = bytearray(request)
    corrupt[-1] ^= 0xFF
    with pytest.raises(m.CrcError):
        m.verify_write_response(request, bytes(corrupt))


# --------------------------------------------------------------------------- #
# Value decoding
# --------------------------------------------------------------------------- #

@pytest.mark.parametrize(
    'raw,expected_pa',
    [
        (9974, 99740.0),
        (9975, 99750.0),
        (10164, 101640.0),
        (0, 0.0),
    ],
)
def test_decode_pressure(raw, expected_pa):
    assert m.decode_pressure(raw) == pytest.approx(expected_pa)


@pytest.mark.parametrize(
    'raw,expected_c',
    [
        (0x0141, 32.1),
        (0x0000, 0.0),
        (0xFF9C, -10.0),
        (0x8000, -3276.8),
    ],
)
def test_decode_temperature(raw, expected_c):
    assert m.decode_temperature(raw) == pytest.approx(expected_c)


def test_to_signed16():
    assert m.to_signed16(0x0000) == 0
    assert m.to_signed16(0x7FFF) == 32767
    assert m.to_signed16(0x8000) == -32768
    assert m.to_signed16(0xFFFF) == -1


def test_baudrate_code_table():
    assert m.BAUDRATE_CODES[9600] == 2
    assert m.BAUDRATE_CODES[1200] == 7
    assert m.BAUDRATE_FROM_CODE[7] == 1200
    assert m.SUPPORTED_BAUDRATES == (1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200)


# --------------------------------------------------------------------------- #
# End to end
# --------------------------------------------------------------------------- #

def test_request_response_round_trip():
    request = m.build_read_pressure_temperature_request(slave_id=1)
    assert request == _hex('01 03 01 00 00 02 C5 F7')
    reading = m.parse_pressure_temperature(_hex('01 03 04 26 F7 01 41 81 29'))
    assert reading.pressure_pa == pytest.approx(99750.0)
    assert reading.temperature_c == pytest.approx(32.1)
