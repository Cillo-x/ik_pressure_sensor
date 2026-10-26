"""
Device layer: turn a :class:`~.transport.Transport` into sensor readings.

The layer owns the things a caller should not have to think about: building
the request, waiting for the whole response, deciding whether a failure is
worth retrying, and keeping counters that a ROS node can turn into
diagnostics.

It performs no ROS specific work and can be exercised entirely with
:class:`~.transport.MockTransport`, so the retry and error behaviour is
testable without a device on the bus.
"""

from __future__ import annotations

from dataclasses import dataclass
import time
from typing import Callable, Optional, Tuple, TypeVar

from . import modbus_rtu as codec
from .transport import Transport, TransportError

__all__ = [
    'DeviceCommunicationError',
    'DeviceError',
    'DeviceStats',
    'DeviceTimeout',
    'PressureSensor',
]

_T = TypeVar('_T')

#: Largest legal Modbus-RTU frame: address + function + byte count + 250 data
#: bytes + CRC.  Anything longer is noise and must not make us wait for it.
_MAX_FRAME_LENGTH = 256


class DeviceError(Exception):
    """Base class for every failure surfaced by :class:`PressureSensor`."""


class DeviceTimeout(DeviceError):
    """The device did not deliver a complete response in time."""


class DeviceCommunicationError(DeviceError):
    """
    No valid response could be obtained, even after retrying.

    ``last_error`` keeps the most recent underlying failure so that callers can
    tell timeouts from corruption without string parsing.
    """

    def __init__(  # noqa: B042 - keep str() readable in logs
        self,
        message: str,
        last_error: Optional[BaseException] = None,
    ) -> None:
        super().__init__(message)
        self.last_error = last_error


@dataclass
class DeviceStats:
    """Counters describing the health of the link, for diagnostics."""

    queries: int = 0
    successes: int = 0
    timeouts: int = 0
    crc_errors: int = 0
    frame_errors: int = 0
    exception_responses: int = 0
    transport_errors: int = 0
    consecutive_failures: int = 0
    last_error: Optional[str] = None


class PressureSensor:
    """Modbus master for one pressure transmitter on the bus."""

    def __init__(
        self,
        transport: Transport,
        slave_id: int = codec.DEFAULT_SLAVE_ID,
        timeout: float = 0.25,
        retries: int = 2,
    ) -> None:
        if retries < 0:
            raise ValueError(f'retries must be >= 0, got {retries}')
        if timeout <= 0:
            raise ValueError(f'timeout must be positive, got {timeout}')
        self._transport = transport
        self._slave_id = slave_id
        self._timeout = timeout
        self._retries = retries
        self.stats = DeviceStats()

    @property
    def transport(self) -> Transport:
        return self._transport

    @property
    def slave_id(self) -> int:
        return self._slave_id

    @property
    def timeout(self) -> float:
        return self._timeout

    @property
    def retries(self) -> int:
        return self._retries

    @property
    def consecutive_failures(self) -> int:
        return self.stats.consecutive_failures

    # ------------------------------------------------------------------ #
    # Public API
    # ------------------------------------------------------------------ #

    def read(self) -> codec.PressureReading:
        """
        Read pressure and temperature in a single bus round trip.

        Raises :class:`DeviceCommunicationError` when no valid answer arrives
        within the retry budget, propagates :class:`~.transport.TransportError`
        when the port itself fails, and propagates
        :class:`~.modbus_rtu.ModbusExceptionResponse` when the device answered
        but refused the request.
        """
        request = codec.build_read_pressure_temperature_request(self._slave_id)
        return self._execute(
            request,
            lambda frame: codec.parse_pressure_temperature(frame, self._slave_id),
        )

    def read_identity(self) -> Tuple[int, int]:
        """
        Read the product model and software version registers.

        Useful as a start-up check: a response here proves the wiring, the
        slave id and the baudrate are all correct.
        """
        request = codec.build_read_request(
            self._slave_id, codec.REG_PRODUCT_MODEL, 2
        )
        return self._execute(
            request,
            lambda frame: codec.parse_read_response(frame, self._slave_id, 2),
        )

    # ------------------------------------------------------------------ #
    # Internals
    # ------------------------------------------------------------------ #

    def _execute(
        self,
        request: bytes,
        decode: Callable[[bytes], _T],
    ) -> _T:
        self.stats.queries += 1
        last_error: Optional[BaseException] = None

        for _attempt in range(self._retries + 1):
            try:
                frame = self._transact(request)
                value = decode(frame)
            except codec.ModbusExceptionResponse as exc:
                # The device understood us and said no; retrying cannot help.
                self.stats.exception_responses += 1
                self._mark_failure(exc)
                raise
            except TransportError as exc:
                # Port gone or unusable: the caller owns reconnection.
                self.stats.transport_errors += 1
                self._mark_failure(exc)
                raise
            except DeviceTimeout as exc:
                self.stats.timeouts += 1
                last_error = exc
            except codec.CrcError as exc:
                self.stats.crc_errors += 1
                last_error = exc
            except codec.ModbusResponseError as exc:
                self.stats.frame_errors += 1
                last_error = exc
            else:
                self.stats.successes += 1
                self.stats.consecutive_failures = 0
                self.stats.last_error = None
                return value

        self._mark_failure(last_error)
        attempts = self._retries + 1
        raise DeviceCommunicationError(
            f'no valid response after {attempts} attempt(s): {last_error}',
            last_error=last_error,
        )

    def _transact(self, request: bytes) -> bytes:
        """
        Send one request and return one complete response frame.

        The response length is derived from the frame itself rather than from
        the request: an exception reply is 5 bytes and a normal reply is
        ``5 + byte_count``.  Waiting for the length implied by the request
        would make exception replies invisible.
        """
        self._transport.reset_input()
        self._transport.write(request)

        deadline = time.monotonic() + self._timeout
        buffer = bytearray()

        # Phase 1: address, function code and the third byte (byte count or
        # exception code) tell us how long the whole frame should be.
        if not self._read_until(buffer, 3, deadline):
            raise DeviceTimeout(
                f'no response header within {self._timeout * 1000.0:.0f} ms '
                f'(received {len(buffer)} byte(s))'
            )

        function_code = buffer[1]
        if function_code & codec.EXCEPTION_BIT:
            expected_length = 5
        else:
            expected_length = 5 + buffer[2]

        if expected_length > _MAX_FRAME_LENGTH:
            raise codec.FrameError(
                f'implausible response length {expected_length} '
                f'(byte count {buffer[2]})'
            )

        # Phase 2: read the rest of the frame.
        if not self._read_until(buffer, expected_length, deadline):
            raise DeviceTimeout(
                f'expected {expected_length} bytes, received {len(buffer)} '
                f'within {self._timeout * 1000.0:.0f} ms'
            )
        return bytes(buffer[:expected_length])

    def _read_until(
        self,
        buffer: bytearray,
        target: int,
        deadline: float,
    ) -> bool:
        """Fill ``buffer`` up to ``target`` bytes; return False on timeout."""
        while len(buffer) < target:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return False
            chunk = self._transport.read(target - len(buffer), remaining)
            if not chunk:
                # A real port returns b'' only after its timeout expired.
                return False
            buffer.extend(chunk)
        return True

    def _mark_failure(self, error: Optional[BaseException]) -> None:
        self.stats.consecutive_failures += 1
        self.stats.last_error = str(error) if error is not None else 'unknown error'
