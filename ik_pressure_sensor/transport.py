"""
Byte transports for the pressure sensor driver.

:class:`Transport` is a deliberately tiny interface -- open, close, write,
read, discard pending input -- so that the device layer can be driven either
by a real serial port or by :class:`MockTransport` in tests.  All the serial
specific behaviour (baudrate, 8N1 framing, timeouts) is confined to
:class:`SerialTransport`.

``pyserial`` is imported defensively: importing this module never fails when
pyserial is absent, the error surfaces only when a :class:`SerialTransport`
is actually opened.  That keeps the test suite runnable on machines without
any serial support installed.
"""

from __future__ import annotations

from collections import deque
import time
from typing import Deque, Iterable, List, Optional

try:  # pragma: no cover - exercised only by environments without pyserial
    import serial as _serial
except ImportError:  # pragma: no cover
    _serial = None

__all__ = [
    'MockTransport',
    'SerialTransport',
    'Transport',
    'TransportError',
]


class TransportError(Exception):
    """The underlying byte channel failed (open, write, read or closed port)."""


class Transport:
    """Minimal half-duplex byte transport."""

    @property
    def is_open(self) -> bool:
        """Whether the channel is currently usable."""
        raise NotImplementedError

    def open(self) -> None:
        """Open the channel.  Idempotent implementations are preferred."""
        raise NotImplementedError

    def close(self) -> None:
        """Close the channel."""
        raise NotImplementedError

    def write(self, data: bytes) -> None:
        """Send ``data`` and make sure it has left the host."""
        raise NotImplementedError

    def read(self, size: int, timeout: float) -> bytes:
        """
        Read up to ``size`` bytes, waiting at most ``timeout`` seconds.

        Returning fewer bytes (including ``b''``) is a normal outcome and means
        the peer went quiet before the frame was complete.
        """
        raise NotImplementedError

    def reset_input(self) -> None:
        """Discard anything still buffered, e.g. noise or a late reply."""
        raise NotImplementedError


class SerialTransport(Transport):
    """
    RS-485 (or any 8N1 serial) transport built on pyserial.

    The adapter is assumed to handle RS-485 direction control itself, which is
    what every normal USB-RS485 dongle does.
    """

    def __init__(
        self,
        port: str,
        baudrate: int = 9600,
        timeout: float = 0.25,
    ) -> None:
        if timeout <= 0:
            raise ValueError(f'timeout must be positive, got {timeout}')
        self._port = port
        self._baudrate = baudrate
        self._timeout = timeout
        self._serial = None

    @property
    def port(self) -> str:
        return self._port

    @property
    def baudrate(self) -> int:
        return self._baudrate

    @property
    def is_open(self) -> bool:
        return self._serial is not None and bool(self._serial.is_open)

    def open(self) -> None:
        if self.is_open:
            return
        if _serial is None:
            raise TransportError(
                'pyserial is not installed; install python3-serial to use '
                'SerialTransport'
            )
        try:
            self._serial = _serial.Serial(
                port=self._port,
                baudrate=self._baudrate,
                bytesize=_serial.EIGHTBITS,
                parity=_serial.PARITY_NONE,
                stopbits=_serial.STOPBITS_ONE,
                timeout=self._timeout,
                write_timeout=self._timeout,
            )
        except ValueError:
            raise
        except (_serial.SerialException, OSError) as exc:
            self._serial = None
            raise TransportError(
                f'cannot open serial port {self._port!r}: {exc}'
            ) from exc

    def close(self) -> None:
        serial_port, self._serial = self._serial, None
        if serial_port is not None:
            try:
                serial_port.close()
            except (_serial.SerialException, OSError) as exc:
                raise TransportError(f'cannot close {self._port!r}: {exc}') from exc

    def write(self, data: bytes) -> None:
        if not self.is_open:
            raise TransportError(f'serial port {self._port!r} is not open')
        try:
            self._serial.write(data)
            # Wait until the bytes have actually left the host before the
            # device starts answering; a half-duplex adapter would otherwise
            # see the tail of our request on the bus while we are reading.
            self._serial.flush()
        except (_serial.SerialException, OSError) as exc:
            raise TransportError(f'write to {self._port!r} failed: {exc}') from exc

    def read(self, size: int, timeout: float) -> bytes:
        if not self.is_open:
            raise TransportError(f'serial port {self._port!r} is not open')
        if size <= 0:
            return b''
        deadline = time.monotonic() + timeout
        chunks = bytearray()
        while len(chunks) < size:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                break
            self._serial.timeout = remaining
            try:
                chunk = self._serial.read(size - len(chunks))
            except (_serial.SerialException, OSError) as exc:
                raise TransportError(
                    f'read from {self._port!r} failed: {exc}'
                ) from exc
            if not chunk:
                break
            chunks.extend(chunk)
        return bytes(chunks)

    def reset_input(self) -> None:
        if not self.is_open:
            return
        try:
            self._serial.reset_input_buffer()
        except (_serial.SerialException, OSError) as exc:
            raise TransportError(
                f'cannot flush {self._port!r}: {exc}'
            ) from exc


class MockTransport(Transport):
    """
    Scripted transport for tests and for dry runs without a device.

    Each ``read`` call consumes the next canned response.  A response may be
    shorter or longer than the requested size: leftovers stay buffered until
    they are read or discarded by :meth:`reset_input`, which mirrors what a
    real port does with stale bytes.
    """

    def __init__(self, responses: Iterable[bytes] = ()) -> None:
        self._responses: Deque[bytes] = deque(bytes(r) for r in responses)
        self.written: List[bytes] = []
        self.opened = 0
        self.closed = 0
        self._buffer = b''
        self._open = False
        #: Set to an exception instance to make the next writes fail.
        self.write_error: Optional[Exception] = None

    @property
    def is_open(self) -> bool:
        return self._open

    def open(self) -> None:
        self.opened += 1
        self._open = True

    def close(self) -> None:
        self.closed += 1
        self._open = False

    def write(self, data: bytes) -> None:
        if self.write_error is not None:
            raise self.write_error
        self.written.append(bytes(data))

    def read(self, size: int, timeout: float) -> bytes:
        if size <= 0:
            return b''
        if not self._buffer:
            if not self._responses:
                return b''
            self._buffer = self._responses.popleft()
        chunk, self._buffer = self._buffer[:size], self._buffer[size:]
        return chunk

    def reset_input(self) -> None:
        self._buffer = b''

    def queue_response(self, data: bytes) -> None:
        """Append one more scripted response."""
        self._responses.append(bytes(data))

    @property
    def pending_responses(self) -> int:
        return len(self._responses)
