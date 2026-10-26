"""
Simulated sensor for dry runs without hardware.

Implements the same tiny surface as :class:`~.device.PressureSensor`
(``read()``, ``read_identity()``, ``stats``), so the ROS node can be started
with ``use_mock:=true`` and exercised end to end on a machine that has no
sensor, no USB-RS485 adapter and no serial port at all.

Values are quantised exactly like the real device -- pressure in steps of
10 Pa, temperature in steps of 0.1 degC -- so downstream code cannot silently
depend on extra precision that the hardware never delivers.
"""

from __future__ import annotations

import math
import random
import time
from typing import Callable, Optional, Tuple

from . import modbus_rtu as codec
from .device import DeviceCommunicationError, DeviceStats

__all__ = ['SimulatedSensor']


class SimulatedSensor:
    """Fake pressure transmitter with a slow sine, noise and optional faults."""

    def __init__(
        self,
        base_pressure_pa: float = 101325.0,
        temperature_c: float = 25.0,
        pressure_amplitude_pa: float = 50.0,
        period_s: float = 60.0,
        pressure_noise_pa: float = 2.0,
        temperature_noise_c: float = 0.05,
        failure_rate: float = 0.0,
        seed: Optional[int] = None,
        clock: Optional[Callable[[], float]] = None,
    ) -> None:
        if period_s <= 0:
            raise ValueError(f'period_s must be positive, got {period_s}')
        if not 0.0 <= failure_rate <= 1.0:
            raise ValueError(f'failure_rate must be in [0, 1], got {failure_rate}')
        if pressure_amplitude_pa < 0 or pressure_noise_pa < 0:
            raise ValueError('amplitude and noise must be non-negative')

        self._base_pressure_pa = base_pressure_pa
        self._temperature_c = temperature_c
        self._amplitude_pa = pressure_amplitude_pa
        self._period_s = period_s
        self._pressure_noise_pa = pressure_noise_pa
        self._temperature_noise_c = temperature_noise_c
        self._failure_rate = failure_rate
        self._rng = random.Random(seed)
        self._clock = clock if clock is not None else time.monotonic
        self._start = self._clock()
        self.stats = DeviceStats()

    # ------------------------------------------------------------------ #
    # Sensor surface
    # ------------------------------------------------------------------ #

    def read(self) -> codec.PressureReading:
        """Return one fake reading, or raise as a real dead link would."""
        self.stats.queries += 1

        if self._rng.random() < self._failure_rate:
            self.stats.consecutive_failures += 1
            self.stats.last_error = 'simulated communication failure'
            raise DeviceCommunicationError('simulated communication failure')

        elapsed = self._clock() - self._start
        pressure_pa = (
            self._base_pressure_pa
            + self._amplitude_pa * math.sin(2.0 * math.pi * elapsed / self._period_s)
            + self._rng.gauss(0.0, self._pressure_noise_pa)
        )
        temperature_c = self._temperature_c + self._rng.gauss(
            0.0, self._temperature_noise_c
        )

        # Quantise through the register values, like the hardware does.
        pressure_raw = int(round(pressure_pa / codec.PRESSURE_LSB_PA))
        temperature_raw = int(round(temperature_c / codec.TEMPERATURE_LSB_C))

        self.stats.successes += 1
        self.stats.consecutive_failures = 0
        self.stats.last_error = None

        return codec.PressureReading(
            pressure_pa=codec.decode_pressure(pressure_raw),
            temperature_c=codec.decode_temperature(temperature_raw),
            pressure_raw=pressure_raw,
            temperature_raw=temperature_raw,
        )

    def read_identity(self) -> Tuple[int, int]:
        """Report the real device's model and software version registers."""
        return (0x271B, 0x1902)

    def close(self) -> None:
        """Present for interface parity; nothing to release."""
