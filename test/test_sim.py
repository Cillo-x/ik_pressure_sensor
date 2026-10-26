"""Unit tests for the simulated sensor used by ``use_mock:=true`` dry runs."""

from ik_pressure_sensor.device import DeviceCommunicationError
from ik_pressure_sensor.sim import SimulatedSensor
import pytest


class FakeClock:
    """Deterministic stand-in for ``time.monotonic``."""

    def __init__(self) -> None:
        self.now = 0.0

    def __call__(self) -> float:
        return self.now

    def advance(self, seconds: float) -> None:
        self.now += seconds


def test_readings_are_quantised_like_the_hardware():
    sensor = SimulatedSensor(seed=1)
    for _ in range(20):
        reading = sensor.read()
        assert reading.pressure_pa == pytest.approx(reading.pressure_raw * 10.0)
        assert reading.temperature_c == pytest.approx(reading.temperature_raw * 0.1)


def test_same_seed_and_clock_give_identical_sequences():
    first, second = SimulatedSensor(seed=7), SimulatedSensor(seed=7)
    for _ in range(5):
        assert first.read() == second.read()


def test_readings_stay_near_the_configured_base():
    clock = FakeClock()
    sensor = SimulatedSensor(
        base_pressure_pa=100000.0,
        pressure_amplitude_pa=50.0,
        pressure_noise_pa=2.0,
        temperature_c=20.0,
        clock=clock,
        seed=3,
    )
    for _ in range(60):
        clock.advance(0.5)
        reading = sensor.read()
        assert 100000.0 - 60.0 <= reading.pressure_pa <= 100000.0 + 60.0
        assert 19.0 <= reading.temperature_c <= 21.0


def test_pressure_follows_the_sine():
    clock = FakeClock()
    sensor = SimulatedSensor(
        base_pressure_pa=100000.0,
        pressure_amplitude_pa=50.0,
        pressure_noise_pa=0.0,
        period_s=20.0,
        clock=clock,
        seed=0,
    )
    clock.advance(0.0)
    at_zero = sensor.read()
    clock.advance(5.0)               # quarter period -> peak
    at_peak = sensor.read()
    clock.advance(10.0)              # half period -> trough
    at_trough = sensor.read()

    assert at_zero.pressure_pa == pytest.approx(100000.0)
    assert at_peak.pressure_pa == pytest.approx(100050.0)
    assert at_trough.pressure_pa == pytest.approx(99950.0)


def test_failure_rate_one_always_raises():
    sensor = SimulatedSensor(failure_rate=1.0, seed=1)
    with pytest.raises(DeviceCommunicationError):
        sensor.read()
    assert sensor.stats.queries == 1
    assert sensor.stats.consecutive_failures == 1
    assert sensor.stats.last_error == 'simulated communication failure'


def test_successful_reads_update_stats():
    sensor = SimulatedSensor(seed=1)
    sensor.read()
    sensor.read()
    assert sensor.stats.queries == 2
    assert sensor.stats.successes == 2
    assert sensor.stats.consecutive_failures == 0
    assert sensor.stats.last_error is None


def test_identity_matches_the_real_device():
    sensor = SimulatedSensor(seed=1)
    assert sensor.read_identity() == (0x271B, 0x1902)


def test_close_is_a_no_op():
    sensor = SimulatedSensor(seed=1)
    sensor.close()


@pytest.mark.parametrize(
    'kwargs',
    [
        {'period_s': 0},
        {'failure_rate': -0.1},
        {'failure_rate': 1.1},
        {'pressure_amplitude_pa': -1.0},
        {'pressure_noise_pa': -1.0},
    ],
)
def test_invalid_configuration_is_rejected(kwargs):
    with pytest.raises(ValueError):
        SimulatedSensor(**kwargs)
