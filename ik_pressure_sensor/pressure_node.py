"""
ROS 2 node publishing barometric pressure and temperature.

All serial I/O runs on one dedicated worker thread, so a silent or unplugged
device can never block the executor.  The worst case blocking time is
``timeout * (retries + 1)`` inside that thread, while the ROS side keeps
spinning and keeps publishing diagnostics.

Publishing is done directly from the worker thread (rclpy publishers are
thread safe); the main thread only owns the diagnostics timer.
"""

from __future__ import annotations

import threading
import time
from typing import Any, List, Optional, Tuple

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import FluidPressure, Temperature

from . import modbus_rtu as codec
from .device import DeviceCommunicationError, DeviceError, PressureSensor
from .sim import SimulatedSensor
from .transport import SerialTransport, TransportError

__all__ = ['PressureSensorNode', 'main']

_DIAGNOSTIC_NAME = 'link'


class PressureSensorNode(Node):
    """Modbus master for one barometric pressure transmitter."""

    def __init__(self) -> None:
        super().__init__('ik_pressure_sensor')
        self._declare_parameters()
        self._fetch_parameters()

        self._sensor: Optional[Any] = None
        self._identity: Optional[Tuple[int, int]] = None
        self._stop_event = threading.Event()
        self._worker: Optional[threading.Thread] = None
        self._failure_logged = False

        qos = self._make_qos()
        self._pressure_pub = self.create_publisher(
            FluidPressure, self._pressure_topic, qos
        )
        self._temperature_pub = None
        if self._publish_temperature:
            self._temperature_pub = self.create_publisher(
                Temperature, self._temperature_topic, qos
            )
        self._diagnostics_pub = self.create_publisher(
            DiagnosticArray, '/diagnostics', 10
        )
        self._diagnostics_timer = self.create_timer(1.0, self._publish_diagnostics)

        self._log_configuration()
        self._worker = threading.Thread(
            target=self._io_loop, name='ik_pressure_sensor-io', daemon=True
        )
        self._worker.start()

    # ------------------------------------------------------------------ #
    # Parameters
    # ------------------------------------------------------------------ #

    def _declare_parameters(self) -> None:
        defaults = {
            'port': '/dev/ttyUSB0',
            'baudrate': 9600,
            'slave_id': 1,
            'timeout': 0.25,
            'retries': 2,
            'poll_rate': 2.0,
            'frame_id': 'pressure_link',
            'pressure_topic': 'pressure',
            'temperature_topic': 'temperature',
            'publish_temperature': True,
            'qos_reliable': False,
            'pressure_offset_pa': 0.0,
            'use_mock': False,
            'mock_pressure_pa': 101325.0,
            'mock_temperature_c': 25.0,
            'mock_failure_rate': 0.0,
            'offline_threshold': 3,
            'reconnect_interval': 2.0,
        }
        for name, default in defaults.items():
            self.declare_parameter(name, default)

    def _fetch_parameters(self) -> None:
        self._port: str = self.get_parameter('port').value
        self._baudrate: int = self.get_parameter('baudrate').value
        self._slave_id: int = self.get_parameter('slave_id').value
        self._timeout: float = self.get_parameter('timeout').value
        self._retries: int = self.get_parameter('retries').value
        self._poll_rate: float = self.get_parameter('poll_rate').value
        self._frame_id: str = self.get_parameter('frame_id').value
        self._pressure_topic: str = self.get_parameter('pressure_topic').value
        self._temperature_topic: str = self.get_parameter('temperature_topic').value
        self._publish_temperature: bool = self.get_parameter('publish_temperature').value
        self._qos_reliable: bool = self.get_parameter('qos_reliable').value
        self._pressure_offset_pa: float = self.get_parameter('pressure_offset_pa').value
        self._use_mock: bool = self.get_parameter('use_mock').value
        self._mock_pressure_pa: float = self.get_parameter('mock_pressure_pa').value
        self._mock_temperature_c: float = self.get_parameter('mock_temperature_c').value
        self._mock_failure_rate: float = self.get_parameter('mock_failure_rate').value
        self._offline_threshold: int = self.get_parameter('offline_threshold').value
        self._reconnect_interval: float = self.get_parameter('reconnect_interval').value

        if not 1 <= self._slave_id <= 247:
            raise ValueError(f'slave_id must be in [1, 247], got {self._slave_id}')
        if self._baudrate not in codec.SUPPORTED_BAUDRATES:
            raise ValueError(
                f'baudrate must be one of {codec.SUPPORTED_BAUDRATES},'
                f' got {self._baudrate}'
            )
        if self._timeout <= 0.0:
            raise ValueError(f'timeout must be positive, got {self._timeout}')
        if self._retries < 0:
            raise ValueError(f'retries must be >= 0, got {self._retries}')
        if self._poll_rate <= 0.0:
            raise ValueError(f'poll_rate must be positive, got {self._poll_rate}')
        if self._offline_threshold < 1:
            raise ValueError(
                f'offline_threshold must be >= 1, got {self._offline_threshold}'
            )
        if self._reconnect_interval <= 0.0:
            raise ValueError(
                f'reconnect_interval must be positive, got {self._reconnect_interval}'
            )
        if not self._use_mock and not self._port:
            raise ValueError('port must be set when use_mock is false')
        if not 0.0 <= self._mock_failure_rate <= 1.0:
            raise ValueError(
                'mock_failure_rate must be in [0, 1],'
                f' got {self._mock_failure_rate}'
            )

    def _make_qos(self) -> QoSProfile:
        if self._qos_reliable:
            return QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        return qos_profile_sensor_data

    def _log_configuration(self) -> None:
        if self._use_mock:
            self.get_logger().info(
                'simulated sensor: base %.1f Pa, %.1f degC, failure_rate %.2f'
                % (self._mock_pressure_pa, self._mock_temperature_c,
                   self._mock_failure_rate)
            )
        else:
            self.get_logger().info(
                'serial %s at %d bit/s, slave id %d'
                % (self._port, self._baudrate, self._slave_id)
            )
        self.get_logger().info(
            'polling at %.2f Hz, publisher %s (timeout %.2f s x %d retries)'
            % (self._poll_rate, 'reliable' if self._qos_reliable else 'best effort',
               self._timeout, self._retries + 1)
        )

    # ------------------------------------------------------------------ #
    # Worker thread
    # ------------------------------------------------------------------ #

    def _io_loop(self) -> None:
        period = 1.0 / self._poll_rate
        while not self._stop_event.is_set():
            started = time.monotonic()

            if self._sensor is None:
                if not self._try_open():
                    self._stop_event.wait(self._reconnect_interval)
                    continue

            try:
                reading = self._sensor.read()
            except DeviceCommunicationError as exc:
                self._report_failure(f'read failed: {exc}', error=False)
                self._stop_event.wait(period)
                continue
            except codec.ModbusExceptionResponse as exc:
                self._report_failure(f'device rejected the request: {exc}', error=True)
                self._stop_event.wait(period)
                continue
            except TransportError as exc:
                self._report_failure(f'serial link failed: {exc}', error=True)
                self._close_sensor()
                self._stop_event.wait(self._reconnect_interval)
                continue
            except Exception as exc:  # noqa: BLE001 - keep the driver alive
                self._report_failure(f'unexpected error: {exc}', error=True)
                self._close_sensor()
                self._stop_event.wait(self._reconnect_interval)
                continue

            self._clear_failure()
            try:
                self._publish_reading(reading)
            except Exception as exc:  # noqa: BLE001 - never kill the io thread
                self.get_logger().error(f'failed to publish: {exc}')

            elapsed = time.monotonic() - started
            self._stop_event.wait(max(0.0, period - elapsed))

    def _try_open(self) -> bool:
        try:
            if self._use_mock:
                sensor: Any = SimulatedSensor(
                    base_pressure_pa=self._mock_pressure_pa,
                    temperature_c=self._mock_temperature_c,
                    failure_rate=self._mock_failure_rate,
                )
            else:
                transport = SerialTransport(
                    self._port, self._baudrate, timeout=self._timeout
                )
                transport.open()
                sensor = PressureSensor(
                    transport,
                    slave_id=self._slave_id,
                    timeout=self._timeout,
                    retries=self._retries,
                )
        except TransportError as exc:
            self._report_failure(f'cannot open {self._port}: {exc}', error=True)
            return False
        except ValueError as exc:
            self._report_failure(f'invalid sensor configuration: {exc}', error=True)
            return False

        self._sensor = sensor
        self._read_identity_safely(sensor)
        return True

    def _read_identity_safely(self, sensor: Any) -> None:
        try:
            model, version = sensor.read_identity()
        except (DeviceError, codec.ModbusError, TransportError) as exc:
            self.get_logger().warn(f'identity read failed: {exc}')
            return
        self._identity = (model, version)
        self.get_logger().info(
            f'device product model 0x{model:04X}, software version 0x{version:04X}'
        )

    def _close_sensor(self) -> None:
        sensor, self._sensor = self._sensor, None
        if sensor is None:
            return
        transport = getattr(sensor, 'transport', None)
        if transport is not None:
            try:
                transport.close()
            except TransportError as exc:
                self.get_logger().debug(f'closing the port failed: {exc}')

    # ------------------------------------------------------------------ #
    # Publishing
    # ------------------------------------------------------------------ #

    def _publish_reading(self, reading: codec.PressureReading) -> None:
        stamp = self.get_clock().now().to_msg()

        pressure_msg = FluidPressure()
        pressure_msg.header.stamp = stamp
        pressure_msg.header.frame_id = self._frame_id
        pressure_msg.fluid_pressure = reading.pressure_pa + self._pressure_offset_pa
        pressure_msg.variance = 0.0
        self._pressure_pub.publish(pressure_msg)

        if self._temperature_pub is not None:
            temperature_msg = Temperature()
            temperature_msg.header.stamp = stamp
            temperature_msg.header.frame_id = self._frame_id
            temperature_msg.temperature = reading.temperature_c
            temperature_msg.variance = 0.0
            self._temperature_pub.publish(temperature_msg)

    def _publish_diagnostics(self) -> None:
        stats = getattr(self._sensor, 'stats', None) if self._sensor is not None else None

        status = DiagnosticStatus()
        status.name = f'{self.get_name()}: {_DIAGNOSTIC_NAME}'
        status.hardware_id = 'mock' if self._use_mock else self._port

        if self._sensor is None:
            status.level = DiagnosticStatus.ERROR
            status.message = 'link down: no sensor connection'
        elif stats is not None and stats.consecutive_failures >= self._offline_threshold:
            status.level = DiagnosticStatus.ERROR
            status.message = (
                f'offline: {stats.consecutive_failures} consecutive failures'
            )
        elif stats is not None and stats.consecutive_failures > 0:
            status.level = DiagnosticStatus.WARN
            status.message = f'{stats.consecutive_failures} consecutive failure(s)'
        else:
            status.level = DiagnosticStatus.OK
            status.message = 'ok'

        status.values = self._diagnostic_values(stats)

        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = [status]
        try:
            self._diagnostics_pub.publish(array)
        except Exception as exc:  # noqa: BLE001 - shutdown race
            self.get_logger().debug(f'diagnostics publish failed: {exc}')

    def _diagnostic_values(self, stats: Any) -> List[KeyValue]:
        values = [
            KeyValue(key='port', value='mock' if self._use_mock else self._port),
            KeyValue(key='baudrate', value=str(self._baudrate)),
            KeyValue(key='slave_id', value=str(self._slave_id)),
            KeyValue(key='poll_rate', value=f'{self._poll_rate:.3f}'),
            KeyValue(key='timeout', value=f'{self._timeout:.3f}'),
            KeyValue(key='attempts', value=str(self._retries + 1)),
        ]
        if self._identity is not None:
            values.append(
                KeyValue(key='product_model', value=f'0x{self._identity[0]:04X}')
            )
            values.append(
                KeyValue(key='software_version', value=f'0x{self._identity[1]:04X}')
            )
        if stats is not None:
            for name in (
                'queries',
                'successes',
                'timeouts',
                'crc_errors',
                'frame_errors',
                'exception_responses',
                'transport_errors',
                'consecutive_failures',
            ):
                values.append(KeyValue(key=name, value=str(getattr(stats, name))))
            values.append(KeyValue(key='last_error', value=stats.last_error or ''))
        return values

    # ------------------------------------------------------------------ #
    # Failure logging (state changes only, so a dead bus cannot flood)
    # ------------------------------------------------------------------ #

    def _report_failure(self, message: str, error: bool) -> None:
        if self._failure_logged:
            self.get_logger().debug(message)
            return
        if error:
            self.get_logger().error(message)
        else:
            self.get_logger().warn(message)
        self._failure_logged = True

    def _clear_failure(self) -> None:
        if self._failure_logged:
            self.get_logger().info('link recovered')
            self._failure_logged = False

    # ------------------------------------------------------------------ #
    # Lifecycle
    # ------------------------------------------------------------------ #

    def destroy_node(self) -> bool:
        self._stop_event.set()
        worker, self._worker = self._worker, None
        if worker is not None and worker.is_alive():
            worker.join(timeout=3.0)
            if worker.is_alive():
                self.get_logger().warn('io thread did not stop in time')
        self._close_sensor()
        return super().destroy_node()


def main(args: Optional[List[str]] = None) -> None:
    """Entry point used by ``ros2 run ik_pressure_sensor pressure_node``."""
    rclpy.init(args=args)
    node: Optional[PressureSensorNode] = None
    try:
        node = PressureSensorNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
