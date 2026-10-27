# AGENTS.md

Agent-facing orientation. Keep it short. Hold only the context an agent needs
to act and the invariants it must not break; detailed process lives in `CONTRIBUTING.md`.

## What this is

ROS 2 Humble driver for the IKMSIK RS-485 Modbus pressure transmitter (BMP280 chip).
It publishes `sensor_msgs/FluidPressure` (Pa) and `sensor_msgs/Temperature` (degC).

This repository ships software only.

## Where to look

- Wire format, register map, scaling, CRC, example frames: `doc/01-485协议与数据域.md`
- Serial device setup, runtime behaviour, diagnostics: `doc/02-使用注意事项.md`
- Frame encode/decode, CRC, scaling — pure, no I/O, no ROS: `ik_pressure_sensor/modbus_rtu.py`
- Byte transport, real serial and mock: `ik_pressure_sensor/transport.py`
- Request/response, retries, link statistics: `ik_pressure_sensor/device.py`
- ROS node, parameters, topics, diagnostics: `ik_pressure_sensor/pressure_node.py`
- Simulated sensor behind `use_mock:=true`: `ik_pressure_sensor/sim.py`
- All runtime parameters, commented (single source of truth): `config/pressure_sensor.yaml`
- Byte-exact test vectors captured from the device: `test/test_modbus_rtu.py`

## Verify

    python -m pip install -r requirements-dev.txt
    python -m flake8 .
    python -m pytest test -q

With ROS 2 Humble:

    colcon build --packages-select ik_pressure_sensor
    colcon test --packages-select ik_pressure_sensor

## Invariants (MUST)

- Keep `ik_pressure_sensor/modbus_rtu.py` pure: no I/O, no ROS imports, unit-testable without hardware.
- Every protocol change updates a byte-for-byte test vector in `test/test_modbus_rtu.py`.
- A new parameter is added to `config/pressure_sensor.yaml` and the node's declared defaults in the same change.
- Runtime dependencies must resolve through rosdep; `python3-serial` is the only one today. Do not introduce pip-only runtime dependencies.
- On link failure, log diagnostics and fail explicitly. Do not publish stale or fabricated values.

## Default conventions (SHOULD)

- Keep Python lines within 99 columns; lint rules live in `setup.cfg` and mirror `ament_flake8`.
  Fix lint findings; if suppression is genuinely needed, use a narrow `# noqa` and say why.
- Write README and `doc/` in Chinese; code, comments and commit messages in English.
- Commits follow Conventional Commits. Branching and CI details live in `CONTRIBUTING.md`.
- Use relative or portable paths in code and config, never absolute or machine-specific ones.
- Keep `test/` runnable with plain `pytest`; guard ROS-only tests with `pytest.importorskip`.