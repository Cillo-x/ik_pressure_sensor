# ik_pressure_sensor

[![CI](https://github.com/Cillo-x/ik_pressure_sensor/actions/workflows/ci.yml/badge.svg)](https://github.com/Cillo-x/ik_pressure_sensor/actions/workflows/ci.yml)
[![Lint](https://github.com/Cillo-x/ik_pressure_sensor/actions/workflows/lint.yml/badge.svg)](https://github.com/Cillo-x/ik_pressure_sensor/actions/workflows/lint.yml)

艾克姆科技 RS485 型气压变送器（BMP280 芯片）的 ROS 2 Humble 驱动。通过 USB-RS485
适配器以 Modbus-RTU 协议读取气压与温度，一次请求同时取回两个量，发布为
`sensor_msgs/msg/FluidPressure`（Pa）与 `sensor_msgs/msg/Temperature`（℃）。

## 特性

- 一次 Modbus 请求读取气压与温度（9 字节响应），无需两次总线往返
- 发布标准消息类型，可直接被 `rqt_plot`、`rosbag` 和现有传感器管道消费
- 纯 Python 实现，运行时仅依赖 `python3-serial`，不需要 pip 安装第三方库
- 串口 I/O 在独立线程，总线异常不会阻塞 executor
- 链路失败显式上报，不发布旧值或伪造值；串口掉线后自动重连
- 内置 `/diagnostics`（链路状态与收发计数），并支持无硬件的模拟模式

## 安装

需要 Ubuntu 22.04 + ROS 2 Humble：

```bash
cd ~/ros2_ws/src
git clone https://github.com/Cillo-x/ik_pressure_sensor.git
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select ik_pressure_sensor
source install/setup.bash
```

## 快速开始

没有硬件时用模拟数据源验证整条链路：

```bash
ros2 run ik_pressure_sensor pressure_node --ros-args -p use_mock:=true
```

接入真实传感器（设备默认 9600 8N1、从机地址 1）：

```bash
sudo usermod -aG dialout $USER          # 只需一次，重新登录后生效
ros2 launch ik_pressure_sensor pressure_sensor.launch.py port:=/dev/ttyUSB0
```

启动日志中出现型号与版本，即说明设备可达且参数匹配：

```
[INFO] [ik_pressure_sensor]: device product model 0x271B, software version 0x1902
```

多传感器场景可加 `namespace:=sensors`，话题随之变为 `/sensors/pressure`。

## 话题

| 话题 | 类型 | 说明 |
|---|---|---|
| `pressure` | `sensor_msgs/msg/FluidPressure` | `fluid_pressure`，单位 Pa |
| `temperature` | `sensor_msgs/msg/Temperature` | 单位 ℃ |
| `/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | 1 Hz，链路状态与计数器 |

默认使用 sensor data QoS（best effort）；需要可靠传输时加 `-p qos_reliable:=true`。

## 参数

| 参数 | 默认值 | 说明 |
|---|---|---|
| `port` | `/dev/ttyUSB0` | 串口设备，建议用 udev 固定名 |
| `baudrate` | `9600` | 设备支持的 8 档波特率之一 |
| `slave_id` | `1` | Modbus 从机地址 |
| `poll_rate` | `2.0` | 读取频率（Hz） |
| `frame_id` | `pressure_link` | 消息坐标系，接入机器人时改为实际 frame 名 |
| `publish_temperature` | `true` | 是否发布温度话题 |
| `timeout` / `retries` | `0.25` / `2` | 单次超时（秒）与重发次数 |

完整参数表（含模拟模式）见 `config/pressure_sensor.yaml`，launch 时可用
`params_file:=<绝对路径>` 覆盖。

## 示例

```bash
ros2 topic echo /pressure --once
ros2 topic hz /pressure                    # 默认 2 Hz
ros2 topic echo /diagnostics --once        # level: OK 表示链路正常
ros2 param list /ik_pressure_sensor
```

```yaml
header:
  frame_id: pressure_link
fluid_pressure: 101325.0
variance: 0.0
```

## 文档

| 文档 | 内容 |
|---|---|
| [doc/01-485协议与数据域.md](doc/01-485协议与数据域.md) | 寄存器映射、帧格式、CRC、单位换算 |
| [doc/02-使用注意事项.md](doc/02-使用注意事项.md) | 串口与权限、udev、轮询与超时、诊断解读 |
| [CONTRIBUTING.md](CONTRIBUTING.md) | 分支模型、提交规范、发版流程 |

## 许可

Apache-2.0，见 [LICENSE](LICENSE)。
