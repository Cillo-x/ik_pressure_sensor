# ik_pressure_sensor

[![CI](https://github.com/Cillo-x/ik_pressure_sensor/actions/workflows/ci.yml/badge.svg)](https://github.com/Cillo-x/ik_pressure_sensor/actions/workflows/ci.yml)
[![Lint](https://github.com/Cillo-x/ik_pressure_sensor/actions/workflows/lint.yml/badge.svg)](https://github.com/Cillo-x/ik_pressure_sensor/actions/workflows/lint.yml)

艾克姆科技 RS485 型气压变送器（Modbus-RTU，BMP280 芯片）的 ROS 2 Humble 驱动。

- 通过 USB-RS485 适配器读取气压与温度，一次请求同时取回两个量
- 发布 `sensor_msgs/msg/FluidPressure`（单位 Pa）与 `sensor_msgs/msg/Temperature`（单位 ℃）
- 串口 I/O 在独立线程，链路卡住不会阻塞 executor
- 纯 Python 实现，Modbus 协议手写，运行时只依赖 `python3-serial`，不需要 pip 装第三方库
- 没有硬件时可以 `use_mock:=true` 跑通整个节点

协议细节、寄存器映射、报文示例见
[`doc/01-485协议与数据域.md`](doc/01-485协议与数据域.md)；
接线、现场施工与部署注意事项见
[`doc/02-使用注意事项.md`](doc/02-使用注意事项.md)。

## 包结构

```
ik_pressure_sensor/
├── ik_pressure_sensor/
│   ├── modbus_rtu.py        # 纯函数编解码：组帧 / CRC / 解析 / 换算，无 I/O
│   ├── transport.py         # Transport 接口 + SerialTransport（pyserial）+ MockTransport
│   ├── device.py            # PressureSensor：组帧、收帧、重试、统计
│   ├── sim.py               # SimulatedSensor：无硬件时的假数据源
│   └── pressure_node.py     # rclpy 节点
├── test/
│   ├── test_modbus_rtu.py   # 编解码 / CRC / 换算（厂家抓包报文）
│   ├── test_transport.py    # mock 传输、串口错误包装
│   ├── test_device.py       # 重试、超时、异常帧、统计恢复
│   ├── test_sim.py          # 模拟数据源
│   ├── test_flake8.py       # 风格检查（无 ROS 时自动跳过）
│   └── test_pep257.py       # docstring 检查（无 ROS 时自动跳过）
├── config/
│   ├── pressure_sensor.yaml        # 默认参数
│   └── pressure_sensor_mock.yaml   # 仅覆盖 use_mock
├── doc/
│   ├── 01-485协议与数据域.md    # 协议、寄存器、报文、CRC
│   └── 02-使用注意事项.md        # 接线、总线、部署、排障
├── launch/pressure_sensor.launch.py
├── udev/99-ik-pressure-sensor.rules
├── .github/
│   ├── workflows/ci.yml            # ROS 2 构建 + 测试
│   ├── workflows/lint.yml          # 风格检查（不装 ROS，30 秒）
│   ├── dependabot.yml
│   └── pull_request_template.md
├── resource/ik_pressure_sensor
├── package.xml
├── setup.py
├── setup.cfg
├── requirements-dev.txt
├── CONTRIBUTING.md
├── LICENSE
└── .gitattributes
```

## 编译

```bash
# Ubuntu 22.04 + ROS 2 Humble
cd ~/ros2_ws/src
git clone https://github.com/Cillo-x/ik_pressure_sensor.git
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y   # 或 sudo apt install python3-serial
colcon build --packages-select ik_pressure_sensor
source install/setup.bash
```

## 运行

### 没有硬件先跑通框架

```bash
ros2 run ik_pressure_sensor pressure_node --ros-args -p use_mock:=true
```

另一个终端：

```bash
ros2 topic list
ros2 topic echo /pressure --once        # sensor_msgs/msg/FluidPressure
ros2 topic echo /temperature --once     # sensor_msgs/msg/Temperature
ros2 topic hz /pressure                 # 应为 poll_rate，默认 2 Hz
ros2 topic echo /diagnostics --once     # level: OK，带各项计数
```

预期：`/pressure` 约 101325 Pa 上下小幅波动，`/temperature` 约 25 ℃，
日志里会打印 `device product model 0x271B, software version 0x1902`（模拟源返回的就是真机型号）。

想验证故障处理，把丢包率调高：

```bash
ros2 run ik_pressure_sensor pressure_node --ros-args \
  -p use_mock:=true -p mock_failure_rate:=0.5
```

会看到 `read failed: ...` 告警（只在状态变化时打印一次，不刷屏），
`/diagnostics` 的 level 在 WARN / ERROR 之间变化。

### 接真实传感器

```bash
# 串口权限（一次性，重新登录生效）
sudo usermod -aG dialout $USER

ros2 launch ik_pressure_sensor pressure_sensor.launch.py port:=/dev/ttyUSB0
```

预期日志：

```
[ik_pressure_sensor]: serial /dev/ttyUSB0 at 9600 bit/s, slave id 1
[ik_pressure_sensor]: polling at 2.00 Hz, publisher best effort (timeout 0.25 s x 3 retries)
[ik_pressure_sensor]: device product model 0x271B, software version 0x1902
```

启动时的型号/版本读取同时是一次链路自检：能读到，说明接线、地址、波特率都对。

推荐给适配器配 udev 固定名字，然后把 `port` 改成 `/dev/ik_pressure_sensor`：

```bash
sudo cp "$(ros2 pkg prefix ik_pressure_sensor)/share/ik_pressure_sensor/udev/99-ik-pressure-sensor.rules" /etc/udev/rules.d/
# 先按文件里的注释填好自己的 idVendor/idProduct/serial
sudo udevadm control --reload-rules && sudo udevadm trigger
```

### 带命名空间（多传感器时）

```bash
ros2 launch ik_pressure_sensor pressure_sensor.launch.py namespace:=sensors
# 话题变成 /sensors/pressure、/sensors/temperature
```

## 话题与诊断

| 话题 | 类型 | 说明 |
|---|---|---|
| `pressure` | `sensor_msgs/msg/FluidPressure` | `fluid_pressure` 单位 Pa，`frame_id` 来自参数 |
| `temperature` | `sensor_msgs/msg/Temperature` | 单位 ℃，`publish_temperature:=false` 可关闭 |
| `/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | 每 1 s 一条，name 为 `<node>: link` |

`variance` 填 0.0（设备不提供该估计）。时间戳取 `node` 时钟。

诊断状态判定：

- `OK`：已连接且连续失败数为 0
- `WARN`：连续失败数 ≥ 1 但小于 `offline_threshold`
- `ERROR`：未连接，或连续失败数 ≥ `offline_threshold`

诊断里还带 `port / baudrate / slave_id / poll_rate / timeout / attempts /
product_model / software_version` 以及全部统计计数，排查时先看这里。

## 参数

| 参数 | 默认 | 说明 |
|---|---|---|
| `port` | `/dev/ttyUSB0` | 串口设备；建议用 udev 固定名 |
| `baudrate` | `9600` | 只接受 8 档之一（1200~115200），写错会在启动时报错 |
| `slave_id` | `1` | Modbus 从机地址 1~247 |
| `timeout` | `0.25` | 单次收帧超时（秒） |
| `retries` | `2` | 失败后重发次数，共 `retries+1` 次尝试 |
| `reconnect_interval` | `2.0` | 串口打不开/掉线后的重连间隔（秒） |
| `poll_rate` | `2.0` | 读取频率（Hz） |
| `frame_id` | `pressure_link` | 消息 header.frame_id |
| `pressure_topic` | `pressure` | 相对话题名，随命名空间解析 |
| `temperature_topic` | `temperature` | 同上 |
| `publish_temperature` | `true` | 是否发布温度 |
| `qos_reliable` | `false` | `false` = best effort（sensor data），`true` = reliable depth 10 |
| `pressure_offset_pa` | `0.0` | 加到每次测量上的偏置，用于微调 |
| `offline_threshold` | `3` | 连续失败多少次判为离线（诊断 ERROR） |
| `use_mock` | `false` | 用模拟数据源，不需要串口 |
| `mock_pressure_pa` | `101325.0` | 模拟基准气压 |
| `mock_temperature_c` | `25.0` | 模拟基准温度 |
| `mock_failure_rate` | `0.0` | 模拟丢包概率 0~1 |

参数文件在 `config/pressure_sensor.yaml`，launch 时可用
`params_file:=<绝对路径>` 换成自己的。

## 设计要点

**串口 I/O 在独立线程。** 单次读取最坏耗时 `timeout × (retries+1)`（默认 0.75 s），
如果放在定时器回调里，一次故障就会把 executor 卡住 0.75 s。
现在 worker 线程负责读取和发布，主线程只跑 1 Hz 的诊断定时器，故障期间 ROS 侧照常响应。

**失败语义。** 通信失败不发布旧值、不发布 0，只更新统计并打一条日志（状态变化时才打，
不会刷屏）。串口本身坏掉时关闭并按 `reconnect_interval` 重连；设备拒答（Modbus 异常帧）
不重试，因为重试没有意义。

**收帧按帧头动态定长。** 正常应答是 `5 + 字节数`，异常应答固定 5 字节。
如果死等"请求的寄存器数对应的长度"，异常帧会被误判为帧不完整而永远看不到。

## 代码风格与持续集成

规则统一放在 `setup.cfg` 的 `[flake8]` 段（99 列、ament 同款忽略列表、google import 风格），
所以本地 `flake8`、CI 里的 `flake8`、以及 ROS 的 `ament_flake8` 三者结论一致。

本地检查（不需要 ROS）：

```bash
python -m pip install -r requirements-dev.txt
python -m flake8 .
python -m pytest test -q
```

CI 有两个 workflow，分两档门禁：

| workflow | 检查内容 | 耗时 | 作用 |
|---|---|---|---|
| `Lint` | `flake8` + `pydocstyle`，纯 Python，不装 ROS | ~30 s | 所有 PR 的快速门禁 |
| `CI` | `rosdep install` → `colcon build` → `colcon test`（含包里两条 ament lint 测试） | ~2–3 min | 合入 `main` 的构建门禁；合入 `dev` 后也会自动跑一次 |

> 设计取舍：功能 PR 合入 `dev` 只跑 30 秒的 Lint，不烧 3 分钟构建；
> 合入 `dev` 之后的 push 会自动跑完整构建，所以集成分支始终有构建记录，
> 而 `main` 的门槛最高 —— PR 必须构建通过才能合并。

为省 runner 时间：只监听 `main` / `dev` 的 push 与 PR、同一分支旧任务自动取消、关闭覆盖率收集。

分支模型（`main` 发布 / `dev` 集成 / `feat/*` 开发）与发布流程见
[CONTRIBUTING.md](CONTRIBUTING.md)。

## 测试

不需要 ROS、也不需要硬件：

```bash
cd ros2_ws/src/ik_pressure_sensor
python -m pytest test -q
```

预期 `112 passed, 2 skipped`（那 2 条是 lint 测试，没有 ROS 时自动跳过）。分层跑：

```bash
python -m pytest test/test_modbus_rtu.py -v   # 协议编解码，含 24 条厂家抓包
python -m pytest test/test_device.py -v       # 重试 / 超时 / 异常帧 / 统计
python -m pytest test/test_transport.py -v    # 传输层
python -m pytest test/test_sim.py -v          # 模拟数据源
```

> 厂家抓包里「单寄存器读软件版本」那条报文是转录错误的
> （`19 04` + CRC `32 15`，而 `32 15` 只对 `19 02` 成立），测试里留了用例记录这个事实，
> 因此版本号按 `0x1902` 处理。

## 排障速查

| 现象 | 先查 |
|---|---|
| 启动后一直 `cannot open /dev/ttyUSB0` | 设备在不在（`ls /dev/ttyUSB*`）、用户是否在 `dialout` 组、是否被别的进程占用 |
| 打开成功但 `read failed: ...` / 连续失败 | 传感器是否独立供电、A/B 是否接反、地址与波特率是否匹配 |
| `device rejected the request` | 寄存器地址或功能码不匹配，多半是固件版本不同 |
| 诊断 `crc_errors` 持续增长 | 波特率偏差、线缆质量、终端电阻、干扰与共地 |
| 有数据但和预期差很多 | 确认是绝对气压、确认单位是 Pa、确认安装位置没有被局部环境影响 |

## 数据来源与许可

- 协议信息、寄存器映射与报文示例整理自厂家《导轨式 MODBUS 气压变送器使用说明书 V2.1》
  及随附的调试记录。说明书原文、厂家例程与调试软件**不随本仓库分发**。
- 本项目以 **Apache-2.0** 许可发布，全文见 [LICENSE](LICENSE)。
