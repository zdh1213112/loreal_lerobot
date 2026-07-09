# bi_dobot_nova5_dh

双臂 Dobot Nova5 机器人模块，集成大寰 AG-95 夹爪通过机械臂末端内置 RS485 接口进行 Modbus RTU 通信。

---

## 背景与目标

标准 `dh_gripper` 模块通过 USB-RS485 转换器（pyserial）直接与大寰 AG-95 夹爪通信，需要额外的串口硬件和 USB 连线。本模块的目标是**去掉 USB 串口线**，改用 Dobot Nova5 末端关节内置的 RS485 接口，让夹爪的 Modbus 报文经机械臂控制器中转，从而简化硬件配线。

---

## 文件结构

```
bi_dobot_nova5_dh/
├── config_dh_gripper_integrated.py   # 夹爪参数配置
├── dh_gripper_integrated.py          # 夹爪驱动（ModbusRTUProtocol + DHGripperIntegrated）
├── config_bi_dobot_nova5_dh.py       # 机器人整体参数配置
├── bi_dobot_nova5_dh.py              # 机器人主类（BiDobotNova5DH）+ Modbus 适配器（_DobotModbusRTU）
├── __init__.py                       # 导出 BiDobotNova5DH, BiDobotNova5DHConfig, ControlMode
└── TCP_IP_Python_V4/
    └── dobot_api.py                  # Dobot 官方 SDK（不修改）
```

---

## 通信原理

Dobot Nova5 控制器（TCP 端口 29999，`DobotApiDashboard`）提供了一组 Modbus RTU 代理 API，将 Modbus 报文经控制器内部 RS485 转发至末端设备：

| SDK 函数 | 对应 Modbus 功能 | 说明 |
|---|---|---|
| `SetToolMode(1, 1, identify)` | — | 将末端接口设置为 RS485 Modbus 工具模式 |
| `SetTool485(baud, "N", 1, identify)` | — | 配置末端 RS485 为 8N1 |
| `ModbusCreate("192.168.201.1", 60000, slave_id, isRTU=True)` | — | 建立 RS485 主站，返回 `master_index` |
| `SetHoldRegs(index, addr, count, val)` | FC=0x06 | 写保持寄存器 |
| `GetHoldRegs(index, addr, count)` | FC=0x03 | 读保持寄存器 |
| `ModbusClose(index)` | — | 关闭主站 |

所有 API 调用均通过已有的 TCP 连接（端口 29999）完成，无需额外硬件。

> **线程安全**：`DobotApiDashboard.sendRecvMsg` 内部持有 `__globalLock`，所有调用自动串行化，后台轮询线程与主控制线程可以安全并发使用同一个 Dashboard 实例。

---

## 架构设计

### 依赖关系

```
BiDobotNova5DH
  ├── DobotApiDashboard          左/右臂 TCP 控制连接
  │
  ├── _DobotModbusRTU            Modbus 适配器（定义于 bi_dobot_nova5_dh.py）
  │     ├── __init__: ModbusCreate(isRTU=True) → 存储 master_index
  │     ├── read_register()  → GetHoldRegs(index, reg, 1)
  │     ├── write_register() → SetHoldRegs(index, reg, 1, val)
  │     └── close()          → ModbusClose(index)
  │
  └── DHGripperIntegrated        夹爪驱动（定义于 dh_gripper_integrated.py）
        connect(modbus: ModbusRTUProtocol)
        ├── _hardware_initialize()
        ├── set_gripper_position()
        ├── get_gripper_position()  ← 后台 20Hz 轮询缓存
        └── disconnect()
```

### 关键设计决策

**适配器模式**：`DHGripperIntegrated` 不持有 `DobotApiDashboard` 引用，仅依赖 `ModbusRTUProtocol`（`typing.Protocol`，定义 `read_register` / `write_register` / `close` 三个方法）。`_DobotModbusRTU` 隐式满足该 Protocol，运行时无任何包装或转换，Protocol 仅在静态类型检查时生效。

`_DobotModbusRTU` 定义于 `bi_dobot_nova5_dh.py` 而非单独文件，因为它与 `DobotApiDashboard` 紧耦合，脱离 `BiDobotNova5DH` 没有独立使用价值。

---

## 大寰 AG-95 寄存器速查

| 地址 | 方向 | 含义 | 值域 |
|---|---|---|---|
| `0x0100` | 写 | 硬件初始化触发 | `0xA5` |
| `0x0101` | 写 | 夹力 | 20–100 (%) |
| `0x0103` | 写 | 目标位置 | 0（全闭）–1000（全开） |
| `0x0104` | 写 | 速度（硬件不响应，无效） | 0–100 (%) |
| `0x0200` | 读 | 初始化状态 | 0=未初始化，1=完成 |
| `0x0201` | 读 | 运动状态 | 0=运动中，1=到位，2=抓住物体，3=物体掉落 |
| `0x0202` | 读 | 当前位置 | 0–1000 |

**位置约定**：
- DH 硬件寄存器：0 = 全闭，1000 = 全开
- lerobot 归一化：0.0 = 全闭，1.0 = 全开

---

## 连接流程

```
BiDobotNova5DH.connect()
│
├─ 1. TCP 连接两臂 Dashboard（port 29999）
├─ 2. 启动 FeedBack 线程（port 30004），读取 RobotMode / qActual / tcpPose
├─ 3. EnableRobot，等待两臂 RobotMode=5（就绪）
│
├─ 4. 对每个夹爪（以右臂为例）：
│     robot.SetToolMode(1, 1, identify)
│     robot.SetTool485(baud, "N", 1, identify)
│     _DobotModbusRTU(self._right_robot, master_ip, master_port, slave_id)
│         └─ robot.ModbusCreate(master_ip, master_port, slave_id, isRTU=True)
│            → 控制器在末端 RS485 建立主站，返回 master_index
│     self._right_gripper.connect(right_modbus)
│         ├─ 读 0x0200：若已初始化则跳过
│         ├─ 否则写 0x0100=0xA5，轮询 0x0200=1（最长 init_timeout 秒）
│         ├─ 写 0x0101=gripper_force
│         ├─ 若 init_open=True，写 0x0103=1000（张开）
│         └─ 启动后台轮询线程（20Hz 读 0x0202）
│
├─ 5. 连接相机
└─ 6. 移动到起始位置
```

## 断开流程

```
BiDobotNova5DH.disconnect()
│
├─ 1. 移动到 home 位置
│
├─ 2. 【先】夹爪 disconnect()       ← 必须在 TCP 关闭前
│     ├─ 停止后台轮询线程
│     ├─ 写 0x0103=1000（张开夹爪）
│     └─ modbus.close() → ModbusClose(index)
│
└─ 3. 【后】robot.Stop() + robot.close()  ← TCP 断开
```

> **顺序说明**：`ModbusClose` 和开爪指令均需通过 TCP 发出，必须在 `robot.close()` 之前执行。

---

## 配置说明

### BiDobotNova5DHConfig

```python
from lerobot.robots.bi_dobot_nova5_dh import BiDobotNova5DH, BiDobotNova5DHConfig, ControlMode

config = BiDobotNova5DHConfig(
    left_robot_ip="192.168.5.101",
    right_robot_ip="192.168.5.102",
    control_mode=ControlMode.JOINT_MOTION,
    use_left_gripper=True,
    use_right_gripper=True,
    left_master_ip="192.168.201.1",
    left_master_port=60000,
    left_tool_identify=1,
    left_dh_gripper_slave_id=1,
    left_dh_gripper_baudrate=115200,
    left_dh_gripper_force=30,       # 20–100 %
    left_dh_gripper_init_open=True,
    right_master_ip="192.168.201.1",
    right_master_port=60000,
    right_tool_identify=1,
    right_dh_gripper_slave_id=1,
    right_dh_gripper_baudrate=115200,
    right_dh_gripper_force=30,
    right_dh_gripper_init_open=True,
)
robot = BiDobotNova5DH(config)
robot.connect()
```

BiDobotNova5DH 夹爪相关参数：

| 参数 | 默认值 | 说明 |
|---|---|---|
| `left/right_master_ip` | `192.168.201.1` | Dobot 末端 RS485 Modbus 代理 IP |
| `left/right_master_port` | `60000` | Dobot 末端 RS485 Modbus 代理端口 |
| `left/right_tool_identify` | `1` | 多航插机型的末端接口编号（1 或 2） |
| `left/right_dh_gripper_slave_id` | `1` | DH 夹爪 Modbus 从站 ID |
| `left/right_dh_gripper_baudrate` | `115200` | DH 夹爪 RS485 波特率 |

### 夹爪开合快速测试

本目录提供了一个只测试 DH AG-95 夹爪开合的脚本。脚本只连接所选机械臂的 Dashboard 端口和末端 RS485 Modbus，不发送机械臂运动指令。

```bash
# 默认测试右侧夹爪：192.168.5.102
python3 src/lerobot/robots/bi_dobot_nova5_dh/test_dh_gripper_open_close.py

# 测试左侧夹爪
python3 src/lerobot/robots/bi_dobot_nova5_dh/test_dh_gripper_open_close.py --side left --left-ip 192.168.5.101

# 修改循环次数和每次动作后的等待时间
python3 src/lerobot/robots/bi_dobot_nova5_dh/test_dh_gripper_open_close.py --cycles 5 --hold-s 1.0

# 如果你的夹爪 RS485 参数不是 115200,N,1，可以显式指定
python3 src/lerobot/robots/bi_dobot_nova5_dh/test_dh_gripper_open_close.py --baudrate 115200 --parity N --stop-bit 1

# 如果 ModbusCreate 成功但 GetHoldRegs/SetHoldRegs 返回 -1，可尝试重启末端工具电源
python3 src/lerobot/robots/bi_dobot_nova5_dh/test_dh_gripper_open_close.py --power-cycle --power-wait-s 3.0
```

### DHGripperIntegratedConfig

| 参数 | 默认值 | 说明 |
|---|---|---|
| `slave_id` | `1` | Modbus 从站 ID（1–247） |
| `baudrate` | `115200` | RS485 波特率，需与夹爪硬件一致（可选 9600/19200/38400/115200） |
| `gripper_force` | `30` | 夹力百分比（20–100 %，最大 160 N） |
| `gripper_speed` | `0` | 速度（硬件不响应，设非零值会触发 UserWarning） |
| `init_open` | `True` | 连接后是否自动张开夹爪 |
| `init_timeout` | `10.0` | 硬件初始化超时（秒） |

---

## 与 dh_gripper 模块的对比

| 项目 | `dh_gripper` | `bi_dobot_nova5_dh`（本模块） |
|---|---|---|
| 通信方式 | USB-RS485 转换器 + pyserial | 机械臂末端内置 RS485，经控制器中转 |
| Modbus 实现 | 手写帧（CRC-16 查表） | Dobot SDK 代理 API |
| 额外硬件 | 需要 USB-RS485 适配器 | 无 |
| 位置反馈 | 后台线程 50Hz 轮询 | 后台线程 20Hz 轮询（受 TCP 共享影响） |
| 适用场景 | 独立夹爪或非 Dobot 机械臂 | 专用于 Dobot Nova5 内置 RS485 |
