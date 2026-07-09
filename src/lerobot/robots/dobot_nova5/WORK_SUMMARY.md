# Dobot Nova5 接入工作总结（更新于 2026-04-23）

## 概览

`dobot_nova5` 模块已从“骨架阶段”进入“真机联调与稳定性修复阶段”。  
本轮重点围绕 `lerobot-teleoperate + pico4` 打通连接、启动位姿、实时控制与异常处理链路，并完成了关键兼容问题修复。

---

## 已完成的核心工作

### 1) 控制链路与连接流程稳定化

- 完成 `connect()` 关键链路稳态处理：
  - 反馈线程启动与首帧等待（避免状态未就绪就执行控制逻辑）。
  - 连接前错误态检测（`RobotMode=9`）与 `ClearError()` 自动清故障。
  - `EnableRobot()` 返回值与 `RobotMode()` 联合判定，避免误判导致中断。
- 完成启动运动与等待逻辑重构：
  - 统一使用 `SpeedFactor` + `MovJ` 启动关节运动。
  - 增加基于 command id 与关节误差双重兜底的结束判定。
  - 增加超时与错误模式硬退出，避免无限等待卡死。

### 2) V4 API 规范化（按当前固件）

- 启动关节运动切回 V4 标准接口调用：
  - 使用 `DobotApiDashboard.MovJ(..., coordinateMode=1, v=...)`。
  - 去掉了此前用于兼容的 `sendRecvMsg("JointMovJ(...)")`/旧语法路径。
- `forward_kinematics` 修正为 V4 `PositiveKin(j1..j6)` 规范调用与解析。

### 3) 错误处理与可观测性增强

- 新增统一响应解析与异常抛出机制：
  - `_parse_dobot_response`
  - `_raise_if_dobot_error`
  - `_dobot_error_detail`
- 异常信息中补充 `GetErrorID`，定位更直接。
- 增加等待阶段周期日志，便于观察 `RobotMode`、`CurrentCommandId`、关节误差等状态演化。

### 4) Pico4 联调关键修复（单位一致性）

- 明确并修复单位链路：
  - Pico/LeRobot 动作位置单位为 `m`。
  - Dobot `ServoP` 与反馈 `ToolVectorActual` 的 xyz 为 `mm`。
- 已完成统一换算：
  - 下发 `ServoP` 时 `m -> mm`。
  - 反馈/观测与 `get_current_tcp_pose_*` 输出统一为 `m`。
- 修复后现象从“旋转可动、平移几乎不动”恢复到正常比例平移。

### 5) 资源管理与断开流程改进

- `disconnect()` 中增加 Dashboard/Feedback 连接关闭逻辑。
- 反馈线程在 socket 关闭场景下可安全退出，降低退出阶段报错概率。

### 6) 文档化

- 新增故障排查文档：
  - `src/lerobot/robots/dobot_nova5/PICO4_DOBOT_NOVA5_TROUBLESHOOTING.md`
- 文档覆盖：典型报错、根因分析、修复点、验证步骤与临时绕过方案。

---

## 当前状态判断

- `dobot_nova5.py` 已具备真机可用的连接与基础 teleop 控制路径。
- 关键历史问题（启动卡死、错误态处理薄弱、V4 调用不规范、m/mm 单位错配）已完成修复。
- 当前重点已从“接口搭建”转为“稳定性与参数细化”。

---

## 仍需继续验证/优化

1. 启动位姿在不同工况下的可达性与稳健性（避免偶发逆解/轨迹失败）。
2. `ServoP/ServoJ` 控制参数（`t/aheadtime/gain`）在不同频率和负载下的跟踪性能调优。
3. 观测字段完整性与噪声鲁棒性（包括相机、夹爪、触觉数据联动场景）。
4. 异常恢复策略在连续运行下的长期稳定性（多次 enable/disable、断开重连、故障注入）。
