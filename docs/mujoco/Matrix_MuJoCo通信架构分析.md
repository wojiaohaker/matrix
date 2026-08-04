# Matrix MuJoCo 模式通信架构分析

> 基于 2026-08-03 17:31 实际运行日志分析（xgb 机器人，yard 场景，勾选 MuJoCo 物理）

## 一、系统组件

| 组件 | 进程 | PID | 角色 |
|------|------|-----|------|
| robot_mujoco | `./robot_mujoco` | 3304068 | MuJoCo 物理仿真 + 桥接 |
| mc_ctrl | `./mc_ctrl r` | 3304522 | 控制算法（FSM、RL策略） |
| UE | `zsibot_mujoco_ue` | 3304069 | 3D 渲染 |

## 二、端口清单

### 核心端口

| 端口 | 协议 | 发送方 | 接收方 | 数据内容 | 频率 |
|------|------|--------|--------|----------|------|
| **9999** | UDP | robot_mujoco (src:55251) | UE (zsibot_mujoco_ue) | 渲染状态（412 bytes/包） | ~100Hz |
| **13001** | UDP | 外部 LiDAR 驱动（当前无数据） | UE | LiDAR 点云数据 | 按需 |
| **14000** | UDP broadcast | mc_ctrl (src:49758) + robot_mujoco (src:38854) | 彼此 | 传感器/控制数据（~8KB/包） | ~500Hz |
| **14001** | UDP broadcast | mc_ctrl + robot_mujoco | 彼此 | 控制指令 | 按需 |
| **14002** | UDP broadcast | mc_ctrl + robot_mujoco | 彼此 | 辅助数据 | 按需 |

### mc_ctrl 专用端口

| 端口 | 用途 |
|------|------|
| 40001 | mc_ctrl 监听端口 |
| 7667 | 组播地址 239.255.76.67:7667 |
| 43900 | 电机平台接收（mp_recv_cp） |
| 43997 | 电机平台接收/高级通道（mp_recv_cp） |
| 49758 | mc_ctrl 发送传感器数据到 14000 |

### robot_mujoco 专用端口

| 端口 | 用途 |
|------|------|
| 55251 | 向 UE:9999 发送渲染状态 |
| 38854 | 向 14000 广播传感器数据 |
| TCP 41591 | Zenoh/ROS2 通信 |

### config.json 中的 25001/25002

```json
"mujoco_running": true,
"state_port": 25001,
"cmd_port": 25002
```

**实际状态**：当前运行时 mc_ctrl **不监听** 25001/25002。

- `motor_platform_type: 5`（eCAL 模式）→ mc_ctrl 通过 eCAL 共享内存与 robot_mujoco 通信
- 25001/25002 是 `motor_platform_type: 8`（UDP 模式）时 mc_ctrl 使用的端口
- config.json 中的 `state_port`/`cmd_port` 是 UE 侧的配置，UE 当前也未使用这两个端口

**结论**：25001/25002 在 MuJoCo 模式（eCAL）下**不活跃**，仅在非 MuJoCo 的 UDP 直连模式下使用。

## 三、数据流架构

```
┌──────────────┐    eCAL 共享内存     ┌──────────────┐
│   mc_ctrl    │◄───────────────────►│robot_mujoco  │
│  (控制算法)   │   mujoco_state/cmd  │ (MuJoCo物理)  │
│              │                     │              │
│ port: 40001  │    UDP broadcast    │ port: 55251  │
│ port: 14000  │◄───────────────────►│ port: 38854  │
│ port: 43900  │    port: 14000-2    │ port: 14000  │
│ port: 49758  │                     │              │
└──────────────┘                     └──────┬───────┘
                                            │
                                   UDP 9999 │ 412 bytes
                                   ~100Hz   │ 渲染状态
                                            ▼
                                    ┌──────────────┐
                                    │     UE       │
                                    │  (3D 渲染)    │
                                    │              │
                                    │ port: 9999   │
                                    │ port: 13001  │
                                    └──────────────┘
```

## 四、关键发现

### 1. UE 在 MuJoCo 模式下**不运行内部物理**

- 日志中**无** `FMujocoWorkerThread` 记录
- 对比：非 MuJoCo 模式下日志有 `FMujocoWorkerThread` 以 ~500Hz 运行
- UE 在 MuJoCo 模式下是**纯渲染器**

### 2. UDP 9999 是渲染数据通道

- robot_mujoco 从端口 55251 发送 412 bytes 到 UE:9999
- 频率 ~100Hz（每包间隔 ~10ms）
- 数据内容：机器人关节状态（位置、速度等），UE 接收后用于更新 mesh 位置
- 412 bytes 对应自定义二进制结构体（RobotState）

### 3. mc_ctrl 与 robot_mujoco 通过 eCAL + UDP 14000 双通道通信

- eCAL：用于 mujoco_state/mujoco_cmd 主题（低延迟共享内存）
- UDP 14000：用于传感器数据广播（~8KB 大包）

### 4. mc_ctrl 状态

- 启动后处于 `JOINT_FREE` 状态（无控制指令输入时）
- 电机平台类型：`motor_platform_type: 5`（eCAL）
- 控制频率：500Hz（`low_level_dt: 0.002`）

## 五、非 MuJoCo 模式对比

| 特性 | MuJoCo 模式 | 非 MuJoCo 模式 |
|------|------------|---------------|
| mc_ctrl motor_platform_type | 5 (eCAL) | 8 (UDP) |
| robot_mujoco 进程 | ✅ 运行 | ❌ 不运行 |
| UE 内部物理 | ❌ 不运行 | ✅ FMujocoWorkerThread ~500Hz |
| UE 角色 | 纯渲染 | 物理+渲染 |
| mc_ctrl↔UE 通信 | eCAL→robot_mujoco→UE:9999 | UDP 25001/25002 直连 |
| mc_ctrl↔robot_mujoco | eCAL + UDP 14000 | N/A |

## 六、对 CarlaUnreal 的启示

CarlaUnreal 需要实现的外部物理模式：

1. **robot_mujoco 角色**：CarlaUnreal 需接收类似 UDP 9999 的渲染状态数据
2. **mc_ctrl 角色**：外部控制器通过 eCAL 或 UDP 发送控制指令
3. **UE 角色**：CarlaUnreal 作为渲染器，接收外部物理引擎的状态数据更新 mesh

当前 CarlaUnreal 的外部物理模式（`bExternalPhysicsMode`）运行的是**内部 mj_step**，这与 Matrix 的架构不同。Matrix 的 UE 是纯渲染器，物理完全由 robot_mujoco 负责。
