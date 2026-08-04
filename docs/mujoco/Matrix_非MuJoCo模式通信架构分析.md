# Matrix 非 MuJoCo 模式通信架构分析

> 基于 2026-08-03 17:41 实际运行日志分析（xgb 机器人，yard 场景，**未勾选** MuJoCo 物理）

## 一、系统组件

| 组件 | 进程 | PID | 角色 |
|------|------|-----|------|
| UE | `zsibot_mujoco_ue` | 3332663 | **物理 + 渲染**（内部 mj_step ~500Hz） |
| mc_ctrl | `./mc_ctrl r` | 3333125 | 控制算法（FSM、RL策略） |
| robot_mujoco | — | — | **未启动** |

关键区别：不勾选 MuJoCo 时，robot_mujoco 进程不启动，UE 自己跑物理。

## 二、mc_ctrl 初始化差异

```
# MuJoCo 模式（勾选）：
[MujocoCommandInterface] initialize...

# 非 MuJoCo 模式（不勾选）：
[UnrealCommandInterface] initialize...
```

mc_ctrl 使用 `UnrealCommandInterface` 通过 UDP 直接与 UE 通信，而非通过 eCAL 与 robot_mujoco 通信。

## 三、端口清单

### 核心通信端口

| 端口 | 协议 | 发送方 | 接收方 | 数据大小 | 频率 | 内容 |
|------|------|--------|--------|----------|------|------|
| **25001** | UDP | UE (src:33126) | mc_ctrl | **303 bytes** | ~500Hz | 机器人状态（关节位置/速度/IMU） |
| **25002** | UDP | mc_ctrl (src:47317) | UE | **365 bytes** | ~500Hz | 电机控制指令（12路扭矩/位置） |

### UE 监听端口

| 端口 | 用途 | 有流量？ |
|------|------|---------|
| **9999** | 渲染状态接收（MuJoCo模式专用） | ❌ 无（robot_mujoco未启动） |
| **25002** | 接收mc_ctrl控制指令 | ✅ 365 bytes @ ~500Hz |
| **13001** | LiDAR数据 | ❌ 无 |
| 11111 | UE内部（用途不明） | ❌ 无 |
| 3002 | UE内部（用途不明） | ❌ 无 |
| 33126 | UE发送状态到mc_ctrl的源端口 | ✅ 303 bytes @ ~500Hz |

### mc_ctrl 监听端口

| 端口 | 用途 | 有流量？ |
|------|------|---------|
| **25001** | 接收UE机器人状态 | ✅ 303 bytes @ ~500Hz |
| 14000-14002 | 传感器广播（MuJoCo模式专用） | 无流量（无robot_mujoco） |
| 43900 | 电机平台接收 | ❌ |
| 43997 | 电机平台高级通道 | ❌ |
| 7667 | 组播 | ❌ |
| 57114 | mc_ctrl内部 | ❌ |
| 43350 | mc_ctrl内部 | ❌ |
| 47317 | mc_ctrl发送控制到UE的源端口 | ✅ 365 bytes @ ~500Hz |

## 四、数据流架构

```
┌──────────────────────────────────────────────────┐
│                     UE                            │
│                                                  │
│  ┌─────────────────┐    ┌─────────────────────┐  │
│  │ FMujocoWorker   │    │ UnrealCommandInterface│  │
│  │ Thread (~500Hz) │    │                     │  │
│  │                 │    │  mj_step() 物理仿真  │  │
│  │  qpos → FK     │    │  qpos → 状态(303B)  │  │
│  │  → mesh渲染     │    │  → 发送到mc_ctrl    │  │
│  │                 │    │                     │  │
│  │  接收控制(365B) │◄───┤  接收控制(365B)     │  │
│  │  → PD控制      │    │  → 写入ctrl数据     │  │
│  └─────────────────┘    └──────────┬──────────┘  │
│                                    │              │
└────────────────────────────────────┼──────────────┘
                                     │
                    UDP 25001 (303B) │ UDP 25002 (365B)
                    UE → mc_ctrl     │ mc_ctrl → UE
                                     │
                          ┌──────────┴──────────┐
                          │      mc_ctrl         │
                          │   (控制算法 500Hz)    │
                          │                      │
                          │  UnrealCommandInterface│
                          │  接收状态 → RL推理    │
                          │  → 输出控制指令       │
                          │                      │
                          │  motor_platform_type:8 │
                          └──────────────────────┘
```

## 五、与 MuJoCo 模式对比

| 特性 | 非 MuJoCo 模式 | MuJoCo 模式 |
|------|---------------|------------|
| mc_ctrl 接口 | `UnrealCommandInterface` | `MujocoCommandInterface` |
| motor_platform_type | **8** (UDP) | **5** (eCAL) |
| robot_mujoco 进程 | ❌ 不启动 | ✅ 运行 |
| UE 内部物理 | ✅ FMujocoWorkerThread ~500Hz | ❌ 不运行 |
| UE 角色 | **物理 + 渲染** | **纯渲染** |
| mc_ctrl ↔ UE | UDP 25001/25002 直连 | eCAL → robot_mujoco → UDP 9999 |
| 状态包大小 | 303 bytes | 412 bytes |
| 控制包大小 | 365 bytes | N/A（经eCAL中转） |
| 通信频率 | ~500Hz | ~500Hz (eCAL) + ~100Hz (渲染) |

## 六、关键发现

### 1. 25001/25002 是非 MuJoCo 模式的核心通道

- **25001**：UE → mc_ctrl，303 bytes，机器人状态
- **25002**：mc_ctrl → UE，365 bytes，电机控制指令
- 双向 ~500Hz，与物理仿真频率同步

### 2. 非 MuJoCo 模式下 mc_ctrl 成功进入 RL_MIX 行走状态

- mc_ctrl 从 JOINT_FREE → RLMIX → RL_Walk
- 狗成功站起来并行走
- 控制闭环：UE物理 → 25001 → mc_ctrl RL推理 → 25002 → UE PD控制 → UE物理

### 3. 9999 端口在非 MuJoCo 模式下闲置

- UE 仍然监听 9999，但无数据流入
- 因为 robot_mujoco 未启动，无渲染状态推送
- UE 自己跑物理，渲染数据来自内部 FK

### 4. 状态包大小差异（303 vs 412 bytes）

- 非 MuJoCo 模式：303 bytes（UE 内部状态，经 UnrealCommandInterface 序列化）
- MuJoCo 模式：412 bytes（robot_mujoco 的 RobotState，经 SDK 序列化）
- 两者包含的信息可能不同（412 bytes 可能包含更多传感器数据）

## 七、总结

非 MuJoCo 模式是一个**自包含**架构：UE 同时负责物理和渲染，mc_ctrl 通过 UDP 25001/25002 直连 UE 形成控制闭环。不需要 robot_mujoco 进程，mc_ctrl 使用 UnrealCommandInterface 替代 MujocoCommandInterface。
