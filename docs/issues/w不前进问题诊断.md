# W 不前进问题诊断

## 问题描述

CarlaUnreal 中按 U 可以站立，但按 W 无法前进。Matrix UE（打包版）中 W 正常。
两端使用**同一个 mc_ctrl 二进制 + 同一份配置 + 同一个 xgb.xml 模型**，唯一区别是 UE 端。

## 日志对比（关键差异）

| 项目 | Matrix UE ✅ | CarlaUnreal ❌ |
|------|-------------|----------------|
| SDK 初始化 | `client_ip: 127.0.0.1` → 成功 | `client_ip: 192.168.234.1` → `bind: Cannot assign requested address` → **sdk init failed!** |
| 数据接收 | `Reciving data size: 303` × **7740次** | `Reciving data size: 303` × **0次** |
| 初始 body height | **0.047540**（趴地折叠） | **0.375018**（已站立） |
| FSM 转换 | `[FSM_RLMIX] on Enter!` ✅ | 永远卡在 `STANDUP` ❌ |
| W 行走 | 正常 | 无反应 |

## 数据链路分析

```
① 键盘捕获        ② 虚拟手柄       ③ mc_ctrl FSM       ④ UE 执行
keyboard_control   →  gamepad    →   STANDUP 卡住    →   Kp=80 只保持站立
   ✅ 正常             ✅ 正常          ❌ 瓶颈              (不响应速度指令)
```

W 控制行走**必须在 RL_MIX 模式**才生效（Kp≈20，RL 策略驱动）。
STANDUP 模式（Kp=80）只保持站立姿态，忽略一切速度指令。

## 根因分析

### 直接原因：STANDUP → RL_MIX 转换条件未满足

mc_ctrl 的 STANDUP 状态机是一个**时序状态机**，不是简单的阈值判断：

```
STANDUP 状态机预期流程：
  1. 检测到 body_height < 阈值（趴地）     ← 必须经历
  2. 执行收腿（Folding legs）
  3. 执行撑起（Stand up）
  4. 确认 body_height ≈ 目标值 且稳定
  5. 转换到 RL_MIX                         ← 才能行走
```

**Matrix UE**：机器人初始趴地（0.047）→ 步骤 1 满足 → 完整执行 2-5 → 进入 RL_MIX ✅
**CarlaUnreal**：机器人初始已站立（0.375）→ 步骤 1 **从未满足** → 卡死在 STANDUP ❌

### 根本原因：CarlaUnreal UE 端的两个问题

**问题 1：初始姿态不是趴地**

`MuJoCoSimulation.cpp` BeginPlay 中设置了半蹲姿态：
```cpp
mData->qpos[2] = 0.15;                    // body z = 0.15（太高）
mData->qpos[7 + j*3 + 1] = 0.2f;         // HIP = 0.2
mData->qpos[7 + j*3 + 2] = -0.4f;        // KNEE = -0.4
```
Matrix UE 的机器人以默认关节角（全 0）启动，在重力下自然塌缩到 body_height ≈ 0.047。

**问题 2：UE 的 PD 控制与 mc_ctrl 抢占控制权**

当 mc_ctrl 以 Kp=80 激活时，UE 的 `ApplyUdpControl()` 立即执行 mc_ctrl 的站立指令，
机器人在 mc_ctrl 的 STANDUP FSM 还没检测到"趴地"状态时就已经被撑起到 0.375。
FSM 永远等不到"从低到高"的站立过程，无法完成转换。

**问题 3：SDK 通信路径不同**

Matrix UE 的 mc_ctrl 通过 SDK（LCM 多播 127.0.0.1）接收数据，有 7740 条 `Reciving data size: 303`。
CarlaUnreal 的 mc_ctrl SDK 绑定 192.168.234.1 失败，回退到 `UnrealCommandInterface`（UDP 直连），
没有 `Reciving data` 日志。两条路径的数据处理逻辑可能存在差异（如关节偏移补偿）。

## 已实施修复

### 修复 1：去掉 BeginPlay 初始姿态覆盖 + 启动延迟 + 拑制状态发送（已完成）

文件：`MuJoCoSimulation.cpp` + `MuJoCoSimulation.h`

**修改前**：
- BeginPlay 手动设置半蹲姿态（body z=0.15, HIP=0.2, KNEE=-0.4）
- UE 立即向 mc_ctrl 发送状态，mc_ctrl 第一次收到就是站立高度 0.375
- STANDUP FSM 从未检测到趴地，卡死

**修改后**（三层保护）：
1. 删除初始姿态设置，使用模型默认 qpos（base z=0.65）
2. 前 3 秒为启动延迟期：零力矩，机器人在重力下自然塌缩到 ~0.05
3. 启动延迟期内**不向 mc_ctrl 发送状态**，避免 FSM 看到站立高度

3 秒后 UE 开始发送状态，此时机器人已趴地，mc_ctrl 的 STANDUP FSM 正常执行：
趴地检测 → 收腿 → 撑起 → 转换 RL_MIX → W 可用

### 修复 2：mc_ctrl 启动参数（需手动）

启动 mc_ctrl 时必须设置：
```bash
export SDK_CLIENT_IP=127.0.0.1   # 与 Matrix UE 一致，否则 SDK bind 失败
export ROBOT_TYPE=XG
taskset -c 7 ./mc_ctrl r 2>&1 | tee /tmp/mc_ctrl_carlaunreal.log
```

## 验证方法

重新编译 CarlaUnreal 后启动，检查 mc_ctrl 日志：

```bash
# 期望看到（与 Matrix UE 一致）：
body height is 0.0xx,Folding legs     # 初始趴地
body height is 0.3xx,Stand up         # 撑起过程
[FSM_RLMIX] on Enter!                 # 成功转换
Operating Mode: NORMAL in RL_MIX      # 行走模式
```

然后按 W 验证前进。
让我分析这三个终端的完整数据链路。先确认一个关键参数：


数据链路分析完成。**按 W 没反应的根本原因不在按键，而在 mc_ctrl 卡死在 STANDUP 模式**。下面是完整分析：

## 数据链路逐段诊断

```
① 键盘捕获        ② 虚拟手柄       ③ mc_ctrl 模式      ④ UE 执行
keyboard_control   →  gamepad    →   STANDUP卡住    →   Kp=80站立
   ✅ 正常             ✅ 正常          ❌ 瓶颈            (不响应速度)
```

**① 按键捕获正常**（Terminal 56-72）：

```
[按键] W(前进)  ABS_Y=-32768 ABS_X=0 RX=0   ← W 被正确捕获并发出
```

**② mc_ctrl 收到并进入 STANDUP**（Terminal 981-1026）：

```
[joystic] find 1 joystic          ← 手柄识别正常
body height is 0.374925, Folding legs
Operating Mode: NORMAL in STANDUP  ← 卡在 STANDUP（迭代9999、19999都是）
```

**③ UE 持续应用 STANDUP 指令**（Terminal 1010-1025）：

```
maxKp=80.0 active=1   ← 一直是 Kp=80（STANDUP），从未变成 Kp=20（RLMIX）
```

## 核心问题：W 为什么无效

| 模式                      | Kp   | 行为            | 速度指令(W)  |
| ------------------------- | ---- | --------------- | ------------ |
| **STANDUP**（当前卡在这） | 80   | 只保持站立姿态  | ❌ **被忽略** |
| **RLMIX**（行走模式）     | 20   | RL 策略驱动行走 | ✅ 响应       |

**W 控制行走必须在 RLMIX 模式才生效**。但 mc_ctrl 从 STANDUP 转换到 RLMIX 的条件没满足，所以一直卡在站立，W 自然没反应。

## 为什么转换失败

STANDUP→RLMIX 的转换条件：**身体高度接近目标值**（误差 < ±10%）

```
mc_ctrl 用 FK(关节角) 算出的身体高度 = 0.3749 m
配置目标 body_height              = 0.32 m
高度比 = 0.3749 / 0.32 = 1.17
转换要求 |1.17 - 1.0| < 0.1  →  0.17 > 0.1  ❌ 失败
```

mc_ctrl 认为机器人"太高了"（0.375 vs 目标 0.32，偏差 17%），所以拒绝进入行走模式。

## 根本原因

这是 **UE 与 mc_ctrl 的关节约定不一致**导致的：UE 发送的站立关节角（hip=0.8, knee=-1.5）在 UE 里实际身体高度是 0.32，但 mc_ctrl 用自己的 FK（含关节偏移量 hip_offset/knee_offset）算出来是 0.375。两边对"同一组关节角"的高度理解不同。

---

**下一步我建议**：抓取 UE 当前实际发送的关节角和机器人真实高度，对比 mc_ctrl 的 FK 计算，确认关节偏移量的具体差异，然后在 UE 发送 RobotState 时补偿偏移量，让 mc_ctrl 算出正确的 0.32 高度，从而正常转换到 RLMIX。

需要我现在就开始排查这个关节偏移量问题吗？