# Matrix 官方 SDK 与关节偏移量问题分析

## 一、分析目标

排查 mc_ctrl 卡死 STANDUP 无法转换到 RLMIX 的关节偏移量问题：
- mc_ctrl 用 FK 算出身体高度 = **0.3749 m**
- 配置目标 body_height = **0.32 m**
- 偏差 17% > 10% 阈值 → 拒绝转换

需要确认：mc_ctrl 的 FK 公式中是否包含 hip_offset/knee_offset，以及 UE 应如何补偿。

---

## 二、MATRiX_Python_SDK 分析

**路径**: `/home/qiyuan/Softwares/MATRiX_Python_SDK`

### 2.1 项目定位

这是 Matrix 官方的 **MuJoCo→UE 状态同步 SDK**，用于：
- 在独立进程中运行 MuJoCo 物理仿真
- 将 qpos/qvel 通过 UDP 发送给 UE 进行渲染
- UE 仅作为可视化前端，不参与物理计算

### 2.2 通信协议（与我们的架构完全不同）

```text
协议格式 (jszr_sdk_bridge.cc 兼容):
  double time
  uint32 nq
  double qpos[nq]     ← 原始 MuJoCo 广义坐标
  uint32 nv
  double qvel[nv]
  uint32 nu
  double act[nu]
目标端口: 9999 (UE渲染同步)
```

**对比我们的架构**:
| | MATRiX_Python_SDK | 我们的 CarlaUE5 |
|---|---|---|
| MuJoCo 位置 | 独立进程 | 嵌入 UE 内部 |
| 通信协议 | 原始 qpos/qvel (double) | Protobuf RobotState/RobotCmd |
| 端口 | 9999 (单向→UE) | 25001/25002 (双向) |
| 控制方 | SDK 内的 controller 回调 | mc_ctrl (RL策略) |
| UE 角色 | 纯渲染 | 物理仿真 + 渲染 |

### 2.3 模型文件（有价值）

SDK 包含 `model/xgw/xgw.xml`，与我们的 `xgb_game.xml` 是**同系列机器人**：

```xml
<!-- xgw 模型关键参数 -->
<body name="FR_ABAD_LINK" pos="0.1745 -0.062 0">
  <joint name="FAR_ABAD_JOINT" axis="1 0 0" range="-0.4887 0.4887"/>
  <body name="FR_HIP_LINK" pos="0 -0.097399 0">
    <joint name="FAR_HIP_JOINT" axis="0 1 0" range="-1.152 2.967"/>
    <body name="FR_KNEE_LINK" pos="0 0 -0.2">        ← L_hip = 0.2m
      <joint name="FAR_KNEE_JOINT" axis="0 1 0" range="-2.723 -0.602"/>
      <body name="FR_FOOT_LINK" pos="0 0 -0.21366">  ← L_knee = 0.21366m
```

### 2.4 关键发现

✅ **确认连杆参数一致**：xgw 与 xgb 完全相同
- L_hip = 0.2 m
- L_knee = 0.21366 m
- Abad 偏移: x=±0.17449, y=±0.062
- Hip 偏移: y=±0.0974

✅ **确认模型无关节偏移**：所有 joint 均为 `pos="0 0 0"`，无 `ref=`/`springref=` 属性

❌ **不包含 FK 代码**：SDK 只做状态转发，不做正向运动学计算

❌ **不包含 mc_ctrl 通信**：没有 Protobuf RobotState/RobotCmd 协议

---

## 三、genisom_robot_sdk 分析

**路径**: `/home/qiyuan/Softwares/genisom_robot_sdk`

### 3.1 项目定位

这是 Matrix 机器人的**真机控制 SDK**（C++ 闭源库），用于：
- 通过 WebSocket/UDP 连接物理机器人
- 发送高层指令（站立/趴下/移动/转弯）
- 接收传感器数据（IMU/关节/电池/里程计）

### 3.2 接口层次

```cpp
// 高层控制指令（不涉及关节级控制）
client.StandUp();
client.LieDown();
client.Move(line, translation, angle);  // 速度指令 (m/s, m/s, rad/s)
client.SetMode(1);  // General/InPlace/Stair

// 传感器数据
struct MotionData {
  float quat[4];        // 姿态四元数 [w,x,y,z]
  float position[3];    // 世界坐标位置 [x,y,z] (m)
  float v_world[3];     // 世界速度
  float omega_body[3];  // 机体角速度
};

struct JointStateData {
  vector<string> names;      // 关节名
  vector<double> positions;  // 关节角 (rad)
  vector<double> velocities; // 关节速度 (rad/s)
  vector<double> efforts;    // 关节力矩 (Nm)
};
```

### 3.3 关键发现

❌ **无 FK 代码**：SDK 是高层接口，不暴露正向运动学

❌ **无关节偏移参数**：没有 hip_offset/knee_offset 定义

❌ **闭源库**：核心逻辑在 `librobot_sdk.so` 中，无法查看

✅ **确认关节数据格式**：JointStateData 使用标准 names/positions/velocities/efforts

✅ **确认速度指令格式**：Move(line, translation, angle) = (前后, 左右, 偏航)

---

## 四、对我们 xgb 模型的验证

我们的 `xgb_game.xml` 与官方 xgw 模型对比：

| 参数 | xgb_game.xml (我们) | xgw.xml (官方SDK) | 一致性 |
|------|---------------------|-------------------|--------|
| L_hip | 0.2 m | 0.2 m | ✅ |
| L_knee | 0.21366 m | 0.21366 m | ✅ |
| Abad X偏移 | ±0.17449 | ±0.1745/0.17449 | ✅ |
| Abad Y偏移 | ±0.062 | ±0.062 | ✅ |
| Hip Y偏移 | ±0.097399/0.097412 | ±0.097399/0.097412 | ✅ |
| ABAD 轴 | 1 0 0 | 1 0 0 | ✅ |
| HIP/KNEE 轴 | 0 1 0 | 0 1 0 | ✅ |
| 关节机械偏移 | 无 (pos="0 0 0") | 无 (pos="0 0 0") | ✅ |

**结论：我们的 MuJoCo 模型运动学参数与官方完全一致，模型本身没有问题。**

---

## 五、核心结论

### 5.1 两个 SDK 对排查关节偏移量问题的帮助

| SDK | 帮助程度 | 原因 |
|-----|---------|------|
| MATRiX_Python_SDK | ⭐⭐ 有限 | 确认了模型参数正确，但架构完全不同（无 mc_ctrl 通信） |
| genisom_robot_sdk | ⭐ 极有限 | 纯高层真机接口，无 FK/偏移量信息 |

### 5.2 问题定位

**偏移量不在 MuJoCo 模型中**（两个官方模型都没有 ref/springref/offset）。

mc_ctrl 日志中的 `body height is 0.374925` 是 mc_ctrl **内部 FK 代码**计算的结果。
这个 FK 公式可能包含：
1. 软件层面的关节零位偏移（joint zero offset）
2. 不同的 FK 公式约定（如角度正方向不同）
3. 或 mc_ctrl 读取到的关节角与 UE 发送的不一致（Protobuf 字段映射问题）

### 5.3 下一步排查方向

1. **抓取 UE 实际发送的关节角**（停 mc_ctrl → 绑 25001 → 抓 Protobuf RobotState）
2. **用简单 FK 验证**：`h = 0.2*cos(q_hip) + 0.21366*cos(q_hip+q_knee)` 是否等于 position.z
3. **反推 mc_ctrl 的 FK**：如果 UE 发 q_hip=0.8, q_knee=-1.5，而 mc_ctrl 算出 0.375，
   则 mc_ctrl 的 FK 一定有额外偏移或不同公式
4. **检查 Matrix 源码中 mc_ctrl 的 FK 实现**（需找到源码，当前只有二进制）
5. **对比 Matrix 自己 UE 发送的关节角**（运行 Matrix 完整栈，抓取 zsibot_mujoco_ue→mc_ctrl 的数据）

---

## 六、附录：文件清单

### MATRiX_Python_SDK 关键文件
```
matrix_sdk/protocol.py    - UDP 序列化（raw qpos/qvel double 数组）
matrix_sdk/sdk.py         - MuJoCo 仿真循环 + UE 同步
matrix_sdk/udp.py         - UDP 发送器
examples/xgb_unreal_sync.py - XGB 模型同步示例
model/xgw/xgw.xml        - XGW 机器人 MuJoCo 模型（与 XGB 同系列）
```

### genisom_robot_sdk 关键文件
```
include/robot_sdk/sdk_type.hpp   - 数据结构定义（RobotState/MotionData/JointStateData）
include/robot_sdk/sdk_client.hpp - SDK 客户端接口
example/control.cpp              - 交互控制示例（StandUp/Move/LieDown）
example/data.cpp                 - 传感器数据订阅示例
lib/x86_64/librobot_sdk.so      - 闭源核心库
docs/protocol/Protocol-1.2.0.pdf - 通信协议文档
```
