# robot_mc 与 robot_mujoco 通信数据结构分析

> 基于 Matrix 原始 robot_mujoco 日志 + 编译后 robot_sdk.pb.h + mc_ctrl 启动日志分析
> 日期: 2026-08-04 | 状态: 站立模式 (STANDUP → RL_MIX)

---

## 1. 通信架构总览

```
┌──────────────┐        eCAL (共享内存)         ┌──────────────────┐
│   mc_ctrl    │                                 │  robot_mujoco    │
│  (robot_mc)  │                                 │  (MuJoCo 3.3.0) │
│              │  ┌─────────────────────────┐    │                  │
│  FSM/RL策略   │  │  topic: "mujoco_cmd"    │    │  ZsibotSdkBridge │
│  500Hz 控制   │──┤  消息: RobotCmd          ├───►│  ↓               │
│              │  │  365 bytes (protobuf)    │    │  eCAL Subscriber │
│              │  └─────────────────────────┘    │  ↓               │
│              │                                  │  ApplyControl()  │
│  reciveData()│  ┌─────────────────────────┐    │  + qfrc_bias     │
│  ← 解析 ←────│  │  topic: "mujoco_state"  │    │  + PD 控制       │
│              │◄─┤  消息: RobotState        ├────│  + mj_step()     │
│              │  │  ~303 bytes (protobuf)   │    │  ↓               │
│              │  └─────────────────────────┘    │  eCAL Publisher  │
│              │                                  │                  │
└──────────────┘                                  │  ↓ UDP 9999      │
                                                  │  (412 bytes)     │
                                                  └────────┬─────────┘
                                                           │
                                                           ▼
                                                  ┌────────────────┐
                                                  │ zsibot_mujoco_ue│
                                                  │ (UE 纯渲染)     │
                                                  │ 端口 9999 监听  │
                                                  └────────────────┘
```

| 链路 | 协议 | Topic/端口 | 消息 | 频率 | 字节数 |
|------|------|-----------|------|------|--------|
| mc_ctrl → robot_mujoco | **eCAL** | `mujoco_cmd` | RobotCmd | 500Hz | ~365B |
| robot_mujoco → mc_ctrl | **eCAL** | `mujoco_state` | RobotState | 500Hz | ~303B |
| robot_mujoco → UE | **UDP** | 127.0.0.1:9999 | 原始二进制 | ~100Hz | 412B |

**日志证据:**
- `run_mc.log`: `[MujocoCommandInterface] initialize...` → motor_platform_type=5 (eCAL)
- `robot_mujoco.log`: `[CheckSensor] num_motor_=12, dim_motor_sensor_=36, nsensor=51, nsensordata=84`
- `robot_mujoco.log`: `[CheckSensor] 找到 IMU 传感器，索引: 36` / `找到 Frame 传感器，索引: 39`

---

## 2. RobotCmd 数据结构 (mc_ctrl → robot_mujoco)

### 2.1 Protobuf 字段定义 (robot_sdk.pb.h 编译确认)

```protobuf
message RobotCmd {
    // ===== 目标关节位置 (rad) =====
    repeated float q_des_abad = 1;   // [4] FR,FL,RR,RL 外展关节
    repeated float q_des_hip  = 2;   // [4] FR,FL,RR,RL 髋关节
    repeated float q_des_knee = 3;   // [4] FR,FL,RR,RL 膝关节
    repeated float q_des_foot = 4;   // [4] xgb 填 0

    // ===== 目标关节速度 (rad/s) =====
    repeated float qd_des_abad = 5;  // [4]
    repeated float qd_des_hip  = 6;  // [4]
    repeated float qd_des_knee = 7;  // [4]
    repeated float qd_des_foot = 8;  // [4] xgb 填 0

    // ===== PD 比例增益 =====
    repeated float kp_abad = 9;      // [4] STANDUP=80, RL=20
    repeated float kp_hip  = 10;     // [4]
    repeated float kp_knee = 11;     // [4]
    repeated float kp_foot = 12;     // [4] xgb 填 0

    // ===== PD 微分增益 =====
    repeated float kd_abad = 13;     // [4]
    repeated float kd_hip  = 14;     // [4] RL=0.7
    repeated float kd_knee = 15;     // [4] RL=0.7
    repeated float kd_foot = 16;     // [4] xgb 填 0

    // ===== 前馈力矩 (N·m) =====
    repeated float tau_abad_ff = 17; // [4]
    repeated float tau_hip_ff  = 18; // [4]
    repeated float tau_knee_ff = 19; // [4]
    repeated float tau_foot_ff = 20; // [4] xgb 填 0
}
```

### 2.2 站立模式实际值 (从 mc_ctrl 日志推断)

| 字段 | 大小 | STANDUP 模式 | RL_MIX 模式 |
|------|------|-------------|-------------|
| q_des_abad | 4f | ≈[-0.14, +0.14, -0.03, +0.03] | 策略输出 |
| q_des_hip | 4f | ≈[0.85, 0.85, 0.82, 0.82] | 策略输出 |
| q_des_knee | 4f | ≈[-1.50, -1.49, -1.50, -1.50] | 策略输出 |
| kp_abad | 4f | 80 | 20 |
| kp_hip | 4f | 80 | 20 |
| kp_knee | 4f | 80 | 20 |
| kd_hip | 4f | 0.7 | 0.7 |
| kd_knee | 4f | 0.7 | 0.7 |

### 2.3 关节顺序

**所有字段按 [FR, FL, RR, RL] 排列:**
- FR (右前) = FAR_ABAD/HIP/KNEE
- FL (左前) = FBL_ABAD/HIP/KNEE
- RR (右后) = RAR_ABAD/HIP/KNEE
- RL (左后) = RBL_ABAD/HIP/KNEE

---

## 3. RobotState 数据结构 (robot_mujoco → mc_ctrl)

### 3.1 Protobuf 字段定义 (robot_sdk.pb.h 编译确认)

```protobuf
message RobotState {
    // ===== 关节位置 (rad) =====
    repeated float q_abad      = 1;   // [4] FR,FL,RR,RL 外展关节角
    repeated float q_hip       = 2;   // [4] 髋关节角
    repeated float q_knee      = 3;   // [4] 膝关节角
    repeated float q_foot      = 4;   // [4] xgb 填 0

    // ===== 关节速度 (rad/s) =====
    repeated float qd_abad     = 5;   // [4] 外展速度
    repeated float qd_hip      = 6;   // [4] 髋速度
    repeated float qd_knee     = 7;   // [4] 膝速度
    repeated float qd_foot     = 8;   // [4] xgb 填 0

    // ===== 力矩反馈 (N·m) =====
    repeated float tau_abad_fb = 9;   // [4] 来自 jointactuatorfrc 传感器
    repeated float tau_hip_fb  = 10;  // [4]
    repeated float tau_knee_fb = 11;  // [4]
    repeated float tau_foot_fb = 12;  // [4] xgb 填 0

    // ===== 姿态 =====
    repeated float quat        = 13;  // [4] 四元数 [w, x, y, z]
    repeated float gyro        = 14;  // [3] 陀螺仪 (rad/s) — 世界坐标系
    repeated float acc         = 15;  // [3] 加速度计 (m/s²) — 世界坐标系

    // ===== 欧拉角 =====
    repeated float rpy         = 16;  // [3] roll, pitch, yaw (rad)

    // ===== 时间戳 =====
    uint64       time_stamp    = 17;  // 纳秒时间戳

    // ===== 位置 =====
    repeated float position    = 19;  // [3] 基座位置 (x, y, z) 世界坐标系

    // ===== 线速度 =====
    repeated float v_world     = 20;  // [3] 世界坐标系线速度 (vx, vy, vz)
}
```

**注意:** field 18 (depth) 为空, field 编号不连续。

### 3.2 MuJoCo sensordata → RobotState 字段映射

```
MuJoCo sensordata 布局 (nsensordata=84):
┌─────────────────────────────────────────────────────────────────────┐
│ [0:11]   jointpos     12×1  (FR,FL,RR,RL × abad,hip,knee)         │
│ [12:23]  jointvel     12×1  (同上)                                  │
│ [24:35]  jointactfrc  12×1  (同上, 力矩反馈)                        │
│ [36:39]  imu_quat      4    (framequat, w,x,y,z)                   │
│ [40:42]  imu_gyro      3    (gyro, x,y,z)                          │
│ [43:45]  imu_acc       3    (accelerometer, x,y,z)                  │
│ [46:48]  frame_pos     3    (framepos, x,y,z)                       │
│ [49:51]  frame_vel     3    (framelinvel, x,y,z)                    │
│ [52:83]  livox/camera IMU (不使用)                                   │
└─────────────────────────────────────────────────────────────────────┘
```

**原始 robot_mujoco 的传感器索引 (从日志确认):**
```
[CheckSensor] 找到 IMU 传感器，索引: 36  → sensordata[36] 开始
[CheckSensor] 找到 Frame 传感器，索引: 39 → sensordata[39] 开始
```

| RobotState 字段 | sensordata 索引 | 维度 | 坐标系 |
|----------------|----------------|------|--------|
| q_abad[0..3] | qpos[7,10,13,16] | 4 | — |
| q_hip[0..3] | qpos[8,11,14,17] | 4 | — |
| q_knee[0..3] | qpos[9,12,15,18] | 4 | — |
| qd_abad[0..3] | qvel[6,9,12,15] | 4 | — |
| qd_hip[0..3] | qvel[7,10,13,16] | 4 | — |
| qd_knee[0..3] | qvel[8,11,14,17] | 4 | — |
| tau_abad_fb[0..3] | sensordata[24,27,30,33] | 4 | — |
| tau_hip_fb[0..3] | sensordata[25,28,31,34] | 4 | — |
| tau_knee_fb[0..3] | sensordata[26,29,32,35] | 4 | — |
| quat[0..3] | sensordata[36:39] | 4 | 世界坐标系 (w,x,y,z) |
| gyro[0..2] | sensordata[40:42] | 3 | **世界坐标系** |
| acc[0..2] | sensordata[43:45] | 3 | **世界坐标系** |
| rpy[0..2] | 由 quat 计算 | 3 | — |
| position[0..2] | sensordata[46:48] | 3 | 世界坐标系 |
| v_world[0..2] | sensordata[49:51] | 3 | **世界坐标系** |

**关键: gyro/acc/v_world 均为世界坐标系**, 直接来自 MuJoCo 传感器, 无旋转变换。

---

## 4. mc_ctrl 内部数据结构 (RL 策略观测)

### 4.1 RL 策略网络架构 (从 run_mc.log 提取)

| 策略 | 模型路径 | 输入维度 | LSTM | 输出 |
|------|---------|---------|------|------|
| **policy_mix_walk** | policy_mix_walk | **48** | 512 | 12 |
| odom_mix_walk | odom_mix_walk | 29 | 512 | 3 |
| policy_mlp | policy_mlp | 45 | — (MLP) | 12 |
| policy_mix_backflip | policy_mix_backflip | 49 | 512 | 12 |
| policy_balancestand | policy_balancestand_withyaw_0423 | 51 | 512 | 12 |

### 4.2 Walk 策略输入推测 (48 维)

基于 RobotState 的 20 个 protobuf 字段:

| 分量 | 维度 | 来源 |
|------|------|------|
| q_joint (12) | 12 | q_abad[4] + q_hip[4] + q_knee[4] |
| qd_joint (12) | 12 | qd_abad[4] + qd_hip[4] + qd_knee[4] |
| tau_fb (12) | 12 | tau_abad_fb[4] + tau_hip_fb[4] + tau_knee_fb[4] |
| quat (4) | 4 | quat[4] |
| gyro (3) | 3 | gyro[3] |
| acc (3) | 3 | acc[3] |
| rpy (3) | 3 | rpy[3] |
| position (3) | 3 | position[3] |
| **小计** | **45** | |
| v_command (3) | 3 | 速度指令 (vx,vy,yaw_rate) |
| **总计** | **48** | |

**注:** tau_foot_fb[4]=0 不计入, v_world[3] 可能被 odom 模型替代。

### 4.3 Odom 模型输入 (29 维)

| 分量 | 维度 |
|------|------|
| q_joint (12) | 12 |
| qd_joint (12) | 12 |
| gyro (3) | 3 |
| quat (3, 可能用 rpy 或 quat_xyz) | 3 |
| **总计** | **29** → 输出 3 (速度估计) |

### 4.4 Walk 策略输出 (12 维)

输出 12 个关节的目标位置, 顺序与 RobotCmd 一致:
```
[FR_abad, FR_hip, FR_knee, FL_abad, FL_hip, FL_knee,
 RR_abad, RR_hip, RR_knee, RL_abad, RL_hip, RL_knee]
```

mc_ctrl 将策略输出包装为 RobotCmd, 附加 PD 增益 (kp, kd)。

---

## 5. 控制律 (robot_mujoco ApplyControl)

```
tau = kp * (q_des - q) + kd * (qd_des - qd) + tau_ff + qfrc_bias
tau = clamp(tau, -28, +28)
```

**关键要素:**
- **qfrc_bias**: MuJoCo 的 `data_->qfrc_bias[vel_idx]` 包含重力项 (C(q,q̇) + G(q)), 用作重力补偿前馈
- **±28 Nm 限幅**: 与 xgb.xml `actuatorfrcrange="-28 28"` 一致
- **无 qfrc_bias 时**: kp=20 无法对抗 ~10Nm 重力矩, 导致关节持续误差 → 策略发散

**日志证据 (robot_mujoco 使用 qfrc_bias):**
```
$ strings robot_mujoco | grep qfrc_bias
qfrc_bias    ← 原始二进制中包含此符号
```

---

## 6. UDP 9999 渲染数据 (robot_mujoco → UE)

| 字段 | 大小 | 说明 |
|------|------|------|
| sim_time | 8B (double) | 仿真时间 (秒) |
| qpos[19] | 76B (19×float) | [0:2]=base_pos, [3:6]=base_quat(w,x,y,z), [7:18]=12关节角 |
| qvel[18] | 72B (18×float) | [0:2]=base_linvel, [3:5]=base_angvel, [6:17]=12关节速度 |
| tau[12] | 48B (12×float) | 12个关节力矩 (sensordata[24:35]) |
| **总计** | **412 bytes** | |

**频率:** ~100Hz (每包间隔 ~10ms), 与 eCAL 500Hz 解耦。

---

## 7. FSM 状态机 (站立模式)

```
PASSIVE → JOINT_PD → STANDUP → RL_MLP → RL_MIX
                                        ↓
                                   RL_Walk (按W)
                                        ↓
                                   RL_FlipOver (失控时)
                                        ↓
                                   RL_Walk (恢复)
```

**当前状态 (从 run_mc.log):**
```
body height is 0.321377,Stand up
[FSM_RLMIX] on Enter!
[RL_Walk] on Enter!
Operating Mode: NORMAL in RL_MIX
```

**控制周期:** 2ms (500Hz), `low_level_dt: 0.002`

---

## 8. 传感器索引完整对照表

| sensor_index | name | dim | sensordata adr | 用途 |
|---|---|---|---|---|
| 0-11 | jointpos (FR→RL) | 12 | 0-11 | 关节位置 |
| 12-23 | jointvel (FR→RL) | 12 | 12-23 | 关节速度 |
| 24-35 | jointactuatorfrc | 12 | 24-35 | 力矩反馈 |
| 36 | **imu_quat** | 4 | **36-39** | 基座四元数 (w,x,y,z) |
| 40 | **imu_gyro** | 3 | **40-42** | 角速度 (世界坐标系) |
| 43 | **imu_acc** | 3 | **43-45** | 加速度 (世界坐标系, 含重力) |
| 46 | **frame_pos** | 3 | **46-48** | 基座位置 (世界坐标系) |
| 49 | **frame_vel** | 3 | **49-51** | 基座线速度 (世界坐标系) |
| 52+ | livox/camera IMU | — | 52-83 | 不使用 |

**注意:** sensor_index 是传感器编号 (0-based), sensordata adr 是数据在 `data_->sensordata[]` 中的起始偏移。两者不同! 例如 sensor_index=40 的 imu_gyro 对应 sensordata adr=40。

