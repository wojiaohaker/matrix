# CarlaUnreal MuJoCo 双模式集成——完整架构分析

## 概述

本文档对 CarlaUnreal MuJoCoUE 插件的两种运行模式进行完整的技术分析。两种模式分别对标 Matrix 的「非 MuJoCo 物理模式」和「MuJoCo 物理模式」，核心区别在于物理引擎运行位置和 mc_ctrl 通信方式。

**模式速查：**

| CarlaUnreal 设置 | Matrix 对应 | 物理引擎位置 | mc_ctrl 通信 |
|---|---|---|---|
| `bExternalPhysicsMode = false` | 不勾选 mujoco 物理 | UE 内部 MuJoCo | type=8, UDP 双向 |
| `bExternalPhysicsMode = true` | 勾选 mujoco 物理 | mujoco_sim 独立进程 | type=5, eCAL |

**整体架构对照：**

```
┌─────────────────────────────────────────────────────────────────────────────────┐
│                          Matrix 原始架构                                          │
│                                                                                  │
│  非 MuJoCo 模式 (type=8):              MuJoCo 模式 (type=5):                      │
│  ┌──────────────┐  UDP  ┌─────────┐    ┌──────────────┐  eCAL  ┌─────────┐     │
│  │  UE (物理+渲染)│◄────►│ mc_ctrl │    │ robot_mujoco │◄──────►│ mc_ctrl │     │
│  │  FMujocoWkr   │ 25001│ type=8  │    │ (物理+GLFW)   │        │ type=5  │     │
│  │  Thread 500Hz │ 25002│         │    │ 500Hz mj_step│        │         │     │
│  └──────────────┘      └─────────┘    └──────┬───────┘        └─────────┘     │
│       2 个进程                                │ UDP 25001                        │
│                                        ┌──────▼───────┐                         │
│                                        │ zsibot_ue    │                         │
│                                        │ (纯 FK 渲染)  │                         │
│                                        └──────────────┘                         │
│                                          3 个进程                                 │
├─────────────────────────────────────────────────────────────────────────────────┤
│                          CarlaUnreal 集成架构                                     │
│                                                                                  │
│  bExternalPhysicsMode=false:           bExternalPhysicsMode=true:                 │
│  ┌──────────────┐  UDP  ┌─────────┐    ┌──────────────┐  eCAL  ┌─────────┐     │
│  │ CarlaUnreal   │◄────►│ mc_ctrl │    │ mujoco_sim   │◄──────►│ mc_ctrl │     │
│  │ (物理+渲染)   │ 25001│ type=8  │    │ (物理+GLFW)   │        │ type=5  │     │
│  │ SimulateMc() │ 25002│         │    │ 500Hz mj_step│        │ type=5  │     │
│  │ mj_step 在    │      │         │    │ 独立物理线程   │        │         │     │
│  │ UE Tick 中   │      │         │    └──────┬───────┘        └─────────┘     │
│  └──────────────┘      └─────────┘           │ UDP 25001                        │
│       2 个进程                         ┌──────▼───────┐                         │
│                                       │ CarlaUnreal   │                         │
│                                       │ (纯 FK 渲染)   │                         │
│                                       │ TickExtPhys() │                         │
│                                       └──────────────┘                         │
│                                         3 个进程                                 │
└─────────────────────────────────────────────────────────────────────────────────┘
```

---

# 第一部分：已实现模式 (bExternalPhysicsMode = false)

## 1. 对标 Matrix 关系

本模式完全对标 Matrix 的「非 MuJoCo 物理模式」（不勾选 mujoco 物理）。Matrix 中 UE 内嵌 MuJoCo 引擎，通过 `FMujocoWorkerThread` 独立线程以 ~500Hz 运行 `mj_step()`，mc_ctrl 通过 `UnrealCommandInterface` (type=8) 以 UDP 双向通信。

CarlaUnreal 中的实现：`AMuJoCoSimulation::SimulateMuJoCo()` 在 UE 的 `Tick()` 中执行物理步进，通过 `UdpReceiverComponent` (ParseRobotCmd 模式) 和 `UdpSenderComponent` 与 mc_ctrl 通信。

## 2. 进程架构

```
┌────────────────────────────────────────────────────────────────────────┐
│                         CarlaUnreal (UE)                                │
│                                                                         │
│  AMuJoCoSimulation Actor                                                │
│  ├── BeginPlay()                                                        │
│  │   ├── LoadModel(XML) → mj_loadXML + mj_makeData                     │
│  │   ├── 设置初始 LIEDOWN 姿态 (HIP=1.4, KNEE=-2.4)                    │
│  │   ├── mj_forward() → 生成 mesh (14 body + 69 geom)                  │
│  │   └── StartSimulation() → bSimulationRunning = true                  │
│  │                                                                      │
│  ├── Tick(DeltaTime) → SimulateMuJoCo(DeltaTime)                        │
│  │   ├── while (mData->time - startTime < DeltaTime):                   │
│  │   │   ├── [启动延迟 3s] → 零力矩, 重力趴下                           │
│  │   │   ├── [UDP 活跃] → ApplyUdpControl(gainScale)                    │
│  │   │   │   ├── 读取 UdpReceiver->LastCommand                          │
│  │   │   │   ├── 关节目标映射: Protocol → MuJoCo                        │
│  │   │   │   ├── 目标混合: lerp(激活角度, mc_ctrl目标, gainScale)        │
│  │   │   │   ├── PD: τ = Kp*(target-q) - Kd*q̇ + τ_ff                  │
│  │   │   │   └── 限幅: ±28 Nm                                           │
│  │   │   ├── [本地站立] → ApplyStandUpControl()                         │
│  │   │   ├── mj_step(mModel, mData) ← 物理步进                          │
│  │   │   └── SendStateToMcCtrl() → UDP 25001                            │
│  │   ├── ExtractCurrentState() → xpos/xquat                             │
│  │   └── UpdateSimulationView() → 更新 UE 组件                           │
│  │                                                                       │
│  ├── UdpReceiverComponent (ParseRobotCmd 模式)                           │
│  │   ├── 端口: 25002                                                     │
│  │   ├── 线程: FUdpSocketReceiver (独立接收线程)                          │
│  │   ├── 解析: robot_sdk::pb::RobotCmd → FUdpCommandData                 │
│  │   ├── 数据: JointTargets[12] + KpValues[12] + KdValues[12] + TauFF[12]│
│  │   └── 可选: AsyncTask(GameThread) 调度到游戏线程                       │
│  │                                                                       │
│  └── UdpSenderComponent                                                  │
│      ├── 目标: 127.0.0.1:25001                                           │
│      ├── 构建: robot_sdk::pb::RobotState                                 │
│      ├── 数据: 关节状态(12) + IMU(10) + 位置(3) + 速度(3)                │
│      ├── 时间戳: CLOCK_REALTIME (纳秒)                                    │
│      └── 序列化: protobuf → UDP sendto                                   │
│                                                                          │
└────────────────────────────┬─────────────────────────────────────────────┘
                             │ UDP 双向通信
                             │
          ┌──────────────────┴──────────────────┐
          │                                      │
          ▼ 25002 (RobotCmd)                     ▼ 25001 (RobotState)
┌──────────────────────────────────────────────────────────────┐
│                        mc_ctrl                                 │
│                                                               │
│  UnrealCommandInterface (type=8)                              │
│  ├── initialize()                                             │
│  │   ├── 创建 UEUDPReceiver 线程 → 监听 25001                 │
│  │   └── 初始化 UDP Sender → 目标 127.0.0.1:25002             │
│  ├── sendCmd()                                                │
│  │   └── 序列化 RobotCmd (protobuf) → UDP → 127.0.0.1:25002  │
│  ├── reciveData()                                             │
│  │   └── 从 UEUDPReceiver 获取最新 RobotState (303 bytes)     │
│  │                                                            │
│  FSM 状态机:                                                   │
│  PASSIVE → JOINT_PD → STANDUP → RL_MLP → RL_MIX → RL_WalkPos │
│                                                               │
│  控制循环: [PeriodicTask] Start robot-control (0s, 2000000ns)  │
│           = 500Hz                                             │
│  手柄输入: [joystic] find 1 joystic (虚拟 F710)                │
└──────────────────────────────────────────────────────────────┘
```

## 3. 关键源码分析

### 3.1 BeginPlay 初始化流程

```cpp
// MuJoCoSimulation.cpp: BeginPlay()
// bExternalPhysicsMode = false 分支:

// ① 设置初始 LIEDOWN 姿态
// 为什么需要? mc_ctrl 的 STANDUP FSM 检查 body_height (通过 FK 计算)。
// 如果初始高度太高 (~0.37m 站立), STANDUP FSM 会卡住。
// 设置 HIP=1.4, KNEE=-2.4 → FK 计算 body_height ≈ 0.05m (趴地)。
if (mModel->nq >= 19) // 7 (freejoint) + 12 joints
{
    mData->qpos[2] = 0.10; // base z: 低, 接近地面
    for (int j = 0; j < 4; j++)
    {
        mData->qpos[7 + j*3 + 0] = 0.0f;   // ABAD = 0
        mData->qpos[7 + j*3 + 1] = 1.4f;   // HIP  = 1.4 (liedown)
        mData->qpos[7 + j*3 + 2] = -2.4f;  // KNEE = -2.4 (liedown)
    }
}
mj_forward(mModel, mData);

// ② 生成渲染 mesh
_info = ExtractModelInfo(mModel);
ConvertMuJoCoModelToProceduralMeshes(mModel, this); // 自定义网格 → ProceduralMesh
GenerateMeshes(_info);                               // body SceneComponent + geom StaticMesh

// ③ 启动物理仿真 (立即开始, 先趴下 3s)
SimStartWallTime = FPlatformTime::Seconds();
StartSimulation(); // bSimulationRunning = true
```

### 3.2 物理步进循环 (SimulateMuJoCo)

```cpp
// MuJoCoSimulation.cpp: SimulateMuJoCo(DeltaTime)
// 在 UE Tick 中被调用, DeltaTime 是帧间隔

double startTime = mData->time;
while (mData->time - startTime < DeltaTime)
{
    // ---- 控制优先级判定 ----

    // 1) 启动延迟期 (前 3s): 零力矩, 让机器人重力趴下
    bool bInStartupDelay = (FPlatformTime::Seconds() - SimStartWallTime) < UDP_STARTUP_DELAY;
    if (bInStartupDelay)
    {
        for (int i = 0; i < mModel->nu; i++)
            mData->ctrl[i] = 0.0; // 零力矩
    }

    // 2) UDP 控制活跃 (mc_ctrl Kp > 10):
    else if (bUdpControlEnabled && UdpReceiver->LastCommand.bValid)
    {
        double age = FPlatformTime::Seconds() - UdpReceiver->LastCommand.ReceiveTime;
        if (age < UDP_CMD_TIMEOUT) // 0.1s 超时
        {
            float maxKp = max(Cmd.KpValues);
            if (maxKp > 10.0f) // mc_ctrl 在主动控制
            {
                // 上升沿: 捕获当前角度, 重置增益爬升计时器
                if (!bUdpWasActive)
                {
                    UdpGainRampTime = 0.0f;
                    for (int j = 0; j < NUM_JOINTS; j++)
                        UdpActivationAngles[j] = mData->qpos[jntIdx]; // 捕获
                }
                UdpGainRampTime += mModel->opt.timestep;
                float gainScale = Clamp(UdpGainRampTime / 1.0f, 0, 1); // 1s 爬升
                ApplyUdpControl(gainScale);
            }
            else // mc_ctrl 在阻尼/空闲模式 (Kp<=10)
            {
                // 轻柔阻尼: 防止不受控摔倒
                mData->ctrl[i] = Clamp(-2.0f * vel, -10.0f, 10.0f);
            }
        }
    }

    // 3) 本地站立控制 (无 mc_ctrl 时):
    else if (bStandUpActive)
    {
        ApplyStandUpControl(); // 三阶段站立轨迹 + IK 步态
    }

    // ---- 物理步进 ----
    mj_step(mModel, mData);

    // ---- 状态反馈 ----
    if (bUdpControlEnabled && !bInStartupDelay)
        SendStateToMcCtrl(); // UDP 25001 → mc_ctrl
}
```

### 3.3 ApplyUdpControl — PD 控制实现

```cpp
// MuJoCoSimulation.cpp: ApplyUdpControl(gainScale)
// 核心: 将 mc_ctrl 的关节目标映射到 MuJoCo 并执行 PD 控制

// 关节顺序映射 (关键!):
// mc_ctrl Protocol 顺序: [abad×4(FR,FL,RR,RL), hip×4, knee×4]
// MuJoCo 执行器顺序:    [leg0(ABAD,HIP,KNEE), leg1, leg2, leg3]
// 映射: MuJoCo(leg*3+jointType) ← Protocol(jointType*4+leg)

for (int leg = 0; leg < 4; leg++)
{
    for (int jointType = 0; jointType < 3; jointType++)
    {
        int mjIdx = leg * 3 + jointType;       // MuJoCo actuator index
        int protoIdx = jointType * 4 + leg;     // Protocol array index

        int jntIdx = mjIdx + 1; // skip freejoint
        float currentPos = mData->qpos[mModel->jnt_qposadr[jntIdx]];
        float currentVel = mData->qvel[mModel->jnt_dofadr[jntIdx]];

        float target = Cmd.JointTargets[protoIdx];
        float kp = Cmd.KpValues[protoIdx]; // 默认 20.0
        float kd = Cmd.KdValues[protoIdx]; // 默认 0.7

        // 目标混合: 从激活瞬间角度平滑过渡到 mc_ctrl 目标
        float blendedTarget = Lerp(UdpActivationAngles[mjIdx], target, gainScale);

        // 增益爬升: 防止激活瞬间的弹射冲击
        kp *= gainScale;
        kd *= gainScale;

        // PD 控制 + 前馈
        float torque = kp * (blendedTarget - currentPos) - kd * currentVel;
        if (Cmd.TauFF.Num() > protoIdx)
            torque += Cmd.TauFF[protoIdx] * gainScale;

        mData->ctrl[mjIdx] = Clamp(torque, -28.0f, 28.0f); // ±28 Nm 限幅
    }
}
```

### 3.4 SendStateToMcCtrl — 状态反馈构建

```cpp
// MuJoCoSimulation.cpp: SendStateToMcCtrl()
// 每物理步调用, 构建 protobuf RobotState 发送给 mc_ctrl

// 关节数据映射 (与 ApplyUdpControl 相反):
// MuJoCo(leg*3+jointType) → Protocol(jointType*4+leg)
for (int leg = 0; leg < 4; leg++)
{
    for (int jointType = 0; jointType < 3; jointType++)
    {
        int mjIdx = leg * 3 + jointType;
        int protoIdx = jointType * 4 + leg;
        JointPos[protoIdx] = mData->qpos[jntIdx];
        JointVel[protoIdx] = mData->qvel[dofIdx];
        JointTau[protoIdx] = mData->qfrc_actuator[...];
    }
}

// IMU 数据:
// Quat: body 1 (torso) 的 xquat [w,x,y,z]
// Gyro: R^T * cvel[angular] (body 帧角速度)
// Acc:  R^T * [0,0,9.81] (body 帧比力, 静止时 = [0,0,+g])
// RPY:  从 quat 计算 (注意: 不发送, 匹配 Matrix 303 bytes 格式)

// 时间戳: CLOCK_REALTIME 纳秒 (必须与 mc_ctrl 时间基准一致)
struct timespec ts;
clock_gettime(CLOCK_REALTIME, &ts);
StateMsg->set_time_stamp(tv_sec * 1e9 + tv_nsec);

// 序列化 + UDP 发送
StateMsg->SerializeToArray(Buffer, MsgSize);
SenderSocket->SendTo(Buffer, MsgSize, BytesSent, *TargetAddr);
```

### 3.5 UdpReceiverComponent — 双模式接收器

```
UdpReceiverComponent 架构:

┌───────────────────────────────────────────────────────────────┐
│  UUdpReceiverComponent                                         │
│                                                                │
│  ParseMode: EUdpParseMode                                      │
│  ├── ParseRobotCmd (模式 0): bExternalPhysicsMode=false 时使用  │
│  │   ├── 端口: 25002                                           │
│  │   ├── 解析: robot_sdk::pb::RobotCmd                         │
│  │   ├── 输出: FUdpCommandData                                 │
│  │   │   ├── JointTargets[12]: [abad×4, hip×4, knee×4]        │
│  │   │   ├── JointVelocities[12]                               │
│  │   │   ├── KpValues[12], KdValues[12], TauFF[12]            │
│  │   │   └── ReceiveTime, bValid                               │
│  │   └── 线程: 接收线程 → AsyncTask(GameThread) 可选调度        │
│  │                                                             │
│  └── ParseRobotState (模式 1): bExternalPhysicsMode=true 时使用 │
│      ├── 端口: 25001                                            │
│      ├── 解析: robot_sdk::pb::RobotState                        │
│      ├── 输出: FUdpStateData                                    │
│      │   ├── QAbad[4], QHip[4], QKnee[4] (关节角度)            │
│      │   ├── QdAbad[4], QdHip[4], QdKnee[4] (关节角速度)       │
│      │   ├── Quat[4] (body 四元数 w,x,y,z)                     │
│      │   ├── Position[3] (body 位置 x,y,z)                     │
│      │   ├── Gyro[3], Acc[3] (IMU)                             │
│      │   └── ReceiveTime, bValid                                │
│      └── 线程: 接收线程直接解析, FScopeLock 保护 LastState       │
│                                                                 │
│  SetParseMode(Mode):                                            │
│  ├── 设置 ParseMode                                             │
│  └── 自动调整端口:                                               │
│      ├── → ParseRobotState: 25002 → 25001                       │
│      └── → ParseRobotCmd:   25001 → 25002                       │
│                                                                 │
│  HandleDataReceived(Data):                                      │
│  ├── if ParseRobotState:                                        │
│  │   ├── ParseProtobufState(Data, StateData)                    │
│  │   ├── FScopeLock → LastState = StateData                     │
│  │   └── OnStateReceived.Broadcast(StateData)                   │
│  └── if ParseRobotCmd:                                          │
│      ├── if bProcessOnGameThread:                               │
│      │   └── AsyncTask(GameThread) → ParseProtobufCommand       │
│      └── else: ParseProtobufCommand → LastCommand               │
└───────────────────────────────────────────────────────────────┘
```

### 3.6 UdpSenderComponent — 状态发送器

```
UdpSenderComponent 架构:

┌───────────────────────────────────────────────────────────────┐
│  UUdpSenderComponent                                           │
│                                                                │
│  配置:                                                         │
│  ├── TargetIP: "127.0.0.1"                                    │
│  ├── TargetPort: 25001 (mc_ctrl state_port)                    │
│  └── bAutoSend: false (由 MuJoCoSimulation 手动调用)            │
│                                                                │
│  UpdateState(JointPos, JointVel, JointTau, Quat, Gyro, Acc,    │
│              RPY, Position, VWorld):                            │
│  ├── 填充 robot_sdk::pb::RobotState 各字段                     │
│  ├── 关节: q_abad/hip/knee + qd_abad/hip/knee + tau_fb        │
│  ├── IMU: quat[4] + gyro[3] + acc[3]                          │
│  ├── 位置: position[3] + v_world[3]                            │
│  └── 时间戳: CLOCK_REALTIME 纳秒                               │
│                                                                │
│  SendState():                                                  │
│  ├── StateMsg->ByteSizeLong() → MsgSize                        │
│  ├── StateMsg->SerializeToArray(Buffer, MsgSize)               │
│  └── SenderSocket->SendTo(Buffer, MsgSize, *TargetAddr)        │
│                                                                │
│  注意: BeginPlay 中 bAutoSend=false, 由 SimulateMuJoCo         │
│  在每个物理步后手动调用 UpdateState()+SendState()               │
└───────────────────────────────────────────────────────────────┘
```

## 4. Protobuf 消息结构

### 4.1 RobotCmd (mc_ctrl → UE, 通过 UDP 25002)

```protobuf
// mc_ctrl 发送的关节控制命令
message RobotCmd {
  // ABAD 关节 (4 条腿)
  repeated float q_des_abad   = 1;   // 目标角度 [FR, FL, RR, RL]
  repeated float qd_des_abad  = 2;   // 目标角速度
  repeated float kp_abad      = 3;   // PD Kp
  repeated float kd_abad      = 4;   // PD Kd
  repeated float tau_abad_ff  = 5;   // 前馈力矩

  // HIP 关节
  repeated float q_des_hip    = 6;
  repeated float qd_des_hip   = 7;
  repeated float kp_hip       = 8;
  repeated float kd_hip       = 9;
  repeated float tau_hip_ff   = 10;

  // KNEE 关节
  repeated float q_des_knee   = 11;
  repeated float qd_des_knee  = 12;
  repeated float kp_knee      = 13;
  repeated float kd_knee      = 14;
  // (knee 无 tau_ff)
}
```

### 4.2 RobotState (UE → mc_ctrl, 通过 UDP 25001, ~303 bytes)

```protobuf
// 状态反馈 (与 Matrix UE 格式完全一致)
message RobotState {
  // 关节角度
  repeated float q_abad    = 1;   // [FR, FL, RR, RL]
  repeated float q_hip     = 2;
  repeated float q_knee    = 3;
  repeated float q_foot    = 4;   // xgb 无足端关节, 填 0

  // 关节角速度
  repeated float qd_abad   = 5;
  repeated float qd_hip    = 6;
  repeated float qd_knee   = 7;
  repeated float qd_foot   = 8;

  // 关节力矩反馈
  repeated float tau_abad_fb  = 9;
  repeated float tau_hip_fb   = 10;
  repeated float tau_knee_fb  = 11;
  repeated float tau_foot_fb  = 12;  // 填 0

  // IMU 数据
  repeated float quat       = 13;  // [w, x, y, z]
  repeated float gyro       = 14;  // [wx, wy, wz] body 帧
  repeated float acc        = 15;  // [ax, ay, az] body 帧

  // 时间戳 (CLOCK_REALTIME 纳秒)
  uint64 time_stamp         = 16;

  // Base 位置
  repeated float position   = 17;  // [x, y, z]

  // RPY (不发送, 匹配 Matrix 303 bytes)
  // repeated float rpy     = 18;

  // Base 世界速度
  repeated float v_world    = 19;  // [vx, vy, vz]
}
```

## 5. 通信时序

```
时间轴 (每 2ms = 500Hz):

mc_ctrl                          CarlaUnreal (UE)
  │                                    │
  │ ① RobotCmd (protobuf)              │
  │──── UDP 25002 ──────────────────►  │
  │    q_des + Kp/Kd + tau_ff          │ UdpReceiver.HandleDataReceived()
  │                                    │ → ParseProtobufCommand()
  │                                    │ → LastCommand (GameThread)
  │                                    │
  │                                    │ SimulateMuJoCo():
  │                                    │   ApplyUdpControl(gainScale)
  │                                    │   mj_step()
  │                                    │   SendStateToMcCtrl()
  │                                    │
  │ ② RobotState (protobuf, ~303B)     │
  │◄─── UDP 25001 ──────────────────── │
  │    q + qd + tau + IMU + pos        │ UdpSender.SendState()
  │                                    │
  │ UEUDPReceiver:                     │
  │   reciveData() → RobotState        │
  │                                    │
  │ FSM 计算下一步:                     │
  │   → 新的 RobotCmd                  │
  │──── UDP 25002 ──────────────────►  │ (下一个 2ms 周期)
```

## 6. 与 Matrix 非 MuJoCo 模式的详细对比

| 对比项 | Matrix 非 MuJoCo | CarlaUnreal (本模式) | 差异说明 |
|--------|-------------------|---------------------|---------|
| **物理线程** | FMujocoWorkerThread (独立线程) | UE Tick 驱动 (SimulateMuJoCo) | Matrix 物理与渲染完全解耦 |
| **物理频率** | ~500Hz (独立线程, 实测 498.70) | ~500Hz × DeltaTime (受帧率影响) | 高帧率时接近, 低帧率时下降 |
| **物理步进** | mj_step() 固定 timestep | while(mData->time < DeltaTime) 循环 | 本模式可能每帧多步或跳步 |
| **mc_ctrl 类型** | type=8 UnrealCommandInterface | 相同 type=8 | 完全一致 |
| **通信协议** | UDP 双向 (25001/25002) | 相同 UDP 双向 | 完全一致 |
| **状态包大小** | 303 bytes (protobuf) | 相同 ~303 bytes | 完全一致 |
| **初始姿态** | LIEDOWN (UE 启动即趴下) | LIEDOWN (HIP=1.4, KNEE=-2.4) | 一致 |
| **启动延迟** | 无特殊处理 | 3s 重力趴下 + 1s 增益爬升 | 本模式更保守, 防止弹射 |
| **键盘控制** | sim_launcher 内置 (XGrabKey+uinput) | keyboard_control.py (uinput) | 功能相同, 实现不同 |
| **GLFW 窗口** | 无 | 无 | 一致 |
| **进程数** | 2 (UE + mc_ctrl) | 2 (UE + mc_ctrl) | 一致 |

---

# 第二部分：待实现模式 (bExternalPhysicsMode = true)

## 7. 对标 Matrix 关系

本模式对标 Matrix 的「MuJoCo 物理模式」（勾选 mujoco 物理）。Matrix 中 `robot_mujoco` 作为独立物理进程运行 `mj_step()` @ 500Hz，通过 eCAL 与 mc_ctrl (type=5) 通信，同时通过 UDP 将 RobotState 发送给 UE 做纯 FK 渲染。

CarlaUnreal 中用 `mujoco_sim` 替代 `robot_mujoco`（开源实现），UE 端通过 `TickExternalPhysics()` 接收 UDP RobotState → 写入 qpos → `mj_kinematics()` (FK) → 更新 mesh。

## 8. 进程架构

```
┌────────────────────────────────────────────────────────────────────┐
│                     mujoco_sim (独立进程)                           │
│                     替代 Matrix robot_mujoco                        │
│                                                                     │
│  MujocoSim 类                                                       │
│  ├── Initialize(config)                                             │
│  │   ├── LoadModel()                                                │
│  │   │   └── mj_loadXML("scene_terrain_yard.xml")                  │
│  │   │   └── mj_makeData() + mj_forward()                          │
│  │   ├── eCAL::Initialize("mujoco_sim")                             │
│  │   │   ├── Publisher: "mujoco_state" (RobotState)                 │
│  │   │   └── Subscriber: "mujoco_cmd" (RobotCmd)                    │
│  │   │       └── OnRobotCmdReceived() → latest_cmd_ (mutex)         │
│  │   └── InitUdp()                                                  │
│  │       └── socket() → dest 127.0.0.1:25001 (CarlaUnreal)         │
│  │                                                                   │
│  ├── Run() → RunWithGui() (enable_gui=1)                            │
│  │   │                                                               │
│  │   ├── 物理线程 (500Hz, std::thread):                              │
│  │   │   while (running_):                                           │
│  │   │     ├── ApplyControl()                                        │
│  │   │     │   └── for each leg×joint:                               │
│  │   │     │       τ = kp*(q_des-q) + kd*(qd_des-qd) + τ_ff         │
│  │   │     │       data_->ctrl[actuator_idx] = τ                     │
│  │   │     ├── PhysicsStep()                                         │
│  │   │     │   └── mj_step(model_, data_)                            │
│  │   │     ├── PublishState()                                        │
│  │   │     │   ├── BuildRobotState() → robot_sdk::pb::RobotState     │
│  │   │     │   ├── eCAL pub->Send(state) → mc_ctrl                   │
│  │   │     │   └── UDP sendto(serialized) → CarlaUnreal:25001        │
│  │   │     └── sleep_until(next_time)  // 精确 2ms 周期              │
│  │   │                                                                │
│  │   └── 渲染主循环 (~60fps, GLFW):                                  │
│  │       mjv_updateScene() + mjr_render() + glfwSwapBuffers()        │
│  │                                                                    │
│  └── 传感器索引 (xgb.xml):                                            │
│      [0:11]  jointpos, [12:23] jointvel, [24:35] jointactuatorfrc    │
│      [36:39] framequat(imu_quat), [39:41] gyro, [42:44] acc         │
│      [45:47] framepos, [48:50] framelinvel                           │
└──────────────┬──────────────────────────────┬────────────────────────┘
               │ eCAL                          │ UDP 25001
               │                               │
    ┌──────────▼──────────┐         ┌──────────▼────────────────────────┐
    │     mc_ctrl          │         │       CarlaUnreal (UE)             │
    │                       │         │                                    │
    │  MujocoCommand        │         │  AMuJoCoSimulation                 │
    │  Interface (type=5)   │         │  bExternalPhysicsMode = true       │
    │                       │         │                                    │
    │  ┌─────────────────┐ │         │  ┌──────────────────────────────┐ │
    │  │ eCAL Subscriber │ │         │  │ BeginPlay():                  │ │
    │  │ ← mujoco_state  │ │         │  │ ├── LoadModel(XML) → mesh    │ │
    │  │ (RobotState)    │ │         │  │ ├── mj_forward() (初始 FK)   │ │
    │  ├─────────────────┤ │         │  │ ├── 禁用 mj_step             │ │
    │  │ eCAL Publisher  │ │         │  │ ├── 禁用 UdpSender           │ │
    │  │ → mujoco_cmd    │ │         │  │ └── UdpReceiver → ParseRobot │ │
    │  │ (RobotCmd)      │ │         │  │     State 模式 (端口 25001)   │ │
    │  ├─────────────────┤ │         │  │                              │ │
    │  │ GamepadReader   │ │         │  │ Tick() → TickExternalPhysics │ │
    │  │ ← 虚拟 F710     │ │         │  │ ├── GetLatestState() (mutex) │ │
    │  │ (joystic)       │ │         │  │ ├── 检查超时 (0.5s)          │ │
    │  └─────────────────┘ │         │  │ ├── 写入 mData->qpos:        │ │
    │                       │         │  │ │   [0..2] = Position (xyz)  │ │
    │  FSM 状态机:           │         │  │ │   [3..6] = Quat (wxyz)    │ │
    │  PASSIVE → JOINT_PD   │         │  │ │   [7..18] = 12 关节角度    │ │
    │  → STANDUP → RL_MIX   │         │  │ ├── mj_kinematics() (纯 FK) │ │
    │  → RL_Walk             │         │  │ ├── mj_comPos()             │ │
    │                       │         │  │ ├── ExtractCurrentState()    │ │
    │  控制循环 500Hz         │         │  │ └── UpdateSimulationView()  │ │
    │  (PeriodicTask 2ms)    │         │  │                             │ │
    └───────────────────────┘         │  └──────────────────────────────┘ │
                                      └────────────────────────────────────┘
```

## 9. mujoco_sim 详细设计

### 9.1 项目结构

```
/home/qiyuan/Softwares/Mujoco330/mujoco_sim/
├── main.cpp           # 入口: 信号处理, 配置加载, 路径解析
├── mujoco_sim.h       # MujocoSim 类 + SimConfig 配置结构
├── mujoco_sim.cpp     # 核心实现 (624 行)
├── config.yaml        # 配置文件 (与 robot_mujoco 兼容)
├── CMakeLists.txt     # 构建系统
└── run.sh             # 构建+运行脚本
```

### 9.2 SimConfig 配置结构

```cpp
// mujoco_sim.h
struct SimConfig {
    std::string robot_model_dir;    // 机器人模型目录
    std::string scene_file;         // 场景 XML 文件名

    // 通信
    std::string ecal_state_topic = "mujoco_state";   // eCAL 发布 topic
    std::string ecal_cmd_topic   = "mujoco_cmd";     // eCAL 订阅 topic
    std::string udp_target_ip    = "127.0.0.1";      // CarlaUnreal IP
    int         udp_target_port  = 25001;             // CarlaUnreal 端口
    bool        enable_udp       = true;
    bool        enable_ecal      = true;

    // 仿真
    double      sim_rate_hz      = 500.0;    // 物理步进频率
    double      publish_rate_hz  = 500.0;    // 状态发布频率
    bool        enable_gui       = true;     // GLFW 窗口
};
```

### 9.3 config.yaml (与 robot_mujoco 兼容)

```yaml
robot: "xgb"
robot_scene: "scene_terrain_yard.xml"

enable_ecal: 1           # eCAL 与 mc_ctrl 通信
enable_udp: 1            # UDP 向 CarlaUnreal 发送状态
udp_target_ip: "127.0.0.1"
udp_target_port: 25001   # CarlaUnreal 监听端口

enable_gui: 1            # GLFW 可视化窗口 (1=弹出, 0=无头)
sim_rate_hz: 500         # 物理步进 500Hz (timestep=0.002s)
publish_rate_hz: 500     # 状态发布 500Hz
```

### 9.4 物理线程核心循环

```cpp
// mujoco_sim.cpp: RunWithGui() 中的物理线程
std::thread physics_thread([this]() {
    const auto period = std::chrono::microseconds(
        static_cast<int64_t>(1e6 / config_.sim_rate_hz)); // 2000μs = 2ms
    auto next_time = std::chrono::steady_clock::now();

    while (running_.load()) {
        next_time += period;

        // ① 应用 PD 控制 (从 eCAL 接收的最新 cmd)
        ApplyControl();

        // ② 物理步进
        PhysicsStep();  // → mj_step(model_, data_)

        // ③ 发布状态 (eCAL + UDP)
        PublishState();

        step_count_++;

        // 精确周期等待 (补偿计算时间)
        std::this_thread::sleep_until(next_time);
    }
});
```

### 9.5 ApplyControl — PD 控制律

```cpp
// mujoco_sim.cpp: ApplyControl()
// 与 Matrix robot_mujoco 完全相同的 PD 控制律
// 与 CarlaUnreal ApplyUdpControl 的区别:
//   - mujoco_sim: 直接写 data_->ctrl, 无 gain ramp
//   - ApplyUdpControl: 有 gain ramp + target blending

for (int leg = 0; leg < 4; leg++) {
    int jnt_idx = 7 + leg * 3;   // qpos 中的关节起始 (skip freejoint)
    int vel_idx = 6 + leg * 3;   // qvel 中的关节起始
    int ctrl_idx = leg * 3;      // ctrl 中的执行器起始

    // ABAD
    double q = data_->qpos[jnt_idx + 0];
    double qd = data_->qvel[vel_idx + 0];
    double tau = cmd.kp_abad(leg) * (cmd.q_des_abad(leg) - q)
               + cmd.kd_abad(leg) * (cmd.qd_des_abad(leg) - qd)
               + cmd.tau_abad_ff(leg);
    data_->ctrl[ctrl_idx + 0] = tau;

    // HIP, KNEE 同理...
}
```

### 9.6 BuildRobotState — 状态构建

```cpp
// mujoco_sim.cpp: BuildRobotState()
// 构建与 Matrix robot_mujoco 完全相同的 RobotState protobuf

// 关节数据 (从 qpos/qvel 直接读取):
for (int leg = 0; leg < 4; leg++) {
    int qpos_idx = 7 + leg * 3;
    int qvel_idx = 6 + leg * 3;
    state.add_q_abad(data_->qpos[qpos_idx + 0]);
    state.add_q_hip(data_->qpos[qpos_idx + 1]);
    state.add_q_knee(data_->qpos[qpos_idx + 2]);
    // ... qd, tau 同理
}

// 传感器数据 (从 sensordata 读取, 索引与 xgb.xml <sensor> 定义对应):
// [36:39] imu_quat (framequat, w,x,y,z)
// [39:41] imu_gyro (gyro)
// [42:44] imu_acc (accelerometer)
// [45:47] frame_pos (framepos, x,y,z)
// [48:50] frame_linvel (framelinvel)

// 时间戳: CLOCK_REALTIME 纳秒 (与 mc_ctrl 时间基准一致)
auto ns = chrono::duration_cast<nanoseconds>(now.time_since_epoch()).count();
state.set_time_stamp(ns);
```

### 9.7 PublishState — 双通道发布

```cpp
// mujoco_sim.cpp: PublishState()
void MujocoSim::PublishState() {
    auto state = BuildRobotState();

    // 通道 1: eCAL → mc_ctrl (type=5)
    if (config_.enable_ecal && ecal_pub_) {
        ecal_pub_->Send(state);  // topic: "mujoco_state"
    }

    // 通道 2: UDP → CarlaUnreal (渲染用)
    if (config_.enable_udp) {
        std::string serialized;
        state.SerializeToString(&serialized);
        SendUdp(serialized);  // → 127.0.0.1:25001
    }

    publish_count_++;
}
```

## 10. CarlaUnreal 外部物理模式实现

### 10.1 BeginPlay 初始化

```cpp
// MuJoCoSimulation.cpp: BeginPlay()
// bExternalPhysicsMode = true 分支:

// ① 加载模型 (仅用于生成渲染 mesh)
mj_forward(mModel, mData);  // 初始 FK, 生成有效姿态
_info = ExtractModelInfo(mModel);
ConvertMuJoCoModelToProceduralMeshes(mModel, this);
GenerateMeshes(_info);

// ② 禁用内部物理循环
bSimulationRunning = false;  // SimulateMuJoCo 不会被调用

// ③ 禁用 mc_ctrl UDP 控制
// 原因: 此模式下 mc_ctrl 通过 eCAL 与 mujoco_sim 通信, 不直接与 UE 交互
bUdpControlEnabled = false;
if (UdpSender) UdpSender->bAutoSend = false;  // 不向 mc_ctrl 发状态

// ④ UdpReceiver 切换到 RobotState 解析模式
// 关键: 从 ParseRobotCmd (25002) 切换到 ParseRobotState (25001)
if (UdpReceiver)
{
    UdpReceiver->StopListening();
    UdpReceiver->SetParseMode(EUdpParseMode::ParseRobotState);
    // SetParseMode 内部: ListenPort 25002 → 25001
    UdpReceiver->bAutoStart = false;
    UdpReceiver->StartListening();
}

// 日志确认
UE_LOG(LogTemp, Warning, TEXT("[EXT-PHYSICS] External physics mode ENABLED. "
    "UdpReceiver listening on port %d for RobotState."), ListenPort);
UE_LOG(LogTemp, Warning, TEXT("[EXT-PHYSICS] Internal mj_step DISABLED. "
    "Rendering driven by UDP RobotState."));
```

### 10.2 TickExternalPhysics — 纯 FK 渲染循环

```cpp
// MuJoCoSimulation.cpp: TickExternalPhysics(DeltaTime)
// 每帧调用, 替代 SimulateMuJoCo

// ① 获取最新状态 (线程安全, FScopeLock)
FUdpStateData State = UdpReceiver->GetLatestState();

if (!State.bValid) return;  // 还没收到数据

// ② 检查数据新鲜度
double age = FPlatformTime::Seconds() - State.ReceiveTime;
if (age > EXT_STATE_TIMEOUT)  // 0.5s 超时
{
    // 超时警告 (节流)
    return;
}

// ③ 写入 mData->qpos (为 FK 计算准备数据)
// Base position [x, y, z] → qpos[0..2]
mData->qpos[0] = State.Position[0];
mData->qpos[1] = State.Position[1];
mData->qpos[2] = State.Position[2];

// Base quaternion [w, x, y, z] → qpos[3..6]
mData->qpos[3] = State.Quat[0]; // w
mData->qpos[4] = State.Quat[1]; // x
mData->qpos[5] = State.Quat[2]; // y
mData->qpos[6] = State.Quat[3]; // z

// 12 个关节角度 → qpos[7..18]
// RobotState: [abad×4, hip×4, knee×4] per type
// MuJoCo qpos: leg0(abad,hip,knee), leg1(abad,hip,knee), ...
for (int leg = 0; leg < 4; leg++)
{
    int qposBase = 7 + leg * 3;
    mData->qpos[qposBase + 0] = State.QAbad[leg];
    mData->qpos[qposBase + 1] = State.QHip[leg];
    mData->qpos[qposBase + 2] = State.QKnee[leg];
}

// ④ 纯运动学 (无动力学! 不调用 mj_step)
mj_kinematics(mModel, mData);  // 正向运动学: 计算所有 body 的 xpos/xquat
mj_comPos(mModel, mData);      // 质心位置

// ⑤ 更新 UE mesh 位置
ExtractCurrentState(_info);     // 从 mData->xpos/xquat 提取
UpdateSimulationView(_info);    // 更新 SceneComponent + StaticMeshComponent
```

**核心区别**: 不调用 `mj_step()` (无动力学), 只做 `mj_kinematics()` (FK)。物理完全由 mujoco_sim 驱动。

### 10.3 Tick 入口

```cpp
// MuJoCoSimulation.cpp: Tick(DeltaTime)
void AMuJoCoSimulation::Tick(float DeltaTime)
{
    Super::Tick(DeltaTime);

    // 外部物理模式: 渲染来自 UDP 状态, 无内部 mj_step
    if (bExternalPhysicsMode)
    {
        TickExternalPhysics(DeltaTime);
        return;  // ← 直接返回, 不走 SimulateMuJoCo
    }

    // 内部物理模式: 正常 SimulateMuJoCo
    if (bSimulationRunning)
        SimulateMuJoCo(DeltaTime);
}
```

## 11. 启动顺序与时序

```
时间轴 ──────────────────────────────────────────────────────────────►

① 启动 mujoco_sim (物理服务器)
   │ $ cd /home/qiyuan/Softwares/Mujoco330/mujoco_sim/build
   │ $ ./mujoco_sim ../config.yaml
   │
   ├── [Config] 加载完成: robot=xgb, scene=scene_terrain_yard.xml
   ├── [MuJoCo] 模型加载成功: xgb/scene_terrain_yard.xml
   │   ├── nq=19, nv=18, nu=12, njnt=13, nbody=15
   │   ├── timestep=0.002 (500Hz)
   │   └── nsensor=51, nsensordata=84
   ├── [eCAL] 初始化完成: pub=mujoco_state, sub=mujoco_cmd
   ├── [UDP] 初始化完成: 127.0.0.1:25001 (CarlaUnreal)
   ├── [GUI] 窗口已打开. ESC=退出, Backspace=重置
   └── 物理线程启动: mj_step @ 500Hz (无控制, 机器人重力趴下)
       ├── step=5000, time=10.0
       ├── step=10000, time=20.0
       └── ... (持续运行)

② 启动 CarlaUnreal (UE 编辑器/打包版)
   │ 关卡中 MuJoCoSimulation Actor: bExternalPhysicsMode=true
   │
   ├── BeginPlay()
   │   ├── LoadModel("model/xgb/scene_terrain_yard.xml")
   │   ├── mj_forward() → 初始 FK
   │   ├── 生成 mesh (14 body SceneComponent + 69 geom StaticMesh)
   │   ├── bSimulationRunning = false (禁用 mj_step)
   │   ├── bUdpControlEnabled = false (禁用 UDP 控制)
   │   ├── UdpSender->bAutoSend = false (禁用状态发送)
   │   └── UdpReceiver: ParseRobotState 模式, 端口 25001
   │       └── "Listening on port 25001 for Protobuf RobotState."
   │
   └── Tick() → TickExternalPhysics()
       ├── 等待 mujoco_sim 的 RobotState...
       ├── 收到第一个 State → 写入 qpos → mj_kinematics → 渲染
       └── 持续渲染 (每帧更新 mesh 位置)

③ sleep 7s (等待 UE 加载完成)

④ 启动 mc_ctrl (Matrix 运动控制器, type=5)
   │ $ cd /home/qiyuan/Softwares/Matrix/src/robot_mc
   │ $ ./run_mc.sh r
   │
   ├── [MujocoCommandInterface] initialize...    ← 确认 type=5
   ├── eCAL 连接: sub=mujoco_state, pub=mujoco_cmd
   ├── 加载 ONNX RL 模型
   ├── 初始化 FSM: PASSIVE → JOINT_PD → STANDUP → RL_MIX → ...
   ├── [joystic] find 1 joystic                  ← 虚拟 F710
   ├── [PeriodicTask] Start robot-control (0s, 2000000ns) ← 500Hz
   │
   └── 控制循环:
       ├── eCAL subscribe → mujoco_state ← mujoco_sim (RobotState)
       ├── FSM 计算 → RobotCmd
       └── eCAL publish → mujoco_cmd → mujoco_sim (RobotCmd)

⑤ 用户按 U 键 (站立)
   │
   ├── sim_launcher / keyboard_control.py
   │   └── 虚拟 F710: LB+Y
   │
   ├── mc_ctrl GamepadReader → FSM: PASSIVE → STANDUP
   │   └── 计算站立轨迹 → 12 关节目标
   │
   ├── eCAL: mujoco_cmd → mujoco_sim
   │   └── OnRobotCmdReceived() → latest_cmd_
   │
   ├── mujoco_sim 物理线程:
   │   ├── ApplyControl() → PD: τ = kp*(q_des-q) + kd*(qd_des-qd)
   │   ├── mj_step() → 物理推进 → 机器人站立
   │   └── PublishState():
   │       ├── eCAL: mujoco_state → mc_ctrl (状态反馈)
   │       └── UDP: RobotState → CarlaUnreal:25001
   │
   └── CarlaUnreal TickExternalPhysics():
       ├── GetLatestState() → 新的 RobotState
       ├── 写入 qpos → mj_kinematics (FK)
       └── UpdateSimulationView() → 渲染站立
```

## 12. 通信协议详细对比

### 12.1 本模式 (bExternalPhysicsMode=true) 通信链路

```
┌──────────────────────────────────────────────────────────────────┐
│                                                                  │
│  mc_ctrl ◄──── eCAL: mujoco_state (RobotState) ──── mujoco_sim  │
│     │                                                  │         │
│     │  eCAL: mujoco_cmd (RobotCmd)                     │         │
│     └──────────────────────────────────────────────────►│         │
│                                                         │         │
│                              UDP 25001 (RobotState)     │         │
│                              ──────────────────────────►│ CarlaUnreal│
│                                                         │         │
│  注意: CarlaUnreal 只接收, 不发送                         │         │
│  注意: mc_ctrl 不直接与 CarlaUnreal 通信                  │         │
│                                                                  │
└──────────────────────────────────────────────────────────────────┘
```

| 链路 | 方式 | Topic/端口 | 消息 | 频率 | 发送方 → 接收方 |
|------|------|-----------|------|------|----------------|
| ① | eCAL | mujoco_cmd | RobotCmd | 500Hz | mc_ctrl → mujoco_sim |
| ② | eCAL | mujoco_state | RobotState | 500Hz | mujoco_sim → mc_ctrl |
| ③ | UDP | 25001 | RobotState (protobuf) | 500Hz | mujoco_sim → CarlaUnreal |

### 12.2 与本模式 (bExternalPhysicsMode=false) 通信对比

| 对比项 | false (内部物理) | true (外部物理) |
|--------|-----------------|----------------|
| mc_ctrl ↔ UE | UDP 双向 (25001+25002) | **无直接通信** |
| mc_ctrl ↔ 物理引擎 | 无 (UE 自己跑物理) | eCAL 双向 (mujoco_cmd/state) |
| 物理引擎 → UE | 无 (UE 自己就是物理) | UDP 25001 (RobotState) |
| UE 发送 | UDP 25001 (RobotState to mc_ctrl) | **不发送** |
| UE 接收 | UDP 25002 (RobotCmd from mc_ctrl) | UDP 25001 (RobotState from mujoco_sim) |
| 总链路数 | 2 (UDP 双向) | 3 (2 eCAL + 1 UDP) |

## 13. 数据流完整路径

### 13.1 关节控制命令路径

```
用户按 W 键
  → keyboard_control.py / sim_launcher → 虚拟 F710 (ABS_Y=+32767)
  → mc_ctrl GamepadReader → FSM (RL_Walk)
  → RL 策略网络 (ONNX) → 12 关节目标 + PD 参数
  → RobotCmd protobuf:
      q_des_abad[4] = [0.0, 0.0, 0.0, 0.0]
      q_des_hip[4]  = [0.8, 0.8, 0.8, 0.8]
      q_des_knee[4] = [-1.5, -1.5, -1.5, -1.5]
      kp_hip[4]     = [20.0, 20.0, 20.0, 20.0]
      kd_hip[4]     = [0.7, 0.7, 0.7, 0.7]
      ...
  → eCAL publish (mujoco_cmd topic)
  → mujoco_sim OnRobotCmdReceived():
      latest_cmd_ = msg (mutex 保护)
  → 下一物理步 ApplyControl():
      τ = 20.0 * (0.8 - q_hip) + 0.7 * (0 - qd_hip) + tau_ff
      data_->ctrl[hip_actuator] = τ
  → mj_step() → 物理推进
```

### 13.2 状态反馈路径 (到 mc_ctrl)

```
mujoco_sim PublishState():
  → BuildRobotState():
      q_abad[4]  = {qpos[7], qpos[10], qpos[13], qpos[16]}
      q_hip[4]   = {qpos[8], qpos[11], qpos[14], qpos[17]}
      q_knee[4]  = {qpos[9], qpos[12], qpos[15], qpos[18]}
      quat[4]    = sensordata[36:39]  (imu_quat w,x,y,z)
      gyro[3]    = sensordata[39:41]  (imu_gyro)
      acc[3]     = sensordata[42:44]  (imu_acc)
      position[3]= sensordata[45:47]  (framepos)
  → eCAL pub->Send(state) (mujoco_state topic)
  → mc_ctrl eCAL Subscriber → reciveData()
  → FSM 状态估计 → 下一步控制
```

### 13.3 渲染路径 (到 CarlaUnreal)

```
mujoco_sim PublishState() (同一帧):
  → state.SerializeToString(&serialized)
  → UDP sendto(127.0.0.1:25001)

CarlaUnreal UdpReceiver (独立接收线程):
  → HandleDataReceived(Data)
  → ParseProtobufState(Data, StateData):
      robot_sdk::pb::RobotState::ParseFromArray(Data)
      StateData.QAbad[4] = StateMsg.q_abad(0..3)
      StateData.QHip[4]  = StateMsg.q_hip(0..3)
      StateData.QKnee[4] = StateMsg.q_knee(0..3)
      StateData.Quat[4]  = StateMsg.quat(0..3)
      StateData.Position[3] = StateMsg.position(0..2)
  → FScopeLock → LastState = StateData

CarlaUnreal TickExternalPhysics() (游戏线程):
  → GetLatestState() → State (FScopeLock 读取)
  → mData->qpos[0..2] = State.Position (base xyz)
  → mData->qpos[3..6] = State.Quat (base wxyz)
  → mData->qpos[7..18] = 12 关节角度
  → mj_kinematics(mModel, mData)
      计算所有 body 的世界坐标 xpos[15×3]
      计算所有 body 的世界四元数 xquat[15×4]
  → mj_comPos(mModel, mData)
      计算质心位置
  → ExtractCurrentState(_info)
      _info.bodies[i].pos = mData->xpos[i*3 : i*3+3]
      _info.bodies[i].quat = mData->xquat[i*4 : i*4+4]
      _info.bodies[i].quat2 = MujocoQuatToUE(quat)  // 坐标转换
  → UpdateSimulationView(_info)
      for each body:
        SceneComponent->SetWorldLocation(MujocoToUE(pos))
        SceneComponent->SetWorldRotation(quat2)
      for each geom:
        StaticMeshComponent->SetWorldLocation(...)
        StaticMeshComponent->SetWorldRotation(...)
```

## 14. 坐标转换

MuJoCo 使用右手系 Z-up，UE 使用左手系 Y-up。所有坐标需要转换：

```cpp
// 位置: X 保持, Y 翻转, 米→厘米 (×100)
FVector MujocoToUE(mjtNum x, mjtNum y, mjtNum z)
{
    return FVector(x * 100.0f, -y * 100.0f, z * 100.0f);
}

// 四元数: MuJoCo [w,x,y,z] → UE [x,y,z,w] + Y 轴翻转
FQuat MujocoQuatToUE(const mjtNum q[4])
{
    // Y 轴翻转 (右手→左手): 翻转 x 和 z 分量
    return FQuat(-q[1], q[2], -q[3], q[0]);
}
```

## 15. 关键注意事项

### 15.1 启动顺序约束

1. **mujoco_sim 必须先于 UE** — UE 启动后立即开始监听 25001，等待 RobotState
2. **UE 先于 mc_ctrl** — mc_ctrl 需要 eCAL topic 就绪
3. **mc_ctrl 最后启动** — 等待 mujoco_sim 的 eCAL publisher

### 15.2 模型一致性

- mujoco_sim 和 CarlaUnreal **必须加载相同的 XML 模型**
- 模型路径: `xgb/scene_terrain_yard.xml`
- mujoco_sim: 从 `config.yaml` 中的 `robot` + `robot_scene` 拼接
- CarlaUnreal: 从 `XmlSourcePath` Blueprint 属性读取

### 15.3 端口分配

| 端口 | 本模式用途 | 内部物理模式用途 |
|------|-----------|----------------|
| 25001 | mujoco_sim → CarlaUnreal (RobotState) | UE → mc_ctrl (RobotState) |
| 25002 | **不使用** | mc_ctrl → UE (RobotCmd) |

### 15.4 与 Matrix 的关键差异

| 差异点 | Matrix robot_mujoco | mujoco_sim | 影响 |
|--------|--------------------|-----------|------|
| 源码 | 闭源二进制 | 开源 (C++17) | 可调试/修改 |
| PD 控制 | 内置 | 自行实现 | 需确保公式一致 |
| eCAL topic | mujoco_cmd/state | 相同 | 兼容 |
| UDP 输出 | sendUdpData() | SendUdp() | 相同格式 |
| GLFW 窗口 | 有 | 有 (可选关闭) | 一致 |
| 传感器索引 | 从日志推断 | 从 xgb.xml 解析 | 需验证一致性 |

### 15.5 性能考虑

- mujoco_sim 物理线程: 独立 std::thread, 500Hz, sleep_until 精确等待
- CarlaUnreal 渲染: UE Tick (30~60fps), 每帧一次 mj_kinematics + mesh 更新
- UDP 接收: 独立 FUdpSocketReceiver 线程, FScopeLock 保护共享状态
- eCAL: 零拷贝发布, 低延迟

### 15.6 切换方式

在 UE 编辑器中选中 MuJoCoSimulation Actor:
- `bExternalPhysicsMode = false` → 内部物理 (UE 跑 mj_step + mc_ctrl UDP)
- `bExternalPhysicsMode = true` → 外部物理 (mujoco_sim 跑 + eCAL + UE 纯渲染)

```cpp
// MuJoCoSimulation.h
UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "External Physics")
bool bExternalPhysicsMode = true;  // 默认外部模式
```

---

# 第三部分：两种模式完整对比总结

## 16. 架构对比表

| 对比项 | bExternalPhysicsMode=false | bExternalPhysicsMode=true |
|--------|---------------------------|--------------------------|
| **Matrix 对应** | 非 MuJoCo (不勾选) | MuJoCo (勾选) |
| **物理引擎** | UE 内部 MuJoCo | mujoco_sim (独立进程) |
| **物理函数** | SimulateMuJoCo() | mujoco_sim 物理线程 |
| **物理步进** | mj_step() 在 UE Tick 中 | mj_step() 在独立线程 |
| **物理频率** | ~500Hz × DeltaTime | 500Hz (精确, sleep_until) |
| **物理与渲染** | 耦合 (同一 Tick) | 解耦 (独立线程 + 渲染) |
| **UE 渲染函数** | UpdateSimulationView() (在 SimulateMuJoCo 内) | TickExternalPhysics() |
| **UE FK 计算** | ExtractCurrentState() (在物理步后) | mj_kinematics() (每帧) |
| **mc_ctrl 接口** | UnrealCommandInterface (type=8) | MujocoCommandInterface (type=5) |
| **mc_ctrl 通信** | UDP 双向 (25001+25002) | eCAL (mujoco_cmd/state) |
| **UE UdpReceiver** | ParseRobotCmd, 端口 25002 | ParseRobotState, 端口 25001 |
| **UE UdpSender** | 启用 (发 RobotState 到 mc_ctrl) | 禁用 |
| **UE mj_step** | 是 (在 SimulateMuJoCo 中) | 否 (纯 FK) |
| **UE mj_kinematics** | 隐含在 ExtractCurrentState 中 | 显式调用 |
| **进程数** | 2 (UE + mc_ctrl) | 3 (mujoco_sim + UE + mc_ctrl) |
| **GLFW 窗口** | 无 | 有 (mujoco_sim, 可选) |
| **键盘控制** | keyboard_control.py | sim_launcher 或 keyboard_control.py |
| **config.json** | mujoco_running=false | mujoco_running=true |
| **启动命令** | UE → sleep 7 → mc_ctrl | mujoco_sim → UE → sleep 7 → mc_ctrl |
| **启动延迟** | 3s 重力趴下 + 1s 增益爬升 | mujoco_sim 趴下 → mc_ctrl 接管 |
| **状态包** | ~303 bytes (protobuf) | ~303 bytes (protobuf, 相同格式) |
| **时间戳基准** | CLOCK_REALTIME (纳秒) | CLOCK_REALTIME (纳秒) |
| **关节映射** | Protocol[jointType*4+leg] ↔ MuJoCo[leg*3+jointType] | 相同 |

## 17. 启动命令参考

### 17.1 模式 A: bExternalPhysicsMode = false (内部物理)

```bash
# ① 启动 CarlaUnreal (UE 编辑器中运行关卡, bExternalPhysicsMode=false)
# ② 等待 UE 加载
sleep 7
# ③ 启动 mc_ctrl (type=8)
cd /home/qiyuan/Softwares/Matrix/src/robot_mc
# 确保 motor_platform_type: 8
export SDK_CLIENT_IP="127.0.0.1"
./run_mc.sh r > run_mc.log 2>&1 &
# ④ 启动键盘控制
python3 keyboard_control.py
```

### 17.2 模式 B: bExternalPhysicsMode = true (外部物理)

```bash
# ① 启动 mujoco_sim (物理服务器)
cd /home/qiyuan/Softwares/Mujoco330/mujoco_sim/build
export LD_LIBRARY_PATH="/home/qiyuan/Softwares/Mujoco330/install/lib:${LD_LIBRARY_PATH}"
./mujoco_sim ../config.yaml > mujoco_sim.log 2>&1 &

# ② 启动 CarlaUnreal (UE 编辑器中运行关卡, bExternalPhysicsMode=true)
# ③ 等待 UE 加载
sleep 7
# ④ 启动 mc_ctrl (type=5)
cd /home/qiyuan/Softwares/Matrix/src/robot_mc
# 确保 motor_platform_type: 5
export SDK_CLIENT_IP="127.0.0.1"
./run_mc.sh r > run_mc.log 2>&1 &
# ⑤ 键盘控制 (sim_launcher 或 keyboard_control.py)
```

## 18. 源码文件索引

| 文件 | 路径 | 用途 |
|------|------|------|
| MuJoCoSimulation.h | Plugins/MuJoCoUE/Public/ | Actor 类声明, 双模式标志 |
| MuJoCoSimulation.cpp | Plugins/MuJoCoUE/Private/ | 核心实现 (1357 行) |
| UdpReceiverComponent.h | Plugins/MuJoCoUE/Public/ | 双模式 UDP 接收器 |
| UdpReceiverComponent.cpp | Plugins/MuJoCoUE/Private/ | protobuf 解析 (334 行) |
| UdpSenderComponent.h | Plugins/MuJoCoUE/Public/ | UDP 状态发送器 |
| UdpSenderComponent.cpp | Plugins/MuJoCoUE/Private/ | protobuf 序列化 (293 行) |
| mujoco_sim.h | Mujoco330/mujoco_sim/ | 物理服务器类声明 |
| mujoco_sim.cpp | Mujoco330/mujoco_sim/ | 物理服务器实现 (624 行) |
| main.cpp | Mujoco330/mujoco_sim/ | 入口: 配置/信号/运行 |
| config.yaml | Mujoco330/mujoco_sim/ | 配置 (与 robot_mujoco 兼容) |
| CMakeLists.txt | Mujoco330/mujoco_sim/ | 构建: mujoco+protobuf+eCAL+glfw |
| run.sh | Mujoco330/mujoco_sim/ | 构建+运行脚本 |
# CarlaUnreal MuJoCo 双模式集成分析

## 概述

本文档分析 CarlaUnreal MuJoCoUE 插件的两种运行模式，对照 Matrix 的对应架构，说明已实现模式和待实现模式的技术细节。

```
模式对照表：
┌──────────────────────────────────┬──────────────────────────────────────┐
│ CarlaUnreal                     │ Matrix 对应                           │
├──────────────────────────────────┼──────────────────────────────────────┤
│ bExternalPhysicsMode = false     │ 非 MuJoCo 模式 (不勾选 mujoco 物理)   │
│   UE 内部跑物理 + mc_ctrl UDP    │   UE 内部 FMujocoWorkerThread + type=8│
│                                  │                                      │
│ bExternalPhysicsMode = true      │ MuJoCo 模式 (勾选 mujoco 物理)        │
│   mujoco_sim 跑物理 + mc_ctrl    │   robot_mujoco 跑物理 + type=5 eCAL   │
│   eCAL + UE 纯渲染               │   + UE 纯 FK 渲染                     │
└──────────────────────────────────┴──────────────────────────────────────┘
```

---

# 第一部分：已实现模式 (bExternalPhysicsMode = false)

## 1. 架构概览

仿 Matrix **非 MuJoCo 模式**（不勾选 mujoco 物理），UE 内部运行 MuJoCo 物理引擎，mc_ctrl 通过 UDP 双向通信。

```
┌──────────────────────────────────────────────────────────────┐
│                    CarlaUnreal (UE)                            │
│                                                               │
│  AMuJoCoSimulation                                            │
│  ├── LoadModel(XML) → 创建 body/mesh                         │
│  ├── SimulateMuJoCo() → mj_step @ 500Hz (在 Tick 中)         │
│  ├── ApplyUdpControl() → PD 控制 (mc_ctrl 目标)               │
│  ├── SendStateToMcCtrl() → UDP 25001 → mc_ctrl               │
│  └── UdpReceiver → 监听 25002 (mc_ctrl RobotCmd)              │
│                                                               │
│  UdpReceiverComponent (ParseRobotCmd 模式)                    │
│  ├── 端口: 25002                                              │
│  ├── 解析: RobotCmd protobuf (关节目标 + PD 参数)              │
│  └── 输出: FUdpCommandData → LastCommand                      │
│                                                               │
│  UdpSenderComponent                                           │
│  ├── 端口: 25001 (目标)                                       │
│  └── 发送: RobotState protobuf (303 bytes)                    │
└────────────────────────────┬───────────────────────────────────┘
                             │ UDP 双向
                             ▼
                    ┌──────────────────┐
                    │    mc_ctrl        │
                    │  type=8           │
                    │  UnrealCommand    │
                    │  Interface        │
                    │                   │
                    │  发送 RobotCmd    │
                    │  → 127.0.0.1:25002│
                    │  接收 RobotState  │
                    │  ← 127.0.0.1:25001│
                    └──────────────────┘
```

## 2. UE 内部流程

### 2.1 BeginPlay 初始化

```cpp
// MuJoCoSimulation.cpp BeginPlay()
// bExternalPhysicsMode = false 时走此分支:

// 1. 设置初始 LIEDOWN 姿态 (HIP=1.4, KNEE=-2.4)
//    → mc_ctrl FK 计算 body_height ≈ 0.05 (趴地)
mData->qpos[2] = 0.10;  // base z
for (int j = 0; j < 4; j++) {
    mData->qpos[7 + j*3 + 0] = 0.0f;   // ABAD = 0
    mData->qpos[7 + j*3 + 1] = 1.4f;   // HIP  = 1.4 (liedown)
    mData->qpos[7 + j*3 + 2] = -2.4f;  // KNEE = -2.4 (liedown)
}
mj_forward(mModel, mData);

// 2. 生成 mesh (14 body + 69 geom)
_info = ExtractModelInfo(mModel);
ConvertMuJoCoModelToProceduralMeshes(mModel, this);
GenerateMeshes(_info);

// 3. 启动物理仿真
SimStartWallTime = FPlatformTime::Seconds();
StartSimulation();  // bSimulationRunning = true
```

### 2.2 物理循环 (SimulateMuJoCo)

```
每帧 Tick(DeltaTime) → SimulateMuJoCo(DeltaTime):

  while (mData->time - startTime < DeltaTime):
    ├── 启动延迟期 (前 3s):
    │   └── 零力矩, 让机器人在重力下趴下
    │
    ├── UDP 控制活跃 (mc_ctrl Kp > 10):
    │   ├── ApplyUdpControl(gainScale)
    │   │   ├── 读取 UdpReceiver->LastCommand
    │   │   ├── 关节目标映射: Protocol[jointType*4+leg] → MuJoCo[leg*3+jointType]
    │   │   ├── 目标混合: lerp(激活角度, mc_ctrl目标, gainScale)
    │   │   ├── PD 控制: τ = Kp*(target-q) - Kd*q̇ + τ_ff
    │   │   └── 力矩限幅: ±28 Nm
    │   └── SendStateToMcCtrl() → UDP 25001
    │
    ├── 本地站立控制 (无 mc_ctrl):
    │   └── ApplyStandUpControl() → 三阶段站立轨迹
    │
    └── mj_step(mModel, mData)  ← 物理步进
```

### 2.3 状态反馈 (SendStateToMcCtrl)

```
UE 每物理步发送 RobotState (protobuf, ~303 bytes):
  ├── JointPos: [abad×4, hip×4, knee×4] (12 个关节角度)
  ├── JointVel: [abad×4, hip×4, knee×4] (12 个关节角速度)
  ├── JointTau: [abad×4, hip×4, knee×4] (12 个关节力矩)
  ├── Quat: [w, x, y, z] (base 四元数)
  ├── Gyro: [wx, wy, wz] (body 帧角速度)
  ├── Acc: [ax, ay, az] (body 帧加速度)
  ├── RPY: [roll, pitch, yaw]
  ├── Position: [x, y, z] (base 位置)
  └── VWorld: [vx, vy, vz] (base 线速度)
```

## 3. 通信协议

| 方向 | 端口 | 消息 | 频率 | 用途 |
|------|------|------|------|------|
| mc_ctrl → UE | 25002 (cmd_port) | RobotCmd (protobuf) | ~500Hz | 关节目标 + PD 参数 |
| UE → mc_ctrl | 25001 (state_port) | RobotState (protobuf, ~303 bytes) | ~500Hz | 状态反馈 |

## 4. 关键特性

| 特性 | 说明 |
|------|------|
| 物理引擎 | UE 内部 MuJoCo (mj_step) |
| 物理频率 | ~500Hz (受 UE Tick 驱动, 非独立线程) |
| mc_ctrl 接口 | UnrealCommandInterface (type=8) |
| 进程数 | 2 (CarlaUnreal + mc_ctrl) |
| 初始姿态 | LIEDOWN (HIP=1.4, KNEE=-2.4) |
| 启动延迟 | 3s 重力趴下 → mc_ctrl PD 接管 |
| 增益爬升 | 1s 线性爬升 (防止弹射) |
| 目标混合 | 激活瞬间角度 → mc_ctrl 目标 (1s 过渡) |

## 5. 与 Matrix 非 MuJoCo 模式的差异

| 对比项 | Matrix 非 MuJoCo | CarlaUnreal (本模式) |
|--------|-------------------|---------------------|
| 物理线程 | FMujocoWorkerThread (独立线程 ~500Hz) | UE Tick 驱动 (受帧率影响) |
| 物理精度 | ~500Hz 稳定 | 受渲染帧率影响 |
| 状态反馈 | 固定 303 bytes | 同样 protobuf ~303 bytes |
| mc_ctrl 类型 | type=8 (UnrealCommandInterface) | 相同 type=8 |
| 键盘控制 | sim_launcher 虚拟 F710 | keyboard_control.py 虚拟 F710 |
| 启动延迟 | 无 (UE 已趴下) | 3s 重力趴下 + 1s 增益爬升 |

**注意**: Matrix 的 FMujocoWorkerThread 是独立线程，物理频率 (~500Hz) 不受渲染帧率 (30fps) 影响。CarlaUnreal 当前的物理步进在 UE Tick 中执行，频率受帧率限制。如需精确 500Hz 物理，需改为独立物理线程。

---

# 第二部分：待实现模式 (bExternalPhysicsMode = true)

## 6. 架构概览

仿 Matrix **MuJoCo 模式**（勾选 mujoco 物理），mujoco_sim 作为独立物理服务器，mc_ctrl 通过 eCAL 通信，CarlaUnreal 纯 FK 渲染。

```
┌─────────────────────┐
│   mujoco_sim         │  ← 替代 Matrix robot_mujoco
│   (MuJoCo 3.3.0)     │
│                       │
│   mj_step @ 500Hz    │  ← 独立物理线程 (500Hz)
│   ApplyControl()     │  ← PD: τ = kp*(q_des-q) + kd*(qd_des-qd) + τ_ff
│   BuildRobotState()  │
│   GLFW 窗口 (可选)    │
│                       │
│   eCAL Publisher      │──→ mujoco_state → mc_ctrl
│   eCAL Subscriber    │←── mujoco_cmd ← mc_ctrl
│   UDP Sender         │──→ RobotState → CarlaUnreal:25001
└───────────────────────┘
         │ eCAL              │ UDP 25001
         ▼                   ▼
┌──────────────────┐  ┌──────────────────────────────────────┐
│    mc_ctrl        │  │    CarlaUnreal (UE)                   │
│  type=5           │  │                                       │
│  MujocoCommand    │  │  AMuJoCoSimulation                     │
│  Interface        │  │  ├── LoadModel(XML) → mesh 生成        │
│                   │  │  ├── TickExternalPhysics()             │
│  eCAL pub/sub     │  │  │   ├── 接收 RobotState (UDP 25001)  │
│  虚拟 F710 手柄   │  │  │   ├── 写入 mData->qpos              │
│                   │  │  │   ├── mj_kinematics() (纯 FK)       │
│                   │  │  │   └── UpdateSimulationView()        │
│                   │  │  └── 无 mj_step, 无 UDP 发送           │
└──────────────────┘  └───────────────────────────────────────┘
```

## 7. mujoco_sim 详细设计

### 7.1 项目结构

```
/home/qiyuan/Softwares/Mujoco330/mujoco_sim/
├── main.cpp           # 入口: 加载配置, 注册信号处理, 运行仿真
├── mujoco_sim.h       # MujocoSim 类定义
├── mujoco_sim.cpp     # 核心实现: 物理/eCAL/UDP/PD控制
├── config.yaml        # 配置文件 (与 robot_mujoco 兼容)
├── CMakeLists.txt     # 构建: mujoco + protobuf + eCAL + glfw
└── run.sh             # 构建+运行脚本
```

### 7.2 config.yaml

```yaml
robot: "xgb"
robot_scene: "scene_terrain_yard.xml"

enable_ecal: 1          # eCAL 与 mc_ctrl 通信
enable_udp: 1           # UDP 向 CarlaUnreal 发送状态
udp_target_ip: "127.0.0.1"
udp_target_port: 25001  # CarlaUnreal 监听端口

enable_gui: 1           # GLFW 可视化窗口
sim_rate_hz: 500        # 物理步进 500Hz
publish_rate_hz: 500    # 状态发布 500Hz
```

### 7.3 内部结构

```
MujocoSim
├── Initialize()
│   ├── LoadModel() → mj_loadXML + mj_makeData + mj_forward
│   ├── eCAL::Initialize → Publisher(mujoco_state) + Subscriber(mujoco_cmd)
│   └── InitUdp() → socket → dest 127.0.0.1:25001
│
├── Run() → RunWithGui() 或 RunHeadless()
│   │
│   ├── 物理线程 (500Hz):
│   │   while running:
│   │     ├── ApplyControl()   ← PD 控制律
│   │     ├── PhysicsStep()    ← mj_step()
│   │     ├── PublishState()   ← eCAL + UDP
│   │     └── sleep_until(next_time)
│   │
│   └── 渲染循环 (~60fps, GLFW):
│       mjv_updateScene + mjr_render + glfwSwapBuffers
│
├── ApplyControl()
│   └── for each leg × joint:
│       τ = kp*(q_des - q) + kd*(qd_des - qd) + τ_ff
│       data_->ctrl[actuator_idx] = τ
│
├── BuildRobotState()
│   ├── q_abad/hip/knee[4] ← qpos[7:18]
│   ├── qd_abad/hip/knee[4] ← qvel[6:17]
│   ├── tau_abad/hip/knee[4] ← sensordata[24:35]
│   ├── quat[4] ← sensordata[36:39] (imu_quat)
│   ├── gyro[3] ← sensordata[39:41] (imu_gyro)
│   ├── acc[3] ← sensordata[42:44] (imu_acc)
│   ├── position[3] ← sensordata[45:47] (framepos)
│   ├── rpy[3] ← 从 quat 计算
│   └── v_world[3] ← sensordata[48:50] (frame_linvel)
│
└── PublishState()
    ├── eCAL pub->Send(state)  → mc_ctrl
    └── UDP sendto(serialized) → CarlaUnreal:25001
```

### 7.4 关节顺序映射

```
MuJoCo qpos[7:18] 与 RobotState protobuf 对应:

  qpos[7]  = FAR_ABAD  (FR外展)  → q_abad[0]
  qpos[8]  = FAR_HIP   (FR髋)    → q_hip[0]
  qpos[9]  = FAR_KNEE  (FR膝)    → q_knee[0]
  qpos[10] = FBL_ABAD  (FL外展)  → q_abad[1]
  qpos[11] = FBL_HIP   (FL髋)    → q_hip[1]
  qpos[12] = FBL_KNEE  (FL膝)    → q_knee[1]
  qpos[13] = RAR_ABAD  (RR外展)  → q_abad[2]
  qpos[14] = RAR_HIP   (RR髋)    → q_hip[2]
  qpos[15] = RAR_KNEE  (RR膝)    → q_knee[2]
  qpos[16] = RBL_ABAD  (RL外展)  → q_abad[3]
  qpos[17] = RBL_HIP   (RL髋)    → q_hip[3]
  qpos[18] = RBL_KNEE  (RL膝)    → q_knee[3]
```

## 8. CarlaUnreal 外部物理模式实现

### 8.1 BeginPlay 初始化

```cpp
// MuJoCoSimulation.cpp BeginPlay()
// bExternalPhysicsMode = true 时:

// 1. 加载模型, 生成 mesh (仅用于渲染)
mj_forward(mModel, mData);
_info = ExtractModelInfo(mModel);
ConvertMuJoCoModelToProceduralMeshes(mModel, this);
GenerateMeshes(_info);

// 2. 禁用内部物理
bSimulationRunning = false;

// 3. 禁用 mc_ctrl UDP 控制 (mujoco_sim 通过 eCAL 直接控制)
bUdpControlEnabled = false;
if (UdpSender) UdpSender->bAutoSend = false;

// 4. UdpReceiver 切换到 RobotState 解析模式 (端口 25001)
UdpReceiver->StopListening();
UdpReceiver->SetParseMode(EUdpParseMode::ParseRobotState);
UdpReceiver->bAutoStart = false;
UdpReceiver->StartListening();
```

### 8.2 渲染循环 (TickExternalPhysics)

```
每帧 Tick(DeltaTime) → TickExternalPhysics(DeltaTime):

  1. 从 UdpReceiver 获取最新 RobotState (线程安全)
     └── FUdpStateData State = UdpReceiver->GetLatestState()

  2. 检查数据新鲜度 (超时 0.5s → 警告)

  3. 写入 mData->qpos:
     ├── qpos[0..2] = State.Position (base x,y,z)
     ├── qpos[3..6] = State.Quat (base qw,qx,qy,qz)
     └── qpos[7..18] = State.QAbad/QHip/QKnee (12 关节)

  4. 纯运动学 (无动力学):
     ├── mj_kinematics(mModel, mData)  ← 正向运动学
     └── mj_comPos(mModel, mData)      ← 质心计算

  5. 更新 mesh 位置:
     ├── ExtractCurrentState(_info)  ← 从 mData 提取 xpos/xquat
     └── UpdateSimulationView(_info) ← 更新 UE 组件
```

**关键**: 不调用 `mj_step()`，只做 `mj_kinematics()` (FK)。物理完全由 mujoco_sim 驱动。

### 8.3 UdpReceiverComponent 双模式

```
EUdpParseMode:
├── ParseRobotCmd (模式 0):  用于 bExternalPhysicsMode=false
│   ├── 端口: 25002
│   ├── 解析: RobotCmd protobuf
│   └── 输出: FUdpCommandData (关节目标 + Kp/Kd/τ_ff)
│
└── ParseRobotState (模式 1): 用于 bExternalPhysicsMode=true
    ├── 端口: 25001
    ├── 解析: RobotState protobuf
    └── 输出: FUdpStateData (关节状态 + IMU + 位置)
```

## 9. 通信协议

| 方向 | 方式 | Topic/端口 | 消息 | 频率 |
|------|------|-----------|------|------|
| mc_ctrl → mujoco_sim | eCAL | mujoco_cmd | RobotCmd | 500Hz |
| mujoco_sim → mc_ctrl | eCAL | mujoco_state | RobotState | 500Hz |
| mujoco_sim → CarlaUnreal | UDP | 25001 | RobotState (protobuf) | 500Hz |

**注意**: CarlaUnreal 在此模式下**只接收不发送**。UDP 25002 不使用。

## 10. 启动顺序

```
时间轴 ──────────────────────────────────────────────────────►

① 启动 mujoco_sim
   ├── 加载 scene_terrain_yard.xml
   ├── 初始化 eCAL (pub: mujoco_state, sub: mujoco_cmd)
   ├── 初始化 UDP (target: 127.0.0.1:25001)
   ├── 启动 GLFW 窗口 (可选)
   └── 开始物理线程: mj_step @ 500Hz (无控制, 机器人趴下)

② 启动 CarlaUnreal
   ├── 加载 XML → 生成 mesh (14 body, 69 geom)
   ├── bExternalPhysicsMode=true → 禁用内部物理
   ├── UdpReceiver → ParseRobotState 模式, 端口 25001
   └── 等待 mujoco_sim 的 RobotState...

③ 启动 mc_ctrl (type=5)
   ├── [MujocoCommandInterface] initialize
   ├── eCAL 连接 mujoco_sim
   ├── 加载 ONNX RL 模型
   ├── 初始化 FSM (PASSIVE → ... → POS_CONTROL)
   ├── [joystic] find 1 joystic (虚拟 F710)
   └── 开始控制循环 500Hz

④ 用户按 U 键
   ├── sim_launcher → 虚拟 F710: LB+Y
   ├── mc_ctrl: PASSIVE → STANDUP
   ├── eCAL: RobotCmd → mujoco_sim
   ├── mujoco_sim: PD + mj_step → 站立
   ├── eCAL: RobotState → mc_ctrl (状态反馈)
   └── UDP: RobotState → CarlaUnreal:25001
       └── UE: FK → 渲染站立
```

## 11. 关键特性

| 特性 | 说明 |
|------|------|
| 物理引擎 | mujoco_sim (独立进程, MuJoCo 3.3.0) |
| 物理频率 | 500Hz (独立线程, 精确) |
| mc_ctrl 接口 | MujocoCommandInterface (type=5, eCAL) |
| 进程数 | 3 (mujoco_sim + CarlaUnreal + mc_ctrl) |
| UE 角色 | 纯 FK 渲染 (无 mj_step) |
| 键盘控制 | 复用 Matrix sim_launcher 虚拟 F710 |
| GLFW 窗口 | mujoco_sim 自带 (可选关闭) |

---

# 第三部分：两种模式完整对比

## 12. 模式对照表

| 对比项 | bExternalPhysicsMode=false | bExternalPhysicsMode=true |
|--------|---------------------------|--------------------------|
| **Matrix 对应** | 非 MuJoCo 模式 (不勾选) | MuJoCo 模式 (勾选) |
| **物理引擎** | UE 内部 MuJoCo | mujoco_sim (独立进程) |
| **物理频率** | ~500Hz (UE Tick 驱动) | 500Hz (独立线程, 精确) |
| **mc_ctrl 类型** | type=8 (UnrealCommandInterface) | type=5 (MujocoCommandInterface) |
| **mc_ctrl 通信** | UDP 双向 (25001/25002) | eCAL (mujoco_cmd/mujoco_state) |
| **UE → mc_ctrl** | UDP 25001 (RobotState ~303B) | 无 (mujoco_sim 通过 eCAL 发) |
| **mc_ctrl → UE** | UDP 25002 (RobotCmd) | 无 (mujoco_sim 通过 eCAL 收) |
| **mujoco_sim → UE** | 无 | UDP 25001 (RobotState) |
| **UE 物理** | mj_step (在 Tick 中) | 无 (纯 mj_kinematics FK) |
| **UE UdpReceiver** | ParseRobotCmd (25002) | ParseRobotState (25001) |
| **UE UdpSender** | 启用 (发 RobotState) | 禁用 |
| **进程数** | 2 (UE + mc_ctrl) | 3 (mujoco_sim + UE + mc_ctrl) |
| **GLFW 窗口** | 无 | 有 (mujoco_sim) |
| **键盘控制** | keyboard_control.py | sim_launcher 虚拟 F710 |
| **config.json** | mujoco_running=false | mujoco_running=true |
| **启动顺序** | UE → sleep 7 → mc_ctrl | mujoco_sim → UE → sleep 7 → mc_ctrl |

## 13. 数据流对比

### 模式 A: bExternalPhysicsMode = false (已实现)

```
用户按 U → 虚拟 F710 → mc_ctrl (type=8)
  ──UDP RobotCmd──→ UE:25002
  UE: ApplyUdpControl() → mj_step → 站立
  UE: SendStateToMcCtrl() ──UDP RobotState──→ mc_ctrl:25001
  UE: ExtractCurrentState → UpdateSimulationView → 渲染
```

### 模式 B: bExternalPhysicsMode = true (待实现)

```
用户按 U → 虚拟 F710 → mc_ctrl (type=5)
  ──eCAL mujoco_cmd──→ mujoco_sim
  mujoco_sim: ApplyControl() → mj_step → 站立
  mujoco_sim: PublishState()
    ├── ──eCAL mujoco_state──→ mc_ctrl (状态反馈)
    └── ──UDP RobotState──→ CarlaUnreal:25001
  CarlaUnreal: TickExternalPhysics()
    ├── 写入 qpos → mj_kinematics (FK)
    └── UpdateSimulationView → 渲染
```

## 14. 代码切换方式

在 UE 编辑器中，选中 MuJoCoSimulation Actor：
- `bExternalPhysicsMode = false` → 内部物理 (已实现)
- `bExternalPhysicsMode = true` → 外部物理 (待实现)

```cpp
// MuJoCoSimulation.h
UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "External Physics")
bool bExternalPhysicsMode = true;  // 默认值, 在 Blueprint 中可切换
```

BeginPlay 中根据此标志走不同分支：
- `true` → 禁用 mj_step, 禁用 UdpSender, UdpReceiver 切 ParseRobotState
- `false` → 启用 mj_step, 启用 UdpSender, UdpReceiver 保持 ParseRobotCmd

## 15. 待实现模式的注意事项

1. **mujoco_sim 必须先于 UE 启动** — UE 需要接收 RobotState 才能渲染
2. **mc_ctrl 最后启动** — 需要等待 mujoco_sim 的 eCAL topic 就绪
3. **XML 模型一致性** — mujoco_sim 和 CarlaUnreal 必须加载相同的 XML 模型
4. **端口不冲突** — mujoco_sim UDP 25001 → UE, mc_ctrl eCAL 不走 UDP
5. **UE 不发送任何数据** — 纯被动接收端，无 UDP 25002 通信
6. **虚拟手柄** — 需要 sim_launcher.bin 或 keyboard_control.py 创建虚拟 F710
7. **mujoco_sim 的 PD 控制** — 与 Matrix robot_mujoco 相同：τ = kp*(q_des-q) + kd*(qd_des-qd) + τ_ff
8. **eCAL 依赖** — mujoco_sim 和 mc_ctrl 都需要 eCAL 库

## 16. 启动命令参考

```bash
# ① 启动 mujoco_sim (物理服务器)
cd /home/qiyuan/Softwares/Mujoco330/mujoco_sim/build
export LD_LIBRARY_PATH="/home/qiyuan/Softwares/Mujoco330/install/lib:${LD_LIBRARY_PATH}"
./mujoco_sim ../config.yaml > mujoco_sim.log 2>&1 &

# ② 启动 CarlaUnreal (UE 编辑器或打包版本)
# 在 UE 编辑器中运行关卡, 确保 bExternalPhysicsMode=true

# ③ 等待 UE 加载完成
sleep 7

# ④ 启动 mc_ctrl (Matrix 运动控制器, type=5)
cd /home/qiyuan/Softwares/Matrix/src/robot_mc
export SDK_CLIENT_IP="127.0.0.1"
# 确保 xg-user-parameters.yaml 中 motor_platform_type: 5
./run_mc.sh r > run_mc.log 2>&1 &

# ⑤ 启动键盘控制 (可选, 如果不用 sim_launcher)
python3 /path/to/keyboard_control.py
```
