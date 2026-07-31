# robot_mc 深入分析文档

## 1. 概述

`robot_mc`（Robot Motion Controller）是 Matrix 仿真平台的**核心运动控制模块**，负责四足机器人的实时运动控制。它以预编译二进制形式分发（无源码），通过 UDP/共享内存与仿真环境（UE/MuJoCo）通信，接收机器人状态并输出关节控制命令。

**在 Matrix 系统中的位置：**

```
┌─────────────────────────────────────────────────────────────┐
│                    Matrix 仿真平台                            │
├─────────────┬──────────────────┬────────────────────────────┤
│  UeSim      │   robot_mc       │   robot_mujoco             │
│  (UE渲染+   │   (运动控制器)    │   (MuJoCo物理仿真)         │
│   物理仿真)  │                  │                            │
└──────┬──────┴────────┬─────────┴─────────────┬──────────────┘
       │    UDP 25001/25002    │               │
       └──────────────────────┘               │
              protobuf (RobotState/RobotCmd)   │
```

## 2. 目录结构

```
src/robot_mc/
├── .gitignore              # 忽略 build 目录，但保留 export 子目录
├── run_mc.sh               # 启动脚本
├── docs/                   # 文档目录
└── build/export/           # 运行时部署目录
    ├── config/             # YAML 配置文件（22个）
    ├── mc/bin/             # 可执行文件和共享库
    ├── onnx_model_crypto/  # 加密的 ONNX RL 模型
    └── mile_data.txt       # 里程计数据（mile: 405535）
```

## 3. 二进制组件

### 3.1 可执行文件

| 文件 | 说明 |
|------|------|
| `mc_ctrl` | 主控制器进程（由 run_mc.sh 启动） |
| `ptp_ctrl` | 点对点控制工具 |
| `pose_calib_test` | 姿态标定测试 |
| `file_crypto_cli` | 模型文件加解密工具 |

### 3.2 共享库

| 库文件 | 大小 | 功能 |
|--------|------|------|
| `librobot.so` | 22.9 MB | **核心控制库**（含调试信息），包含全部控制逻辑 |
| `libP2.so` | 3.6 MB | IMU 硬件驱动（Lord IMU 串口通信） |
| `libqpOASES.so` | 1.7 MB | 二次规划求解器（MPC/WBC 使用） |
| `libonnx_model.so` | 1.1 MB | ONNX Runtime 推理封装 |
| `libWBC.so` | 765 KB | 全身控制（Whole-Body Control） |
| `libWBC_CTRL.so` | 442 KB | WBC 控制器 |
| `libGoldfarb_Optimizer.so` | 463 KB | Goldfarb-Idnani QP 求解器 |
| `libdynacore_yaml-cpp.so` | 475 KB | YAML 配置解析 |
| `libfilecrypto_shared.so` | 288 KB | 模型文件解密（zsibot::mc::FileCrypto） |
| `libdynacore_param_handler.so` | 65 KB | 参数管理 |
| `libinih.so` | 64 KB | INI 文件解析 |

## 4. 核心架构（从符号表逆向）

### 4.1 类层次结构

```
main_helper(int, char**, RobotController*)
    │
    ├── RobotInterface          ← 硬件/仿真接口层
    │   ├── initCommon()
    │   ├── loadParameters()
    │   ├── handleGamepadLCM()  ← 手柄输入
    │   ├── imuResultCallBack() ← IMU 数据回调
    │   └── run()
    │
    ├── RobotRunner             ← 主控制循环
    │   ├── init()
    │   ├── run()               ← 500Hz 主循环
    │   ├── setupStep()
    │   ├── finalizeStep()
    │   ├── initializeStateEstimator()
    │   ├── pub_leg_command()   ← 发布关节命令
    │   ├── pub_high_state()    ← 发布高层状态
    │   ├── pub_low_state()     ← 发布低层状态
    │   ├── pub_sdk_state()     ← 发布 SDK 状态
    │   ├── pub_app_state()     ← 发布 APP 状态
    │   ├── run_lowlevelsdk()   ← 低层 SDK 接口
    │   └── run_highlevelsdk()  ← 高层 SDK 接口
    │
    ├── CommandInterface        ← 通信接口工厂
    │   ├── getCmdInterface(int motor_platform_type)
    │   └── setCommandAndState(SpiCommand*, SpiData*, VectorNavData*)
    │
    ├── RobotMonitor            ← 安全监控
    │   ├── fsm_state_check()
    │   ├── imu_data_check()
    │   ├── motor_data_check()
    │   ├── rl_loadpolicy_check()
    │   ├── sdk_init_check()
    │   └── app_Node_check()
    │
    └── ContactEstimator<float> ← 足端接触估计
```

### 4.2 通信接口（CommandInterface 子类）

`motor_platform_type` 参数决定使用哪个通信后端：

| motor_platform_type | 接口类 | 通信方式 | 使用场景 |
|---------------------|--------|----------|----------|
| 0 | SPI 直连 | 硬件 SPI 总线 | 实机（默认） |
| 1 | Unitree | Unitree SDK | Unitree 电机 |
| 5 | `MujocoCommandInterface` | UDP protobuf | MuJoCo 仿真 |
| 6 | SHM | 共享内存 | 本地进程间 |
| 7 | `ShmContainerCmdInterface` | Boost.Interprocess SHM | 容器化部署 |
| **8** | **`UnrealCommandInterface`** | **UDP protobuf** | **UE 仿真** |

#### UnrealCommandInterface（type=8）关键方法：

```cpp
UnrealCommandInterface::initialize()   // 绑定 UDP 25001，启动 UEUDPReceiver 线程
UnrealCommandInterface::reciveData()   // 检查数据有效性（flag 0x241 + 0x242 + 时间戳<10ms）
UnrealCommandInterface::sendCmd()      // 发送 RobotCmd 到 UE（port 25002）
UnrealCommandInterface::getInstance()  // 单例模式
```

#### UEUDPReceiver 线程：

```cpp
UEUDPReceiver::run()  // recvfrom → protobuf ParseFromArray → GenericSwap → 设置 flags
```

### 4.3 数据流

```
UE/MuJoCo 仿真器
    │
    │ UDP port 25001 (protobuf RobotState, 303 bytes)
    │ 包含: 关节角×12, 关节速度×12, IMU(四元数+角速度+加速度), 时间戳
    ▼
UEUDPReceiver::run()  [独立线程]
    │ ParseFromArray → 字节序转换 → 写入内部状态
    ▼
UnrealCommandInterface::reciveData()  [500Hz 主循环调用]
    │ 三重验证: flag_valid + flag_fresh + timestamp < 10ms
    ▼
RobotRunner::run()  [500Hz 控制循环]
    │
    ├── StateEstimator    → 状态估计（IMU + 足端接触）
    ├── ControlFSM        → 有限状态机（PASSIVE→STANDUP→RL_MIX）
    ├── RL Policy (ONNX)  → 强化学习策略推理
    ├── MPC               → 模型预测控制（可选）
    ├── WBC               → 全身控制（可选）
    └── GaitScheduler     → 步态调度
    │
    ▼
UnrealCommandInterface::sendCmd()
    │ UDP port 25002 (protobuf RobotCmd)
    │ 包含: 目标关节角×12, Kp×12, Kd×12
    ▼
UE/MuJoCo 仿真器（执行 PD 控制）
```

## 5. 配置系统

### 5.1 配置文件分类

| 文件 | 用途 |
|------|------|
| `robot-defaults.yaml` | 全局控制参数（dt=0.002s, 噪声参数） |
| `simulator-defaults.yaml` | 仿真器参数（物理 dt=0.001s, 地面刚度） |
| `{type}-user-parameters.yaml` | 机器人专属 PD 增益、MPC 权重、关节限位 |
| `{type}-motion_config.yaml` | 预定义动作序列（站立、坐下、握手等） |
| `rl_onnx_config.yaml` | ONNX 模型路径映射（x86 GPU/CPU） |
| `rl_rknn_config.yaml` | RKNN 模型路径映射（瑞芯微 NPU） |
| `sdk_config.yaml` | SDK 通信配置（target_ip:port） |
| `default-terrain.yaml` | 地形定义（平面、台阶、斜坡） |
| `dance_config.yaml` | 舞蹈动作编排 |

### 5.2 支持的机器人类型

| 类型代码 | 描述 | motor_platform_type | 身体高度 |
|----------|------|---------------------|----------|
| XG | 标准四足（小狗） | 8 (UE) | 0.32m |
| XGW | XG + 轮足 | 5 (MuJoCo) | — |
| ZG | 大型四足 | 5 (MuJoCo) | — |
| ZGW | ZG + 轮足 | 7 (SHM) | — |
| ZGWS | ZG + 双轮足 | 5 (MuJoCo) | — |
| XXG | 中型四足 | 5 (MuJoCo) | — |

### 5.3 关键控制参数（XG 为例）

```yaml
# 控制频率
controller_dt: 0.002          # 500Hz 控制循环

# FSM PD 增益
FSM_jointPD_Kp: 80            # 站立/被动模式 Kp
FSM_jointPD_Kd: 1.0
FSM_RL_ABAD_Kp: 20.0         # RL 模式 abduction Kp
FSM_RL_HIP_Kp: 20.0          # RL 模式 hip Kp
FSM_RL_KNEE_Kp: 20.0         # RL 模式 knee Kp
FSM_RL_Kd: 0.7

# MPC 参数
cmpc_x_vel: 3.0              # 最大前进速度 m/s
cmpc_y_vel: 1.0              # 最大侧移速度 m/s
cmpc_yaw_vel: 3.0            # 最大偏航角速度 rad/s
body_height: 0.32            # 目标身体高度 m
mpc_horizon_length: 14       # MPC 预测步长

# 关节限位
JPos_limit_low: [-0.48, -1.15, -2.7]
JPos_limit_high: [0.48, 2.97, -0.65]
jointVelocityLimit: 24.0     # rad/s
```

## 6. RL 策略模型系统

### 6.1 模型架构

每个运动模式由两个 ONNX 模型组成：
- **policy**: 策略网络，输入观测→输出目标关节角
- **odom**: 状态估计网络（odometry），输入历史观测→估计速度/位置

### 6.2 支持的运动模式（XG 平台，共 20+ 种）

| 运动模式 | 策略模型 | 说明 |
|----------|----------|------|
| mix_walk | policy_mix_walk | 混合行走（默认 RL 步态） |
| gait_walk | policy_gait_walk | 步态行走 |
| balancestand | policy_balancestand_withyaw_0423 | 平衡站立（含偏航） |
| backflip | policy_mix_backflip | 后空翻 |
| frontflip | policy_mix_frontflip | 前空翻 |
| sideflip | policy_mix_sideflip | 侧空翻 |
| jump | policy_jump_fix2_step3 | 跳跃 |
| upright | policy_upright | 直立恢复 |
| tracking | policy_tracking | 轨迹跟踪 |
| walkpos | policy_walkpos | 位置行走 |
| crawl | policy_xg_crawl_260127 | 匍匐 |
| moonwalk | policy_moonwalk | 太空步 |
| ik | policy_ik | 逆运动学 |
| mlp | policy_mlp | 基础 MLP 策略 |
| measured | policy_measured_0918 | 测量步态 |
| soft_walk | policy_soft_walk | 柔性行走 |
| slow_walk_gait | policy_slow_walk_gait | 慢速步态 |
| flipover | policy_mix_flipover | 翻转恢复 |
| stair | — | 楼梯（XG 不支持） |
| climb | — | 攀爬（XG 不支持） |

### 6.3 模型加密

- 所有模型文件经过 `zsibot::mc::FileCrypto` 加密
- 运行时由 `libfilecrypto_shared.so` 解密后加载
- 支持 ONNX（x86）和 RKNN（ARM NPU）两种推理后端

## 7. 预定义动作系统

### 7.1 动作配置格式

```yaml
XG_shakehand:                    # 动作名称（{类型}_{动作}）
  meta:
    motionClass: no-interruptible  # 不可中断 / interruptible
  sequence:                        # 动作序列（按时间顺序执行）
    - motionType: transition       # 过渡（平滑插值到目标姿态）
      heightCmd: 0.32              # 目标身体高度
      rpyCmd: [0.0, 0.0, 0.0]     # 目标姿态角
      xyzCmd: [0.0, 0.0, 0.0]     # 目标位置偏移
      FRfootCmd: [0.0, 0.0, 0.0]  # 前右足端偏移
      FLfootCmd: [0.0, 0.0, 0.0]  # 前左足端偏移
      RRfootCmd: [0.0, 0.0, 0.0]  # 后右足端偏移
      RLfootCmd: [0.0, 0.0, 0.0]  # 后左足端偏移
      duration: 0.8                # 持续时间（秒）
    - motionType: pose             # 姿态保持
    - motionType: foot             # 足端运动
```

### 7.2 XG 平台预定义动作

| 动作 | 可中断 | 说明 |
|------|--------|------|
| XG_defaultStand | 否 | 默认站立 |
| XG_sitdown | 是 | 坐下 |
| XG_pee | 否 | 抬腿（撒尿姿态） |
| XG_stretchleg | 否 | 伸腿 |
| XG_shakehand | 否 | 握手 |
| XG_greet | 否 | 打招呼（鞠躬+挥手） |

## 8. 通信协议

### 8.1 UE 仿真通信（motor_platform_type=8）

```
UE → mc_ctrl (port 25001): protobuf RobotState, 303 bytes
  ├── time_stamp: uint64 (CLOCK_REALTIME 纳秒)
  ├── joint_angle: float[12] (4腿×3关节)
  ├── joint_velocity: float[12]
  ├── imu_quaternion: float[4]
  ├── imu_gyroscope: float[3]
  ├── imu_accelerometer: float[3]
  └── (无 rpy 字段)

mc_ctrl → UE (port 25002): protobuf RobotCmd
  ├── joint_position: float[12] (目标关节角)
  ├── kp: float[12]
  └── kd: float[12]
```

### 8.2 数据有效性验证（reciveData 三重检查）

1. **flag 0x241** (data_valid): UEUDPReceiver 成功解析 protobuf
2. **flag 0x242** (fresh): 数据未被消费过
3. **时间戳检查**: `|CLOCK_REALTIME_now - msg.time_stamp| < 10ms`

### 8.3 SDK 通信

- 端口: 43988（由 sdk_config.yaml 配置）
- 协议: zsibot::rmw 自定义中间件
- 话题: RobotState, SDKRobotState, AppCmd, SDKCmd, VisualData, BatteryState

## 9. 控制频率与时序

```
┌─────────────────────────────────────────────────┐
│ 高层控制: 500Hz (dt=0.002s)                      │
│   RobotRunner::run()                             │
│   ├── reciveData()     ← 读取仿真状态            │
│   ├── StateEstimator   ← IMU + 接触估计          │
│   ├── ControlFSM       ← 状态机转换              │
│   ├── RL/MPC/WBC       ← 计算控制量              │
│   └── sendCmd()        ← 发送关节命令            │
├─────────────────────────────────────────────────┤
│ 低层控制: 5000Hz (dt=0.0002s)                    │
│   PD 控制器（在仿真器/硬件侧执行）               │
├─────────────────────────────────────────────────┤
│ 物理仿真: 1000Hz (dt=0.001s)                     │
│   MuJoCo/UE 物理步进                             │
└─────────────────────────────────────────────────┘
```

## 10. 启动流程

### 10.1 run_mc.sh

```bash
export LD_LIBRARY_PATH=${DIR}/build/export/mc/bin
export ROBOT_TYPE=XG
cd ${DIR}/build/export/mc/bin/
taskset -c 7 ./mc_ctrl r    # 绑定 CPU 核心 7，实时运行
```

### 10.2 Matrix 完整启动顺序（run_sim.sh）

```
1. MuJoCo 物理仿真器启动（可选）
2. UE 渲染+物理仿真器启动
3. 等待 7 秒（仿真器初始化）
4. mc_ctrl 启动
   ├── 加载 robot-defaults.yaml
   ├── 加载 {ROBOT_TYPE}-user-parameters.yaml
   ├── 加载 {ROBOT_TYPE}-motion_config.yaml
   ├── 根据 motor_platform_type 创建 CommandInterface
   ├── 加载 RL 模型（rl_onnx_config.yaml）
   ├── 启动 UEUDPReceiver 线程
   └── 进入 500Hz 控制循环
```

## 11. FSM 状态机

```
PASSIVE ──(启动)──→ STANDUP ──(站立完成)──→ RL_MIX
   │                    │                      │
   │                    │                      ├── balancestand
   │                    │                      ├── mix_walk
   │                    │                      ├── gait_walk
   │                    │                      ├── backflip/jump/...
   │                    │                      └── tracking
   │                    │
   └──(异常)──→ FOLDLEGS ←──(保护)──────────────┘
```

- **PASSIVE**: 电机阻尼模式（Kd=3.0），无位置控制
- **STANDUP**: 从趴下姿态插值到站立关节角（Kp=80）
- **RL_MIX**: RL 策略控制（Kp=20, Kd=0.7），支持多种运动模式切换

## 12. 依赖关系

### 12.1 运行时依赖

- Eigen3（线性代数）
- Protobuf（UDP 通信序列化）
- ONNX Runtime（RL 推理）
- Boost.Interprocess（共享内存通信）
- LCM（轻量级通信库，手柄/调试）
- qpOASES / Goldfarb（QP 求解）
- yaml-cpp（配置解析）
- zsibot::rmw（自研中间件）

### 12.2 硬件依赖（实机模式）

- Lord IMU（串口，libP2.so）
- SPI 电机驱动板
- 手柄（LCM gamepad_lcmt）

## 13. 关键设计特点

1. **多平台统一架构**: 通过 `motor_platform_type` 切换通信后端，同一控制逻辑适配实机/仿真
2. **RL + 传统控制混合**: FSM 管理状态转换，RL 策略负责运动生成，MPC/WBC 提供稳定性保障
3. **模型加密保护**: RL 模型文件加密存储，运行时解密，防止知识产权泄露
4. **实时性保证**: `taskset -c 7` CPU 核绑定，500Hz 确定性控制循环
5. **安全监控**: RobotMonitor 独立检查 IMU/电机/FSM 状态，异常时触发保护
6. **多机器人适配**: 通过配置文件（非代码）适配 XG/ZG/XGW/ZGWS 等不同构型
