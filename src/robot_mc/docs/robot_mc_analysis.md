# robot_mc 深入分析与实现计划

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
| `libbiomimetics.so` | — | 基础工具库（PeriodicTask、ControlParameters、SpineBoard、DanceController） |
| `libP2.so` | 3.6 MB | IMU 硬件驱动（Lord IMU 串口通信） |
| `libqpOASES.so` | 1.7 MB | 二次规划求解器（MPC/WBC 使用） |
| `libonnx_model.so` | 1.1 MB | ONNX Runtime 推理封装（inference::ModelInference） |
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
    │   ├── initCommon()        ← 初始化 zsibot::rmw 订阅（AppCmd, BatteryState, VisualData, SDKCmd）
    │   ├── loadParameters()    ← 加载 YAML 配置
    │   ├── handleGamepadLCM()  ← 手柄输入
    │   ├── handleInterfaceLCM()
    │   ├── imuResultCallBack() ← IMU 数据回调
    │   └── run()
    │
    ├── RobotRunner             ← 主控制循环（500Hz PeriodicTask）
    │   ├── init()
    │   ├── run()               ← 500Hz 主循环
    │   ├── setupStep()
    │   ├── finalizeStep()      ← 发布 LCM 调试数据
    │   ├── initializeStateEstimator()
    │   ├── initEcalData()      ← eCAL 通信初始化
    │   ├── pub_leg_command()   ← 发布关节命令
    │   ├── pub_leg_data()
    │   ├── pub_high_state()    ← 发布高层状态
    │   ├── pub_low_state()     ← 发布低层状态
    │   ├── pub_sdk_state()     ← 发布 SDK 状态
    │   ├── pub_app_state()     ← 发布 APP 状态
    │   ├── run_lowlevelsdk()   ← 低层 SDK 接口
    │   ├── run_highlevelsdk()  ← 高层 SDK 接口
    │   ├── read_data_from_file()
    │   ├── save_data_periodically()
    │   └── signal_handler()
    │
    ├── CommandInterface        ← 通信接口工厂
    │   ├── getCmdInterface(int motor_platform_type)
    │   └── setCommandAndState(SpiCommand*, SpiData*, VectorNavData*)
    │
    ├── RobotMonitor            ← 安全监控（独立线程）
    │   ├── fsm_state_check()
    │   ├── imu_data_check()
    │   ├── motor_data_check()
    │   ├── rl_loadpolicy_check()
    │   ├── sdk_init_check()
    │   ├── app_Node_check()
    │   └── run()
    │
    └── ContactEstimator<float> ← 足端接触估计
```

### 4.2 通信接口（CommandInterface 子类）

`motor_platform_type` 参数决定使用哪个通信后端：

| motor_platform_type | 接口类 | 通信方式 | 使用场景 |
|---------------------|--------|----------|----------|
| 0 | SPI 直连 | 硬件 SPI 总线 | 实机（默认） |
| 1 | Unitree | Unitree SDK | Unitree 电机 |
| 5 | `MujocoCommandInterface` | eCAL + UDP protobuf | MuJoCo 仿真 |
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

#### MujocoCommandInterface（type=5）关键方法：

```cpp
MujocoCommandInterface::initialize()   // eCAL 订阅 RobotState + lambda 回调
MujocoCommandInterface::reciveData()   // 读取 eCAL 数据
MujocoCommandInterface::sendCmd()      // 发送命令
MujocoCommandInterface::getInstance()  // 单例模式
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

### 4.4 ONNX 推理引擎

`libonnx_model.so` 导出符号：

```cpp
inference::ModelInference::inferLoop()              // 纯策略推理（无状态估计）
inference::ModelInference::inferLoopWithEstimator()  // 策略推理 + 里程计估计
inference::ModelInference::reset()                   // 重置推理状态
```

推理在独立线程中运行，由 `ModelInference` 管理。支持 TensorRT 加速（`trt_max_workspace_size`, `trt_min_subgraph_size`）。

### 4.5 底层控制库

`libbiomimetics.so` 提供基础控制框架：

```cpp
PeriodicTask::start()/stop()/loopFunction()    // 实时任务调度
PeriodicTaskManager::addTask()/stopAll()       // 任务管理器
ControlParameters::generateUnitializedList()   // 参数管理
SpineBoard::init()/run()/resetCommand()        // 电机驱动抽象
DanceController::loadAllDances()/onEnter()     // 舞蹈控制
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
FSM_passive_Kd: 3.0           # 被动模式阻尼
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
use_wbc: 1                   # 启用 WBC

# 关节限位
JPos_limit_low: [-0.48, -1.15, -2.7]
JPos_limit_high: [0.48, 2.97, -0.65]
jointVelocityLimit: 24.0     # rad/s

# 关节偏移（站立姿态）
abad_stand_pos:   [0., 0., 0., 0.]
hip_stand_pos:    [0.8, 0.8, 0.8, 0.8]
knee_stand_pos:   [-1.5, -1.5, -1.5, -1.5]

# 关节偏移（趴下姿态）
abad_liedown_pos: [0., 0., 0., 0.]
hip_liedown_pos:  [1.4, 1.4, 1.4, 1.4]
knee_liedown_pos: [-2.4, -2.4, -2.4, -2.4]

# 腿侧符号
abad_side_sign: [-1., -1., 1., 1.]
hip_side_sign:  [1., -1., 1., -1.]
knee_side_sign: [1., -1., 1., -1.]

# 关节偏移量
abad_offset: [0.488, -0.488, -0.488, 0.488]
hip_offset:  [-2.967, 2.967, -2.967, 2.967]
knee_offset: [2.723, -2.723, 2.723, -2.723]
```

### 5.4 仿真器参数

```yaml
dynamics_dt: 0.001             # 1000Hz 物理仿真
high_level_dt: 0.002           # 500Hz 高层控制
low_level_dt: 0.0002           # 5000Hz 低层控制（PD）
floor_kp: 500000               # 地面刚度
floor_kd: 5000                 # 地面阻尼
vectornav_imu_accelerometer_noise: 0.005
vectornav_imu_gyro_noise: 0.005
vectornav_imu_quat_noise: 0.003
```

## 6. RL 策略模型系统

### 6.1 模型架构

每个运动模式由两个 ONNX 模型组成：
- **policy**: 策略网络，输入观测→输出目标关节角
- **odom**: 状态估计网络（odometry），输入历史观测→估计速度/位置

### 6.2 ONNX 模型内部张量（从 libonnx_model.so 逆向）

从 `libonnx_model.so` 的字符串表和调试信息中提取的关键张量名：

| 张量名 | 用途 |
|--------|------|
| `policy_obs` | 策略网络输入张量 |
| `estimator_obs` | 里程计网络输入张量 |
| `estimator_outputs_` | 里程计网络输出张量 |
| `obs_scales_lin_vel` | 线速度观测缩放系数（外部参数） |

**关键发现**：
- 策略输入名为 `policy_obs`（非 RSL_RL 默认的 `obs`），说明是**自研训练框架**
- `obs_scales_lin_vel` 作为外部参数传入，说明线速度缩放在推理时动态配置
- 支持 `inferLoop()`（纯策略）和 `inferLoopWithEstimator()`（策略+里程计联合）两种模式

### 6.3 原始开发环境（从调试信息泄露）

从 `libonnx_model.so` 的 DW_AT_comp_dir 调试属性提取：

```
源码路径: /home/xin/Toolkit/zsibot_sim/src/jszr_mc/onnx_infer/src/
├── Inference.cpp      ← 推理主逻辑
├── OnnxModel.cpp      ← ONNX 模型加载/管理
└── CMakeLists.txt     ← 构建配置
```

- 开发者用户名: `xin`
- 项目名: `zsibot_sim`（zsibot 仿真工具包）
- 模块名: `jszr_mc`（zsibot robot motion controller）

### 6.4 RL 训练框架判定

**结论：自研训练框架，非 RSL_RL / legged_gym / stable-baselines3 等开源方案。**

判定依据：

| 特征 | robot_mc | RSL_RL / legged_gym | 开源框架典型 |
|------|----------|---------------------|-------------|
| 策略输入张量名 | `policy_obs` | `obs` / `observations` | 各异 |
| 里程计模型 | 独立 odom ONNX | 无（直接用 sim 速度） | 通常无 |
| 推理封装 | 自研 `inference::ModelInference` | 无 C++ 推理封装 | torch.onnx 直出 |
| 模型加密 | FileCrypto 自研加密 | 无加密 | 无 |
| TensorRT 支持 | 内置 trt 参数 | 无 | 需自行集成 |
| 双模型架构 | policy + odom 联合推理 | 单策略 | 单策略 |

**训练框架推测**：
- 基于 PyTorch 训练（ONNX 导出）
- 自研 PPO 实现或修改版（非标准 RSL_RL）
- 自定义观测归一化（`obs_scales_lin_vel` 外部传入而非模型内置）
- 里程计网络为自创设计（从历史观测窗口估计速度）

### 6.5 支持的运动模式（XG 平台，共 20+ 种）

| 运动模式 | 策略模型 | 说明 |
|----------|----------|------|
| mix_walk | policy_mix_walk | 混合行走（默认 RL 步态） |
| gait_walk | policy_gait_walk | 步态行走 |
| balancestand | policy_balancestand_withyaw_0423 | 平衡站立（含偏航） |
| backflip | policy_mix_backflip | 后空翻 |
| frontflip | policy_mix_frontflip | 前空翻 |
| sideflip | policy_mix-sideflip | 侧空翻 |
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

### 6.6 模型加密

- 所有模型文件经过 `zsibot::mc::FileCrypto` 加密
- 运行时由 `libfilecrypto_shared.so` 解密后加载
- 支持 ONNX（x86）和 RKNN（ARM NPU）两种推理后端
- 加密模型存放: `onnx_model_crypto/xg/` 目录

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
      FLfootCmd: [0.0, 0.0, 0.0]
      RRfootCmd: [0.0, 0.0, 0.0]
      RLfootCmd: [0.0, 0.0, 0.0]
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
7. **自研 RL 框架**: 非开源框架，自研训练代码 + 推理封装 + 模型加密

---

## 14. robot_mc 闭源分析：核心黑盒内容

robot_mc 中**无法获取源码**的部分及其功能：

### 14.1 RL 策略模型（最核心）

- **加密的 ONNX 文件**: policy_* 和 odom_* 模型，经过 FileCrypto 加密
- **观测空间 → 动作空间的映射**: 完全编码在神经网络权重中，无法提取
- **训练参数**: 奖励函数、课程学习策略、域随机化参数等全部不可知
- **训练框架**: 自研（非 RSL_RL），基于 PyTorch + 自研 PPO

### 14.2 状态估计器（StateEstimator）

- IMU 数据融合算法（互补滤波 / EKF / UKF）
- 足端接触估计（ContactEstimator<float>）
- 里程计估计（由 odom ONNX 模型辅助）

### 14.3 控制 FSM（ControlFSM）

- 状态转移条件和逻辑
- 各状态下的控制律（PASSIVE/STANDUP/RL_MIX/FOLDLEGS）
- 模式切换的平滑过渡逻辑

### 14.4 MPC + WBC（可选模块）

- MPC 的具体实现（虽然参数可见，但目标函数、约束条件的细节在 librobot.so 中）
- WBC 的力分配策略（libWBC.so + libWBC_CTRL.so 无导出符号，完全闭源）
- QP 问题构建（libqpOASES.so 和 libGoldfarb_Optimizer.so 是开源的，但问题建模闭源）

### 14.5 预定义动作系统

- motionType (transition/pose/foot) 的插值算法
- 动作序列的时序调度
- 可中断/不可中断的状态管理

---

## 15. 实现 robot_mc 的计划（路线 B：重新训练 RL）

### 15.1 核心思路

不依赖加密的 ONNX 模型，从零搭建 RL 训练环境，训练自己的策略网络，然后部署到自研控制框架中。

**优势**: 完全自主可控，不依赖任何闭源组件，可针对特定需求优化。
**挑战**: 需要 RL 训练经验，调参周期长（2-4 个月），效果需要迭代优化。

### 15.2 总体架构

```
┌──────────────────────────────────────────────────────────────┐
│                    路线 B 总体架构                              │
├─────────────────────┬────────────────────────────────────────┤
│   训练阶段（离线）    │   部署阶段（在线）                       │
│                     │                                        │
│  MuJoCo XML 模型    │  自研 robot_mc 控制器                    │
│        ↓            │  ├── UDP 通信层 (25001/25002)           │
│  Isaac Lab / RSL_RL │  ├── 配置系统 (yaml-cpp)                │
│  PPO 训练           │  ├── FSM 状态机                         │
│        ↓            │  ├── ONNX 推理（自训模型）               │
│  ONNX 模型导出      │  ├── 状态估计器                          │
│        ↓            │  └── 安全监控                            │
│  部署到 C++ 控制器   │                                        │
└─────────────────────┴────────────────────────────────────────┘
```

### 15.3 阶段 B1：MuJoCo 机器人模型准备

**目标**: 建立与 XG 机器人一致的 MuJoCo 仿真模型。

```
实现内容:
├── XML 模型文件
│   ├── 躯干: 质量、惯量矩阵、碰撞体
│   ├── 4 条腿 × 3 关节 (abad/hip/knee)
│   ├── 关节限位: abad ±0.48, hip [-1.15, 2.97], knee [-2.7, -0.65]
│   ├── 电机力矩限制: ±28 Nm
│   ├── 地面: plane + 摩擦系数
│   └── 传感器: IMU, 关节位置/速度, 足端接触
│
├── 关键参数（从 xg-user-parameters.yaml 提取）
│   ├── body_half_length: 0.175 m, body_height: 0.32 m
│   ├── 关节偏移/侧向符号
│   ├── 站立姿态: hip=0.8, knee=-1.5
│   └── 趴下姿态: hip=1.4, knee=-2.4
│
└── 验证: 静止站立稳定、关节限位正确、质量/惯量合理
```

**现有资源**: mujoco_sim 已有 XG 的 XML 模型，可直接复用。

### 15.4 阶段 B2：RL 训练环境搭建

```
技术选型（二选一）:

方案 A: Isaac Lab + RSL_RL（推荐）
  ├── GPU 并行数千环境，训练速度快（小时级）
  ├── 硬件需求: NVIDIA GPU (RTX 3090/4090/A100)
  └── 参考: legged_gym, unitree_rl_gym

方案 B: MuJoCo + stable-baselines3
  ├── 与部署物理一致，sim-to-real gap 小
  ├── 多进程并行，训练速度慢（天级）
  └── 适合: 无 GPU 或快速验证
```

### 15.5 阶段 B3：观测空间和动作空间设计

**观测空间（参考 robot_mc 的 `policy_obs` 张量 + 业界标准）:**

| 序号 | 观测项 | 维度 | 说明 |
|------|--------|------|------|
| 1 | 躯干姿态 (rpy) | 3 | 从 IMU 四元数提取 |
| 2 | 躯干角速度 (wx, wy, wz) | 3 | body-frame gyro（cvel + R^T 变换） |
| 3 | 速度命令 (vx, vy, wz) | 3 | 归一化到 [-1, 1] |
| 4 | 关节位置偏差 (q - q_default) | 12 | 相对于默认站立姿态 |
| 5 | 关节速度 (qd) | 12 | |
| 6 | 上次动作 (last_action) | 12 | 动作平滑正则化 |
| 7 | 步态相位 (phase) | 4 | sin/cos 编码 |
| 8 | 接触状态 (contact) | 4 | 足端是否着地 |
| **总计** | | **53** | |

> **注意**: robot_mc 原始模型的 `policy_obs` 确切维度需加载 ONNX 后验证 input tensor shape。
> 以上 53 维是基于业界标准和 robot_mc 推断的合理估计。

**动作空间:**

| 项目 | 维度 | 范围 | 说明 |
|------|------|------|------|
| 目标关节位置偏移 | 12 | [-1, 1] (tanh) | q_target = q_default + scale * action |

```python
action_scale = 0.25  # 最大偏移约 0.25 rad
q_target = q_default + action_scale * np.tanh(action)
```

**PPO 超参数（RSL_RL 默认值作为起点）:**

```yaml
algorithm: PPO
max_iterations: 1500
num_envs: 4096
num_steps_per_env: 24
learning_rate: 1e-3
gamma: 0.99
lam: 0.95
actor_hidden_dims: [512, 256, 128]
critic_hidden_dims: [512, 256, 128]
```

### 15.6 阶段 B4：奖励函数设计

```
正向奖励:
  ├── tracking_lin_vel:  exp(-||v_cmd - v_actual||² / 0.25²)    w=1.0
  ├── tracking_ang_vel:  exp(-||ω_cmd - ω_actual||² / 0.25²)   w=0.5
  └── alive:             1.0                                      w=0.5

负向惩罚:
  ├── torque:            ||τ||²                                   w=-0.001
  ├── action_rate:       ||a_t - a_{t-1}||²                       w=-0.01
  ├── lin_vel_z:         |vz|²                                    w=-0.5
  ├── ang_vel_xy:        |wx|² + |wy|²                            w=-0.05
  ├── collision:         body 碰撞地面                              w=-2.0
  ├── fall:              |roll|>0.5 or |pitch|>0.5                 w=-200.0
  └── out_of_bounds:     关节超限位                                 w=-10.0
```

### 15.7 阶段 B5：课程学习策略

```
Phase 1: 平衡站立 (0-500 iter)
  ├── 速度命令: vx=0, vy=0, wz=0
  └── 目标: 学会站立并保持平衡

Phase 2: 低速行走 (500-1000 iter)
  ├── 速度命令: vx ∈ [0, 0.5], vy=0, wz=0
  └── 目标: 学会向前慢走

Phase 3: 全向行走 (1000-1500 iter)
  ├── 速度命令: vx ∈ [0, 1.5], vy ∈ [-0.5, 0.5], wz ∈ [-1, 1]
  └── 目标: 全向运动

Phase 4: 极限运动 (1500+ iter)
  ├── 速度命令: vx ∈ [0, 3.0], vy ∈ [-1.0, 1.0], wz ∈ [-3.0, 3.0]
  └── 与 Phase 3 混合训练
```

### 15.8 阶段 B6：里程计模型（可选但推荐）

robot_mc 分离了 policy + odom 两个 ONNX 模型。odom 从历史观测窗口估计速度。

```
odom 网络:
  ├── 输入: 过去 N 步观测窗口 (N=10~20), 每步 ~34 维
  ├── 输出: 估计线速度 [vx, vy, vz] (3 维)
  ├── 网络: MLP [256, 128, 64] + GRU 隐层
  └── 训练目标: MSE(估计速度, 真实速度)
```

### 15.9 阶段 B7：模型导出与部署

```
PyTorch → ONNX (torch.onnx.export)
  ├── 输入张量命名: policy_obs（与 robot_mc 风格一致）
  ├── 固定输入 shape, opset_version=11+
  ├── 验证: numpy 推理结果一致
  └── 可选量化: FP32 → FP16 (GPU) / INT8 (嵌入式)

C++ 推理: ONNX Runtime
  ├── 推理延迟: < 0.5ms (GPU), < 2ms (CPU)
  └── 500Hz 控制循环内可接受
```

### 15.10 阶段 B8：自研控制框架

```
robot_mc_open/
├── CMakeLists.txt
├── src/
│   ├── main.cpp                  # 入口
│   ├── udp/
│   │   ├── udp_receiver.cpp      # 接收 RobotState (port 25001)
│   │   └── udp_sender.cpp        # 发送 RobotCmd (port 25002)
│   ├── config/
│   │   └── config_loader.cpp     # YAML 配置加载
│   ├── fsm/
│   │   └── control_fsm.cpp       # PASSIVE → STANDUP → RL_MIX
│   ├── policy/
│   │   ├── policy_inference.cpp  # ONNX 推理
│   │   └── odom_inference.cpp    # 里程计推理（可选）
│   ├── state/
│   │   └── state_estimator.cpp   # IMU 处理 + 接触估计
│   ├── safety/
│   │   └── robot_monitor.cpp     # 安全监控
│   └── proto/
│       └── robot_sdk.pb.cc       # protobuf 生成
├── config/                        # 复用 robot_mc 配置
└── models/                        # 自训练 ONNX 模型
```

**控制主循环 (500Hz):**

```cpp
void controlLoop() {
    receiveData();                    // UDP 25001 → RobotState
    stateEstimator.update(imu, q, qd);
    
    switch (fsm.getState()) {
        case PASSIVE:  cmd = passiveControl();  break;  // Kd=3.0 阻尼
        case STANDUP:  cmd = standUpControl();  break;  // Kp=80, Kd=1.0
        case RL_MIX:   obs = buildObs(stateEstimator, velCmd);
                       action = policy.infer(obs);
                       cmd = rlControl(action);   break;  // Kp=20, Kd=0.7
    }
    
    sendCmd(cmd);                     // UDP 25002 → RobotCmd
    robotMonitor.check();
}
```

### 15.11 工作量估算

| 阶段 | 内容 | 工作量 | 依赖 |
|------|------|--------|------|
| B1: MuJoCo 模型 | XML 建模 + 参数调校 | 1-2 周 | 无 |
| B2: 训练环境 | Isaac Lab / MuJoCo + RSL_RL | 1-2 周 | B1 |
| B3: 空间设计 | 观测/动作空间定义 | 3-5 天 | B2 |
| B4: 奖励函数 | 奖励设计 + 权重调参 | 2-4 周 | B3 |
| B5: 课程学习 | 课程策略 + 训练到收敛 | 1-2 周 | B4 |
| B6: 里程计 | odom 网络训练 | 1 周 | B5（可并行） |
| B7: 模型导出 | PyTorch → ONNX → C++ | 3-5 天 | B5 |
| B8: 控制框架 | 通信 + FSM + 推理 + 安全 | 2-3 周 | B7 |
| **总计** | | **2-4 个月** | |

### 15.12 关键里程碑

```
M1 (第1月): MuJoCo 中 PD 站立成功
M2 (第2月): 仿真中稳定行走
M3 (第3月): ONNX 部署到 C++ 控制器
M4 (第4月): CarlaUnreal 端到端行走
```

### 15.13 风险和对策

| 风险 | 对策 |
|------|------|
| 奖励函数调参困难 | 从 legged_gym 默认参数出发，逐步调整 |
| Sim-to-Sim gap (MuJoCo→UE) | 域随机化 + 在 UE 物理参数下微调 |
| 训练速度慢 | 优先用 Isaac Gym (GPU 并行) |
| 观测空间不匹配 | 严格对照 robot_mc 推断维度（policy_obs） |
| 站立阶段不稳定 | 先用 PD 站立（非 RL），RL 只负责行走 |
| 无 GPU 硬件 | 备选: MuJoCo + 多进程 CPU 训练 |

### 15.14 推荐参考项目

| 项目 | 用途 |
|------|------|
| RSL_RL (leggedrobotics) | ETH 四足 RL 训练框架 |
| legged_gym (leggedrobotics) | Isaac Gym 四足训练环境 |
| unitree_rl_gym (unitreerobotics) | Unitree 官方 RL 训练 |
| walk-these-ways (Improbable-AI) | 四足通用 RL 框架 |
| simple_qd (qiayuanliao) | 简洁四足 RL 实现 |
| Cheetah-Software (MIT) | MIT 猎豹控制框架 |
# robot_mc 深入分析与实现计划

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
| `libbiomimetics.so` | — | 基础工具库（PeriodicTask、ControlParameters、SpineBoard、DanceController） |
| `libP2.so` | 3.6 MB | IMU 硬件驱动（Lord IMU 串口通信） |
| `libqpOASES.so` | 1.7 MB | 二次规划求解器（MPC/WBC 使用） |
| `libonnx_model.so` | 1.1 MB | ONNX Runtime 推理封装（inference::ModelInference） |
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
    │   ├── initCommon()        ← 初始化 zsibot::rmw 订阅（AppCmd, BatteryState, VisualData, SDKCmd）
    │   ├── loadParameters()    ← 加载 YAML 配置
    │   ├── handleGamepadLCM()  ← 手柄输入
    │   ├── handleInterfaceLCM()
    │   ├── imuResultCallBack() ← IMU 数据回调
    │   └── run()
    │
    ├── RobotRunner             ← 主控制循环（500Hz PeriodicTask）
    │   ├── init()
    │   ├── run()               ← 500Hz 主循环
    │   ├── setupStep()
    │   ├── finalizeStep()      ← 发布 LCM 调试数据
    │   ├── initializeStateEstimator()
    │   ├── initEcalData()      ← eCAL 通信初始化
    │   ├── pub_leg_command()   ← 发布关节命令
    │   ├── pub_leg_data()
    │   ├── pub_high_state()    ← 发布高层状态
    │   ├── pub_low_state()     ← 发布低层状态
    │   ├── pub_sdk_state()     ← 发布 SDK 状态
    │   ├── pub_app_state()     ← 发布 APP 状态
    │   ├── run_lowlevelsdk()   ← 低层 SDK 接口
    │   ├── run_highlevelsdk()  ← 高层 SDK 接口
    │   ├── read_data_from_file()
    │   ├── save_data_periodically()
    │   └── signal_handler()
    │
    ├── CommandInterface        ← 通信接口工厂
    │   ├── getCmdInterface(int motor_platform_type)
    │   └── setCommandAndState(SpiCommand*, SpiData*, VectorNavData*)
    │
    ├── RobotMonitor            ← 安全监控（独立线程）
    │   ├── fsm_state_check()
    │   ├── imu_data_check()
    │   ├── motor_data_check()
    │   ├── rl_loadpolicy_check()
    │   ├── sdk_init_check()
    │   ├── app_Node_check()
    │   └── run()
    │
    └── ContactEstimator<float> ← 足端接触估计
```

### 4.2 通信接口（CommandInterface 子类）

`motor_platform_type` 参数决定使用哪个通信后端：

| motor_platform_type | 接口类 | 通信方式 | 使用场景 |
|---------------------|--------|----------|----------|
| 0 | SPI 直连 | 硬件 SPI 总线 | 实机（默认） |
| 1 | Unitree | Unitree SDK | Unitree 电机 |
| 5 | `MujocoCommandInterface` | eCAL + UDP protobuf | MuJoCo 仿真 |
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

#### MujocoCommandInterface（type=5）关键方法：

```cpp
MujocoCommandInterface::initialize()   // eCAL 订阅 RobotState + lambda 回调
MujocoCommandInterface::reciveData()   // 读取 eCAL 数据
MujocoCommandInterface::sendCmd()      // 发送命令
MujocoCommandInterface::getInstance()  // 单例模式
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

### 4.4 ONNX 推理引擎

`libonnx_model.so` 导出符号：

```cpp
inference::ModelInference::inferLoop()              // 纯策略推理（无状态估计）
inference::ModelInference::inferLoopWithEstimator()  // 策略推理 + 里程计估计
inference::ModelInference::reset()                   // 重置推理状态
```

推理在独立线程中运行，由 `ModelInference` 管理。

### 4.5 底层控制库

`libbiomimetics.so` 提供基础控制框架：

```cpp
PeriodicTask::start()/stop()/loopFunction()    // 实时任务调度
PeriodicTaskManager::addTask()/stopAll()       // 任务管理器
ControlParameters::generateUnitializedList()   // 参数管理
SpineBoard::init()/run()/resetCommand()        // 电机驱动抽象
DanceController::loadAllDances()/onEnter()     // 舞蹈控制
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
| `sdk_config.yaml` | SDK 通信配置（target_ip:port） |

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
controller_dt: 0.002          # 500Hz 控制循环
FSM_jointPD_Kp: 80            # 站立/被动模式 Kp
FSM_jointPD_Kd: 1.0
FSM_passive_Kd: 3.0           # 被动模式阻尼
FSM_RL_ABAD_Kp: 20.0         # RL 模式 Kp
FSM_RL_HIP_Kp: 20.0
FSM_RL_KNEE_Kp: 20.0
FSM_RL_Kd: 0.7
body_height: 0.32             # 目标身体高度 m
mpc_horizon_length: 14        # MPC 预测步长
use_wbc: 1                    # 启用 WBC
JPos_limit_low: [-0.48, -1.15, -2.7]
JPos_limit_high: [0.48, 2.97, -0.65]
jointVelocityLimit: 24.0      # rad/s
abad_stand_pos:   [0., 0., 0., 0.]
hip_stand_pos:    [0.8, 0.8, 0.8, 0.8]
knee_stand_pos:   [-1.5, -1.5, -1.5, -1.5]
abad_liedown_pos: [0., 0., 0., 0.]
hip_liedown_pos:  [1.4, 1.4, 1.4, 1.4]
knee_liedown_pos: [-2.4, -2.4, -2.4, -2.4]
```

## 6. RL 策略模型系统

每个运动模式由两个 ONNX 模型组成：
- **policy**: 策略网络，输入观测→输出目标关节角
- **odom**: 里程计网络，输入历史观测→估计速度/位置

### 6.1 支持的运动模式（XG 平台）

| 运动模式 | 说明 |
|----------|------|
| mix_walk | 混合行走（默认） |
| gait_walk | 步态行走 |
| balancestand | 平衡站立（含偏航） |
| backflip / frontflip / sideflip | 翻转特技 |
| jump | 跳跃 |
| upright | 直立恢复 |
| tracking | 轨迹跟踪 |
| crawl | 匍匐 |
| moonwalk | 太空步 |
| soft_walk | 柔性行走 |
| flipover | 翻转恢复 |

### 6.2 模型加密

- 所有模型经 `zsibot::mc::FileCrypto` 加密
- 运行时由 `libfilecrypto_shared.so` 解密后加载
- 支持 ONNX（x86）和 RKNN（ARM NPU）两种推理后端

## 7. 通信协议

```
UE → mc_ctrl (port 25001): protobuf RobotState
  ├── time_stamp: uint64 (CLOCK_REALTIME 纳秒)
  ├── joint_angle: float[12], joint_velocity: float[12]
  ├── imu_quaternion: float[4], imu_gyroscope: float[3], imu_accelerometer: float[3]

mc_ctrl → UE (port 25002): protobuf RobotCmd
  ├── joint_position: float[12], kp: float[12], kd: float[12]
```

数据有效性三重验证: flag 0x241 (valid) + flag 0x242 (fresh) + 时间戳 < 10ms

## 8. FSM 状态机

```
PASSIVE ──(启动)──→ STANDUP ──(站立完成)──→ RL_MIX
   │                    │                      │
   └──(异常)──→ FOLDLEGS ←──(保护)──────────────┘
```

- **PASSIVE**: 阻尼模式（Kd=3.0），无位置控制
- **STANDUP**: PD 控制到站立姿态（Kp=80），从 liedown 插值到 stand
- **RL_MIX**: RL 策略控制（Kp=20, Kd=0.7），支持多种运动模式切换

## 9. 控制频率

```
高层控制: 500Hz (dt=0.002s) — RobotRunner::run()
低层控制: 5000Hz (dt=0.0002s) — PD 控制器（仿真器侧执行）
物理仿真: 1000Hz (dt=0.001s) — MuJoCo/UE 物理步进
```

---

## 10. robot_mc 闭源分析

robot_mc 中无法获取源码的部分：

| 黑盒 | 内容 | 影响 |
|------|------|------|
| RL 策略模型 | 加密 ONNX 文件，观测→动作映射不可知 | 最核心，无法复制 |
| 状态估计器 | IMU 融合 + 接触估计 + 里程计 | 可用开源方案替代 |
| 控制 FSM | 状态转移条件 + 各状态控制律 | 可从配置推断 |
| MPC + WBC | 目标函数/约束/力分配 | libWBC.so 无导出符号 |
| 预定义动作 | 插值算法 + 时序调度 | 可从配置格式推断 |

---

## 11. 实现 robot_mc 的计划（路线 B：重新训练 RL）

### 11.1 核心思路

不依赖加密的 ONNX 模型，从零搭建 RL 训练环境，训练自己的策略网络，然后部署到自研控制框架中。

**优势**: 完全自主可控，不依赖任何闭源组件，可针对特定需求优化。
**挑战**: 需要 RL 训练经验，调参周期长（2-4 个月），效果需要迭代优化。

### 11.2 总体架构

```
┌──────────────────────────────────────────────────────────────┐
│                    路线 B 总体架构                              │
├─────────────────────┬────────────────────────────────────────┤
│   训练阶段（离线）    │   部署阶段（在线）                       │
│                     │                                        │
│  MuJoCo XML 模型    │  自研 robot_mc 控制器                    │
│        ↓            │  ├── UDP 通信层 (25001/25002)           │
│  Isaac Lab / RSL_RL │  ├── 配置系统 (yaml-cpp)                │
│  PPO 训练           │  ├── FSM 状态机                         │
│        ↓            │  ├── ONNX 推理（自训模型）               │
│  ONNX 模型导出      │  ├── 状态估计器                          │
│        ↓            │  └── 安全监控                            │
│  部署到 C++ 控制器   │                                        │
└─────────────────────┴────────────────────────────────────────┘
```

### 11.3 阶段 B1：MuJoCo 机器人模型准备

**目标**: 建立与 XG 机器人一致的 MuJoCo 仿真模型。

```
实现内容:
├── XML 模型文件
│   ├── 躯干: 质量、惯量矩阵、碰撞体
│   ├── 4 条腿 × 3 关节 (abad/hip/knee)
│   ├── 关节限位: abad ±0.48, hip [-1.15, 2.97], knee [-2.7, -0.65]
│   ├── 电机力矩限制: ±28 Nm
│   ├── 地面: plane + 摩擦系数
│   └── 传感器: IMU, 关节位置/速度, 足端接触
│
├── 关键参数（从 xg-user-parameters.yaml 提取）
│   ├── body_half_length: 0.175 m, body_height: 0.32 m
│   ├── 关节偏移/侧向符号
│   ├── 站立姿态: hip=0.8, knee=-1.5
│   └── 趴下姿态: hip=1.4, knee=-2.4
│
└── 验证: 静止站立稳定、关节限位正确、质量/惯量合理
```

**现有资源**: mujoco_sim 已有 XG 的 XML 模型，可直接复用。

### 11.4 阶段 B2：RL 训练环境搭建

```
技术选型（二选一）:

方案 A: Isaac Lab + RSL_RL（推荐）
  ├── GPU 并行数千环境，训练速度快（小时级）
  ├── 硬件需求: NVIDIA GPU (RTX 3090/4090/A100)
  └── 参考: legged_gym, unitree_rl_gym

方案 B: MuJoCo + stable-baselines3
  ├── 与部署物理一致，sim-to-real gap 小
  ├── 多进程并行，训练速度慢（天级）
  └── 适合: 无 GPU 或快速验证
```



Matrix RL 训练环境分析

**不是 stable-baselines3**。robot_mc 的训练环境是 **zsibot 自研框架**，但物理引擎确实是 MuJoCo。

## robot_mc 实际训练环境

| 组件         | 实际情况                                                     |
| ------------ | ------------------------------------------------------------ |
| **物理引擎** | MuJoCo（`zsibot_robots/xgw/xgw.xml` 等模型文件证实）         |
| **RL 框架**  | **zsibot_sim**（自研，非 stable-baselines3 / RSL_RL / legged_gym） |
| **模型输出** | 加密 ONNX（`onnx_model_crypto/` 目录，`libfilecrypto_shared.so` 解密） |
| **网络架构** | **双模型**：policy 网络 + estimator 网络（学习型里程计）     |
| **推理加速** | 支持 TensorRT                                                |
| **地形训练** | 多种 terrain XML（flat, sloped, stair, crowd, zombie 等）    |

## 关键证据

从 [rl_onnx_config.yaml](file:///home/qiyuan/Softwares/Matrix/src/robot_mc/build/export/config/rl_onnx_config.yaml) 可以看到，系统为**每种技能**都训练了独立的 policy + odom 模型对：

```
policy_mix_walk_path     → 行走策略
policy_mix_flipover_path → 翻身策略  
policy_mix_backflip_path → 后空翻策略
policy_balancestand_path → 平衡站立策略
policy_mix_climb_path    → 攀爬策略
policy_mix_crawl_path    → 爬行策略
...
```

这说明训练框架支持**多任务/多技能**的独立策略训练，每个技能一对 ONNX 模型（policy + odom），运行时由 FSM 状态机切换。

## 与你方案的对比

|          | robot_mc（自研）       | 方案 B（SB3）    |
| -------- | ---------------------- | ---------------- |
| 物理引擎 | MuJoCo ✅               | MuJoCo ✅         |
| RL 算法  | 自研 PPO（大概率）     | SB3 PPO          |
| 并行化   | 自研大规模并行         | 多进程（慢）     |
| 模型管理 | 加密 ONNX + 多技能切换 | 单模型           |
| 可复现性 | ❌ 闭源不可复现         | ✅ 完全开源可复现 |

**结论**：物理引擎选 MuJoCo 是对的，但 RL 框架不是 SB3——是 zsibot 团队自研的内部工具，不对外开源。如果你要自己训练，用 MuJoCo + SB3/RSL_RL 是可以的，只是训练效率和奖励设计需要自己摸索。



根据逆向分析，zsibot_sim 主要做了以下工作：

## zsibot_sim 框架核心工作

### 1. 训练侧（Python，不开放）

| 模块           | 内容                                                         |
| -------------- | ------------------------------------------------------------ |
| **物理环境**   | 基于 MuJoCo XML 模型构建仿真环境（多种 terrain：flat/stair/sloped/crowd 等） |
| **RL 算法**    | 自研 PPO 实现（非 RSL_RL/SB3），基于 PyTorch 训练            |
| **观测设计**   | 自定义 `policy_obs` 张量（包含 IMU、关节状态、历史动作、速度命令等） |
| **双模型架构** | 同时训练 **policy 网络**（输出动作）+ **estimator 网络**（学习型里程计，从历史观测估计速度） |
| **奖励设计**   | 自定义奖励函数（速度跟踪、姿态稳定、能量效率等，具体不可见） |
| **多技能训练** | 为 20+ 种运动模式分别训练独立策略（行走、后空翻、跳跃、匍匐、太空步...） |
| **模型加密**   | 训练后 ONNX 模型用 `libfilecrypto_shared.so` 加密保护        |

### 2. 推理侧（C++，部署在 robot_mc 中）

从 `libonnx_model.so` 逆向得到的导出函数：

```
inference::ModelInference
├── loadPolicy(path)              // 加载单个策略模型
├── loadPolicy(policy_path, odom_path)  // 加载 policy + odom 双模型
├── inferLoop()                   // 纯策略推理（无里程计）
├── inferLoopWithEstimator()      // policy + estimator 联合推理
├── updateObs(obs, reset)         // 更新观测输入
└── reset()                       // 重置隐状态（LSTM）
```

| 模块                  | 功能                                                         |
| --------------------- | ------------------------------------------------------------ |
| **ONNX Runtime 推理** | 加载加密 ONNX 模型，解密后运行推理                           |
| **观测预处理**        | 组装 `policy_obs` 向量，应用 `obs_scales_lin_vel` 等缩放参数 |
| **双模型联合推理**    | policy 输出动作 + estimator 输出速度估计，形成闭环           |
| **TensorRT 加速**     | 支持 TensorRT 后端加速推理（配置项 `trt`）                   |
| **LSTM 隐状态管理**   | 支持带 LSTM 的循环策略，`reset()` 清除隐状态                 |

### 3. 与传统开源框架的关键区别

|                | zsibot_sim                      | RSL_RL           | SB3           |
| -------------- | ------------------------------- | ---------------- | ------------- |
| **里程计**     | 独立 estimator 网络学习         | 直接用仿真真值   | 无            |
| **推理封装**   | C++ `inference::ModelInference` | 无（需自己部署） | 无            |
| **模型保护**   | 加密 ONNX                       | 无               | 无            |
| **多技能管理** | 20+ 策略由 FSM 切换             | 单策略           | 单策略        |
| **观测命名**   | `policy_obs`（自定义）          | `obs`（标准）    | `obs`（标准） |

**一句话总结**：zsibot_sim 的核心创新是**双模型架构**（policy + learned estimator）和**多技能策略库**（20+ 独立 ONNX 模型），配合自研加密和 C++ 推理引擎，形成了一套完整的从训练到部署的闭环。训练部分完全闭源，部署部分只暴露 3 个核心函数。



### 11.5 阶段 B3：观测空间和动作空间设计

**观测空间（参考业界标准 + robot_mc 推断）:**

| 序号 | 观测项 | 维度 | 说明 |
|------|--------|------|------|
| 1 | 躯干姿态 (rpy) | 3 | 从 IMU 四元数提取 |
| 2 | 躯干角速度 (wx, wy, wz) | 3 | body-frame gyro |
| 3 | 速度命令 (vx, vy, wz) | 3 | 归一化到 [-1, 1] |
| 4 | 关节位置偏差 (q - q_default) | 12 | 相对于默认站立姿态 |
| 5 | 关节速度 (qd) | 12 | |
| 6 | 上次动作 (last_action) | 12 | 动作平滑正则化 |
| 7 | 步态相位 (phase) | 4 | sin/cos 编码 |
| 8 | 接触状态 (contact) | 4 | 足端是否着地 |
| **总计** | | **53** | |

**动作空间:**

| 项目 | 维度 | 范围 | 说明 |
|------|------|------|------|
| 目标关节位置偏移 | 12 | [-1, 1] (tanh) | q_target = q_default + scale * action |

```python
action_scale = 0.25  # 最大偏移约 0.25 rad
q_target = q_default + action_scale * np.tanh(action)
```

**PPO 超参数（RSL_RL 默认值）:**

```yaml
algorithm: PPO
max_iterations: 1500
num_envs: 4096
num_steps_per_env: 24
learning_rate: 1e-3
gamma: 0.99
lam: 0.95
actor_hidden_dims: [512, 256, 128]
critic_hidden_dims: [512, 256, 128]
```

### 11.6 阶段 B4：奖励函数设计

```
正向奖励:
  ├── tracking_lin_vel:  exp(-||v_cmd - v_actual||² / 0.25²)    w=1.0
  ├── tracking_ang_vel:  exp(-||ω_cmd - ω_actual||² / 0.25²)   w=0.5
  └── alive:             1.0                                      w=0.5

负向惩罚:
  ├── torque:            ||τ||²                                   w=-0.001
  ├── action_rate:       ||a_t - a_{t-1}||²                       w=-0.01
  ├── lin_vel_z:         |vz|²                                    w=-0.5
  ├── ang_vel_xy:        |wx|² + |wy|²                            w=-0.05
  ├── collision:         body 碰撞地面                              w=-2.0
  ├── fall:              |roll|>0.5 or |pitch|>0.5                 w=-200.0
  └── out_of_bounds:     关节超限位                                 w=-10.0
```

### 11.7 阶段 B5：课程学习策略

```
Phase 1: 平衡站立 (0-500 iter)
  ├── 速度命令: vx=0, vy=0, wz=0
  └── 目标: 学会站立并保持平衡

Phase 2: 低速行走 (500-1000 iter)
  ├── 速度命令: vx ∈ [0, 0.5], vy=0, wz=0
  └── 目标: 学会向前慢走

Phase 3: 全向行走 (1000-1500 iter)
  ├── 速度命令: vx ∈ [0, 1.5], vy ∈ [-0.5, 0.5], wz ∈ [-1, 1]
  └── 目标: 全向运动

Phase 4: 极限运动 (1500+ iter)
  ├── 速度命令: vx ∈ [0, 3.0], vy ∈ [-1.0, 1.0], wz ∈ [-3.0, 3.0]
  └── 与 Phase 3 混合训练
```

### 11.8 阶段 B6：里程计模型（可选但推荐）

robot_mc 分离了 policy + odom 两个 ONNX 模型。odom 从历史观测窗口估计速度。

```
odom 网络:
  ├── 输入: 过去 N 步观测窗口 (N=10~20), 每步 34 维
  ├── 输出: 估计线速度 [vx, vy, vz] (3 维)
  ├── 网络: MLP [256, 128, 64] + GRU 隐层
  └── 训练目标: MSE(估计速度, 真实速度)
```

### 11.9 阶段 B7：模型导出与部署

```
PyTorch → ONNX (torch.onnx.export)
  ├── 固定输入 shape, opset_version=11+
  ├── 验证: numpy 推理结果一致
  └── 可选量化: FP32 → FP16 (GPU) / INT8 (嵌入式)

C++ 推理: ONNX Runtime
  ├── 推理延迟: < 0.5ms (GPU), < 2ms (CPU)
  └── 500Hz 控制循环内可接受
```

### 11.10 阶段 B8：自研控制框架

```
robot_mc_open/
├── CMakeLists.txt
├── src/
│   ├── main.cpp                  # 入口
│   ├── udp/
│   │   ├── udp_receiver.cpp      # 接收 RobotState (port 25001)
│   │   └── udp_sender.cpp        # 发送 RobotCmd (port 25002)
│   ├── config/
│   │   └── config_loader.cpp     # YAML 配置加载
│   ├── fsm/
│   │   └── control_fsm.cpp       # PASSIVE → STANDUP → RL_MIX
│   ├── policy/
│   │   ├── policy_inference.cpp  # ONNX 推理
│   │   └── odom_inference.cpp    # 里程计推理（可选）
│   ├── state/
│   │   └── state_estimator.cpp   # IMU 处理 + 接触估计
│   ├── safety/
│   │   └── robot_monitor.cpp     # 安全监控
│   └── proto/
│       └── robot_sdk.pb.cc       # protobuf 生成
├── config/                        # 复用 robot_mc 配置
└── models/                        # 自训练 ONNX 模型
```

**控制主循环 (500Hz):**

```cpp
void controlLoop() {
    receiveData();                    // UDP 25001 → RobotState
    stateEstimator.update(imu, q, qd);
    
    switch (fsm.getState()) {
        case PASSIVE:  cmd = passiveControl();  break;  // Kd=3.0 阻尼
        case STANDUP:  cmd = standUpControl();  break;  // Kp=150, Kd=2.0
        case RL_MIX:   obs = buildObs(stateEstimator, velCmd);
                       action = policy.infer(obs);
                       cmd = rlControl(action);   break;  // Kp=20, Kd=0.7
    }
    
    sendCmd(cmd);                     // UDP 25002 → RobotCmd
    robotMonitor.check();
}
```

### 11.11 工作量估算

| 阶段 | 内容 | 工作量 | 依赖 |
|------|------|--------|------|
| B1: MuJoCo 模型 | XML 建模 + 参数调校 | 1-2 周 | 无 |
| B2: 训练环境 | Isaac Lab / MuJoCo + RSL_RL | 1-2 周 | B1 |
| B3: 空间设计 | 观测/动作空间定义 | 3-5 天 | B2 |
| B4: 奖励函数 | 奖励设计 + 权重调参 | 2-4 周 | B3 |
| B5: 课程学习 | 课程策略 + 训练到收敛 | 1-2 周 | B4 |
| B6: 里程计 | odom 网络训练 | 1 周 | B5（可并行） |
| B7: 模型导出 | PyTorch → ONNX → C++ | 3-5 天 | B5 |
| B8: 控制框架 | 通信 + FSM + 推理 + 安全 | 2-3 周 | B7 |
| **总计** | | **2-4 个月** | |

### 11.12 关键里程碑

```
M1 (第1月): MuJoCo 中 PD 站立成功
M2 (第2月): 仿真中稳定行走
M3 (第3月): ONNX 部署到 C++ 控制器
M4 (第4月): CarlaUnreal 端到端行走
```

### 11.13 风险和对策

| 风险 | 对策 |
|------|------|
| 奖励函数调参困难 | 从 legged_gym 默认参数出发，逐步调整 |
| Sim-to-Sim gap (MuJoCo→UE) | 域随机化 + 在 UE 物理参数下微调 |
| 训练速度慢 | 优先用 Isaac Gym (GPU 并行) |
| 观测空间不匹配 | 严格对照 robot_mc 推断维度 |
| 站立阶段不稳定 | 先用 PD 站立（非 RL），RL 只负责行走 |
| 无 GPU 硬件 | 备选: MuJoCo + 多进程 CPU 训练 |

### 11.14 推荐参考项目

| 项目 | 用途 |
|------|------|
| RSL_RL (leggedrobotics) | ETH 四足 RL 训练框架 |
| legged_gym (leggedrobotics) | Isaac Gym 四足训练环境 |
| unitree_rl_gym (unitreerobotics) | Unitree 官方 RL 训练 |
| walk-these-ways (Improbable-AI) | 四足通用 RL 框架 |
| simple_qd (qiayuanliao) | 简洁四足 RL 实现 |
| Cheetah-Software (MIT) | MIT 猎豹控制框架 |
# robot_mc 深入分析与实现计划

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
| `libbiomimetics.so` | — | 基础工具库（PeriodicTask、ControlParameters、SpineBoard、DanceController） |
| `libP2.so` | 3.6 MB | IMU 硬件驱动（Lord IMU 串口通信） |
| `libqpOASES.so` | 1.7 MB | 二次规划求解器（MPC/WBC 使用） |
| `libonnx_model.so` | 1.1 MB | ONNX Runtime 推理封装（inference::ModelInference） |
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
    │   ├── initCommon()        ← 初始化 zsibot::rmw 订阅（AppCmd, BatteryState, VisualData, SDKCmd）
    │   ├── loadParameters()    ← 加载 YAML 配置
    │   ├── handleGamepadLCM()  ← 手柄输入
    │   ├── handleInterfaceLCM()
    │   ├── imuResultCallBack() ← IMU 数据回调
    │   └── run()
    │
    ├── RobotRunner             ← 主控制循环（500Hz PeriodicTask）
    │   ├── init()
    │   ├── run()               ← 500Hz 主循环
    │   ├── setupStep()
    │   ├── finalizeStep()      ← 发布 LCM 调试数据
    │   ├── initializeStateEstimator()
    │   ├── initEcalData()      ← eCAL 通信初始化
    │   ├── pub_leg_command()   ← 发布关节命令
    │   ├── pub_leg_data()
    │   ├── pub_high_state()    ← 发布高层状态
    │   ├── pub_low_state()     ← 发布低层状态
    │   ├── pub_sdk_state()     ← 发布 SDK 状态
    │   ├── pub_app_state()     ← 发布 APP 状态
    │   ├── run_lowlevelsdk()   ← 低层 SDK 接口
    │   ├── run_highlevelsdk()  ← 高层 SDK 接口
    │   ├── read_data_from_file()
    │   ├── save_data_periodically()
    │   └── signal_handler()
    │
    ├── CommandInterface        ← 通信接口工厂
    │   ├── getCmdInterface(int motor_platform_type)
    │   └── setCommandAndState(SpiCommand*, SpiData*, VectorNavData*)
    │
    ├── RobotMonitor            ← 安全监控（独立线程）
    │   ├── fsm_state_check()
    │   ├── imu_data_check()
    │   ├── motor_data_check()
    │   ├── rl_loadpolicy_check()
    │   ├── sdk_init_check()
    │   ├── app_Node_check()
    │   └── run()
    │
    └── ContactEstimator<float> ← 足端接触估计
```

### 4.2 通信接口（CommandInterface 子类）

`motor_platform_type` 参数决定使用哪个通信后端：

| motor_platform_type | 接口类 | 通信方式 | 使用场景 |
|---------------------|--------|----------|----------|
| 0 | SPI 直连 | 硬件 SPI 总线 | 实机（默认） |
| 1 | Unitree | Unitree SDK | Unitree 电机 |
| 5 | `MujocoCommandInterface` | eCAL + UDP protobuf | MuJoCo 仿真 |
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

#### MujocoCommandInterface（type=5）关键方法：

```cpp
MujocoCommandInterface::initialize()   // eCAL 订阅 RobotState + lambda 回调
MujocoCommandInterface::reciveData()   // 读取 eCAL 数据
MujocoCommandInterface::sendCmd()      // 发送命令
MujocoCommandInterface::getInstance()  // 单例模式
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

### 4.4 ONNX 推理引擎

`libonnx_model.so` 导出符号：

```cpp
inference::ModelInference::inferLoop()              // 纯策略推理（无状态估计）
inference::ModelInference::inferLoopWithEstimator()  // 策略推理 + 里程计估计
inference::ModelInference::reset()                   // 重置推理状态
```

每个运动模式由两个 ONNX 模型组成：
- **policy**: 策略网络，输入观测→输出目标关节角
- **odom**: 里程计网络（odometry），输入历史观测→估计速度/位置

推理在独立线程中运行，由 `ModelInference` 管理。

### 4.5 底层控制库

`libbiomimetics.so` 提供基础控制框架：

```cpp
// 周期任务管理
PeriodicTask::start()/stop()/loopFunction()    // 实时任务调度
PeriodicTaskManager::addTask()/stopAll()       // 任务管理器

// 控制参数
ControlParameters::generateUnitializedList()   // 参数管理

// 脊柱板（电机驱动抽象）
SpineBoard::init()/run()/resetCommand()/resetData()

// 舞蹈控制
DanceController::loadAllDances()/onEnter()/dancePlan()

// 工具
getLcmUrl()/getConfigDirectoryPath()
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
FSM_passive_Kd: 3.0           # 被动模式阻尼
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
use_wbc: 1                   # 启用 WBC

# 关节限位
JPos_limit_low: [-0.48, -1.15, -2.7]
JPos_limit_high: [0.48, 2.97, -0.65]
jointVelocityLimit: 24.0     # rad/s

# 关节偏移（站立姿态）
abad_stand_pos:   [0., 0., 0., 0.]
hip_stand_pos:    [0.8, 0.8, 0.8, 0.8]
knee_stand_pos:   [-1.5, -1.5, -1.5, -1.5]

# 关节偏移（趴下姿态）
abad_liedown_pos: [0., 0., 0., 0.]
hip_liedown_pos:  [1.4, 1.4, 1.4, 1.4]
knee_liedown_pos: [-2.4, -2.4, -2.4, -2.4]

# 腿侧符号
abad_side_sign: [-1., -1., 1., 1.]
hip_side_sign:  [1., -1., 1., -1.]
knee_side_sign: [1., -1., 1., -1.]

# 关节偏移量
abad_offset: [0.488, -0.488, -0.488, 0.488]
hip_offset:  [-2.967, 2.967, -2.967, 2.967]
knee_offset: [2.723, -2.723, 2.723, -2.723]
```

### 5.4 仿真器参数

```yaml
dynamics_dt: 0.001             # 1000Hz 物理仿真
high_level_dt: 0.002           # 500Hz 高层控制
low_level_dt: 0.0002           # 5000Hz 低层控制（PD）
floor_kp: 500000               # 地面刚度
floor_kd: 5000                 # 地面阻尼
vectornav_imu_accelerometer_noise: 0.005
vectornav_imu_gyro_noise: 0.005
vectornav_imu_quat_noise: 0.003
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
      FLfootCmd: [0.0, 0.0, 0.0]
      RRfootCmd: [0.0, 0.0, 0.0]
      RLfootCmd: [0.0, 0.0, 0.0]
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

---

## 14. robot_mc 闭源分析：核心黑盒内容

robot_mc 中**无法获取源码**的部分及其功能：

### 14.1 RL 策略模型（最核心）

- **加密的 ONNX 文件**: policy_* 和 odom_* 模型，经过 FileCrypto 加密
- **观测空间 → 动作空间的映射**: 完全编码在神经网络权重中，无法提取
- **训练参数**: 奖励函数、课程学习策略、域随机化参数等全部不可知

### 14.2 状态估计器（StateEstimator）

- IMU 数据融合算法（互补滤波 / EKF / UKF）
- 足端接触估计（ContactEstimator<float>）
- 里程计估计（由 odom ONNX 模型辅助）
- 从符号表看：`initializeStateEstimator()` 在 RobotRunner 中调用

### 14.3 控制 FSM（ControlFSM）

- 状态转移条件和逻辑
- 各状态下的控制律（PASSIVE/STANDUP/RL_MIX/FOLDLEGS）
- 模式切换的平滑过渡逻辑

### 14.4 MPC + WBC（可选模块）

- MPC 的具体实现（虽然参数可见，但目标函数、约束条件的细节在 librobot.so 中）
- WBC 的力分配策略（libWBC.so + libWBC_CTRL.so 无导出符号，完全闭源）
- QP 问题构建（libqpOASES.so 和 libGoldfarb_Optimizer.so 是开源的，但问题建模闭源）

### 14.5 预定义动作系统

- motionType (transition/pose/foot) 的插值算法
- 动作序列的时序调度
- 可中断/不可中断的状态管理

---

## 15. 实现 robot_mc 的计划

### 15.0 核心判断

robot_mc 的本质是一个**RL 运动控制器**，其核心价值在于：
1. 训练好的 RL 策略模型（policy + odom ONNX 文件）
2. 围绕 RL 策略的状态估计、FSM 管理、安全保护框架

**RL 模型是加密的 ONNX 文件，无法提取权重。** 因此实现 robot_mc 有两条路：

| 路线 | 策略 | 可行性 | 效果 |
|------|------|--------|------|
| **A: 复用加密模型** | 实现控制框架，加载现有加密 ONNX 模型 | 需要破解 FileCrypto 或使用 libfilecrypto_shared.so | 与原版完全一致 |
| **B: 重新训练 RL** | 从零训练新的 RL 策略 | 需要 MuJoCo/Isaac Gym 训练环境 | 可能更好或更差 |
| **C: 传统控制替代** | 用 MPC+WBC 替代 RL | 完全开源实现 | 性能有差距但可行 |

### 15.1 路线 A：复用加密模型（推荐）

**核心思路**: 不破解加密，而是直接调用 `libfilecrypto_shared.so` 解密 + `libonnx_model.so` 推理。

#### 阶段 1：通信框架

**目标**: 实现 UDP 通信层，替代 `UnrealCommandInterface`。

```
实现内容:
├── UdpReceiver: 监听 25001 端口，接收 protobuf RobotState
├── UdpSender: 发送 protobuf RobotCmd 到 25002 端口
├── 数据验证: flag 0x241/0x242 + 时间戳 < 10ms
└── 线程模型: 接收线程 + 控制线程（500Hz）
```

**可参考**: UE 插件中的 `UdpReceiverComponent.cpp` / `UdpSenderComponent.cpp`

#### 阶段 2：配置系统

**目标**: 加载所有 YAML 配置参数。

```
实现内容:
├── 加载 robot-defaults.yaml (controller_dt, 噪声参数)
├── 加载 {type}-user-parameters.yaml (PD增益, MPC权重, 关节限位)
├── 加载 {type}-motion_config.yaml (预定义动作)
├── 加载 rl_onnx_config.yaml (模型路径映射)
├── 加载 simulator-defaults.yaml (仿真参数)
└── 参数热更新（可选）
```

**技术选型**: yaml-cpp（与原版一致）

#### 阶段 3：RL 推理引擎

**目标**: 加载加密 ONNX 模型并执行推理。

```
方案 1（推荐）: 直接链接 libfilecrypto_shared.so + libonnx_model.so
  ├── 调用 zsibot::mc::FileCrypto 解密模型
  ├── 调用 inference::ModelInference 执行推理
  └── 无需知道加密算法

方案 2: 绕过加密
  ├── 运行时从 mc_ctrl 进程内存中提取解密后的模型
  ├── 或用 GDB/LD_PRELOAD hook FileCrypto 解密函数
  └── 风险: 可能违反许可协议
```

**推理接口（从符号表推断）**:

```cpp
// ModelInference 大致接口
class ModelInference {
    void loadPolicy(const std::string& model_path);
    void loadOdom(const std::string& model_path);
    void inferLoop();                    // 纯策略推理
    void inferLoopWithEstimator();       // 策略 + 里程计
    void reset();
};
```

**关键问题**: 需要确定 RL 策略的**观测空间**和**动作空间**。

从配置和符号表推断（XG 平台 mix_walk）：

```
观测空间 (典型四足 RL):
  ├── 躯干姿态 (roll, pitch, yaw)           — 3
  ├── 躯干角速度 (wx, wy, wz)              — 3
  ├── 关节位置 (12)                         — 12
  ├── 关节速度 (12)                         — 12
  ├── 上次动作 (12)                         — 12
  ├── 速度命令 (vx, vy, wz)                — 3
  ├── 步态相位 (4)                          — 4
  └── 总计: ~49 维（具体需验证）

动作空间:
  └── 目标关节位置 (12)                     — 12
```

> **注意**: 确切维度和含义需要从 ONNX 模型的输入/输出 tensor shape 中验证。
> 加载模型后用 ONNX Runtime 的 `GetInputCount()/GetOutputCount()/GetTensorShape()` 即可确认。

#### 阶段 4：FSM 状态机

**目标**: 实现 PASSIVE → STANDUP → RL_MIX 的状态转换。

```
实现内容:
├── PASSIVE 状态
│   └── 阻尼模式: tau = -Kd * qvel (Kd=3.0)
├── STANDUP 状态
│   └── PD 控制: tau = Kp*(q_stand - q) - Kd*qvel (Kp=80, Kd=1.0)
│   └── 从 liedown 插值到 stand (hip: 1.4→0.8, knee: -2.4→-1.5)
├── RL_MIX 状态
│   └── RL 策略推理: Kp=20, Kd=0.7
│   └── 支持运动模式切换（mix_walk, balancestand, ...）
├── FOLDLEGS 保护状态
│   └── 异常触发，折叠腿部
└── 状态转移条件
    ├── PASSIVE → STANDUP: 启动后自动
    ├── STANDUP → RL_MIX: 关节误差 < 阈值
    └── ANY → FOLDLEGS: IMU异常 / 电机异常 / 关节超限
```

#### 阶段 5：状态估计器

**目标**: 从 IMU + 关节数据估计机器人状态。

```
实现内容:
├── 简单方案（快速实现）
│   ├── 躯干姿态: 直接从四元数读取
│   ├── 角速度: 直接从 gyro 读取
│   ├── 线速度: 从 RL odom 模型估计
│   └── 接触估计: 从关节力矩阈值判断
│
├── 进阶方案
│   ├── 互补滤波 / Madgwick 滤波
│   ├── 基于运动学的里程计
│   └── 贝叶斯接触估计
```

#### 阶段 6：安全监控（RobotMonitor）

**目标**: 独立线程的安全检查。

```
实现内容:
├── imu_data_check(): IMU 数据新鲜度、合理性
├── motor_data_check(): 关节位置/速度/温度
├── fsm_state_check(): FSM 状态合法性
├── rl_loadpolicy_check(): RL 模型加载状态
├── sdk_init_check(): SDK 连接状态
├── app_Node_check(): APP 命令合法性
└── 异常处理: 触发 FOLDLEGS
```

#### 阶段 7：预定义动作系统

**目标**: 支持 sitdown, shakehand, greet 等动作。

```
实现内容:
├── 动作解析器: 读取 motion_config.yaml
├── 插值器
│   ├── transition: 线性/平滑插值到目标
│   ├── pose: 保持当前姿态
│   └── foot: 足端轨迹跟踪
├── 中断管理: interruptible vs no-interruptible
└── 动作队列: 按时间顺序执行 sequence
```

#### 阶段 8：SDK 通信（可选）

**目标**: 提供外部控制接口。

```
实现内容:
├── zsibot::rmw 中间件替代方案（可用 UDP/ROS2）
├── 话题: RobotState, SDKRobotState, AppCmd, SDKCmd
├── 端口: 43988
└── 高层/低层 SDK 控制接口
```

### 15.2 实现优先级和工作量估算

| 阶段 | 优先级 | 工作量 | 依赖 |
|------|--------|--------|------|
| 阶段 1: 通信框架 | P0 | 2-3 天 | 无 |
| 阶段 2: 配置系统 | P0 | 1-2 天 | 无 |
| 阶段 3: RL 推理引擎 | P0 | 3-5 天 | 阶段 1, 2 |
| 阶段 4: FSM 状态机 | P0 | 2-3 天 | 阶段 1, 3 |
| 阶段 5: 状态估计器 | P1 | 2-3 天 | 阶段 1 |
| 阶段 6: 安全监控 | P1 | 1-2 天 | 阶段 4 |
| 阶段 7: 预定义动作 | P2 | 2-3 天 | 阶段 4 |
| 阶段 8: SDK 通信 | P3 | 3-5 天 | 阶段 1 |

**最小可行系统（P0）**: 约 10-15 天

### 15.3 技术栈选择

```
语言: C++17（与原版一致）
构建: CMake
通信: 原生 UDP socket + protobuf
配置: yaml-cpp
推理: ONNX Runtime（链接 libonnx_model.so 或直接用 onnxruntime）
线性代数: Eigen3
线程: std::thread + 实时优先级
调试: LCM（可选）
```

### 15.4 关键风险和对策

| 风险 | 影响 | 对策 |
|------|------|------|
| ONNX 模型加密无法加载 | 无法使用 RL 策略 | 方案1: 链接 libfilecrypto_shared.so; 方案2: 重新训练 |
| 观测空间维度不匹配 | RL 推理失败 | 加载模型后检查 input tensor shape |
| 控制时序不匹配 | 行走不稳定 | 严格保持 500Hz，与原版 dt=0.002s 一致 |
| 状态估计差异 | RL 策略输入错误 | 先直接用原始 IMU 数据，逐步优化 |
| libfilecrypto_shared.so 依赖环境 | 解密失败 | 需要完整的运行时环境（可能与硬件绑定） |

### 15.5 路线 C：纯开源替代（MPC + WBC）

如果无法复用加密 RL 模型，可用传统控制方法替代：

```
实现内容:
├── 逆运动学 (IK): 从速度命令计算足端轨迹
├── 步态调度器 (GaitScheduler): 对角小跑/trot 步态
├── MPC: 用 qpOASES 求解最优地面反力
│   ├── 状态: [roll, pitch, yaw, x, y, z, roll_d, pitch_d, yaw_d, x_d, y_d, z_d]
│   ├── 约束: 力在摩擦锥内, Fz >= 0
│   └── 目标: 跟踪 MPC 参考轨迹
├── WBC: 从地面反力计算关节力矩
│   ├── 动力学: M*qdd + C + G = J^T * F + tau
│   └── QP: min ||WBC误差|| s.t. 接触约束
└── PD 控制: tau = kp*(q_des-q) - kd*qvel + tau_ff
```

**参考开源项目**:
- MIT Cheetah Software (github.com/mit-biomimetics/Cheetah-Software)
- OCS2 (github.com/leggedrobotics/ocs2)
- Simple Quadruped (github.com/qiayuanliao/terra)

### 15.6 推荐实施路径

```
Phase 1 (1周): 最小通信 + 配置加载
  → 能接收 RobotState，能发送 RobotCmd
  → 验证与 UE/MuJoCo 的通信链路

Phase 2 (1周): RL 推理 + 简单 FSM
  → 加载加密 ONNX 模型（链接 libfilecrypto_shared.so）
  → 实现 PASSIVE → STANDUP → RL_MIX
  → 验证 RL 策略能输出合理的关节目标

Phase 3 (1周): 状态估计 + 安全监控
  → 基本 IMU 处理
  → 安全保护逻辑
  → 端到端测试：UE 渲染 + robot_mc 控制 → 行走

Phase 4 (持续): 功能完善
  → 预定义动作
  → SDK 通信
  → 性能优化
```

### 15.7 验证清单

- [ ] UDP 25001 接收 RobotState protobuf
- [ ] UDP 25002 发送 RobotCmd protobuf
- [ ] 数据有效性三重验证
- [ ] YAML 配置加载（所有参数）
- [ ] ONNX 模型解密和加载
- [ ] RL 推理输出维度正确
- [ ] FSM: PASSIVE → STANDUP 过渡
- [ ] FSM: STANDUP → RL_MIX 过渡
- [ ] 端到端行走测试
- [ ] 安全保护（FOLDLEGS 触发）
- [ ] 500Hz 实时性验证
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
