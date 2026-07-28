# Matrix 仿真启动与按键控制完整流程

## 1. 启动命令完整流程

### 1.1 用户操作（sim_launcher GUI）

```
./bin/sim_launcher
```

sim_launcher 是一个 bash 包装脚本：
```bash
export RMW_IMPLEMENTATION="rmw_zenoh_cpp"   # ROS2 通信中间件
export ROS_DOMAIN_ID="89"                    # ROS2 域隔离
export SDK_CLIENT_IP="127.0.0.1"            # 本机通信
exec sim_launcher.bin "$@"                   # 启动 Qt GUI
```

用户在 GUI 中选择：
| 选项 | 示例 | 对应参数 |
|------|------|----------|
| Robot | zsl-1 (xgb) | ROBOT_ARG=1 |
| Map | 3 - YARD | SCENE_ID=3 |
| Control Mode | Keyboard Control | 启用 KeyListener |
| Headless | 关 | OFFSCREEN=0 |
| Pixel Streaming | 关 | PIXELSTREAM=0 |
| MuJoCo Physics | 关 | MUJOCORUNNING=0 |
| RoamerX Open | 关 | 无 roamerx_link.state |

点击 **"Launch Simulation"** 后，sim_launcher.bin 调用：
```bash
scripts/run_sim.sh <robotId> <mapId> <headless> <pixelStream> <mujocoRunning>
# 实际示例:
scripts/run_sim.sh 1 3 0 0 0
```

### 1.2 run_sim.sh 执行步骤

```
┌─────────────────────────────────────────────────────────────────┐
│ run_sim.sh 1 3 0 0 0                                           │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│ ① 环境检查                                                      │
│    scripts/check_env.sh runtime --robot 1 --scene 3 ...         │
│                                                                 │
│ ② 杀旧进程                                                      │
│    pkill -TERM -f "robot_mujoco|zsibot_mujoco_ue|mc_ctrl|..."   │
│                                                                 │
│ ③ 场景配置 (SCENE_ID=3)                                         │
│    SCENE="scene_terrain_yard.xml"                               │
│    MAPNAME="/Game/Maps/YardWorld"                               │
│    → 修改 config.yaml 中 robot_scene                            │
│                                                                 │
│ ④ 机器人配置 (ROBOT_ARG=1 → xgb)                                │
│    ROBOTTYPE="xgb"                                              │
│    ENABLE_MC=true                                               │
│    → sed run_mc.sh: export ROBOT_TYPE=XG                        │
│    → sed xg-user-parameters.yaml: motor_platform_type: 8        │
│                                                                 │
│ ⑤ JSON 同步                                                     │
│    → jq 更新 config/config.json (robot_type, ports等)           │
│    → cp config.json → UE/Content/model/config/                  │
│    → cp scene.json  → UE/Content/model/SceneLoder/              │
│                                                                 │
│ ⑥ UE 场景入口同步                                               │
│    → cp scene_terrain_yard.xml → scene_terrain.xml              │
│                                                                 │
│ ⑦ 机器人初始位姿                                                │
│    → sed xgb.xml: base_link pos="0 0 0.65"                     │
│                                                                 │
│ ⑧ 启动进程（见下节）                                            │
│                                                                 │
│ ⑨ wait（阻塞直到所有子进程退出）                                 │
└─────────────────────────────────────────────────────────────────┘
```

### 1.3 进程启动顺序

```bash
# ======== 进程① MuJoCo 独立物理（当前不启动）========
# mujoco_running=false → 跳过
# 若启用:
cd src/robot_mujoco/simulate/build
LD_LIBRARY_PATH="..." ./robot_mujoco > robot_mujoco.log 2>&1 &

# ======== 进程② UE 渲染 + 内嵌物理（始终启动）========
cd src/UeSim/Linux
LD_LIBRARY_PATH="..." ./zsibot_mujoco_ue.sh \
    -game /Game/Maps/YardWorld \
    -ExecCmds="t.MaxFPS 30" \
    > zsibot_mujoco_ue.log 2>&1 &

# ======== 等待 UE 就绪 ========
sleep 7

# ======== 进程③ 运动控制器 mc_ctrl ========
cd src/robot_mc
export SDK_CLIENT_IP="127.0.0.1"
# 无 RoamerX 时:
LD_LIBRARY_PATH="..." ./run_mc.sh r mc_enable=true > run_mc.log 2>&1 &
# run_mc.sh 内部实际执行:
#   export ROBOT_TYPE=XG
#   cd build/export/mc/bin/
#   taskset -c 7 ./mc_ctrl r
```

### 1.4 最终运行的进程

| 进程 | 可执行文件 | 作用 |
|------|-----------|------|
| sim_launcher.bin | Qt GUI | 键盘拦截 + 虚拟手柄 + 进程管理 |
| zsibot_mujoco_ue | UE Shipping | 渲染 + 内嵌 MuJoCo 物理 |
| mc_ctrl | RL 控制器 | 策略推理 → 关节指令 |

---

## 2. 按键控制完整数据流

### 2.1 总体架构

```
┌─────────────────────────────────────────────────────────────────────┐
│                    sim_launcher.bin (Qt GUI)                         │
│                                                                     │
│  ┌─────────────────────────────────────────────────────────────┐   │
│  │ KeyListener (XGrabKey 全局键盘钩子)                          │   │
│  │                                                             │   │
│  │  X11 事件循环 → 拦截按键 → 转换为手柄事件 → 写入 uinput     │   │
│  │                                                             │   │
│  │  注意: UE 窗口完全收不到这些按键！                           │   │
│  └─────────────────────────────────────────────────────────────┘   │
│                              │                                      │
│                              ▼                                      │
│  ┌─────────────────────────────────────────────────────────────┐   │
│  │ UInputGamepad (虚拟罗技 F710)                                │   │
│  │                                                             │   │
│  │  设备名: "Logitech Gamepad F710 (Virtual)"                  │   │
│  │  通过 /dev/uinput 创建                                      │   │
│  │  系统识别为 /dev/input/js0 (或 js1)                         │   │
│  └─────────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────────┘
                               │
                    /dev/input/js0 (joystick 设备)
                               │
                               ▼
┌─────────────────────────────────────────────────────────────────────┐
│                    mc_ctrl (500Hz RL 推理)                           │
│                                                                     │
│  GamepadReader:                                                     │
│    open("/dev/input/js0") → 读取 js_event                          │
│    → 解析摇杆值 → GamepadCommand                                   │
│    → DesiredStateCommand.handleGamePadCommand()                     │
│    → 提取 vx, vy, vyaw + 模式切换                                  │
│                                                                     │
│  RL 策略推理:                                                       │
│    observation = [关节角, 关节速度, IMU, 上一步动作, 速度指令, 相位] │
│    action = policy_network(observation)                             │
│    → 12 个关节目标角度                                             │
│                                                                     │
│  发送: UDP → 127.0.0.1:25002 (robotCmd, 248 bytes)                 │
│  接收: UDP ← 127.0.0.1:25001 (robotState, 224 bytes)               │
└─────────────────────────────────────────────────────────────────────┘
                               │
                        UDP:25002 (robotCmd)
                               │
                               ▼
┌─────────────────────────────────────────────────────────────────────┐
│                    zsibot_mujoco_ue (UE 进程)                        │
│                                                                     │
│  接收 robotCmd → 12 个关节目标角 + Kp/Kd                           │
│  PD 控制 (1000Hz):                                                  │
│    torque = Kp × (target - current) - Kd × velocity                │
│    ctrl[i] = clamp(torque, -28, +28)                               │
│  MuJoCo 物理: mj_step()                                            │
│  渲染: 30 FPS                                                      │
│                                                                     │
│  发送 robotState → UDP:25001 (关节角/速度/IMU)                     │
└─────────────────────────────────────────────────────────────────────┘
```

### 2.2 按键映射表（从 sim_launcher.bin 逆向提取）

| 按键 | 手柄事件 | 功能 |
|------|----------|------|
| **W** | ABS_Y = -32768 | 前进 (vx = +max) |
| **S** | ABS_Y = +32767 | 后退 (vx = -max) |
| **A** | ABS_X = -32768 | 左移 (vy = +max) |
| **D** | ABS_X = +32767 | 右移 (vy = -max) |
| **Q** | ABS_RX = -32768, ABS_Z = -32768 | 左转 (vyaw = +max) |
| **E** | ABS_RX = +32767, ABS_Z = +32767 | 右转 (vyaw = -max) |
| **R** | ABS_RY = -32768, ABS_RZ = -32768 | 相机/高度+ |
| **F** | ABS_RY = +32767, ABS_RZ = +32767 | 相机/高度- |
| **U** | **LB + Y** (组合键) | 站立 |
| **Space** | **RB + LB** (组合键) | 蹲下/趴下 |
| **Return** | START | 确认/切换 |

### 2.3 关键发现：U 和 Space 是组合键！

```
U 站立   → 同时按下 LB (0x136) + Y (0x134)
Space 蹲下 → 同时按下 RB (0x137) + LB (0x136)
```

mc_ctrl 的 `GamepadCommand` 类检测这些组合键来切换 FSM 状态：
- LB+Y → 进入站立模式 (FSM_STAND)
- RB+LB → 进入被动/趴下模式 (FSM_PASSIVE)

### 2.4 GamepadForwarder 的作用

sim_launcher 有两种输入模式：

| 模式 | 数据流 |
|------|--------|
| **Keyboard Control** | KeyListener → XGrabKey → 直接写入 UInputGamepad |
| **Gamepad Control** | 物理手柄 → GamepadForwarder 读取 → 转发到 UInputGamepad |

两种模式最终都写入同一个虚拟 F710 设备，mc_ctrl 无感知差异。

### 2.5 mc_ctrl 如何读取手柄

```
配置文件: robot-defaults.yaml
  use_gamepad: 1          ← 默认开启，无需命令行参数

GamepadReader:
  - 打开 /dev/input/js0 (或扫描第一个可用 joystick)
  - 以 PeriodicTask 周期读取 struct js_event
  - 解析为 GamepadCommand (摇杆归一化 + 死区 + 按键状态)
  - 调用 DesiredStateCommand.handleGamePadCommand()
  - 转换为: vx, vy, vyaw + robot_state_flag
```

### 2.6 mc_ctrl 启动参数

```bash
# sim_launcher 实际执行的命令（通过 run_mc.sh）:
taskset -c 7 ./mc_ctrl r

# 无需额外参数！
# use_gamepad 由 robot-defaults.yaml 配置
# ROBOT_TYPE 由环境变量设置 (export ROBOT_TYPE=XG)
```

---

## 3. 通信协议详情

### 3.1 UDP 端口分配

| 端口 | 方向 | 内容 | 大小 |
|------|------|------|------|
| 25001 | UE → mc_ctrl | robotState (关节状态 + IMU) | 224 bytes |
| 25002 | mc_ctrl → UE | robotCmd (关节目标 + PD增益) | 248 bytes |
| 43988 | mc_ctrl → RoamerX | 状态转发 (仅 RoamerX 模式) | - |
| 43997 | RoamerX → mc_ctrl | 高层指令 (仅 RoamerX 模式) | - |

### 3.2 robotState 结构体 (UE → mc_ctrl, 224 bytes)

```c
#pragma pack(push, 4)
struct robotState {
    uint16_t head;              // 0x5AA5
    uint16_t len;               // sizeof(robotState)
    float q_abad[4];            // ABAD 关节角 (FR,FL,RR,RL)
    float q_hip[4];             // HIP 关节角
    float q_knee[4];            // KNEE 关节角
    float qd_abad[4];           // ABAD 关节速度
    float qd_hip[4];            // HIP 关节速度
    float qd_knee[4];           // KNEE 关节速度
    float tau_abad_fb[4];       // ABAD 力矩反馈
    float tau_hip_fb[4];        // HIP 力矩反馈
    float tau_knee_fb[4];       // KNEE 力矩反馈
    float quat[4];              // 身体四元数 [w,x,y,z]
    float gyro[3];              // 角速度 [rad/s]
    float acc[3];               // 线加速度 [m/s²]
    float rpy[3];               // 欧拉角 [rad]
    float time_stamp;           // 时间戳
    float desire_vel_x;         // 期望前进速度 (m/s)
    float desire_vel_y;         // 期望横移速度 (m/s)
    float desire_yaw_rate;      // 期望偏航角速度 (rad/s)
    uint16_t in_rl_state;       // RL 模式标志
    uint32_t checksum;          // 校验和
};
#pragma pack(pop)
```

### 3.3 robotCmd 结构体 (mc_ctrl → UE, 248 bytes)

```c
#pragma pack(push, 4)
struct robotCmd {
    uint16_t head;              // 0x5AA5
    uint16_t len;               // sizeof(robotCmd)
    float q_des_abad[4];        // ABAD 目标角
    float q_des_hip[4];         // HIP 目标角
    float q_des_knee[4];        // KNEE 目标角
    float qd_des_abad[4];       // ABAD 目标速度
    float qd_des_hip[4];        // HIP 目标速度
    float qd_des_knee[4];       // KNEE 目标速度
    float kp_abad[4];           // ABAD Kp
    float kp_hip[4];            // HIP Kp
    float kp_knee[4];           // KNEE Kp
    float kd_abad[4];           // ABAD Kd
    float kd_hip[4];            // HIP Kd
    float kd_knee[4];           // KNEE Kd
    float tau_abad_ff[4];       // ABAD 前馈力矩
    float tau_hip_ff[4];        // HIP 前馈力矩
    float tau_knee_ff[4];       // KNEE 前馈力矩
    uint32_t checksum;          // 校验和
};
#pragma pack(pop)
```

---

## 4. 在 UE 里按键控制机械狗的完整流程

### 4.1 关键事实：UE 收不到按键！

```
⚠️ sim_launcher 使用 XGrabKey 在 X11 层面全局拦截键盘。
   当 sim_launcher 运行时，UE 窗口完全收不到 W/A/S/D/U/Space 等按键。
   按键不会进入 UE 的 PlayerInput 系统。
```

### 4.2 实际控制链路

```
用户按下 W 键
     │
     ▼ (X11 XGrabKey 拦截，UE 无感知)
┌─────────────────────────────────────────┐
│ sim_launcher.bin → KeyListener           │
│                                         │
│ XKeyEvent → 识别为 W                    │
│ → 写入 uinput: ABS_Y = -32768          │
│ → 虚拟 F710 手柄产生摇杆事件           │
└────────────────────┬────────────────────┘
                     │
                     ▼ /dev/input/js0
┌─────────────────────────────────────────┐
│ mc_ctrl → GamepadReader (周期读取)       │
│                                         │
│ js_event: type=ABS, number=1, val=-32768│
│ → 归一化: leftStickY = -32768/32768     │
│ → 死区过滤                              │
│ → vx = +1.0 m/s (前进)                 │
│                                         │
│ DesiredStateCommand:                    │
│   des_vel_x = 1.0                       │
│   des_vel_y = 0.0                       │
│   des_yaw_rate = 0.0                    │
└────────────────────┬────────────────────┘
                     │
                     ▼ RL 策略推理 (500Hz)
┌─────────────────────────────────────────┐
│ mc_ctrl → RL Policy Network             │
│                                         │
│ observation (~50维):                    │
│   ├── 12 关节角度 (from robotState)     │
│   ├── 12 关节速度                       │
│   ├── 身体四元数 (IMU)                  │
│   ├── 身体角速度                        │
│   ├── 上一步动作 (12维)                 │
│   ├── 速度指令 (vx=1.0, vy=0, vyaw=0) │
│   └── 步态相位                          │
│                                         │
│ action = policy(observation)            │
│ → 12 个关节目标角度                    │
│ → Kp=20, Kd=0.7 (RL 行走增益)         │
└────────────────────┬────────────────────┘
                     │
                     ▼ UDP:25002 (248 bytes)
┌─────────────────────────────────────────┐
│ zsibot_mujoco_ue (UE)                   │
│                                         │
│ 接收 robotCmd                           │
│ PD 控制 (每个物理步 1ms):              │
│   for i in 0..11:                       │
│     torque = Kp*(q_des[i] - q[i])      │
│            - Kd * qd[i]                 │
│            + tau_ff[i]                  │
│     ctrl[i] = clamp(torque, -28, +28)  │
│                                         │
│ mj_step(model, data)  → 物理仿真       │
│ 更新渲染网格 → 30 FPS 输出             │
│                                         │
│ 发送 robotState → UDP:25001            │
└─────────────────────────────────────────┘
```

### 4.3 站立/趴下切换流程

```
用户按下 U 键:
  KeyListener: U → 同时按下 LB + Y (组合键)
  → 虚拟 F710 产生 button press 事件
  → mc_ctrl GamepadReader 检测到 LB+Y
  → FSM 切换到 STANDING 模式
  → RL 策略输出站立姿态关节角
  → 机械狗从趴地状态站起

用户按下 Space 键:
  KeyListener: Space → 同时按下 RB + LB (组合键)
  → mc_ctrl 检测到 RB+LB
  → FSM 切换到 PASSIVE 模式
  → 输出零力矩或收腿姿态
  → 机械狗蹲下/趴下
```

### 4.4 速度指令的两条可能路径

mc_ctrl 获取速度指令有两条路径（可能同时存在）：

| 路径 | 来源 | 接口 |
|------|------|------|
| **路径A: GamepadReader** | 虚拟手柄 /dev/input/js0 | js_event → GamepadCommand → vx/vy/vyaw |
| **路径B: robotState 内嵌** | UE 发送的 robotState | desire_vel_x/y/yaw_rate 字段 |

在 Matrix 仿真中，**路径A 是主要的**（sim_launcher → 虚拟手柄 → mc_ctrl 直接读取）。
路径B 可能用于 SDK/RoamerX 模式或作为备用。

---

## 5. 关键配置参数

### 5.1 mc_ctrl 配置 (robot-defaults.yaml)

```yaml
use_gamepad: 1                    # 启用手柄输入
controller_type: 0                # 控制器类型
enable_stunts: 1                  # 允许特技动作
enable_position_tracking: 0       # 位置跟踪关闭
```

### 5.2 PD 增益 (xg-user-parameters.yaml)

```yaml
# RL 行走模式
FSM_RL_ABAD_Kp: 20.0
FSM_RL_HIP_Kp: 20.0
FSM_RL_KNEE_Kp: 20.0
FSM_RL_Kd: 0.7

# 站立模式
FSM_jointPD_Kp: 80
FSM_jointPD_Kd: 1.0

# 平衡站立
FSM_RLBalanceStand_Kp: 20.0
FSM_RLBalanceStand_Kd: 0.7
```

### 5.3 控制频率 (simulator-defaults.yaml)

```yaml
dynamics_dt: 0.001       # 物理仿真 1000Hz
high_level_dt: 0.002     # RL 策略推理 500Hz
low_level_dt: 0.0002     # PD 力矩环 5000Hz
```

---

## 6. 文件路径索引

| 文件 | 路径 |
|------|------|
| 启动包装脚本 | `/home/qiyuan/Softwares/Matrix/bin/sim_launcher` |
| 启动器二进制 | `/home/qiyuan/Softwares/Matrix/bin/sim_launcher.bin` |
| 编排脚本 | `/home/qiyuan/Softwares/Matrix/scripts/run_sim.sh` |
| UE 启动脚本 | `/home/qiyuan/Softwares/Matrix/src/UeSim/Linux/zsibot_mujoco_ue.sh` |
| MC 启动脚本 | `/home/qiyuan/Softwares/Matrix/src/robot_mc/run_mc.sh` |
| MC 二进制 | `/home/qiyuan/Softwares/Matrix/src/robot_mc/build/export/mc/bin/mc_ctrl` |
| 全局配置 | `/home/qiyuan/Softwares/Matrix/config/config.json` |
| 手柄配置 | `.../config/robot-defaults.yaml` (use_gamepad: 1) |
| PD 参数 | `.../config/xg-user-parameters.yaml` |
| RL 模型配置 | `.../config/rl_onnx_config.yaml` |
| 仿真频率 | `.../config/simulator-defaults.yaml` |
| 机器人模型 | `/home/qiyuan/Softwares/Matrix/src/robot_mujoco/zsibot_robots/xgb/xgb.xml` |

---

## 7. 对 CarlaUnreal 的启示

### 7.1 为什么我们的 UE 收不到按键？

如果 sim_launcher 正在运行，XGrabKey 会拦截所有按键。我们的 CarlaUE5 窗口无法收到 W/U/Space 等按键。

### 7.2 可行的接入方案

| 方案 | 原理 | 难度 |
|------|------|------|
| **A: 虚拟手柄** | 创建 "Logitech Gamepad F710 (Virtual)" 设备，mc_ctrl 直接读取 | 中 |
| **B: robotState 内嵌速度** | 在 UE 发送的 robotState 中填入 desire_vel_* 字段 | 低 |
| **C: 独立键盘脚本** | Python 监听键盘 → UDP → UE → 填入 robotState | 低 |

### 7.3 方案A 的关键要求

如果要让 mc_ctrl 通过 GamepadReader 读取虚拟手柄：
1. 设备名必须是 **"Logitech Gamepad F710 (Virtual)"**
2. VID/PID 应为 Logitech: **0x046d / 0xc219**
3. 站立/趴下是**组合键**: U=LB+Y, Space=RB+LB
4. mc_ctrl 启动时 `use_gamepad: 1` 已默认开启
5. 设备需要在 mc_ctrl 启动**之前**创建（或 mc_ctrl 会热扫描）



让我帮你诊断。需要确认 mc_ctrl 是否真的读到了虚拟手柄，以及是否在发 UDP 给 UE：


**问题找到了：mc_ctrl 没有打开 `/dev/input/js0`！**

虚拟手柄存在且设备名正确，但 mc_ctrl 没有读取它。可能是 mc_ctrl 先启动、手柄后创建，mc_ctrl 不支持热插拔。


问题确认了：

- **mc_ctrl 没有打开 `/dev/input/js0`** — 因为 mc_ctrl 启动时虚拟手柄还不存在
- UDP 链路正常：mc_ctrl 监听 25001，UE 监听 25002 ✓

**原因**：sim_launcher 的顺序是**先创建虚拟手柄，再启动 mc_ctrl**。你的顺序反了。

**解决方法**：先启动键盘脚本（创建手柄），再启动 mc_ctrl：

```bash
# 终端1: 先启动键盘脚本（创建虚拟F710）
sudo python3 /home/qiyuan/UnrealEngine/CarlaUE5/Unreal/CarlaUnreal/Plugins/MuJoCoUE/Scripts/keyboard_control.py

# 终端2: 等手柄创建后，再启动 mc_ctrl
taskset -c 7 ./mc_ctrl r
```

先 Ctrl+C 停掉当前的 keyboard_control.py，然后按上面的顺序重来：先跑脚本 → 看到 "虚拟手柄已创建" → 再启动 mc_ctrl。



好的，让我深入分析 protobuf 消息结构。先从二进制中提取描述符，再多抓几包对比：


找到了所有字段名！让我继续提取更多方法来确定完整结构：


现在让我精确确定字段编号。用 protoc --decode_raw 来做标准解析：


有 protoc。让我抓包并用 `protoc --decode_raw` 标准解析：


让我用 Python 精确解析每个字段的 float 值：


非常清晰了！现在让我检查 43997 端口（highLevelCmd）的数据：


43997 没有数据（等待外部 App 发送 highLevelCmd）。现在让我根据分析结果重建 .proto 定义，并验证编码是否匹配：


**重编码验证通过！** 303字节完全匹配。现在让我写 .proto 定义文件，并验证 RobotCmd 也能正确编解码：


两个消息都验证通过。现在让我写 .proto 定义文件：


现在让我验证一下能否用 Python protobuf 从零构造一个有效的 RobotState 消息：


**分析完成！** 构造的消息也是 303 字节，protoc 验证通过。

---

## 完整 Protobuf 消息结构总结

### 通信架构
```
Matrix UE (zsibot_mujoco_ue)  ←→  mc_ctrl
         port 25001 (RobotState, 303B, Protobuf)  →
         port 25002 (RobotCmd, 365B, Protobuf)    ←
```

### RobotState (UE→mc_ctrl, 303字节)

| Field | 名称         | 大小   | 当前值（趴地）               |
| ----- | ------------ | ------ | ---------------------------- |
| 1     | q_abad       | 4f     | [-0.14, +0.14, -0.03, +0.03] |
| 2     | q_hip        | 4f     | [+0.85, +0.85, +0.82, +0.82] |
| 3     | q_knee       | 4f     | [-1.50, -1.49, -1.50, -1.50] |
| 4-6   | qd_*         | 4f×3   | 全0（静止）                  |
| 7     | omega_body   | 4f     | 全0                          |
| 8-12  | tau/position | 4f×5   | 部分有值                     |
| 13    | quat         | 4f     | [1, 0, 0, 0]                 |
| 14    | gyro         | 3f     | ≈0                           |
| 15    | acc          | 3f     | [0, 0, 9.8]                  |
| 17    | time_stamp   | varint | 纳秒时间戳                   |
| 19-20 | rpy/omega    | 3f×2   | 小值                         |

### RobotCmd (mc_ctrl→UE, 365字节)

| Field | 名称          | 大小 | 当前值（趴地）               |
| ----- | ------------- | ---- | ---------------------------- |
| 1     | q_des_abad    | 4f   | [0.10, -0.10, 0.09, -0.09]   |
| 2     | q_des_hip     | 4f   | [0.80, 0.80, 0.80, 0.80]     |
| 3     | q_des_knee    | 4f   | [-1.26, -1.26, -1.32, -1.31] |
| 4-8   | qd_des/tau_ff | 4f×5 | 全0                          |
| 9     | **kp_abad**   | 4f   | **[20, 20, 20, 20]**         |
| 10    | **kp_hip**    | 4f   | **[20, 20, 20, 20]**         |
| 11    | **kp_knee**   | 4f   | **[20, 20, 20, 20]**         |
| 12    | kd_abad       | 4f   | [0, 0, 0, 0]                 |
| 13    | kd_hip        | 4f   | [0.7, 0.7, 0.7, 0.7]         |
| 14    | kd_knee       | 4f   | [0.7, 0.7, 0.7, 0.7]         |
| 15-20 | foot相关      | 4f×6 | 部分0                        |

.proto 文件已保存到 [robot_sdk.proto](/home/qiyuan/Softwares/Matrix/docs/MatrixDocuments/robot_sdk.proto)。

**关键结论**：之前我们的 UE 发 224 字节 0x5AA5 格式，mc_ctrl 完全无法解析（它期望 303 字节 Protobuf）。这就是为什么 mc_ctrl 日志一直打印 "Reciving data size: 303" 但不响应——它收到的是 Matrix 自己 UE 的正确数据，而我们发的错误格式数据可能被丢弃了。





测试流程

一、UE

启动项目

```
/home/qiyuan/UnrealEngine/UnrealEngine5_carla/Engine/Binaries/Linux/UnrealEditor /home/qiyuan/UnrealEngine/CarlaUE5/Unreal/CarlaUnreal/CarlaUnreal.uproject
```

二、启动键盘脚本

```
sudo python3 /home/qiyuan/UnrealEngine/CarlaUE5/Unreal/CarlaUnreal/Plugins/MuJoCoUE/Scripts/keyboard_control.py
```

三、mc_ctrl

```
taskset -c 7 ./mc_ctrl r
```





