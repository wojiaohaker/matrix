# Matrix MuJoCo 物理模式启动流程分析

## 概述

当 Matrix Simulation Launcher 选择 **MuJoCo Physics** 模式时，系统启动独立的 MuJoCo 物理引擎进程（`robot_mujoco`），UE 仅负责渲染。控制回路通过 eCAL 中间件完成，键盘输入通过虚拟手柄传递给 mc_ctrl。

---

## 一、进程架构

```
┌───────────────────────────────────────────────────────────────────────┐
│                        sim_launcher.bin (Qt/QML)                       │
│  ┌─────────────────┐  ┌─────────────────┐  ┌──────────────────────┐  │
│  │  KeyListener    │  │  uinput 虚拟     │  │  GamepadForwarder    │  │
│  │  (XGrabKey)     │──│  F710 手柄       │──│  (/dev/input/js0)    │  │
│  └─────────────────┘  └─────────────────┘  └──────────────────────┘  │
│         点击 "Launch Simulation" → 调用 run_sim.sh                     │
└───────────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
┌───────────────────────────────────────────────────────────────────────┐
│                          run_sim.sh 启动脚本                            │
│                                                                       │
│  参数: run_sim.sh <robot_id> <scene_id> <offscreen> <pixelstream>     │
│                   <mujoco_running>                                     │
│                                                                       │
│  当 mujoco_running=1 时:                                              │
│    • motor_platform_type → 5 (MujocoCommandInterface)                 │
│    • config.json → "mujoco_running": true                             │
│    • 启动 robot_mujoco + UE + mc_ctrl                                 │
└───────────────────────────────────────────────────────────────────────┘
                                    │
                    ┌───────────────┼───────────────┐
                    ▼               ▼               ▼
          ┌──────────────┐  ┌──────────────┐  ┌──────────────┐
          │ robot_mujoco │  │zsibot_mujoco │  │   mc_ctrl    │
          │ (物理引擎)    │  │_ue (渲染)    │  │ (运动控制)    │
          └──────┬───────┘  └──────▲───────┘  └──────┬───────┘
                 │                 │                  │
                 │   eCAL          │  UDP state       │
                 │◄──────────────────────────────────►│
                 │                                    │
                 └────── UDP RobotState ─────────────►│
```

---

## 二、启动顺序（run_sim.sh 源码分析）

### 2.1 配置阶段

```bash
# 文件: scripts/run_sim.sh

# 1. 根据 MUJOCORUNNING 参数设置 motor_platform_type
if [[ "$MUJOCORUNNING" == "1" ]]; then
    ENABLE_MUJOCO=true
    # 设置为 5 → MujocoCommandInterface (eCAL 通信)
    sed -i 's/motor_platform_type: .*/motor_platform_type: 5/' \
        src/robot_mc/build/export/config/xg-user-parameters.yaml
else
    ENABLE_MUJOCO=false
    # 设置为 8 → UnrealCommandInterface (UDP 通信)
    sed -i 's/motor_platform_type: .*/motor_platform_type: 8/' \
        src/robot_mc/build/export/config/xg-user-parameters.yaml
fi

# 2. 同步 config.json (UE 读取此文件决定运行模式)
jq --argjson mujoco_running true \
   '.robot.mujoco_running = $mujoco_running
    | .robot.state_port = 25001
    | .robot.cmd_port = 25002' \
   config/config.json > tmp && mv tmp config.json

# 3. 同步到 UE Content 目录
cp config/config.json \
   src/UeSim/Linux/zsibot_mujoco_ue/Content/model/config/config.json

# 4. 设置机器人初始位置 (XML 中 base_link pos)
sed -i "s/<body name=\"base_link\" pos=\"[^\"]*\"/
    <body name=\"base_link\" pos=\"${ROBOT_X} ${ROBOT_Y} 0.65\"/" "$XML_FILE"
```

### 2.2 进程启动顺序

```bash
# ① 启动 robot_mujoco (MuJoCo 物理引擎 + GLFW 窗口)
cd src/robot_mujoco/simulate/build
LD_LIBRARY_PATH="$(mujoco_ld_library_path)" ./robot_mujoco > robot_mujoco.log 2>&1 &

# ② 启动 UE (zsibot_mujoco_ue 渲染引擎)
cd src/UeSim/Linux
LD_LIBRARY_PATH="$(ue_ld_library_path)" ./zsibot_mujoco_ue.sh -game "$MAPNAME" \
    -ExecCmds="t.MaxFPS 30" > zsibot_mujoco_ue.log 2>&1 &

# ③ 等待 7 秒 (等 UE 完成加载)
sleep 7

# ④ 启动 mc_ctrl (运动控制器)
cd src/robot_mc
export SDK_CLIENT_IP="127.0.0.1"
LD_LIBRARY_PATH="$(mc_ld_library_path)" ./run_mc.sh r > run_mc.log 2>&1 &
```

**关键：mc_ctrl 最后启动，等待 UE 和 robot_mujoco 就绪后再初始化。**

---

## 三、各进程职责

### 3.1 robot_mujoco（物理引擎）

| 属性 | 值 |
|------|-----|
| 二进制路径 | `src/robot_mujoco/simulate/build/robot_mujoco` |
| 配置文件 | `src/robot_mujoco/simulate/config.yaml` |
| 物理频率 | 500Hz (timestep=0.002s) |
| GUI | GLFW 窗口 (MuJoCo 内置渲染) |
| 源码状态 | **闭源二进制** |

**内部结构（从日志和 strings 分析）：**
```
robot_mujoco
├── MuJoCo 主循环 (mj_step @ 500Hz)
├── ZsibotSdkBridge (通信桥接线程)
│   ├── eCAL Subscriber ← 接收 mc_ctrl 关节目标
│   ├── eCAL Publisher  → 发布状态给 mc_ctrl
│   └── sendUdpData()   → UDP 发送 RobotState 给 UE
├── GLFW 渲染线程 (被动渲染，不影响物理)
└── ROS2 传感器发布 (IMU, Camera, LiDAR via Zenoh)
```

**config.yaml：**
```yaml
robot: "xgb"
robot_scene: "scene_terrain_yard.xml"
print_scene_information: 1
enable_elastic_band: 0
```

### 3.2 zsibot_mujoco_ue（UE 渲染）

| 属性 | 值 |
|------|-----|
| 二进制路径 | `src/UeSim/Linux/zsibot_mujoco_ue.sh` |
| 配置来源 | `Content/model/config/config.json` |
| 渲染模式 | 外部物理 (mujoco_running=true) |
| 帧率限制 | t.MaxFPS 30 |

**UE 启动行为（从日志分析）：**
1. 加载 `config.json` → 检测 `mujoco_running: true`
2. 加载 MuJoCo XML 模型（仅用于生成 mesh）
3. 创建所有 body 的 SceneComponent 层级
4. 创建所有 geom 的 StaticMeshComponent
5. 初始化 UDP 接收器 → 监听 robot_mujoco 的 RobotState
6. **不运行 mj_step** — 纯 FK 渲染模式

### 3.3 mc_ctrl（运动控制）

| 属性 | 值 |
|------|-----|
| 二进制路径 | `src/robot_mc/build/export/mc/bin/mc_ctrl` |
| 启动脚本 | `src/robot_mc/run_mc.sh` |
| 通信接口 | MujocoCommandInterface (type=5, eCAL) |
| CPU 绑定 | taskset -c 7 |
| FSM 状态 | 12 个（见下文完整列表） |
| RL 模型 | 17 个 ONNX 模型（后空翻、翻身、侧翻、匍匐、太空步、舞蹈等） |

**mc_ctrl 初始化（从 run_mc.log）：**
```
[MujocoCommandInterface] initialize...          ← type=5 (eCAL)
Load rl odom model → ../../onnx_model_crypto/xg/odom_2ms_notau
```

**完整 FSM 状态初始化顺序（12 个状态）：**
```
Initialized FSM state: PASSIVE           ← 被动/趴下
Initialized FSM state: JOINT_PD          ← 关节 PD 控制
Initialized FSM state: STANDUP           ← 站立状态机
Initialized FSM state: JOINT_FREE        ← 自由关节（零力矩）
Initialized FSM state: SAFE_PROTECT      ← 安全保护
Initialized FSM state: RL_MLP            ← RL 纯策略（无估算器）
Initialized FSM state: RL_MIX            ← RL 混合（策略+估算器）
Initialized FSM state: RL_BALANCE_STAND  ← RL 平衡站立
Initialized FSM state: RL_WalkPos        ← RL 位置行走
Initialized FSM state: RL_IK             ← RL 逆运动学
Initialized FSM state: JOINT_LOCK        ← 关节锁定
Initialized FSM state: POS_CONTROL       ← 位置控制
```

**加载的 RL ONNX 模型清单：**
| 模型 | 路径 | 用途 |
|------|------|------|
| odom | odom_2ms_notau | 里程计估算器 (input=29, output=3) |
| mlp | policy_mlp | 纯策略网络 (input=45, output=12) |
| backflip | policy_mix_backflip | 后空翻 |
| flipover | policy_mix_flipover | 翻身 |
| walk | policy_mix_walk | 行走 |
| sideflip | policy_mix_sideflip | 侧翻 |
| upright | policy_upright | 扶正 |
| tracking | policy_tracking | 跟踪 |
| gait | policy_slow_walk_gait | 慢速步态 |
| gait_walk | policy_gait_walk | 步态行走 |
| crawl | policy_xg_crawl_260127 | 匍匐 |
| moonwalk | policy_moonwalk | 太空步 |
| balanceStand | policy_balancestand_withyaw_0423 | 平衡站立 |
| walkpos | policy_walkpos | 位置行走 (input=51, output=12) |
| IK | policy_ik | 逆运动学 (input=93, output=12) |
| jump | policy_jump_fix2_step3 | 跳跃 |
| measured | policy_measured_0918 | 测量策略 (input=169) |

**Dance 舞蹈功能（run_mc.log 新发现）：**
```
Skipping 'dance_2': not a valid sequence.
Skipping 'dance_3': not a valid sequence.
Loaded dance sequences:
Dance: ZGWS_dance_1
  - stand, duration: 0s, type: consecutive, control_mode: 1
  - twist, duration: 1s, type: continuous, control_mode: 21, angular: (0, 0, 0.5)
  - twist, duration: 1s, type: continuous, control_mode: 21, angular: (0, 0, -0.5)
  ...
  - end, duration: 0s, type: consecutive, control_mode: 18
```
mc_ctrl 支持预编排的舞蹈动作序列，通过 control_mode=21 触发。

**FSM PD 参数（xg-user-parameters.yaml）：**
```yaml
FSM_jointPD_Kp: 80       # 站立 PD Kp
FSM_jointPD_Kd: 1.0      # 站立 PD Kd
FSM_passive_Kd: 3.0      # 被动阻尼
FSM_RL_ABAD_Kp: 20.0     # RL 模式 Kp
FSM_RL_HIP_Kp: 20.0
FSM_RL_KNEE_Kp: 20.0
FSM_RL_Kd: 0.7
```

---

## 四、键盘控制流程

### 4.1 Matrix 原生实现（sim_launcher.bin 内置）

sim_launcher.bin 是 Qt/QML 应用，**内置**以下功能：

```
sim_launcher.bin 启动时:
  ├── 创建虚拟罗技 F710 手柄 (Linux uinput)
  │   └── 日志: "虚拟罗技 F710 手柄已创建"
  ├── 注册 XGrabKey 全局键盘拦截
  │   └── 拦截键: W A S D Q E R F U Space Return
  └── 启动 GamepadForwarder
      └── 日志: "[GamepadForwarder] Started. Forwarding from /dev/input/js0"
```

**按键映射（XGrabKey → 虚拟 F710）：**

| 键盘按键 | 手柄输出 | 功能 |
|----------|---------|------|
| U | LB + Y | 站立 (PASSIVE→STANDUP) |
| Space | RB + LB | 趴下 (→PASSIVE) |
| W/S | ABS_Y ±32767 | 前进/后退 |
| A/D | ABS_X ±32767 | 左移/右移 |
| Q/E | ABS_RX + ABS_Z | 左转/右转 |
| R | BTN_START | 切换运动模式 |
| F | BTN_SELECT | 特殊功能 |

### 4.2 按键控制数据流

```
┌─────────────────────────────────────────────────────────────────┐
│  用户按 U 键                                                     │
│       ↓                                                          │
│  sim_launcher.bin XGrabKey 捕获                                  │
│       ↓                                                          │
│  转换为 LB+Y 组合键 → 写入 /dev/uinput                           │
│       ↓                                                          │
│  虚拟 F710 手柄 (/dev/input/jsX) 产生 joy_event                  │
│       ↓                                                          │
│  mc_ctrl GamepadReader 线程读取手柄事件                            │
│       ↓                                                          │
│  FSM 状态切换: PASSIVE → JOINT_PD → STANDUP                      │
│       ↓                                                          │
│  mc_ctrl 计算站立轨迹 → 生成 12 个关节目标                         │
│       ↓                                                          │
│  eCAL publish (mujoco_cmd topic) → robot_mujoco                  │
│       ↓                                                          │
│  robot_mujoco 执行 PD 控制: τ = Kp*(q_target - q) - Kd*q̇       │
│       ↓                                                          │
│  mj_step() 推进物理 → GLFW 窗口显示站立                           │
│       ↓ (同时)                                                    │
│  robot_mujoco sendUdpData() → UDP RobotState → UE               │
│       ↓                                                          │
│  UE 接收 → 写 qpos → mj_kinematics() FK → 更新 mesh → 渲染站立  │
└─────────────────────────────────────────────────────────────────┘
```

---

## 五、通信协议

### 5.1 eCAL 通信（mc_ctrl ↔ robot_mujoco）

| 方向 | Topic | 消息类型 | 频率 |
|------|-------|---------|------|
| mc_ctrl → robot_mujoco | mujoco_cmd | RobotCmd (protobuf) | 500Hz |
| robot_mujoco → mc_ctrl | mujoco_state | RobotState (protobuf) | 500Hz |

**RobotCmd 字段：**
- q_des_abad[4], q_des_hip[4], q_des_knee[4] — 目标关节角
- qd_des_abad[4], qd_des_hip[4], qd_des_knee[4] — 目标角速度
- kp_abad[4], kp_hip[4], kp_knee[4] — PD Kp
- kd_abad[4], kd_hip[4], kd_knee[4] — PD Kd
- tau_abad_ff[4], tau_hip_ff[4], tau_knee_ff[4] — 前馈力矩

**RobotState 字段：**
- q_abad[4], q_hip[4], q_knee[4] — 当前关节角
- qd_abad[4], qd_hip[4], qd_knee[4] — 当前角速度
- tau_abad[4], tau_hip[4], tau_knee[4] — 当前力矩
- quat[4], position[3], gyro[3], acc[3] — IMU 数据

### 5.2 UDP 通信（robot_mujoco → UE）（2026.08.03 实测修正）

| 方向 | 端口 | 消息类型 | 用途 |
|------|------|---------|------|
| robot_mujoco → UE | **9999** | 自定义二进制 (412 bytes) | 渲染状态 |

**UE 在 MuJoCo Physics 模式下：**
- 只接收，不发送
- 不运行内部物理
- 不向 mc_ctrl 发送任何数据
- UUdpReceiverComponent 监听端口 9999

---

## 六、motor_platform_type 对照表

| 值 | 接口类 | 通信方式 | 物理引擎 | 使用场景 |
|---|--------|---------|---------|---------|
| 5 | MujocoCommandInterface | eCAL | robot_mujoco (外部) | MuJoCo Physics 模式 |
| 8 | UnrealCommandInterface | UDP 25001/25002 | UE 内嵌 MuJoCo | UE 内部物理模式 |
| 7 | 实机接口 | CAN/EtherCAT | 无 | 真实机器人 |

---

## 七、config.json 关键字段

```json
{
  "robot": {
    "robot_type": "xgb",
    "mujoco_running": true,       // ← MuJoCo Physics 模式标志
    "state_port": 25001,          // ← 配置值，但实际 UE 监听端口为 9999 (2026.08.03 实测)
    "cmd_port": 25002,            // ← UE 内部物理模式用（此模式下不使用）
    "EgoView": true,
    "position": {"x": 0, "y": 0, "z": 0}
  }
}
```

**注意：** config.json 中 state_port 配置为 25001，但实测 UE 在 MuJoCo 模式下监听的是端口 **9999**。25001 仅在非 MuJoCo 模式下使用（UE → mc_ctrl 状态反馈）。

---

## 八、启动时序图

```
时间轴 ─────────────────────────────────────────────────────────►

sim_launcher.bin
  │ 创建虚拟手柄
  │ 注册 XGrabKey
  │
  │ [用户点击 Launch]
  │──── 调用 run_sim.sh ─────────────────────────────────────────
  │
  │   run_sim.sh
  │     │ 设置 motor_platform_type=5
  │     │ 同步 config.json (mujoco_running=true)
  │     │
  │     ├── 启动 robot_mujoco ────────────────────────────────────
  │     │     │ 加载 XML 模型
  │     │     │ 初始化 GLFW 窗口
  │     │     │ 启动 ZsibotSdkBridge (eCAL + UDP)
  │     │     │ 开始 mj_step 循环 (500Hz)
  │     │     │
  │     ├── 启动 zsibot_mujoco_ue ────────────────────────────────
  │     │     │ 加载 config.json
  │     │     │ 检测 mujoco_running=true → 外部渲染模式
  │     │     │ 加载 XML → 生成 mesh
  │     │     │ 初始化 UDP 接收器
  │     │     │ 等待 RobotState 数据...
  │     │     │
  │     │   sleep 7s
  │     │     │
  │     └── 启动 mc_ctrl ─────────────────────────────────────────
  │           │ 加载参数 (motor_platform_type=5)
  │           │ [MujocoCommandInterface] initialize
  │           │ 加载 ONNX RL 模型
  │           │ 初始化 FSM (PASSIVE)
  │           │ 连接 eCAL → robot_mujoco
  │           │ 读取虚拟手柄...
  │           │
  │           │ [用户按 U]
  │           │ GamepadReader: LB+Y → STANDUP
  │           │ eCAL cmd → robot_mujoco
  │           │ robot_mujoco: PD + mj_step → 站立
  │           │ robot_mujoco: UDP state → UE
  │           │ UE: FK → 渲染站立
  │           │
  │           │ [3s 后站立完成 → RL_MIX]
```

---

## 九、与替代方案（mujoco_sim + CarlaUnreal）的对应关系

| Matrix 组件 | 替代组件 | 说明 |
|------------|---------|------|
| robot_mujoco | mujoco_sim (Mujoco330) | 物理引擎 + eCAL + UDP |
| zsibot_mujoco_ue | CarlaUnreal (外部物理模式) | FK 渲染 |
| mc_ctrl | mc_ctrl (不变) | type=5, eCAL |
| sim_launcher.bin 键盘功能 | keyboard_control.py | 虚拟手柄 |
| config.json (mujoco_running) | bExternalPhysicsMode | 模式标志 |
| UDP 状态端口 | UDP 9999 | CarlaUnreal 使用 UDP 25001 |
| 状态包协议 | 自定义二进制 (412 bytes) | CarlaUnreal 使用 Protobuf (~303 bytes) |
| 渲染方式 | Skeletal Mesh | CarlaUnreal 使用 StaticMeshComponent |

---

## 十、关键注意事项

1. **mc_ctrl 必须最后启动** — 需要等待 robot_mujoco 的 eCAL topic 就绪
2. **UE 在 MuJoCo Physics 模式下不发送命令** — 它是纯被动接收端
3. **config.json 同步** — run_sim.sh 会将 config.json 复制到 UE Content 目录
4. **XML 模型一致性** — robot_mujoco 和 UE 必须加载相同的 XML 模型
5. **虚拟手柄必须先于 mc_ctrl** — mc_ctrl 启动时如果找不到手柄设备会报警告
6. **UDP 端口差异** — Matrix MuJoCo 模式下 UE 监听端口 **9999**（非 config.json 中的 25001），25001 仅在非 MuJoCo 模式下使用

---

## 十A、实际运行日志分析（MuJoCo 物理模式，2026.08.03 实测）

### A.1 环境确认

```bash
# config.json 状态
mujoco_running: true
state_port: 25001          # ← 配置值，但实际 UE 监听端口为 9999
cmd_port: 25002

# mc_ctrl 配置
motor_platform_type: 5   # MujocoCommandInterface (eCAL)

# 实测 UDP 端口 (2026.08.03)
$ ss -tunlp | grep 9999
udp  UNCONN  0  0  0.0.0.0:9999  0.0.0.0:*  users:(("zsibot_mujoco_u",pid=2987722,fd=260))
```

### A.2 robot_mujoco 启动日志

```
[main] 已安装崩溃信号处理器 (SIGSEGV, SIGBUS, SIGFPE, SIGILL, SIGABRT)
MuJoCo version 3.3.0
[BridgeControl] starting bridge thread
[ZsibotSdkBridgeThread] 线程启动，等待 MuJoCo 数据...
[ZsibotSdkBridgeThread] MuJoCo 数据已准备就绪
[ZsibotSdkBridgeThread] 创建 ZsibotSdkBridge 对象...
[CheckSensor] num_motor_=12, dim_motor_sensor_=36, nsensor=51, nsensordata=84
[CheckSensor] 找到 IMU 传感器，索引: 36
[CheckSensor] 找到 Frame 传感器，索引: 39

# 机器人结构 (14 body, 12 joint, 12 actuator, 84维传感器)
<<--- Link --->>
link_index: 0, name: world
link_index: 1, name: base_link
link_index: 2, name: FAR_ABAD_LINK
... (共 14 个 link)

<<--- Joint --->>
joint_index: 1, name: FAR_ABAD_JOINT
... (共 12 个关节)

<<--- Actuator --->>
actuator_index: 0, name: FAR_ABAD_LINK
... (共 12 个执行器)

<<--- Sensor --->>
sensor_index: 0, name: FR_hip_pos, dim: 1
... (关节位置/速度/力矩各 12 个 = 36 维)
sensor_index: 36, name: imu_quat, dim: 4
sensor_index: 40, name: imu_gyro, dim: 3
sensor_index: 43, name: imu_acc, dim: 3
sensor_index: 46, name: frame_pos, dim: 3
sensor_index: 49, name: frame_vel, dim: 3
# 另有 livox_imu_* 和 camera_imu_* 传感器各 13 维

# 控制循环启动
[ZsibotSdkBridgeThread] 开始运行控制循环...

# Zenoh 警告 (非致命，ROS2 传感器发布降级)
[WARN] [rmw_zenoh_cpp]: Unable to connect to a Zenoh router.

# 物理循环持续运行
[ZsibotSdkBridge::Run] 循环计数: 1000, mj_data_=0x709e9a832ec0, mj_model_=0x709ea1b944c0, num_motor_=12
[ZsibotSdkBridge::Run] 循环计数: 2000, ...
...
[ZsibotSdkBridge::Run] 循环计数: 59000, ...  ← 持续运行中
```

**关键观察：**
- MuJoCo 3.3.0 启动，ZsibotSdkBridge 线程运行控制循环
- 14 个 body、12 个关节、12 个执行器、84 维传感器数据
- 传感器包含 3 组 IMU：主 IMU、livox_imu、camera_imu
- 循环计数每 1000 打印一次，持续增长 → 物理引擎持续运行
- Zenoh 路由器警告不影响运行（ROS2 传感器发布降级）

### A.3 mc_ctrl 启动日志（type=5 eCAL 模式）

```
run robot
Loading parameters from file...
Loaded robot parameters
HardwareBridge Read user parameters from yaml file: ../../config/xg-user-parameters.yaml
Loaded user parameters
Got all parameters, starting up!
client_ip: 127.0.0.1
mp_recv_cp: 127.0.0.1:43997
mp_recv_cp: 127.0.0.1:43900
[MujocoCommandInterface] initialize...          ← 确认 type=5 (eCAL)
[PeriodicTask] Start robot-monitor (0 s, 19999999 ns)

# 加载 ONNX RL 模型 (17 个)
Load rl odom model → odom_2ms_notau
Load mlp model → policy_mlp
Load backflip model → policy_mix_backflip
Load flipover model → policy_mix_flipover
Load walk model → policy_mix_walk
Load sideflip model → policy_mix_sideflip
Load UPright model → policy_upright
Load tracking model → policy_tracking
Load Gait model → policy_slow_walk_gait
Load gait walk model → policy_gait_walk
Load Crawl model → policy_xg_crawl_260127
Load walk moonmodel → policy_moonwalk
Load Softwalk model → policy_mix_walk
Load balanceStand model → policy_balancestand_withyaw_0423
Load walkpos model → policy_walkpos
Load RLIK model → policy_ik
Load jump model → policy_jump_fix2_step3
Load measured model → policy_measured_0918

# Dance 舞蹈序列
Skipping 'dance_2': not a valid sequence.
Skipping 'dance_3': not a valid sequence.
Loaded dance sequences: Dance: ZGWS_dance_1

# 初始化 FSM 状态 (12 个)
Initialized FSM state: PASSIVE
Initialized FSM state: JOINT_PD
Initialized FSM state: STANDUP
Initialized FSM state: JOINT_FREE
Initialized FSM state: SAFE_PROTECT
Initialized FSM state: RL_MLP
Initialized FSM state: RL_MIX
Initialized FSM state: RL_BALANCE_STAND
Initialized FSM state: RL_WalkPos
Initialized FSM state: RL_IK
Initialized FSM state: JOINT_LOCK
Initialized FSM state: POS_CONTROL

# 控制启动
Data loaded from file: 405616
[joystic] find 1 joystic                         ← 找到虚拟 F710 手柄
[PeriodicTask] Start robot-control (0 s, 2000000 ns)  ← 控制循环 500Hz
[PeriodicTask] Start GamepadReader (0 s, 2000000 ns)
 body height is 0.047542,Folding legs            ← 初始趴地姿态（高度 ~4.75cm）
[FSM_RLMIX] on Enter!                            ← 自动进入 RL_MIX
[RL_Walk] on Enter!                              ← 进入行走模式

# 持续运行
Printing FSM Info...
Iteration: 9999
Operating Mode: NORMAL in RL_MIX
...
Iteration: 49999
Operating Mode: NORMAL in RL_MIX
```

**关键观察：**
- `[MujocoCommandInterface] initialize...` — 使用 eCAL 与 robot_mujoco 通信
- `[joystic] find 1 joystic` — 找到 sim_launcher 创建的虚拟 F710
- `body height is 0.047542,Folding legs` — 初始趴地姿态（高度 ~4.75cm）
- `[FSM_RLMIX] on Enter!` → `[RL_Walk] on Enter!` — 自动进入行走模式
- 控制循环频率: `2000000 ns = 2ms = 500Hz`
- 加载了 **17 个 ONNX RL 模型**（后空翻、翻身、侧翻、匍匐、太空步、舞蹈等）
- 初始化了 **12 个 FSM 状态**（含 JOINT_FREE、SAFE_PROTECT、RL_BALANCE_STAND 等）
- 支持 Dance 舞蹈序列（ZGWS_dance_1，通过 control_mode=21 触发）

### A.4 UE 启动日志（外部渲染模式）

```
# 启动命令
zsibot_mujoco_ue -game /Game/Maps/YardWorld -ExecCmds="t.MaxFPS 30"

# 引擎版本
UE5-CL-0, version 5.5.4-0+UE5, Ubuntu 22.04.5 LTS, CPU: i7-14700F

# 插件加载
LogPluginManager: Mounting Project plugin MuJoCoUE
LogPluginManager: Mounting Project plugin UDPWrapper

# World 开始播放
[16:10:42:353] Bringing World /Game/Maps/YardWorld up for play
[16:10:42:355] UUdpReceiverComponent: UDP receiver stopped and socket closed.
[16:10:42:355] UUdpReceiverComponent: Started listening for UDP messages on port 9999
[16:10:42:356] Loading MuJoCo model from: .../Content/model/xgb/scene_terrain_yard.xml

# 创建 body 层级 (14个body)
[16:10:42:632] Creating body: world (ID 0)
[16:10:42:632] Creating body: base_link (ID 1)
[16:10:42:632] Creating body: FAR_ABAD_LINK (ID 2)
... (共 14 个 body)

# 创建 mesh (场景障碍物 + 机器人几何体)
[16:10:42:632] Creating mesh: floor (ID 0), Visible: 0
[16:10:42:632] Creating mesh: box_OBS1 (ID 1), Visible: 1
... (共 69 个 mesh)

# 传感器初始化
[16:10:42:636] URGBCaptureComponent: Found camera Front!
[16:10:42:888] LiDAR Simulator for BP_Mid360_C started at 10.000000 Hz
[16:10:42:888] UUdpReceiverComponent: Started listening on port 13001  ← LiDAR 数据端口

# 加载完成
[16:10:42:888] LoadMap took 1.324784 seconds
[16:10:42:897] Engine Initialization Total time: 3.80 seconds
[16:10:43:107] t.MaxFPS = "30"

# 非致命错误：Vulkan 纹理布局警告
[16:10:43:522] Ensure condition failed: VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL
  Expected source texture in TRANSFER_SRC_OPTIMAL, actual SHADER_READ_ONLY_OPTIMAL
  File: VulkanTexture.cpp Line: 2515
  → 不影响运行，仅 RGB 相机拷贝纹理时的布局不匹配

# 之后无 FMujocoWorkerThread 日志 ← 确认 UE 不运行内部物理
# 尾部持续显示 ROS2 Node 未初始化警告（每 5s 一次）
```

**关键观察（2026.08.03 实测修正）：**
- UE 加载模型后 **没有** `FMujocoWorkerThread` 日志 → 确认不运行内部物理
- UE 在 MuJoCo 模式下是纯被动渲染器
- **UE 监听 UDP 9999 端口接收 robot_mujoco 的渲染状态**（非 25001）
- robot_mujoco 通过 ZsibotSdkBridge 发送 UDP 数据包到端口 9999，包大小 412 bytes，频率 ~100Hz
- ROS2 Node 未初始化警告不影响运行

### A.5 MuJoCo 模式实际数据流（2026.08.03 实测修正）

**UDP 9999 通信验证：**
```bash
# tcpdump 抓包结果
$ sudo tcpdump -i lo -n 'port 9999' -c 10
15:46:24.232421 IP 127.0.0.1.32920 > 127.0.0.1.9999: UDP, length 412
15:46:24.244916 IP 127.0.0.1.32920 > 127.0.0.1.9999: UDP, length 412
...

# ss 查看监听端口
$ ss -tunlp | grep 9999
udp   UNCONN 0      0                 0.0.0.0:9999       0.0.0.0:*    users:("zsibot_mujoco_u",pid=2987722,fd=260))
```

**完整数据流架构：**
```
┌─────────────────────────────────────────────────────────────┐
│  sim_launcher.bin (内置键盘拦截 + 虚拟手柄)                  │
│       │                                                      │
│       │ [用户按 U]                                           │
│       ▼                                                      │
│  虚拟 F710 手柄 (/dev/input/js0)                             │
│       │                                                      │
│       ▼                                                      │
│  mc_ctrl (type=5, MujocoCommandInterface)                    │
│       │                                                      │
│       │ ① eCAL publish (mujoco_cmd) → robot_mujoco          │
│       │    ──────────────────────────────────────►            │
│       │                                                      │
│       │ ② eCAL subscribe (mujoco_state) ← robot_mujoco      │
│       │    ◄──────────────────────────────────────            │
│       │                                                      │
│       ▼                                                      │
│  robot_mujoco (MuJoCo 3.3.0)                                 │
│       ├── mj_step @ 500Hz (物理仿真)                         │
│       ├── ZsibotSdkBridge (eCAL ↔ mj_step)                   │
│       │     ├── eCAL Publisher  → mc_ctrl (mujoco_state)    │
│       │     ├── eCAL Subscriber ← mc_ctrl (mujoco_cmd)      │
│       │     └── sendUdpData() → UDP 9999 → UE (渲染状态)    │
│       └── GLFW 窗口 (本地可视化)                              │
│                                                              │
│       ▼ UDP 9999 (412 bytes/packet, ~100Hz)                  │
│  zsibot_mujoco_ue (UE)                                       │
│       ├── UUdpReceiverComponent 监听端口 9999                │
│       ├── 接收 RobotState → 写 qpos → FK → 更新 mesh         │
│       ├── 渲染线程 (30fps)                                   │
│       └── 无内部物理 (无 FMujocoWorkerThread)                │
─────────────────────────────────────────────────────────────┘
```

**UDP 9999 数据包结构分析：**
- 包大小：412 bytes
- 发送方：robot_mujoco (动态端口 32920)
- 接收方：zsibot_mujoco_ue (端口 9999)
- 协议：二进制自定义格式（非 protobuf）
- 内容推测：包含关节角、IMU、位置等状态数据

**与 CarlaUnreal 的对比：**
| 项目 | Matrix (robot_mujoco) | CarlaUnreal (mujoco_sim) |
|------|-----------------------|--------------------------|
| UDP 端口 | 9999 | 25001 |
| 包大小 | 412 bytes | ~303 bytes (protobuf) |
| 发送频率 | ~100Hz | 500Hz |
| 协议 | 自定义二进制 | Protobuf RobotState |
| 渲染方式 | Skeletal Mesh (抗抖动) | StaticMeshComponent (可能抖动) |

### A.6 与非 MuJoCo 模式的关键差异（实测确认）

| 对比项 | MuJoCo 模式 (实测) | 非 MuJoCo 模式 (实测) |
|--------|---------------------|------------------------|
| mc_ctrl 初始化 | `[MujocoCommandInterface] initialize...` | `[UnrealCommandInterface] initialize...` |
| mc_ctrl 通信 | eCAL (protobuf) | UDP (protobuf) |
| 物理进程 | robot_mujoco (独立 GLFW 窗口) | UE 内部 FMujocoWorkerThread |
| 物理频率 | 500Hz (robot_mujoco) | ~500Hz (UE FMujocoWorkerThread) |
| UE 物理线程 | **无** FMujocoWorkerThread | **有** FMujocoWorkerThread |
| UE 接收状态端口 | **UDP 9999** (robot_mujoco → UE) | **UDP 25001** (UE → mc_ctrl 状态反馈) |
| UE 发送状态 | 不发送 | UDP 25001 → mc_ctrl (303 bytes) |
| body height | 0.047286 (Folding legs) | 由 UE 内部物理决定 |
| 启动后自动状态 | PASSIVE → RL_MIX → RL_Walk | 等待 mc_ctrl 指令 |
| UE 日志特征 | 无物理线程日志 | 有 FMujocoWorkerThread FPS 日志 |

---
---

# Matrix 非 MuJoCo 物理模式（UE 内部物理）启动流程分析

## 概述

当 Matrix Simulation Launcher 选择 **非 MuJoCo Physics** 模式（即不勾选 MuJoCo 物理）时，`robot_mujoco` 进程**不启动**。UE 内嵌 MuJoCo 引擎自行运行物理仿真（`mj_step`），mc_ctrl 通过 **UDP** 与 UE 直接通信（而非 eCAL）。此模式下 UE 既是渲染器又是物理引擎。

---

## 十一、进程架构（非 MuJoCo 模式）

```
┌───────────────────────────────────────────────────────────────────────┐
│                        sim_launcher.bin (Qt/QML)                       │
│  ┌─────────────────┐  ┌─────────────────┐  ┌──────────────────────┐  │
│  │  KeyListener    │  │  uinput 虚拟     │  │  GamepadForwarder    │  │
│  │  (XGrabKey)     │──│  F710 手柄       │──│  (/dev/input/js0)    │  │
│  └─────────────────┘  └─────────────────┘  └──────────────────────┘  │
│         点击 "Launch Simulation" → 调用 run_sim.sh                     │
└───────────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
┌───────────────────────────────────────────────────────────────────────┐
│                          run_sim.sh 启动脚本                            │
│                                                                       │
│  当 mujoco_running=0 时:                                              │
│    • motor_platform_type → 8 (UnrealCommandInterface)                 │
│    • config.json → "mujoco_running": false                            │
│    • 不启动 robot_mujoco                                              │
│    • 只启动 UE + mc_ctrl                                              │
└───────────────────────────────────────────────────────────────────────┘
                                    │
                    ┌───────────────┴───────────────┐
                    ▼                               ▼
          ┌──────────────────────┐          ┌──────────────┐
          │  zsibot_mujoco_ue    │          │   mc_ctrl    │
          │  (物理 + 渲染)        │          │ (运动控制)    │
          │                      │          │              │
          │  内嵌 MuJoCo:        │          │  UnrealCommand│
          │  • mj_step @ 500Hz   │◄────────►│  Interface   │
          │  • PD 控制           │ UDP双向   │  (type=8)    │
          │  • 渲染              │          │              │
          └──────────────────────┘          └──────────────┘
```

**与 MuJoCo 模式的关键区别：**
- 没有 `robot_mujoco` 进程
- UE 内部运行 `mj_step`（物理 + 渲染一体）
- mc_ctrl 通过 UDP（而非 eCAL）与 UE 通信
- 只有 2 个进程（UE + mc_ctrl），而非 3 个

---

## 十二、启动顺序（非 MuJoCo 模式）

### 12.1 配置阶段

```bash
# 文件: scripts/run_sim.sh

# 1. MUJOCORUNNING=0 → 设置 motor_platform_type=8
if [[ "$MUJOCORUNNING" == "1" ]]; then
    ...
else
    ENABLE_MUJOCO=false
    # 设置为 8 → UnrealCommandInterface (UDP 通信)
    sed -i 's/motor_platform_type: .*/motor_platform_type: 8/' \
        src/robot_mc/build/export/config/xg-user-parameters.yaml
fi

# 2. 同步 config.json (mujoco_running=false)
MUJOCO_RUNNING_JSON=false
jq --argjson mujoco_running false \
   '.robot.mujoco_running = $mujoco_running
    | .robot.state_port = 25001
    | .robot.cmd_port = 25002' \
   config/config.json > tmp && mv tmp config.json

# 3. 同步到 UE Content 目录
cp config/config.json \
   src/UeSim/Linux/zsibot_mujoco_ue/Content/model/config/config.json
```

### 12.2 进程启动顺序

```bash
# ① robot_mujoco 不启动 (ENABLE_MUJOCO=false，跳过)
cd src/robot_mujoco/simulate/build
if $ENABLE_MUJOCO; then
    # 此分支不执行
    ./robot_mujoco > robot_mujoco.log 2>&1 &
fi

# ② 启动 UE (内嵌物理引擎)
cd src/UeSim/Linux
LD_LIBRARY_PATH="$(ue_ld_library_path)" ./zsibot_mujoco_ue.sh -game "$MAPNAME" \
    -ExecCmds="t.MaxFPS 30" > zsibot_mujoco_ue.log 2>&1 &

# ③ 等待 7 秒 (等 UE 完成加载)
sleep 7

# ④ 启动 mc_ctrl (运动控制器)
cd src/robot_mc
export SDK_CLIENT_IP="127.0.0.1"
LD_LIBRARY_PATH="$(mc_ld_library_path)" ./run_mc.sh r > run_mc.log 2>&1 &
```

**关键区别：只有 UE 和 mc_ctrl 两个进程，无 robot_mujoco。**

---

## 十三、各进程职责（非 MuJoCo 模式）

### 13.1 zsibot_mujoco_ue（物理 + 渲染一体）

| 属性 | 值 |
|------|-----|
| 二进制路径 | `src/UeSim/Linux/zsibot_mujoco_ue.sh` |
| 配置来源 | `Content/model/config/config.json` |
| 渲染模式 | **内部物理** (mujoco_running=false) |
| 物理引擎 | 内嵌 MuJoCo (mj_step) |
| 物理频率 | **~500Hz** (FMujocoWorkerThread 独立线程) |
| 渲染帧率 | t.MaxFPS 30 |

**实际运行日志（zsibot_mujoco_ue.log）：**
```
# 1. 加载 MuJoCo 模型
[02.11.54:008] Loading model from: model/xgb/scene_terrain_yard.xml
[02.11.54:008] Loading MuJoCo model from: .../Content/model/xgb/scene_terrain_yard.xml

# 2. 创建 body 层级 (14个body: 1 world + 1 base + 12 joints)
[02.11.54:291] Creating body: world (ID 0)
[02.11.54:291] Creating body: base_link (ID 1)
[02.11.54:291] Creating body: FAR_ABAD_LINK (ID 2)
... (共14个body)

# 3. 创建 mesh (可见+不可见)
[02.11.54:291] Creating mesh: floor (ID 0), Visible: 0
[02.11.54:291] Creating mesh: box_OBS1 (ID 1), Visible: 1
... (共69个mesh)

# 4. Actor 实例: MujoCoSim_Xgb_C
LogUObjectGlobals: MujoCoSim_Xgb_C_2147482340._Geom14

# 5. 物理线程启动 — 独立于渲染线程，~500Hz
[02.12.04:294] [FMujocoWorkerThread] FPS: 498.70
[02.12.14:294] [FMujocoWorkerThread] FPS: 498.20
[02.12.24:294] [FMujocoWorkerThread] FPS: 498.70
... (持续运行)
```

**关键发现：物理与渲染解耦**
- `FMujocoWorkerThread` 是独立线程，以 ~500Hz 运行 `mj_step`
- 渲染帧率限制为 30fps（t.MaxFPS 30），但物理不受影响
- 物理线程每步完成后同步 mesh 位置给渲染线程

**UE 启动行为（非 MuJoCo 模式）：**
1. 加载 `config.json` → 检测 `mujoco_running: false`
2. 加载 MuJoCo XML 模型（`scene_terrain_yard.xml`）
3. 创建所有 body 的 SceneComponent 层级（14个body）
4. 创建所有 geom 的 StaticMeshComponent（69个mesh）
5. **启动 FMujocoWorkerThread** — 独立线程以 ~500Hz 调用 `mj_step()`
6. 初始化 UDP 双向通信：
   - **监听 cmd_port (25002)** — 接收 mc_ctrl 的 RobotCmd
   - **发送到 state_port (25001)** — 向 mc_ctrl 发送 RobotState (303 bytes)
7. 设置初始 LIEDOWN 姿态（等待 mc_ctrl 站立指令）

**内部物理循环（FMujocoWorkerThread，~500Hz）：**
```
FMujocoWorkerThread::Run() @ ~500Hz:
  1. 检查 UDP 是否收到新的 RobotCmd
  2. 如果有 → 写入 mData->ctrl (关节目标/PD参数)
  3. 执行 mj_step(mModel, mData) — 推进物理
  4. 从 mData 提取新状态 (qpos, qvel, IMU)
  5. 同步 mesh 位置给渲染线程
  6. 通过 UDP 发送 RobotState (303 bytes) 给 mc_ctrl
```

### 13.2 mc_ctrl（运动控制）

| 属性 | 值 |
|------|-----|
| 二进制路径 | `src/robot_mc/build/export/mc/bin/mc_ctrl` |
| 启动脚本 | `src/robot_mc/run_mc.sh` |
| 通信接口 | **UnrealCommandInterface** (type=8, UDP) |
| CPU 绑定 | taskset -c 7 |
| FSM 状态 | PASSIVE → JOINT_PD → STANDUP → RL_MIX |

**mc_ctrl 初始化（实际运行日志 run_mc.log）：**
```
run robot
Loading parameters from file...
Loaded robot parameters
HardwareBridge Read user parameters from yaml file: ../../config/xg-user-parameters.yaml
Loaded user parameters
Got all parameters, starting up!
client_ip: 127.0.0.1
mp_recv_cp: 127.0.0.1:43997
mp_recv_cp: 127.0.0.1:43900
[UnrealCommandInterface] initialize...          ← 确认 type=8
#################
../../onnx_model_crypto/xg/odom_2ms_notau
Load rl odom model
Start loadPolicy ../../onnx_model_crypto/xg/odom_2ms_notau
[PeriodicTask] Start robot-monitor (0 s, 19999999 ns)
...
Initialized FSM state: PASSIVE
Initialized FSM state: JOINT_PD
Initialized FSM state: STANDUP
Initialized FSM state: RL_MLP
Initialized FSM state: RL_MIX
...
Initialized FSM state: RL_WalkPos
Initialized FSM state: RL_IK
Initialized FSM state: JOINT_LOCK
Initialized FSM state: POS_CONTROL
Data loaded from file: 405616
[joystic] find 1 joystic                         ← 找到虚拟 F710 手柄
[PeriodicTask] Start robot-control (0 s, 2000000 ns)  ← 控制循环启动 (500Hz)
Reciving data size: 303                          ← 开始接收 UE 的 RobotState (303 bytes)
Reciving data size: 303
...
```

**UnrealCommandInterface 内部结构（从 librobot.so strings 分析）：**
```
UnrealCommandInterface (type=8)
├── initialize()
│   ├── 创建 UEUDPReceiver 线程
│   │   └── run() → 持续监听 UDP 25001 (RobotState from UE)
│   └── 初始化 UDP Sender → 目标 127.0.0.1:25002
├── sendCmd()
│   └── 序列化 RobotCmd (protobuf) → UDP 发送到 UE:25002
└── reciveData()
    └── 从 UEUDPReceiver 获取最新 RobotState
```

---

## 十四、键盘控制流程（非 MuJoCo 模式）

### 14.1 按键控制数据流

```
┌─────────────────────────────────────────────────────────────────┐
│  用户按 U 键                                                     │
│       ↓                                                          │
│  sim_launcher.bin XGrabKey 捕获                                  │
│       ↓                                                          │
│  转换为 LB+Y 组合键 → 写入 /dev/uinput                           │
│       ↓                                                          │
│  虚拟 F710 手柄 (/dev/input/jsX) 产生 joy_event                  │
│       ↓                                                          │
│  mc_ctrl GamepadReader 线程读取手柄事件                            │
│       ↓                                                          │
│  FSM 状态切换: PASSIVE → JOINT_PD → STANDUP                      │
│       ↓                                                          │
│  mc_ctrl 计算站立轨迹 → 生成 12 个关节目标                         │
│       ↓                                                          │
│  UnrealCommandInterface.sendCmd()                                │
│  → UDP RobotCmd (protobuf) → 127.0.0.1:25002 → UE              │
│       ↓                                                          │
│  UE 接收 RobotCmd → 写入 mData->ctrl                             │
│       ↓                                                          │
│  UE 执行 mj_step() → 物理推进 → 机械狗站立                       │
│       ↓ (同时)                                                    │
│  UE 发送 RobotState → UDP 25001 → mc_ctrl                       │
│       ↓                                                          │
│  mc_ctrl UEUDPReceiver 接收 → 更新状态估计 → 下一步控制            │
└─────────────────────────────────────────────────────────────────┘
```

**与 MuJoCo 模式的对比：**
- MuJoCo 模式：mc_ctrl → eCAL → robot_mujoco → UDP → UE（单向渲染）
- 非 MuJoCo 模式：mc_ctrl → UDP → UE（UE 自己跑物理，双向通信）

---

## 十五、通信协议（非 MuJoCo 模式）

### 15.1 UDP 双向通信（mc_ctrl ↔ UE）

| 方向 | 端口 | 消息类型 | 频率 | 用途 |
|------|------|---------|------|------|
| mc_ctrl → UE | 25002 (cmd_port) | RobotCmd (protobuf) | ~500Hz | 关节目标 |
| UE → mc_ctrl | 25001 (state_port) | RobotState (protobuf, 303 bytes) | ~500Hz | 状态反馈 |

**注意：非 MuJoCo 模式下没有 eCAL 通信，所有数据通过 UDP 传输。**

**RobotCmd 字段（与 MuJoCo 模式相同）：**
- q_des_abad[4], q_des_hip[4], q_des_knee[4] — 目标关节角
- qd_des_abad[4], qd_des_hip[4], qd_des_knee[4] — 目标角速度
- kp_abad[4], kp_hip[4], kp_knee[4] — PD Kp
- kd_abad[4], kd_hip[4], kd_knee[4] — PD Kd
- tau_abad_ff[4], tau_hip_ff[4], tau_knee_ff[4] — 前馈力矩

**RobotState 字段（与 MuJoCo 模式相同）：**
- q_abad[4], q_hip[4], q_knee[4] — 当前关节角
- qd_abad[4], qd_hip[4], qd_knee[4] — 当前角速度
- tau_abad[4], tau_hip[4], tau_knee[4] — 当前力矩
- quat[4], position[3], gyro[3], acc[3] — IMU 数据

### 15.2 通信频率对比

| 模式 | 物理频率 | 状态反馈频率 | 瓶颈 |
|------|---------|------------|------|
| MuJoCo 模式 | 500Hz (robot_mujoco) | 500Hz (eCAL) | 无 |
| 非 MuJoCo 模式 | ~500Hz (FMujocoWorkerThread) | ~500Hz (UDP) | 无 |

**重要发现：** 非 MuJoCo 模式的物理频率与 MuJoCo 模式相同，均为 ~500Hz。UE 通过独立的 `FMujocoWorkerThread` 线程运行物理，不受渲染帧率（30fps）限制。两种模式的物理仿真精度相当。

---

## 十六、config.json 关键字段（非 MuJoCo 模式）

```json
{
  "robot": {
    "robot_type": "xgb",
    "mujoco_running": false,      // ← 非 MuJoCo 模式标志
    "state_port": 25001,          // ← UE 发送 RobotState 到 mc_ctrl 的端口
    "cmd_port": 25002,            // ← UE 监听 mc_ctrl RobotCmd 的端口
    "EgoView": true,
    "position": {"x": 0, "y": 0, "z": 0}
  }
}
```

**端口用途对比：**

| 端口 | MuJoCo 模式 | 非 MuJoCo 模式 |
|------|------------|---------------|
| 25001 (state_port) | robot_mujoco → UE (状态) | UE → mc_ctrl (状态) |
| 25002 (cmd_port) | 不使用 | mc_ctrl → UE (命令) |

---

## 十七、启动时序图（非 MuJoCo 模式）

```
时间轴 ─────────────────────────────────────────────────────────►

sim_launcher.bin
  │ 创建虚拟手柄
  │ 注册 XGrabKey
  │
  │ [用户点击 Launch]
  │──── 调用 run_sim.sh ─────────────────────────────────────────
  │
  │   run_sim.sh
  │     │ 设置 motor_platform_type=8
  │     │ 同步 config.json (mujoco_running=false)
  │     │
  │     ├── [跳过 robot_mujoco] ─────────────────────────────────
  │     │
  │     ├── 启动 zsibot_mujoco_ue ────────────────────────────────
  │     │     │ 加载 config.json
  │     │     │ 检测 mujoco_running=false → 内部物理模式
  │     │     │ 加载 XML → 生成 mesh (14 body, 69 mesh)
  │     │     │ 设置初始 LIEDOWN 姿态
  │     │     │ 启动 FMujocoWorkerThread (~500Hz mj_step)
  │     │     │ 初始化 UDP:
  │     │     │   • 监听 25002 (接收 RobotCmd)
  │     │     │   • 发送到 25001 (发送 RobotState, 303 bytes)
  │     │     │ 等待 mc_ctrl 命令...
  │     │     │
  │     │   sleep 7s
  │     │     │
  │     └── 启动 mc_ctrl ─────────────────────────────────────────
  │           │ 加载参数 (motor_platform_type=8)
  │           │ [UnrealCommandInterface] initialize
  │           │ 创建 UEUDPReceiver 线程 (监听 25001)
  │           │ 加载 ONNX RL 模型
  │           │ 初始化 FSM (PASSIVE → ... → POS_CONTROL)
  │           │ [joystic] find 1 joystic
  │           │ [PeriodicTask] Start robot-control (500Hz)
  │           │ Reciving data size: 303 ← 开始接收 UE 状态
  │           │
  │           │ [用户按 U]
  │           │ GamepadReader: LB+Y → STANDUP
  │           │ UDP RobotCmd → UE:25002
  │           │ UE: 写 ctrl → mj_step → 站立
  │           │ UE: UDP RobotState → mc_ctrl:25001
  │           │ mc_ctrl: 更新状态 → 下一步控制
  │           │
  │           │ [3s 后站立完成 → RL_MIX]
```

---

## 十八、两种模式完整对比

| 对比项 | MuJoCo Physics 模式 | 非 MuJoCo (UE内部物理) 模式 |
|--------|--------------------|-----------------------------|
| 进程数 | 3 (robot_mujoco + UE + mc_ctrl) | 2 (UE + mc_ctrl) |
| 物理引擎 | robot_mujoco (独立进程) | UE 内嵌 MuJoCo (FMujocoWorkerThread) |
| 物理频率 | 500Hz | **~500Hz** (独立线程，不受帧率限制) |
| mc_ctrl 通信 | eCAL (type=5) | UDP (type=8) |
| UE 角色 | 纯渲染 (FK) | 物理 + 渲染 |
| UE 是否跑 mj_step | 否 | 是 |
| config.json | mujoco_running=true | mujoco_running=false |
| motor_platform_type | 5 | 8 |
| 端口 25001 | robot_mujoco → UE (状态) | UE → mc_ctrl (状态, 303 bytes) |
| 端口 25002 | 不使用 | mc_ctrl → UE (命令) |
| 端口 9999 | robot_mujoco → UE (渲染状态, 412B) | 不使用 |
| RL 模型数 | 17 个 ONNX (后空翻、舞蹈等) | 17 个 ONNX (相同) |
| FSM 状态数 | 12 个 | 12 个 (相同) |
| Dance 功能 | 支持 (control_mode=21) | 支持 (相同) |
| 适用场景 | 高保真物理仿真 | 轻量级/快速预览 |
| RL 训练适用性 | 高（500Hz 精确物理） | 高（同样 500Hz 物理） |

---

## 十九、关键注意事项（非 MuJoCo 模式）

1. **物理与渲染解耦** — `FMujocoWorkerThread` 独立线程以 ~500Hz 运行物理，不受 `t.MaxFPS 30` 限制
2. **mc_ctrl 必须最后启动** — 需要等待 UE 的 UDP 端口就绪
3. **UDP 双向通信** — UE 既是命令接收方又是状态发送方，与 MuJoCo 模式的单向 UDP 不同
4. **初始姿态为 LIEDOWN** — UE 启动时设置趴下姿态，等待 mc_ctrl 发送站立指令
5. **无 robot_mujoco 进程** — 不会出现 GLFW 窗口，只有 UE 渲染窗口
6. **物理精度与 MuJoCo 模式相当** — 同样 ~500Hz 物理步进，RL 策略可直接复用

---

## 二十、实际运行日志分析（2026.08.03 实测）

### 20.1 环境确认

```bash
# config.json 状态
mujoco_running: false
state_port: 25001
cmd_port: 25002

# mc_ctrl 配置
motor_platform_type: 8   # UnrealCommandInterface
```

### 20.2 UE 启动日志（zsibot_mujoco_ue.log）

```
[02.11.50:847] 启动命令: zsibot_mujoco_ue -game /Game/Maps/YardWorld -ExecCmds="t.MaxFPS 30"
[02.11.54:007] UUdpReceiverComponent: UDP receiver stopped and socket closed.
[02.11.54:008] UUdpReceiverComponent: Started listening for UDP messages on port 9999
[02.11.54:008] Loading model from: model/xgb/scene_terrain_yard.xml
[02.11.54:008] Loading MuJoCo model from: .../Content/model/xgb/scene_terrain_yard.xml
[02.11.54:291] Creating body: world (ID 0)
[02.11.54:291] Creating body: base_link (ID 1)
[02.11.54:291] Creating body: FAR_ABAD_LINK (ID 2)
... (共 14 个 body: 1 world + 1 base + 4×3 joints)
[02.11.54:291] Creating mesh: floor (ID 0), Visible: 0
[02.11.54:291] Creating mesh: box_OBS1 (ID 1), Visible: 1
... (共 69 个 mesh: 障碍物可见 + 机器人几何体不可见)
[02.11.54:293] MujoCoSim_Xgb_C_2147482340._Geom14  ← Actor 实例
[02.11.54:295] URGBCaptureComponent: Found camera Front!
[02.11.54:542] LiDAR Simulator for BP_Mid360_C started at 10.000000 Hz
[02.11.54:543] UUdpReceiverComponent: Started listening on port 13001
[02.11.54:543] LoadMap took 1.273577 seconds

# 物理线程启动（加载完成后）
[02.12.04:294] [FMujocoWorkerThread] FPS: 498.70  ← ~500Hz 物理线程运行中
[02.12.14:294] [FMujocoWorkerThread] FPS: 498.20
[02.12.24:294] [FMujocoWorkerThread] FPS: 498.70
... (持续运行)
```

**关键观察：**
- UE 加载模型后，`FMujocoWorkerThread` 立即以 ~500Hz 启动物理仿真
- 物理线程与渲染线程完全解耦（渲染 30fps，物理 500Hz）
- 有一个 Vulkan texture layout 的 ensure 错误（非致命，不影响运行）

### 20.3 mc_ctrl 启动日志（run_mc.log）

```
run robot
Loading parameters from file...
Loaded robot parameters
HardwareBridge Read user parameters from yaml file: ../../config/xg-user-parameters.yaml
Loaded user parameters
Got all parameters, starting up!
client_ip: 127.0.0.1
mp_recv_cp: 127.0.0.1:43997
mp_recv_cp: 127.0.0.1:43900
[UnrealCommandInterface] initialize...          ← 确认 type=8
#################
../../onnx_model_crypto/xg/odom_2ms_notau
Load rl odom model
Start loadPolicy ../../onnx_model_crypto/xg/odom_2ms_notau
[PeriodicTask] Start robot-monitor (0 s, 19999999 ns)
...
Initialized FSM state: PASSIVE
Initialized FSM state: JOINT_PD
Initialized FSM state: STANDUP
Initialized FSM state: RL_MLP
Initialized FSM state: RL_MIX
...
Initialized FSM state: RL_WalkPos
Initialized FSM state: RL_IK
Initialized FSM state: JOINT_LOCK
Initialized FSM state: POS_CONTROL
Data loaded from file: 405616
[joystic] find 1 joystic                         ← 找到虚拟 F710 手柄
[PeriodicTask] Start robot-control (0 s, 2000000 ns)  ← 控制循环 500Hz
Reciving data size: 303                          ← 开始接收 UE 的 RobotState
Reciving data size: 303
... (持续接收)
```

**关键观察：**
- `[UnrealCommandInterface] initialize...` — 确认使用 type=8 接口
- `[joystic] find 1 joystic` — 成功找到 sim_launcher 创建的虚拟 F710 手柄
- `Reciving data size: 303` — 持续接收 UE 发送的 RobotState（303 bytes/包）
- 控制循环频率: `2000000 ns = 2ms = 500Hz`

### 20.4 实际数据流确认

```
┌─────────────────────────────────────────────────────────────┐
│  sim_launcher.bin (内置键盘拦截 + 虚拟手柄)                  │
│       │                                                      │
│       │ [用户按 U]                                           │
│       ▼                                                      │
│  虚拟 F710 手柄 (/dev/input/js0)                             │
│       │                                                      │
│       ▼                                                      │
│  mc_ctrl (type=8, UnrealCommandInterface)                    │
│       │                                                      │
│       │ ① 发送 RobotCmd (UDP → 127.0.0.1:25002)            │
│       │    ──────────────────────────────────────►            │
│       │                                                      │
│       │ ② 接收 RobotState (UDP ← 127.0.0.1:25001)          │
│       │    ◄──────────────────────────────────────            │
│       │    Reciving data size: 303                           │
│       │                                                      │
│       ▼                                                      │
│  zsibot_mujoco_ue (UE)                                       │
│       ├── FMujocoWorkerThread (~500Hz mj_step)               │
│       ├── 渲染线程 (30fps)                                   │
│       └── UDP 双向通信                                       │
└─────────────────────────────────────────────────────────────┘
```

### 20.5 与 MuJoCo 模式的关键差异（实测确认）

| 对比项 | MuJoCo 模式 (type=5) | 非 MuJoCo 模式 (type=8) |
|--------|---------------------|------------------------|
| mc_ctrl 初始化日志 | `[MujocoCommandInterface] initialize...` | `[UnrealCommandInterface] initialize...` |
| 物理进程 | robot_mujoco (独立 GLFW 窗口) | UE 内部 FMujocoWorkerThread |
| 物理频率 | 500Hz | ~500Hz (实测 498.70) |
| 状态包大小 | eCAL (protobuf) | UDP 303 bytes |
| UE 日志特征 | 无 FMujocoWorkerThread | 有 FMujocoWorkerThread FPS 日志 |
| GLFW 窗口 | 有 (robot_mujoco) | 无 |
