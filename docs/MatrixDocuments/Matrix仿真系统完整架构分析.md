# Matrix 仿真系统完整架构分析

## 1. 系统总览

Matrix 仿真系统由 **3 个独立进程** 协作完成机械狗的仿真与控制：

```
┌─────────────────────────────────────────────────────────────────────────┐
│                        sim_launcher.bin (Qt GUI)                         │
│  职责: 启动器 + 键盘/手柄输入捕获 + 进程管理                              │
│  技术: Qt Quick/QML, XGrabKey (X11全局键盘钩子), 虚拟罗技F710手柄         │
└────────────────────────────────┬────────────────────────────────────────┘
                                 │ 启动 run_sim.sh
                                 ▼
┌─────────────────────────────────────────────────────────────────────────┐
│                          run_sim.sh (编排脚本)                            │
│  职责: 环境检查 → 配置同步 → 按序启动3个核心进程 → 进程监控               │
└───┬─────────────────────────┬──────────────────────────┬────────────────┘
    │                         │                          │
    ▼                         ▼                          ▼
┌────────────┐         ┌──────────────┐          ┌──────────────┐
│ 进程①      │         │ 进程②        │          │ 进程③        │
│robot_mujoco│         │zsibot_mujoco │          │  mc_ctrl     │
│(可选)      │         │_ue (UE)      │          │(运动控制)    │
│独立MuJoCo  │         │渲染+内嵌物理 │          │RL+MPC+PD    │
└────────────┘         └──────────────┘          └──────────────┘
```

---

## 2. 启动流程详解

### 2.1 第一层：sim_launcher（用户入口）

```bash
./bin/sim_launcher
```

**sim_launcher** 是一个 bash 包装脚本：
```bash
export RMW_IMPLEMENTATION="rmw_zenoh_cpp"   # ROS2 通信中间件
export ROS_DOMAIN_ID="89"                    # ROS2 域隔离
export SDK_CLIENT_IP="127.0.0.1"            # 本机通信
exec sim_launcher.bin "$@"                   # 启动 Qt GUI
```

**sim_launcher.bin** 是预编译的 Qt/QML 应用：
- 显示启动界面（选择机器人类型、地图、模式）
- 通过 **XGrabKey** 全局拦截键盘（W/A/S/D/Q/E/R/F/U/Space）
- 创建虚拟罗技 F710 手柄（uinput）
- 点击 Launch 后调用 `scripts/run_sim.sh`

### 2.2 第二层：run_sim.sh（进程编排）

```bash
run_sim.sh <robotId> <mapId> <headless> <pixelStream> <mujocoRunning>
# 示例:    1(xgb)    0       0          0             0
```

执行步骤：

| 步骤 | 操作 | 说明 |
|------|------|------|
| 1 | 环境检查 | check_env.sh 验证依赖 |
| 2 | 杀旧进程 | pkill robot_mujoco/zsibot_mujoco_ue/mc_ctrl |
| 3 | 配置场景 | 选择 scene_terrain_*.xml + UE Map |
| 4 | 配置机器人 | ROBOT_TYPE=XG, motor_platform_type |
| 5 | 同步 config.json | 复制到 UE Content/model/config/ |
| 6 | 同步场景 XML | 覆盖 UE 入口 scene_terrain.xml |
| 7 | 设置初始位姿 | 修改 xgb.xml 中 base_link pos |
| 8 | 启动进程 | 见下方 |

### 2.3 第三层：核心进程启动顺序

```bash
# ① MuJoCo 独立物理（当前配置 mujoco_running=false，不启动）
if $ENABLE_MUJOCO; then
    ./robot_mujoco &
fi

# ② UE 渲染 + 内嵌 MuJoCo（始终启动）
./zsibot_mujoco_ue.sh -game /Game/Maps/SceneWorld -MaxFPS 30 &

# ③ 等待 UE 就绪
sleep 7

# ④ 运动控制器（RL 大脑）
taskset -c 7 ./mc_ctrl r mc_enable=true &
```

---

## 3. 三大核心进程职责

### 3.1 zsibot_mujoco_ue（UE 进程）

**路径**: `src/UeSim/Linux/zsibot_mujoco_ue/`

**职责**:
- 内嵌 MuJoCo 物理引擎（mujoco_running=false 时物理在 UE 内运行）
- 3D 渲染（30FPS）
- 传感器仿真（相机、深度图、LiDAR）
- 通过 UDP 端口与 mc_ctrl 通信

**通信端口**（来自 config.json）:
```json
"state_port": 25001,   // UE → mc_ctrl: 发送机器人状态
"cmd_port": 25002      // mc_ctrl → UE: 接收关节控制指令
```

**UE 内部做的事**:
```
每 1ms (1000Hz):
  1. 从 UDP :25002 读取 12 个关节目标角度
  2. PD 控制: torque = Kp × (目标 - 当前) - Kd × 速度
  3. ctrl[i] = clamp(torque, -28, +28)
  4. mj_step() → 物理仿真
  5. 更新渲染网格
```

**关键**: UE **不做任何运动决策**，只是一个"执行器"。

### 3.2 mc_ctrl（运动控制大脑）

**路径**: `src/robot_mc/build/export/mc/bin/mc_ctrl`

**职责**:
- 接收速度指令 (vx, vy, vyaw) 来自 sim_launcher
- 运行 RL 策略网络推理（加密 ONNX）
- MPC 轨迹优化
- WBC 全身控制
- 输出 12 个关节目标角度 → UDP :25002

**控制频率**（来自 simulator-defaults.yaml）:
```yaml
dynamics_dt   : 0.001    # 物理仿真 1000Hz
high_level_dt : 0.002    # RL 策略推理 500Hz
low_level_dt  : 0.0002   # PD 力矩环 5000Hz
```

**PD 增益**（来自 xg-user-parameters.yaml）:
```yaml
# RL 行走模式
FSM_RL_ABAD_Kp : 20.0    # ABAD 关节 Kp
FSM_RL_HIP_Kp  : 20.0    # HIP 关节 Kp
FSM_RL_KNEE_Kp : 20.0    # KNEE 关节 Kp
FSM_RL_Kd      : 0.7     # 所有关节 Kd

# 站立/动作模式
FSM_jointPD_Kp : 80       # 站立 Kp
FSM_jointPD_Kd : 1.0      # 站立 Kd

# 平衡站立
FSM_RLBalanceStand_Kp : 20.0
FSM_RLBalanceStand_Kd : 0.7
```

**RL 策略网络**（来自 rl_onnx_config.yaml，XG 机器人）:
```
policy_mix_walk       ← 正常行走（WASD）
policy_balancestand   ← 平衡站立
policy_backflip       ← 后空翻
policy_jump           ← 跳跃
policy_upright        ←  upright 恢复
policy_gait_walk      ← 慢速步态
policy_moonwalk       ← 太空步
policy_walkpos        ← 位置控制行走
policy_ik             ← IK 控制
policy_crawl          ← 匍匐
policy_stair          ← 爬楼梯
policy_mlp            ← MLP 基础策略
```

每个策略都是**加密的 ONNX 神经网络文件**，无法查看内部结构。

### 3.3 robot_mujoco（独立物理进程，可选）

**路径**: `src/robot_mujoco/simulate/build/robot_mujoco`

**当前状态**: config.json 中 `mujoco_running: false` → **不启动**

当 `mujoco_running: true` 时：
- 物理仿真从 UE 内部移到独立进程
- UE 变为纯渲染器
- 通过共享内存/UDP 交换物理状态

---

## 4. 按键控制完整数据流

```
用户按下 W 键
     │
     ▼
┌─────────────────────────────────────────┐
│ sim_launcher.bin                         │
│ XGrabKey 全局拦截（UE 收不到按键！）      │
│ 转换: W → vx=+1.0 m/s                  │
│       A → vy=+1.0 m/s                  │
│       Q → vyaw=+1.0 rad/s              │
│ 发送速度指令 → mc_ctrl                   │
└────────────────────┬────────────────────┘
                     │ (ROS2/zenoh 或 UDP)
                     ▼
┌─────────────────────────────────────────┐
│ mc_ctrl (500Hz)                          │
│                                          │
│ 输入向量 (~50维):                         │
│   ├── 12 关节角度 (从 UE :25001)         │
│   ├── 12 关节速度                        │
│   ├── 身体姿态 (IMU 四元数)              │
│   ├── 身体角速度                         │
│   ├── 上一步动作                         │
│   ├── 速度指令 (vx, vy, vyaw)           │
│   └── 步态相位                           │
│                                          │
│ RL 推理:                                 │
│   action = policy_network(observation)   │
│   → 12 个关节目标角度                    │
│                                          │
│ 发送 → UDP :25002                        │
└────────────────────┬────────────────────┘
                     │
                     ▼
┌─────────────────────────────────────────┐
│ UE (zsibot_mujoco_ue)                    │
│                                          │
│ 接收 12 个目标角度                       │
│ PD 控制 (5000Hz):                        │
│   torque = 20×(target - current)         │
│          - 0.7 × velocity                │
│   ctrl = clamp(torque, -28, +28)        │
│                                          │
│ MuJoCo 物理 (1000Hz):                    │
│   mj_step(model, data)                   │
│                                          │
│ 渲染 (30FPS):                            │
│   更新网格 → 输出画面                    │
└─────────────────────────────────────────┘
```

---

## 5. 与 CarlaUnreal MuJoCoUE 的对比

### 5.1 架构对比

```
Matrix 架构（3 进程）:
┌──────────┐    ┌──────────┐    ┌──────────┐
│sim_launch│───→│ mc_ctrl  │───→│    UE    │
│(输入)    │    │(RL大脑)  │    │(物理+渲染)│
└──────────┘    └──────────┘    └──────────┘

CarlaUnreal 架构（1 进程）:
┌─────────────────────────────────────────┐
│              UE (全部在一个进程)          │
│  键盘输入 → 手写IK → PD → MuJoCo → 渲染 │
└─────────────────────────────────────────┘
```

### 5.2 逐层对比

| 层级 | Matrix | CarlaUnreal MuJoCoUE | 差距 |
|------|--------|---------------------|------|
| **输入** | XGrabKey 全局钩子 | UE PlayerInput | 无本质差异 |
| **运动规划** | RL 神经网络 (百万参数) | 手写 IK 公式 (几行代码) | **巨大** |
| **状态感知** | IMU + 关节反馈 + 接触 | 仅关节角度 | **大** |
| **协调性** | 12 关节联合输出 | 12 关节独立计算 | **大** |
| **PD 增益** | Kp=20, Kd=0.7 (柔顺) | Kp=150, Kd=2.0 (僵硬) | **大** |
| **物理引擎** | MuJoCo (相同 xgb.xml) | MuJoCo (相同 xgb.xml) | 无差异 |
| **渲染** | UE (预打包二进制) | UE (源码编译) | 无差异 |

### 5.3 为什么 CarlaUnreal 控制不好？

**根本原因：缺少 mc_ctrl 这一层"大脑"**

具体表现：

| 问题 | 原因 |
|------|------|
| 行走翻滚 | Kp=150 → 力矩饱和(±28Nm) → Bang-Bang 控制 |
| 各腿不协调 | IK 独立计算每条腿，无全身耦合 |
| 无法抗扰动 | 无 IMU 反馈，不知道身体在倾斜 |
| 横移漂移 | 无接触感知，不知道哪只脚着地 |
| 动作僵硬 | 高增益 PD 无法柔顺跟踪 |

**Matrix 的 RL 策略解决了什么**:
1. **协调性**: 网络同时输出 12 个关节角，天然耦合
2. **平衡**: 输入含 IMU，身体歪了自动补偿
3. **柔顺**: Kp=20 低增益，力矩在线性区，动作平滑
4. **适应**: 根据实时状态调整，不是固定轨迹
5. **接触**: 知道哪只脚着地，只在支撑相施力

---

## 6. 关键配置参数汇总

### 6.1 控制频率

| 参数 | 值 | 来源 |
|------|-----|------|
| 物理仿真 | 1000 Hz (dt=0.001) | simulator-defaults.yaml |
| RL 推理 | 500 Hz (dt=0.002) | simulator-defaults.yaml |
| PD 力矩 | 5000 Hz (dt=0.0002) | simulator-defaults.yaml |
| UE 渲染 | 30 FPS | run_sim.sh (-MaxFPS 30) |

### 6.2 运动参数

| 参数 | 值 | 说明 |
|------|-----|------|
| body_height | 0.32 m | 站立身体高度 |
| leg_height | 0.1 m | 抬腿高度 |
| gait_period_time | 0.2 s | 步态周期 |
| cmpc_x_vel | 3.0 m/s | 最大前进速度 |
| cmpc_y_vel | 1.0 m/s | 最大横移速度 |
| cmpc_yaw_vel | 3.0 rad/s | 最大转向速度 |

### 6.3 PD 增益对比

| 模式 | Matrix Kp | Matrix Kd | CarlaUnreal Kp | CarlaUnreal Kd |
|------|-----------|-----------|----------------|----------------|
| 站立 | 80 | 1.0 | 150 | 2.0 |
| RL 行走 | 20 | 0.7 | 150 (未切换!) | 2.0 |
| 平衡站立 | 20 | 0.7 | - | - |

### 6.4 通信端口

| 端口 | 方向 | 内容 |
|------|------|------|
| 25001 | UE → mc_ctrl | 机器人状态 (qpos, qvel, IMU) |
| 25002 | mc_ctrl → UE | 关节目标角度 (12维) |
| 43988 | mc_ctrl → SDK | 状态转发 (RoamerX) |
| 43997 | SDK → mc_ctrl | 高层指令 (RoamerX) |

---

## 7. 结论

### 7.1 为什么 Matrix "只启动 UE" 就能丝滑移动？

**不是"只启动 UE"**。实际启动了 3 个进程：
- UE 只是"肌肉 + 眼睛"（执行 + 渲染）
- mc_ctrl 是"小脑"（运动决策）
- sim_launcher 是"手指"（输入捕获）

用户只看到一个 UE 窗口，但后台有 mc_ctrl 在以 500Hz 运行 RL 推理。

### 7.2 CarlaUnreal 控制不好的根本原因

**是的，就是因为缺少 mc_ctrl 这一层。**

你在 UE 内部用按键 → 手写 IK → PD → MuJoCo 的方式，相当于：
- 跳过了 RL 策略（没有"大脑"）
- 用 Kp=150 做行走（Matrix 用 Kp=20）
- 12 个关节各自为政（无协调）
- 无姿态/接触反馈（无感知）

这不是"调参"能解决的问题——是**架构层面的缺失**。

### 7.3 可能的改进方向

| 方案 | 难度 | 效果 | 说明 |
|------|------|------|------|
| 降低行走 Kp (150→30) | 低 | 中 | 减少力矩饱和 |
| 添加 IMU 姿态反馈 | 中 | 中 | 倾斜补偿 |
| 添加接触感知 | 中 | 中 | 只在着地时推 |
| 接入 mc_ctrl | 高 | 高 | 复用 Matrix 的 RL |
| 自训练 RL 策略 | 极高 | 极高 | 需要 Isaac Gym 等 |

---

## 附录：文件路径索引

| 文件 | 路径 |
|------|------|
| 启动脚本 | `/home/qiyuan/Softwares/Matrix/bin/sim_launcher` |
| 编排脚本 | `/home/qiyuan/Softwares/Matrix/scripts/run_sim.sh` |
| UE 启动 | `/home/qiyuan/Softwares/Matrix/src/UeSim/Linux/zsibot_mujoco_ue.sh` |
| MC 启动 | `/home/qiyuan/Softwares/Matrix/src/robot_mc/run_mc.sh` |
| 全局配置 | `/home/qiyuan/Softwares/Matrix/config/config.json` |
| PD 参数 | `/home/qiyuan/Softwares/Matrix/src/robot_mc/build/export/config/xg-user-parameters.yaml` |
| RL 模型配置 | `/home/qiyuan/Softwares/Matrix/src/robot_mc/build/export/config/rl_onnx_config.yaml` |
| 仿真频率 | `/home/qiyuan/Softwares/Matrix/src/robot_mc/build/export/config/simulator-defaults.yaml` |
| MuJoCo 配置 | `/home/qiyuan/Softwares/Matrix/src/robot_mujoco/simulate/config.yaml` |
| 机器人模型 | `/home/qiyuan/Softwares/Matrix/src/robot_mujoco/zsibot_robots/xgb/xgb.xml` |





## 三个项目分析

### 1. QTPAD — 虚拟手柄生成器

```
┌─────────────────────────────────────────┐
│  QTPAD (Qt5 桌面应用)                    │
│                                         │
│  鼠标拖拽圆圈 → 模拟摇杆值              │
│       │                                 │
│       ▼                                 │
│  VirtualXboxController                  │
│  ├── /dev/uinput (Linux 虚拟设备)       │
│  ├── setLeftStick(x, y)                │
│  ├── setRightStick(x, y)               │
│  ├── tapButton(code)                   │
│  └── tapTrigger(axis, value)           │
│       │                                 │
│       ▼                                 │
│  系统识别为 "Xbox 360 手柄"             │
│  → 任何应用（包括 UE）都能读到           │
└─────────────────────────────────────────┘
```

**核心能力**: 通过 `uinput` 创建虚拟 Xbox 手柄，程序化注入摇杆/按键事件。

### 2. genisom_roamerx_open — 导航遥控 SDK

```
┌─────────────────────────────────────────────────────┐
│  RoamerX Lite (ROS2 导航栈)                          │
│                                                     │
│  输入层:                                            │
│  ├── gamepad_lcmt (LCM) ← 手柄摇杆值               │
│  ├── HighCmd.msg ← velocity[2], yaw_speed          │
│  └── HighLevelCmd.msg ← cmd_vel (vx, vy, vyaw)    │
│                                                     │
│  通信层:                                            │
│  ├── UDP (target_ip:43988)                         │
│  ├── LCM (gamepad_lcmt)                            │
│  └── ROS2 Topic (zenoh 中间件)                     │
│                                                     │
│  功能层:                                            │
│  ├── SLAM 建图                                     │
│  ├── NavFn 路径规划                                 │
│  ├── MPPI 控制器                                    │
│  └── 行为树 (自主导航)                              │
└─────────────────────────────────────────────────────┘
```

**核心能力**: 把遥控指令（手柄/键盘）→ 速度命令 → 发给 mc_ctrl 驱动机器人。

### 3. Matrix sim_launcher — 输入拦截 + 进程管理

```
┌─────────────────────────────────────────────────────┐
│  sim_launcher.bin (Qt/QML)                          │
│                                                     │
│  输入拦截:                                          │
│  ├── XGrabKey (X11 全局键盘钩子)                    │
│  │   → W/A/S/D/Q/E/Space 被截获                    │
│  │   → UE 窗口完全收不到这些按键！                   │
│  ├── 虚拟罗技 F710 手柄 (uinput)                   │
│  │   → GamepadForwarder 转发 /dev/input/js0        │
│  └── 键盘/手柄模式切换                              │
│                                                     │
│  指令转发:                                          │
│  └── 速度指令 → mc_ctrl (ROS2/zenoh)               │
└─────────────────────────────────────────────────────┘
```

---

## 你能实现类似的输入拦截吗？

**能。有三种方案：**

### 方案 A：XGrabKey 全局键盘钩子（Matrix 方式）

```cpp
// 原理：X11 API 在窗口管理器层面截获按键
Display* dpy = XOpenDisplay(nullptr);
Window root = DefaultRootWindow(dpy);

// 抓取 W 键（keycode=25），全窗口生效
XGrabKey(dpy, XKeysymToKeycode(dpy, XK_w), 
         None, root, True, GrabModeAsync, GrabModeAsync);

// 事件循环中读取
XEvent ev;
XNextEvent(dpy, &ev);
if (ev.type == KeyPress) {
    // W 被按下 → 发送 vx=1.0 给你的控制器
}
```

**优点**: 和 Matrix 完全一致，UE 收不到按键  
**缺点**: 需要独立 X11 事件循环，与 Qt/UE 主循环冲突需处理

### 方案 B：虚拟手柄注入（QTPAD 方式）

```cpp
// 原理：创建虚拟 Xbox 手柄，UE 自动识别
VirtualXboxController pad;  // 来自 QTPAD

// 按 W 时：
pad.setLeftStick(0, 32767);   // 左摇杆推满 = 前进
// 松开 W 时：
pad.setLeftStick(0, 0);       // 回中
```

**优点**: UE 原生支持手柄输入，无需修改 UE 代码  
**缺点**: 需要 UE 配置手柄映射；模拟量不如键盘精确

### 方案 C：UDP 外部指令（RoamerX 方式）

```python
# 独立 Python 进程，监听键盘 → UDP 发给 UE
import socket, keyboard

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

while True:
    vx = 1.0 if keyboard.is_pressed('w') else 0.0
    vy = 1.0 if keyboard.is_pressed('a') else 0.0
    vyaw = 1.0 if keyboard.is_pressed('q') else 0.0
    
    data = struct.pack('fff', vx, vy, vyaw)
    sock.sendto(data, ('127.0.0.1', 25002))  # 发给 UE
    time.sleep(0.002)  # 500Hz
```

**优点**: 最简单，UE 端只需加个 UDP 监听  
**缺点**: 需要修改 UE 代码接收 UDP

---

## 但是——拦截输入不是你的核心问题

```
Matrix:  按键 → [拦截] → mc_ctrl(RL) → 12个协调关节角 → UE(PD) → 丝滑
                                         ↑
你的:    按键 → [拦截?] → 手写IK → 12个独立关节角 → UE(PD) → 不稳
                                         ↑
                                    问题在这里，不在输入！
```

**即使你完美复刻了 sim_launcher 的输入拦截，如果后面接的还是手写 IK，效果不会变好。**

输入拦截只解决"谁来发指令"的问题，不解决"怎么把指令变成协调动作"的问题。

### 真正有价值的方向

| 优先级 | 方向                       | 效果                 |
| ------ | -------------------------- | -------------------- |
| 1      | 降低行走 Kp (150→30)       | 立即减少翻滚         |
| 2      | 加 IMU 姿态反馈补偿        | 抗倾斜               |
| 3      | 接入 mc_ctrl（UDP :25002） | **直接获得 RL 效果** |
| 4      | 输入拦截（锦上添花）       | 架构更清晰           |

**方案 3 是最有前途的**：你的 UE 已经有 MuJoCo 物理，只需要：
1. 加一个 UDP 监听（接收 12 个关节目标）
2. 启动 Matrix 的 mc_ctrl 进程
3. 让 mc_ctrl 通过 UDP 发指令给你的 UE

这样你就能**直接复用 Matrix 的 RL 策略**，不需要自己训练。需要我帮你实现这个 UDP 通信接口吗？



现在我完全理解了协议。**完全可以！而且这正是 RoamerX SDK 的做法。**

让我看看 mc_ctrl 的完整通信协议：


现在我完全搞清楚了协议。**完全可以，而且这正是 RoamerX SDK 的做法！**

## 完整架构

```
Python (键盘) ──UDP:43997──→ mc_ctrl ──UDP:25002──→ UE (CarlaUnreal)
                               ↑                        │
                               └────UDP:25001────────────┘
                                     (状态反馈)
```

## 关键发现：mc_ctrl 用的是二进制结构体，不是 JSON

从 [common.h](file:///home/qiyuan/Softwares/genisom_roamerx_open/src/navigation/src/robot_navigo/udp/include/common.h) 看到的协议：

**Python → mc_ctrl (port 43997)：`highLevelCmd` 36字节**
```c
struct highLevelCmd {
  uint16_t head = 0x5AA5;       // 魔术头
  uint16_t len = 36;            // 结构体大小
  float vx, vy;                 // 速度指令
  float roll_rate, pitch_rate, yaw_rate;  // 角速度
  float height_rate;            // 体高变化率
  uint16_t motion_mode;
  uint16_t control_mode = 18;   // K_RL_MIX = RL行走
  uint16_t enable_control_mode = 1;
  unsigned int checksum;
};
```

**mc_ctrl → UE (port 25002)：`robotCmd` 208字节**
```c
struct robotCmd {
  uint16_t head, len;
  float q_des_abad[4], q_des_hip[4], q_des_knee[4];   // 12关节目标角
  float qd_des_abad[4], qd_des_hip[4], qd_des_knee[4]; // 12关节目标速度
  float kp_abad[4], kp_hip[4], kp_knee[4];             // Kp
  float kd_abad[4], kd_hip[4], kd_knee[4];             // Kd
  float tau_abad_ff[4], tau_hip_ff[4], tau_knee_ff[4]; // 前馈力矩
  unsigned int checksum;
};
```

**UE → mc_ctrl (port 25001)：`robotState` 状态反馈**
```c
struct robotState {
  uint16_t head, len;
  float q_abad[4], q_hip[4], q_knee[4];     // 12关节当前角
  float qd_abad[4], qd_hip[4], qd_knee[4];  // 12关节当前速度
  float tau_abad_fb[4], ...;                  // 力矩反馈
  float quat[4], gyro[3], acc[3], rpy[3];    // IMU
  float time_stamp, desire_vel_x/y, desire_yaw_rate;
  uint16_t in_rl_state;
  unsigned int checksum;
};
```

## 你需要做的事

| 序号 | 工作               | 说明                                            |
| ---- | ------------------ | ----------------------------------------------- |
| 1    | Python 键盘脚本    | struct.pack 发送 highLevelCmd → 127.0.0.1:43997 |
| 2    | UE 接收 robotCmd   | 端口 25002，**二进制解析**（不是 JSON！）       |
| 3    | UE 发送 robotState | 端口 25001，500Hz 发回关节状态                  |
| 4    | 启动 mc_ctrl       | `taskset -c 7 ./mc_ctrl r mc_enable=true`       |

**重要**：mc_ctrl 发给 UE 的是二进制 `robotCmd`（208字节），不是 JSON。所以我们的 `UdpReceiverComponent` 需要改为解析二进制结构体。同时还需要一个 `UdpSenderComponent` 把状态发回给 mc_ctrl。

需要我：
1. 写 Python 键盘控制脚本？
2. 把 UdpReceiverComponent 改为解析二进制 robotCmd？
3. 创建 UdpSenderComponent 发送 robotState？





好的，全部实现。让我先重写 `UdpReceiverComponent` 来解析二进制 `robotCmd`，然后创建 `UdpSenderComponent`，最后写 Python 脚本。


先重写头文件，定义与 Matrix 协议完全匹配的二进制结构体：


现在重写 cpp 实现，解析二进制协议：


现在创建 `UdpSenderComponent`，发送 `robotState` 到 mc_ctrl:25001：


现在检查 Build.cs 依赖并编译：


依赖已经齐全（Sockets, Networking）。Json 不再需要但留着无害。直接编译：


零错误！确认编译成功：


编译通过。现在写 Python 键盘控制脚本：


验证 Python 结构体大小是否正确：


结构体大小不对！Python 的 `struct` 模块在 `I` 前自动加了 padding。C++ `alignas(4)` 结构体实际是 36 字节（3个 uint16 紧挨着无 padding）。修复：


需要确认 C++ 结构体的真实大小。`alignas(4)` 不消除内部 padding，uint32 仍需 4 字节对齐：


确认了实际大小：
- `highLevelCmd`: **40 字节**（offset 34-35 有 2 字节 padding）
- `robotCmd`: **248 字节**
- `robotState`: **224 字节**

我的 UE C++ 结构体用了 `#pragma pack(4)` 与自然对齐结果一致。现在修复 Python 脚本的 padding 问题：


验证修复后的 Python 打包：


40 字节，完美匹配 C++ 结构体。设置脚本可执行权限并完成：


全部完成！以下是完整实现总结：

## 实现的文件

| 文件                                                         | 功能                                                         |
| ------------------------------------------------------------ | ------------------------------------------------------------ |
| [UdpReceiverComponent.h](file:///home/qiyuan/UnrealEngine/CarlaUE5/Unreal/CarlaUnreal/Plugins/MuJoCoUE/Source/MuJoCoUE/Public/UdpReceiverComponent.h) | 接收 mc_ctrl 的二进制 `robotCmd`（248字节），解析 12 关节目标 + PD 增益 + 前馈力矩 |
| [UdpReceiverComponent.cpp](file:///home/qiyuan/UnrealEngine/CarlaUE5/Unreal/CarlaUnreal/Plugins/MuJoCoUE/Source/MuJoCoUE/Private/UdpReceiverComponent.cpp) | 端口 25002 监听，校验 0x5AA5 头，memcpy 解析结构体           |
| [UdpSenderComponent.h](file:///home/qiyuan/UnrealEngine/CarlaUE5/Unreal/CarlaUnreal/Plugins/MuJoCoUE/Source/MuJoCoUE/Public/UdpSenderComponent.h) | 发送 `robotState`（224字节）到 mc_ctrl:25001                 |
| [UdpSenderComponent.cpp](file:///home/qiyuan/UnrealEngine/CarlaUE5/Unreal/CarlaUnreal/Plugins/MuJoCoUE/Source/MuJoCoUE/Private/UdpSenderComponent.cpp) | 500Hz Tick 自动发送，包含关节角/速度/力矩/IMU                |
| [keyboard_control.py](file:///home/qiyuan/UnrealEngine/CarlaUE5/Unreal/CarlaUnreal/Plugins/MuJoCoUE/Scripts/keyboard_control.py) | Python 键盘遥控，WASD/QE → `highLevelCmd`(40字节) → mc_ctrl:43997 |

## 运行时的完整数据流

```
┌─────────────────┐         ┌──────────────┐         ┌──────────────────┐
│  Python 键盘     │  UDP    │   mc_ctrl    │  UDP    │  UE (CarlaUnreal)│
│  keyboard_control│──43997─→│  RL 策略推理  │──25002─→│  UdpReceiver     │
│  W/A/S/D/Q/E    │  40B    │  500Hz       │  248B   │  → 12关节目标     │
└─────────────────┘         │              │         │  → PD → MuJoCo   │
                            │              │←─25001──│  UdpSender       │
                            │              │  224B   │  ← 关节状态+IMU  │
                            └──────────────┘         └──────────────────┘
```

## 启动步骤

```bash
# 1. 启动 UE（确保 MuJoCo 仿真 Actor 上挂了两个组件）

# 2. 启动 mc_ctrl
cd /home/qiyuan/Softwares/Matrix/src/robot_mc/build/export/mc/bin
export RMW_IMPLEMENTATION="rmw_zenoh_cpp"
export ROS_DOMAIN_ID="89"
taskset -c 7 ./mc_ctrl r mc_enable=true

# 3. 启动键盘控制
sudo python3 .../Plugins/MuJoCoUE/Scripts/keyboard_control.py
```

## 下一步集成

在 `AMuJoCoSimulation` 的 Tick 中需要：
1. 从 `UdpReceiverComponent.LastCommand.JointTargets` 读取 12 个目标角
2. 用 `KpValues`/`KdValues` 做 PD 控制 → `mj_setCtrl`
3. 从 MuJoCo 读取当前 qpos/qvel → 调用 `UdpSenderComponent.UpdateState()` 发回

需要我帮你把这两个组件集成到 `MuJoCoSimulation` 中吗？





现在集成到 MuJoCoSimulation。先修改头文件：


现在修改 cpp 文件，集成 UDP 控制逻辑：


现在在文件末尾添加 `ApplyUdpControl()` 和 `SendStateToMcCtrl()` 实现：


编译验证：


零错误！确认编译成功：


全部完成，编译通过。

## 集成总结

`AMuJoCoSimulation` 现在的工作流程：

```
每个物理步 (1000Hz):
  ┌─ UDP 数据新鲜 (<100ms)?
  │   YES → ApplyUdpControl(): mc_ctrl 的 12 关节目标 + Kp/Kd → PD → ctrl[i]
  │   NO  → bStandUpActive? → ApplyStandUpControl(): 本地步态
  │
  ├─ mj_step()
  │
  └─ SendStateToMcCtrl(): qpos/qvel/IMU → robotState → UDP:25001
```

**控制优先级**: `mc_ctrl UDP` > `本地步态` > `被动`

## 完整文件清单

| 文件                                                         | 作用                                              |
| ------------------------------------------------------------ | ------------------------------------------------- |
| [UdpReceiverComponent.h/.cpp](file:///home/qiyuan/UnrealEngine/CarlaUE5/Unreal/CarlaUnreal/Plugins/MuJoCoUE/Source/MuJoCoUE/Public/UdpReceiverComponent.h) | 监听 :25002，解析 248B `robotCmd`                 |
| [UdpSenderComponent.h/.cpp](file:///home/qiyuan/UnrealEngine/CarlaUE5/Unreal/CarlaUnreal/Plugins/MuJoCoUE/Source/MuJoCoUE/Public/UdpSenderComponent.h) | 发送 224B `robotState` → :25001                   |
| [MuJoCoSimulation.h/.cpp](file:///home/qiyuan/UnrealEngine/CarlaUE5/Unreal/CarlaUnreal/Plugins/MuJoCoUE/Source/MuJoCoUE/Public/MuJoCoSimulation.h) | 集成：`ApplyUdpControl()` + `SendStateToMcCtrl()` |
| [keyboard_control.py](file:///home/qiyuan/UnrealEngine/CarlaUE5/Unreal/CarlaUnreal/Plugins/MuJoCoUE/Scripts/keyboard_control.py) | 键盘 → 40B `highLevelCmd` → mc_ctrl:43997         |

## 启动顺序

```bash
# 1. 启动 UE 编辑器/游戏 (UdpReceiver 自动监听 25002, UdpSender 自动发送到 25001)

# 2. 启动 mc_ctrl
cd /home/qiyuan/Softwares/Matrix/src/robot_mc/build/export/mc/bin
export RMW_IMPLEMENTATION="rmw_zenoh_cpp"
export ROS_DOMAIN_ID="89"
taskset -c 7 ./mc_ctrl r mc_enable=true

# 3. 启动键盘遥控
sudo python3 /home/qiyuan/UnrealEngine/CarlaUE5/Unreal/CarlaUnreal/Plugins/MuJoCoUE/Scripts/keyboard_control.py
```

按 W 键后：Python → mc_ctrl(RL推理) → UE(PD+MuJoCo) → 丝滑行走。





需要启动 zenoh 路由器。让我检查并启动：


果然没运行。启动它：


路由器启动了。现在查看 mc_ctrl 的 ROS2 topics：


mc_ctrl 的 topics 不可见——可能是 mc_ctrl 在路由器之前启动的，需要**重启 mc_ctrl**。

但在重启前，让我先确认 mc_ctrl 到底用什么接口接收指令（LCM 还是 ROS2）：


找到了！mc_ctrl 用的是 **LCM**（不是 ROS2，也不是 UDP:43997）：

- `handleGamePadCommand` — 手柄 LCM 消息
- `handleHighLevelCommand` — 高层 LCM 指令
- `gamepad_lcmt` / `desired_cmd_lcmt` — LCM 消息类型

让我找到具体的 LCM channel 名和消息格式：


找到了！mc_ctrl 通过 **LCM 多播**接收指令：

- LCM URL: `udpm://239.255.76.67:7667?ttl=255`
- Channel: `"vel_cmd_lcm_data"`
- 消息: `gamepad_lcmt`（摇杆映射为速度）

让我确认 mc_ctrl 订阅的 channel 和状态标志：


现在清楚了！sim_launcher 通过 **uinput 虚拟手柄** + **LCM** 两条路径控制 mc_ctrl。

mc_ctrl 的 LCM 接口：
- 多播: `239.255.76.67:7667`
- Channel: `"des_vel_cmd"`
- 消息: `gamepad_lcmt`（hash: `0xef429e39e2e19db1`）

我来改写 Python 脚本，用 **LCM UDP 多播**直接发给 mc_ctrl：


LCM 编码验证通过（84字节）。现在改写脚本为 LCM 多播方式：


已改写为 LCM 多播方式。现在运行测试：

```bash
sudo python3 /home/qiyuan/UnrealEngine/CarlaUE5/Unreal/CarlaUnreal/Plugins/MuJoCoUE/Scripts/keyboard_control.py
```

**关键变化：**
- 不再发 UDP:43997（mc_ctrl 没监听）
- 改为 LCM 多播 `239.255.76.67:7667`，channel `"des_vel_cmd"`
- 消息格式：`gamepad_lcmt`（84字节，big-endian）
- 速度映射：`leftStickAnalog[1]`=前进, `[0]`=横移, `rightStickAnalog[0]`=偏航（取反）
- 站立/趴下：`robot_state_flag` = 1/0

如果 mc_ctrl 还是不响应，可能需要确认 `use_gamepad` 参数是否开启。你之前启动 mc_ctrl 的命令是 `./mc_ctrl r mc_enable=true`，也许需要加 `use_gamepad=true`。
