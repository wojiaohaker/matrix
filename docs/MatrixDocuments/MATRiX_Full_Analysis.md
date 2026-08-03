# MATRiX 项目全面深入分析

> **分析日期:** 2026-07-22
> **项目版本:** 0.1.2
> **仓库地址:** https://github.com/zsibot/matrix

---

## 一、项目概述

MATRiX 是一个高级机器人仿真平台，集成了三大核心引擎：

| 引擎 | 角色 | 说明 |
|------|------|------|
| **MuJoCo** | 物理仿真 | 提供高保真物理动力学、接触力学、传感器模拟 |
| **Unreal Engine 5** | 渲染引擎 | 提供照片级真实感渲染、Nanite/Lumen 视觉 |
| **CARLA** | 场景资源 | 提供地图资产、行人/车辆等动态元素 |

**核心架构理念：** Software-in-the-Loop (SiL)，通过 UDP 通信实现各模块间的松耦合，支持 sim-to-real 迁移。

---

## 二、系统架构

### 2.1 整体架构

```
┌─────────────────────────────────────────────────────────────────────────┐
│                         用户层 (User Layer)                              │
│  sim_launcher (GUI/CLI)  │  run_sim.sh  │  check_env.sh  │  Docker     │
├─────────────────────────────────────────────────────────────────────────┤
│                       仿真运行时 (Runtime)                               │
│                                                                         │
│  ┌──────────────┐   UDP 25001/   ┌──────────────┐                     │
│  │  MuJoCo      │   25002        │  UE5         │                     │
│  │  (物理引擎)   │◄──────────────►│  (渲染引擎)   │                     │
│  │  robot_mujoco│   State/Cmd    │  zsibot_ue   │                     │
│  └──────┬───────┘                └──────┬───────┘                     │
│         │ UDP                           │ 读取 config.json             │
│  ┌──────▼───────┐                       │ + scene.json                │
│  │  MC 控制器    │  mc_ctrl              │                             │
│  │  (运动控制)   │  端口: 43988/43997    │                             │
│  └──────────────┘  /25005              │                             │
│                                                                         │
│  ┌──────────────────────────────────────────────────────┐              │
│  │  ROS 2 Humble (可选)                                  │              │
│  │  - TF 发布、传感器话题、Nav2 集成                      │              │
│  │  - RoamerX Lite 导航栈 / robot_forward TF 桥接        │              │
│  └──────────────────────────────────────────────────────┘              │
├─────────────────────────────────────────────────────────────────────────┤
│                       数据层 (Data Layer)                                │
│  config/config.json │ scene/scene.json │ config.yaml                    │
│  deps/*.deb         │ releases/*.tar.gz │ dynamicmaps/                  │
└─────────────────────────────────────────────────────────────────────────┘
```

### 2.2 三大核心进程

| 进程 | 二进制 | 启动条件 | 通信方式 | 职责 |
|------|--------|---------|---------|------|
| **MuJoCo 物理** | `robot_mujoco` | `mujoco_running=1` | UDP 25001(状态)/25002(指令) | 刚体动力学仿真、关节力矩计算、传感器数据生成 |
| **UE5 渲染** | `zsibot_mujoco_ue-Linux-Shipping` | 始终启动 | 读取 config.json/scene.json | 高质量 3D 渲染、传感器模拟（相机/LiDAR）、场景管理 |
| **MC 控制器** | `mc_ctrl` | 非 go2/go2w 机器人 | UDP 43988/43997/25005 | 接收高层指令（行走/跳跃），生成底层关节力矩 |

### 2.3 启动流程

启动顺序：**MuJoCo → (sleep 7s) → UE5 → MC 控制器**

```bash
# run_sim.sh 中的启动代码
cd src/robot_mujoco/simulate/build
LD_LIBRARY_PATH="$(mujoco_ld_library_path)" ./robot_mujoco > robot_mujoco.log 2>&1 &

cd ../../../UeSim/Linux
LD_LIBRARY_PATH="$(ue_ld_library_path)" ./zsibot_mujoco_ue.sh -game "$MAPNAME" \
    -ExecCmds="t.MaxFPS 30" $USE_OFFSCREEN $USE_PIXELSTREAMER > zsibot_mujoco_ue.log 2>&1 &

sleep 7  # 等待 UE5 初始化

cd ../../robot_mc
LD_LIBRARY_PATH="$(mc_ld_library_path)" ./run_mc.sh r 25001 25002 43988 43997 25005 > run_mc.log 2>&1 &
```

`run_sim.sh` 接受 7 个参数：

| 参数 | 含义 | 默认值 |
|------|------|--------|
| `$1` | 机器人类型 (xgb/xgw/zgws/go2/go2w/custom) | xgb |
| `$2` | 场景 ID (0-22) | 1 |
| `$3` | 离屏渲染 (0/1) | 0 |
| `$4` | Pixel Streaming (0/1) | 0 |
| `$5` | MuJoCo 运行 (0/1) | 0 |
| `$6` | 自定义 URDF 路径 | 空 |
| `$7` | 自定义机器人名称 | 空 |

---

## 三、UE5 ↔ MuJoCo 通信协议

### 3.1 UDP 双端口架构

通信基于 **UDP 协议**，使用两个端口：

| 端口 | 名称 | 方向 | 数据内容 | 频率 |
|------|------|------|---------|------|
| **25001** | `state_port` | MuJoCo → UE5 | 机器人状态：关节角度、关节速度、IMU、位姿、速度 | 默认 10Hz |
| **25002** | `cmd_port` | UE5/MC → MuJoCo | 控制指令：关节力矩/位置命令、状态机切换 | 按需发送 |

### 3.2 同步模式

```json
{
    "synchronous_mode": false,
    "synchronous_frequency": 10
}
```

- **`synchronous_mode: false`**（默认）：MuJoCo 和 UE5 各自以自身节奏运行。MuJoCo 以物理仿真步长推进，UE5 以渲染帧率推进（`t.MaxFPS 30`）。状态数据按可用频率发送，不等待对方确认。
- **`synchronous_mode: true`**：MuJoCo 每推进一步，等待 UE5 渲染一帧后再继续。`synchronous_frequency` 控制同步频率（Hz）。物理和渲染严格同步，适合需要精确数据对齐的研究场景。

### 3.3 配置同步机制

启动前，`run_sim.sh` 会将配置同步到两个运行时位置：

```
config/config.json
    ├── → src/UeSim/Linux/zsibot_mujoco_ue/Content/model/config/config.json  (UE5 读取)
    └── → config/config.json  (MuJoCo/MC 读取)

scene/scene.json
    └── → src/UeSim/Linux/zsibot_mujoco_ue/Content/model/SceneLoder/scene.json (UE5 读取)
```

同步过程使用 `jq` 注入运行时参数：

```bash
jq --arg robot_type "$ROBOTTYPE" \
   --arg weapon "$WEAPON" \
   --argjson mujoco_running "$MUJOCO_RUNNING_JSON" \
   '.robot.robot_type = $robot_type
    | .robot.weapon = $weapon
    | .robot.mujoco_running = $mujoco_running
    | .robot.state_port = (.robot.state_port // 25001)
    | .robot.cmd_port = (.robot.cmd_port // 25002)' config/config.json
```

### 3.4 场景同步机制

UE5 和 MuJoCo 使用各自的场景 XML 文件，但通过启动脚本保持同步：

```
MuJoCo 侧: src/robot_mujoco/zsibot_robots/<robot>/<robot>.xml
UE5 侧:    src/UeSim/Linux/zsibot_mujoco_ue/Content/model/<robot>/scene_terrain.xml
```

`run_sim.sh` 中的 `sync_ue_runtime_scene()` 函数确保 UE5 读取正确的场景变体。
`run_custom_urdf.sh` 中的 `sync_runtime_layout()` 函数在自定义机器人导入时同时更新两侧。

场景 ID 与地图的映射关系：

| ID | 场景 XML | UE 地图路径 | 描述 |
|----|---------|------------|------|
| 0 | scene_terrain_custom.xml | /Game/Maps/CustomWorld | 自定义 |
| 1 | scene_terrain_wh.xml | /Game/Maps/SceneWorld | 仓库场景 |
| 2 | scene_terrain_t10.xml | /Game/Maps/Town10World | 大型城镇 |
| 3 | scene_terrain_yard.xml | /Game/Maps/YardWorld | 庭院 |
| 4 | scene_terrain_crowd.xml | /Game/Maps/CrowdWorld | 拥挤场景 |
| 5 | scene_terrain_venice.xml | /Game/Maps/VeniceWorld | 威尼斯 |
| 6 | scene_terrain_house.xml | /Game/Maps/HouseWorld | 三居室 |
| 7 | scene_terrain_rw.xml | /Game/Maps/RunningWorld | 拾币游戏 |
| 8 | scene_terrain_zombie.xml | /Game/Maps/Town10Zombie | 僵尸游戏 |
| 9 | scene_terrain_flat.xml | /Game/Maps/IROSFlatWorld | IROS 平地 |
| 10 | scene_terrain_sloped.xml | /Game/Maps/IROSSlopedWorld | IROS 坡地 |
| 11-12 | scene_terrain_flat25/sloped25.xml | IROS 2025 | IROS 2025 |
| 13 | scene_terrain_office.xml | /Game/Maps/OfficeWorld | 办公楼 |
| 14/16/17 | 3dgs.xml | /Game/Maps/3DGSWorld | 3D 高斯溅射 |
| 15 | scene_terrain_moon_dynamic.xml | /Game/Maps/MoonWorld | 月球 (动态地形) |
| 20 | scene_terrain_cali.xml | /Game/Maps/CaliWorld | 校准场 |
| 21 | scene_terrain_apart2.xml | /Game/Maps/ApartmentWorld | 公寓 |
| 22 | scene_terrain_meet.xml | /Game/Maps/MeetRoomWorld | 会议室 |

---

## 四、通信数据流详解

### 4.1 State 数据流（MuJoCo → UE5，端口 25001）

MuJoCo 物理引擎每个仿真步计算后，向 UE5 广播以下状态数据：

```
┌─────────────────────────────────────────────────────┐
│              State Packet (UDP 25001)               │
├─────────────────────────────────────────────────────┤
│  关节数据:                                           │
│    - 各关节角度 (joint positions)                    │
│    - 各关节速度 (joint velocities)                   │
│                                                     │
│  IMU 数据:                                          │
│    - 加速度 (body_acc_x/y/z)  单位: m/s²            │
│    - 角速度 (body_gyro_x/y/z) 单位: rad/s           │
│                                                     │
│  位姿数据:                                           │
│    - 世界坐标位置 (pos_world_x/y/z) 单位: m          │
│    - 姿态角 (roll/pitch/yaw)         单位: rad       │
│                                                     │
│  速度数据:                                           │
│    - 世界坐标系速度 (world_vel_x/y/z) 单位: m/s      │
│    - 机体坐标系速度 (body_vel_x/y/z)   单位: m/s     │
└─────────────────────────────────────────────────────┘
```

UE5 接收到状态数据后执行：
1. **骨骼动画驱动**：关节角度 → 机械狗模型网格变形
2. **位姿更新**：更新机器人在场景中的位置和朝向
3. **里程计发布**：发布 `/odom/mujoco_odom` 供导航栈使用
4. **传感器渲染**：相机图像 → `/image_raw/compressed`，LiDAR → `/livox/lidar`

### 4.2 Command 数据流（→ MuJoCo，端口 25002）

控制指令来源取决于运行模式：

| 模式 | 控制链路 | 延迟 |
|------|---------|------|
| **手柄/键盘** | 手柄/键盘 → UE5 → MC → UDP 25002 → MuJoCo | ~10ms |
| **导航栈** | Nav2 目标点 → 路径规划 → velocity cmd → MC → UDP 25002 → MuJoCo | ~50ms |
| **外部算法** | Python/C++ 算法 → UDP 25002 → MuJoCo（直接关节力矩） | <1ms |

### 4.3 HighLevel API 控制接口

项目提供封装好的 HighLevel API（C++ / Python），通过 UDP 与 MC 通信：

**控制函数：**

| 函数 | 说明 | 参数/范围 |
|------|------|----------|
| `initRobot(local_ip, local_port, dog_ip, dog_port)` | 建立通信 | 默认 dog_ip="192.168.234.1" |
| `standUP()` | 站立 | 无 |
| `lieDown()` | 趴下 | 无 |
| `passive()` | 被动模式（阻尼） | 无 |
| `move(vx, vy, yaw_rate)` | 速度控制 | vx: ±0.4~3.0 m/s, vy: ±0.2~2.0 m/s, yaw: ±2.0 rad/s |
| `jump()` | 原地跳 | 无 |
| `frontJump()` | 前跳 | 无 |
| `backflip()` | 后空翻 | 无 |
| `attitudeControl(yaw, roll, pitch, height)` | 姿态控制 | 各轴 ±0.5 rad/s, 高度 ±0.5 m/s |

**状态查询函数：**

| 函数 | 返回值 | 坐标系 |
|------|--------|--------|
| `getRoll/Pitch/Yaw()` | 姿态角 (rad) | 开机原点 Z 轴 |
| `getPosWorldX/Y/Z()` | 位置 (m) | 世界坐标（前X/左Y/上Z） |
| `getWorldVelX/Y/Z()` | 速度 (m/s) | 世界坐标 |
| `getBodyVelX/Y/Z()` | 速度 (m/s) | 机体坐标（前X/左Y/上Z） |
| `getBodyAccX/Y/Z()` | 加速度 (m/s²) | 机体坐标 |
| `getBodyGyroX/Y/Z()` | 角速度 (rad/s) | 机体坐标 |

---

## 五、控制流拓扑：谁在控制机械狗？

### 5.1 模式 A：默认架构 — MC 控制 + MuJoCo 物理 + UE5 显示

**默认模式下确实是"UE 里面控制，MuJoCo 同步"**：

```
┌──────────────────────────────────────────────────────────────────────┐
│                    默认控制流（mujoco_running=true）                   │
│                                                                      │
│  ┌──────────┐    输入事件     ┌──────────┐    UDP 25002    ┌──────┐ │
│  │ 手柄/键盘 │ ─────────────► │   UE5    │ ──────────────► │  MC  │ │
│  └──────────┘                │ (渲染+   │                │mc_ctrl│ │
│                              │  输入)   │                └──┬───┘ │
│                              └──────────┘                   │     │
│                                                             │     │
│                                    UDP 25002                │     │
│                                                             ▼     │
│  ┌──────────┐   关节力矩/     ┌──────────┐   State 25001   │     │
│  │  MuJoCo  │ ◄────────────── │  MuJoCo  │ ◄───────────────┘     │
│  │ (物理引擎) │   位置命令      │ (仿真器)  │   关节状态/IMU/位姿    │
│  └──────────┘                └──────────┘                        │
│       │                                                          │
│       │ ROS 2 Topics                                             │
│       ▼                                                          │
│  ┌──────────┐                                                    │
│  │ 传感器数据 │ → /image_raw, /livox/lidar, /tf                  │
│  └──────────┘                                                    │
└──────────────────────────────────────────────────────────────────────┘
```

**关键点**：
- **MC (mc_ctrl)** 是运动控制核心，接收高层指令（速度/姿态），通过内部 RL 策略或传统控制算法计算关节力矩
- **MuJoCo** 执行关节力矩，计算物理仿真，返回状态
- **UE5** 主要是**显示端**，同时作为手柄/键盘输入入口，将指令转发给 MC

### 5.2 模式 B：MuJoCo 控制 + UE5 纯显示

设置 `mujoco_running=true` 时，MuJoCo 同时承担物理仿真和运动控制角色：

```
┌──────────┐   关节力矩      ┌──────────┐   State 25001   ┌──────────┐
│  MuJoCo  │ ──────────────► │  MuJoCo  │ ──────────────► │   UE5    │
│ (控制器)  │                │ (仿真器)  │                │ (纯显示)  │
└──────────┘                └──────────┘                └──────────┘
     ▲
     │
┌──────────┐
│ 外部算法  │ ─── 直接写入关节力矩 ──► MuJoCo
└──────────┘
```

### 5.3 模式 C：外部算法控制 MuJoCo + UE5 显示（最灵活）

**外部算法通过 UDP 25002 直接向 MuJoCo 发送关节力矩，UE5 自动渲染**：

```
┌──────────┐   UDP 25002    ┌──────────┐   物理仿真    ┌────────┐
│  你的     │ ─────────────► │  MuJoCo  │ ────────────► │ 关节状态│
│  算法    │                │ (仿真器)  │              └───┬────┘
│ (Python/ │  关节力矩/      └──────────┘                  │
│  C++)    │  位置命令             ▲                       │
└──────────┘                      │ State 25001           │
     ▲                            │                       ▼
     │                      ┌──────────┐              ┌──────────┐
     └───────────────────── │  UE5     │ ◄─────────── │  传感器   │
      State 25001           │ (渲染)   │              │  数据     │
                            └──────────┘              └──────────┘
                                  │
                            ROS 2 Topics
                            /image_raw, /livox/lidar
```

**实现方式：**

1. **通过 HighLevel API（推荐）**：
   ```python
   import robot_control
   robot = robot_control.initRobot("127.0.0.1", 25002, "127.0.0.1", 25002)
   robot.standUP()
   robot.move(vx=1.0, vy=0.0, yaw_rate=0.0)
   pos_x = robot.getPosWorldX()
   ```

2. **直接 UDP 通信（底层）**：
   ```python
   import socket, struct
   sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
   cmd_data = struct.pack('fffffffffffffff', *joint_torques)
   sock.sendto(cmd_data, ('127.0.0.1', 25002))
   
   state_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
   state_sock.bind(('127.0.0.1', 25001))
   state_data = state_sock.recv(4096)
   ```

3. **通过 ROS 2 话题（导航级）**：
   ```bash
   ros2 launch robot_navigo navigation_bringup.launch.py \
       platform:=UE mc_controller_type:=RL_TRACK_VELOCITY communication_type:=UDP
   ```

### 5.4 控制模式对比总结

| 模式 | 控制方 | 物理方 | 渲染方 | 适用场景 |
|------|--------|--------|--------|---------|
| **默认（MC 控制）** | MC (mc_ctrl) | MuJoCo | UE5 | 交互式仿真、手柄控制 |
| **MuJoCo 控制** | MuJoCo 内置控制器 | MuJoCo | UE5 | 简化控制链、快速原型 |
| **外部算法控制** | 你的算法 (Python/C++) | MuJoCo | UE5 | RL 训练、自定义控制策略 |
| **导航栈控制** | RoamerX Nav2 | MuJoCo | UE5 | 自主导航、路径规划 |
| **纯 UE 模式** | UE5/MC | 无（UE 内置物理） | UE5 | `mujoco_running=false`，无 MuJoCo |

### 5.5 run_mc.sh 参数传递分析

`run_sim.sh` 调用 MC 时传递了 5 个端口参数：

```bash
./run_mc.sh r 25001 25002 43988 43997 25005
```

| 参数 | 端口 | 用途 |
|------|------|------|
| `$2` | 25001 | state_port — 接收 MuJoCo 状态 |
| `$3` | 25002 | cmd_port — 发送控制指令到 MuJoCo |
| `$4` | 43988 | RoamerX SDK target_port — 导航栈速度指令接收端口 |
| `$5` | 43997 | RoamerX highlevel port — 高层控制指令端口 |
| `$6` | 25005 | 辅助通信端口 |

> **注意**：当前 `run_mc.sh` 中 `./mc_ctrl r` 没有显式传递这些参数。`mc_ctrl` 二进制可能通过 `config.json` 或 `sdk_config.yaml` 读取端口配置。

---

## 六、目录结构详解

```
Matrix/
├── bin/                        # 可执行文件 (sim_launcher 等)
├── config/
│   └── config.json             # 核心配置: 机器人类型、传感器、位姿、端口
├── deps/                       # 本地 .deb 依赖包 (lcm, ecal, mujoco, onnx 等)
├── docs/                       # 文档目录
├── dynamicmaps/                # 动态地图数据 (如月球地形 moonworld.bin)
├── releases/                   # 发布包存储 (tar.gz, manifest, checksums)
├── rviz/
│   └── matrix.rviz             # RViz2 可视化配置
├── scene/
│   ├── scene.json              # 场景元素定义 (行人轨迹、障碍物位置)
│   └── scene_example_*.json    # 场景示例
├── scripts/
│   ├── build.sh                # 一键构建入口
│   ├── build_mc.sh             # 编译 MC 控制器
│   ├── build_navigo.sh         # 编译 ROS 2 导航包
│   ├── check_env.sh            # 环境检查 (6 种模式, 518 行)
│   ├── install_deps.sh         # 系统依赖安装 (151 行)
│   ├── modify_config.sh        # MC 配置修改
│   ├── run_sim.sh              # 仿真启动主脚本 (519 行)
│   ├── run_custom_urdf.sh      # 自定义机器人导入管线 (2031 行)
│   ├── run_roamerx_lite_link.sh# RoamerX 导航栈集成 (381 行)
│   ├── validate_xml_contract.py# XML 合约验证
│   ├── ci/check_repo.py        # CI 仓库检查
│   ├── docker/
│   │   ├── docker_run_gpu.sh   # Docker GPU 启动
│   │   └── entrypoint.sh       # Docker 入口
│   └── release_manager/        # 发布管理工具链
│       ├── common.sh           # 公共函数库 (410 行)
│       ├── install_chunks.sh   # 在线分块安装 (1320 行)
│       ├── install_chunks_local.sh  # 离线本地安装
│       ├── release_pipeline.sh # 端到端发布管线 (462 行)
│       ├── package_assets.sh   # 资产打包
│       ├── split_large_files.sh# 大文件分割 (>2GB)
│       └── upload_to_release.sh# 上传 GitHub Release
├── src/
│   ├── UeSim/Linux/            # UE5 仿真器
│   │   ├── Engine/             # UE5 引擎运行时
│   │   └── zsibot_mujoco_ue/  # UE5 项目 (Content/Paks, model/)
│   ├── robot_mc/               # MC 运动控制器
│   │   ├── build/              # 编译产物 (mc_ctrl)
│   │   └── run_mc.sh           # MC 启动脚本
│   └── robot_mujoco/           # MuJoCo 物理仿真
│       ├── simulate/           # 仿真配置 (config.yaml, build/)
│       └── zsibot_robots/      # 机器人模型 (XML + mesh)
└── VERSION                     # 版本号 (单一事实来源)
```

---

## 七、支持的机器人

| ID | 类型 | 描述 | MC 控制 | 自由度 |
|----|------|------|---------|--------|
| 1 | **xgb** (XG) | 标准四足机器人 | 是 | 12-DOF |
| 2 | **xgw** (XGW) | 轮腿式机器人 | 是 | 16-DOF |
| 3 | **zgws** (ZGWS) | 中型机器狗 | 是 | 16-DOF |
| 4 | **go2** (GO2) | Unitree Go2 四足 | 否 | 12-DOF |
| 5 | **go2w** (GO2W) | Unitree Go2 轮式 | 否 | - |
| 7 | **custom** | 自定义 URDF 导入 | 是 (按 reference_profile) | 可变 |

### 7.1 自定义机器人导入管线

`run_custom_urdf.sh` 是项目中最复杂的脚本（2031 行），实现多阶段流水线：

```
URDF 输入 → 参考轮廓检测 → 网格验证 → 路径归一化
    → URDF→MJCF 转换 (urdf2mjcf)
    → MJCF 归一化 → 视觉网格修复 → 固定链接修复
    → 通用运行时布局恢复 → XML 合约验证
    → 同步到 MuJoCo 和 UE5 双运行时树
```

**关键特性：**
- **缓存机制：** 基于源文件 SHA256 + PIPELINE_VERSION=13 的缓存身份，避免重复转换
- **参考轮廓匹配：** 自动识别 URDF 文件名匹配已知机器人族（xgb/xgw/zg/go2 等）
- **双树同步：** MuJoCo 和 UE5 各有独立的运行时目录树，从同一暂存结果同步
- **网格代理：** 对超大 STL 文件自动生成边界框代理

### 7.2 自定义机器人工作流

```
1. 准备 MuJoCo XML + 网格资源
2. 修改 UE 侧 custom.xml
3. 更新 IMU/传感器定义
4. 同步 custom.xml 和 assets/ 到 MuJoCo 侧
5. 在 launcher 选择 custom 机器人 + CustomWorld
6. 启用 MuJoCo 模式
7. 运行你的 MuJoCo 控制器
8. UE5 自动同步显示 MuJoCo 状态
```

---

## 八、支持的地图/场景

共 **20+ 张地图**，按 Chunk ID 组织为模块化包：

| Chunk ID | 地图名 | 大小 | 描述 |
|----------|--------|------|------|
| 0 | EmptyWorld | - | 空白世界 (基础包内) |
| 11 | SceneWorld | ~423MB | 仓库场景 |
| 12 | Town10World | ~1.1GB | 大型城镇 |
| 13 | YardWorld | ~695MB | 庭院场景 |
| 14 | CrowdWorld | ~60MB | 50 个随机行人的拥挤场景 |
| 15 | VeniceWorld | ~328MB | 威尼斯风格城镇 |
| 16 | RunningWorld | ~36MB | 拾币游戏 |
| 17 | HouseWorld | ~265MB | 三居室公寓 |
| 18 | IROSFlatWorld | ~300KB | IROS 竞赛平地 |
| 19 | IROSSlopedWorld | ~250MB | IROS 竞赛坡地 |
| 20 | Town10Zombie | ~628MB | 僵尸游戏 |
| 21-22 | IROS 2025 | ~148KB | IROS 2025 竞赛地图 |
| 23 | OfficeWorld | ~418MB | 两层办公楼 |
| 24 | CustomWorld | ~22MB | 自定义场景 |
| 25 | 3DGSWorld | ~206MB | 3D 高斯溅射重建地图 |
| 26 | MoonWorld | ~603MB | 月球表面，动态地形 |

**特殊地图：**
- **MoonWorld (ID=15)**：唯一支持动态地形的地图，需要 `dynamicmaps/moonworld.bin`，同时拷贝到 MuJoCo 和 UE5 两侧
- **CrowdWorld (ID=4)**：通过 `scene.json` 定义 50 个动态行人轨迹
- **ZombieWorld (ID=8)**：自动启用武器系统 (`WEAPON="gun"`)

---

## 九、配置系统

### 9.1 核心配置文件

**`config/config.json`** — 运行时配置的单一事实来源：

```json
{
  "robot": {
    "robot_type": "xgb",                    // 机器人类型
    "weapon": "",                            // 武器（僵尸模式自动设为 "gun"）
    "position": { "x": 0, "y": 0, "z": 0 }, // 初始位姿
    "rotation": { "roll": 0, "pitch": 0, "yaw": 0 },
    "mujoco_running": true,                  // MuJoCo 是否启用
    "state_port": 25001,                     // 状态广播端口
    "cmd_port": 25002,                       // 指令接收端口
    "EgoView": true,                         // 相机跟随
    "synchronous_mode": false,               // 同步模式
    "synchronous_frequency": 10,             // 同步频率 (Hz)
    "sensors": {
      "camera": {
        "position": { "x": 29, "y": 0, "z": 1 },
        "rotation": { "roll": 0, "pitch": 15, "yaw": 0 },
        "height": 1080, "width": 1920,
        "sensor_type": "rgb",
        "topic": "/image_raw/compressed",
        "fov": 90, "frequency": 10
      },
      "depth_sensor": {
        "height": 480, "width": 640,
        "sensor_type": "depth",
        "topic": "/image_raw/compressed/depth"
      },
      "lidar": {
        "sensor_type": "mid360",
        "topic": "/livox/lidar",
        "draw_points": false, "frequency": 10
      }
    }
  }
}
```

**`src/robot_mujoco/simulate/config.yaml`** — MuJoCo 仿真配置：

```yaml
robot: "xgb"                           # 当前机器人类型
robot_scene: "scene_terrain_t10.xml"   # 当前场景 XML
print_scene_information: 1             # 打印链路/关节/传感器信息
enable_elastic_band: 0                 # 虚拟弹簧带（用于提升 H1）
```

**`scene/scene.json`** — 场景动态元素定义：

```json
{
    "Element1": {
        "name": "pedestrian1",
        "type": "dynamic",          // dynamic=有轨迹, static=固定
        "model": "human1",
        "position": { "x": 2000, "y": 0, "z": 90 },
        "velocity": 0.5,
        "trajectory": { "point1": { "x": -1000, "y": 0, "z": 0 } }
    }
}
```

### 9.2 配置同步机制

`run_sim.sh` 启动时执行多级配置同步：

```
1. sed 修改 config.yaml → 写入 robot 和 robot_scene
2. jq 更新 config.json → 注入 robot_type, weapon, mujoco_running, state_port, cmd_port
3. cp config.json → UE5 Content/model/config/config.json
4. cp scene.json → UE5 Content/model/SceneLoder/scene.json
5. sync_ue_runtime_scene() → 同步 UE 运行时场景入口 XML
6. sed 修改机器人 XML → 更新 base_link 初始位姿
```

### 9.3 多机器人配置

多机器人场景下，每个机器人使用独立的配置块和端口对：

```json
{
    "robot1": { "state_port": 25001, "cmd_port": 25002 },
    "robot2": { "state_port": 25011, "cmd_port": 25012 }
}
```

| 机器人 | state_port | cmd_port | 偏移规则 |
|--------|-----------|---------|----------|
| Robot 1 | 25001 | 25002 | 基准 |
| Robot 2 | 25011 | 25012 | +10 |
| Robot N | 25001+10(N-1) | 25002+10(N-1) | 按此递增 |

---

## 十、模块化发布系统

### 10.1 分块包架构

项目采用模块化发布系统，将大型仿真资产拆分为独立可下载包：

```
manifest-{version}.json          # 清单文件 (版本、包信息、SHA256)
├── assets-{version}.tar.gz      # 资源包: sim_launcher + 核心二进制 (必需)
├── base-{version}.tar.gz        # 基础包: EmptyWorld + Chunk 0 (必需, ~2.3GB)
├── shared-{version}.tar.gz      # 共享资源: Fab/CARLA 共享资产 (推荐, ~3.3GB)
└── {MapName}-{version}.tar.gz   # 地图包: 按需下载
```

### 10.2 下载与安装流程

`install_chunks.sh` 执行 7 步安装：

```
[0] 刷新 manifest 文件 (SHA256 验证)
[1] 用户选择要安装的地图包
[2] 下载并安装资源包 (assets) → 验证 sim_launcher 完整性
[3] 下载并安装基础包 (base) → 拷贝模型到 robot_mujoco
[4] 下载并安装共享资源包 (shared) → 支持分片合并
[5] 下载并安装选中的地图包
[6] 验证安装 (pakchunk 文件检查)
```

**下载工具优先级：** aria2c (16线程) → axel → wget → curl

**断点续传：** 完整支持，包括 aria2 断点文件检测和跨工具安全回退。

**幂等安装：** 每个包执行三级判断：
1. 运行时文件已存在且完整 → 跳过
2. 已下载且校验通过 → 直接解压
3. 否则 → 下载 → 校验 → 解压

### 10.3 发布管线

`release_pipeline.sh` 端到端 10 步发布流程：

```
[1] UE 分块打包 → [2] 构建发布 tarball → [3] 复制产物到 releases
[4] 打包运行时资产 → [5] 分割大文件 >2GB → [6] 重新生成校验和与清单
[7] 安装系统依赖 (可选) → [8] 本地安装测试 → [9] 环境检查 → [10] 上传 GitHub Release
```

---

## 十一、依赖管理

### 11.1 系统依赖 (`install_deps.sh`)

**APT 包（约 30 个）：**

| 类别 | 包 |
|------|-----|
| 编译工具 | gcc, g++, cmake, protobuf-compiler |
| 图形/窗口 | libglfw3, mesa-common, freeglut3, Qt5 全套 |
| 数学/科学 | libeigen3, libblas, liblapack, libipopt, libhdf5 |
| 机器人 | libopencv, libpcl-common, libyaml-cpp, libboost-all |
| ROS 2 | ros-humble-desktop, image-transport, rmw-zenoh-cpp |

**本地 .deb 包（deps/ 目录）：**

| 包 | 说明 |
|-----|------|
| `lcm_*.deb` | LCM 通信库 |
| `zsibot_common_*.deb` | 项目公共库 |
| `ecal_*.deb` | eCAL 中间件 |
| `mujoco_*.deb` | MuJoCo 物理引擎 |
| `onnx_*.deb` | ONNX Runtime |
| `robot-forward_*.deb` | 机器人转发模块 |

**重要风险点：** 本地 deb 使用 `--allow-downgrades` 安装，可能降级系统已有的更高版本包（如与 CARLA 共享的依赖）。建议：
- 添加版本检测逻辑，安装前检查是否已有更高版本
- 提供 `--skip-existing` 选项
- 优先使用 Docker 隔离

### 11.2 运行时资产依赖

通过 `install_chunks.sh` 下载，不进入系统包管理器，完全隔离在 `src/UeSim/` 和 `bin/` 内。

---

## 十二、环境检查系统

`check_env.sh`（518 行）支持 6 种检查模式：

| 模式 | 检查内容 |
|------|----------|
| `runtime` | 运行时完整检查: 命令、文件、UE 二进制、共享库 (ldd)、ROS 2、显示 |
| `custom` | 自定义 URDF: python3, jq, urdf2mjcf 模块, validate_xml_contract.py |
| `install` | 安装器: tar, gzip, sha256sum, 下载工具 |
| `local-install` | 本地安装: tar, gzip, sha256sum |
| `build` | 构建: gcc, g++, cmake, qmake, protoc |
| `roamerx` | RoamerX: ros2, rviz2, ss |

**共享库检查：** 使用 `ldd` 检测缺失的 `.so` 文件和版本不兼容问题，覆盖 UE 二进制、MuJoCo 二进制和 MC 控制器。

---

## 十三、进程管理

`run_sim.sh` 实现了健壮的进程生命周期管理：

```
启动前:
  1. kill 所有已知模式的旧进程 (TERM)
     模式: robot_mujoco, jszr_mujoco_ue, zsibot_mujoco_ue,
           UnrealGame, UE4Editor, mc_ctrl
  2. 启动看门狗进程 (监控父进程存活)

运行中:
  - 3 个后台进程: MuJoCo, UE5, MC (按条件启动)
  - wait 阻塞等待所有子进程

退出时 (trap cleanup EXIT SIGINT SIGTERM SIGHUP):
  1. 停止看门狗
  2. 调度强制清理 (1秒后 TERM, 再1秒后 KILL)
  3. 优雅关闭所有 PIDS 中的进程
  4. 兜底: kill 所有已知模式进程 (TERM → KILL)
```

---

## 十四、ROS 2 集成

### 14.1 传感器话题

| 传感器 | ROS 话题 | 类型 | 频率 |
|--------|---------|------|------|
| RGB 相机 | `/image_raw/compressed` | compressed Image | 10Hz |
| 深度传感器 | `/image_raw/compressed/depth` | compressed Depth | 10Hz |
| LiDAR | `/livox/lidar` | Livox 点云 | 10Hz |
| 里程计 | `/odom/mujoco_odom` | Odometry | 由 MuJoCo 驱动 |
| TF 变换 | `/tf` | TFMessage | 由 robot_forward 发布 |

### 14.2 RoamerX Lite 导航集成

`run_roamerx_lite_link.sh` 建立完整的导航栈：

```
┌──────────────┐   Nav2 Goal    ┌──────────────┐
│   RViz2      │ ─────────────► │  Nav2 导航栈  │
│  (目标点)     │               │  (路径规划)   │
└──────────────┘               └──────┬───────┘
                                      │ velocity cmd
                                      ▼
                               ┌──────────────┐
                               │  mc_ctrl     │
                               │  (MC 控制器)  │
                               └──────┬───────┘
                                      │ UDP 25002
                                      ▼
                               ┌──────────────┐   State 25001   ┌──────────────┐
                               │  MuJoCo      │ ──────────────► │  UE5         │
                               │  (物理仿真)   │                │  (渲染+TF)    │
                               └──────────────┘                └──────┬───────┘
                                                                      │
                                                               /odom/mujoco_odom
                                                                      │
                                                                      ▼
                                                               ┌──────────────┐
                                                               │robot_forward │
                                                               │ (TF 桥接)    │
                                                               └──────────────┘
```

关键组件：
- **`robot_forward`**：将 MuJoCo 的 `/odom/mujoco_odom` 转换为导航栈需要的 TF 变换
- **`pub_tf`**：发布 MuJoCo 坐标系下的 TF 树
- **`vel_cmd_udp_pub`**：将导航栈的速度指令通过 UDP 发送给 MC
- **`rmw_zenohd`**：Zenoh DDS 中间件守护进程（ROS_DOMAIN_ID=89）

---

## 十五、Docker 支持

通过 `scripts/docker/docker_run_gpu.sh` 在 GPU 容器中运行：

| 特性 | 说明 |
|------|------|
| GPU 直通 | NVIDIA Container Toolkit |
| X11 转发 | `xhost +local:root`，支持 GUI |
| 网络模式 | `--network host`（ROS/MuJoCo/launcher 通信） |
| 工作目录 | 仓库挂载到 `/workspace` |
| 默认镜像 | `zsibot/matrix:latest` |
| 容器名 | `matrix-sim` |

启动流程：
```
host: docker_run_gpu.sh → docker run zsibot/matrix:latest
  → container: entrypoint.sh → source ROS → ./bin/sim_launcher
```

---

## 十六、多机器人通信隔离

多机器人场景下，每个机器人使用独立的端口对和 MC 实例：

| 机器人 | state_port | cmd_port | MC 实例 |
|--------|-----------|---------|--------|
| Robot 1 | 25001 | 25002 | 独立进程 |
| Robot 2 | 25011 | 25012 | 独立进程 |
| Robot N | 25001+10(N-1) | 25002+10(N-1) | 独立进程 |

每个机器人的 MC 实例、MuJoCo 实例、UE5 模型都通过独立端口通信，互不干扰。支持：
- **分散控制**：独立 agent 控制每个机器人
- **集中控制**：单一脚本向多个端口发送命令

---

## 十七、关键设计模式

### 17.1 单一事实来源 (Single Source of Truth)

| 数据 | 来源 |
|------|------|
| 版本号 | `VERSION` 文件 |
| 机器人配置 | `config/config.json` |
| 场景配置 | `scene/scene.json` |
| MuJoCo 场景 | `src/robot_mujoco/simulate/config.yaml` |
| 发布版本 | `common.sh` 中的 `read_project_version()` |

### 17.2 防御性编程

- `set -euo pipefail` 严格模式
- 所有脚本以 `SCRIPT_DIR` / `PROJECT_ROOT` 定位，不依赖 cwd
- 环境变量覆盖机制 (`MATRIX_SKIP_ENV_CHECK`, `SIM_LAUNCHER_FORCE_REIMPORT_CUSTOM_URDF`)
- 中断信号处理 (trap cleanup EXIT SIGINT SIGTERM SIGHUP)

### 17.3 幂等安装

`install_chunks.sh` 对每个包执行三级判断：
1. 运行时文件已存在且完整 → 跳过
2. 已下载且校验通过 → 直接解压
3. 否则 → 下载 → 校验 → 解压

---

## 十八、代码规模统计

| 文件 | 行数 | 复杂度 |
|------|------|--------|
| `run_custom_urdf.sh` | 2031 | 极高 (多阶段管线 + 内嵌 Python) |
| `install_chunks.sh` | 1320 | 高 (多工具降级 + 分片 + 校验) |
| `run_sim.sh` | 519 | 中 (进程管理 + 配置同步) |
| `check_env.sh` | 518 | 中 (多模式检查) |
| `release_pipeline.sh` | 462 | 中 (10 步管线) |
| `run_roamerx_lite_link.sh` | 381 | 中 (导航栈生命周期管理) |
| `common.sh` | 410 | 低 (公共函数库) |
| `install_deps.sh` | 151 | 低 (apt + deb 安装) |

**总计：** 约 5800+ 行 shell 脚本 + 内嵌 Python

---

## 十九、潜在风险与改进建议

### 19.1 依赖冲突风险

`install_deps.sh` 中的本地 `.deb` 包使用 `--allow-downgrades` 安装，与 CARLA 等共享系统依赖的项目可能冲突。建议：
- 添加版本检测逻辑，安装前检查是否已有更高版本
- 提供 `--skip-existing` 选项
- 优先使用 Docker 隔离

### 19.2 配置竞态

`run_sim.sh` 在启动时通过 `sed -i` 和 `jq` 原地修改 `config.json` 和 `config.yaml`，如果多个实例同时启动可能产生竞态。建议：
- 使用临时文件 + 原子 mv
- 或为每个实例生成独立的配置副本

### 19.3 run_mc.sh 参数传递缺失

`run_sim.sh` 向 `run_mc.sh` 传递了 5 个端口参数（25001, 25002, 43988, 43997, 25005），但 `run_mc.sh` 中 `./mc_ctrl r` 没有显式传递这些参数。`mc_ctrl` 可能通过配置文件读取端口，但这些脚本参数可能是预留给后续版本。建议：
- 在 `run_mc.sh` 中显式传递参数给 `mc_ctrl`
- 或移除 `run_sim.sh` 中的多余参数以避免混淆

### 19.4 自定义机器人管线复杂度

`run_custom_urdf.sh` 超过 2000 行，包含大量内嵌 Python。建议：
- 将 Python 处理逻辑提取为独立 `.py` 文件
- 为管线各阶段添加单元测试

### 19.5 硬编码路径

部分脚本中存在硬编码的 IP 地址（如 `192.168.50.40:8081`）和端口号。建议通过配置文件或环境变量统一管理。

---

## 二十、技术栈总结

| 层级 | 技术 |
|------|------|
| 物理引擎 | MuJoCo (Google DeepMind) |
| 渲染引擎 | Unreal Engine 5 (Epic Games) |
| 场景资产 | CARLA |
| 操作系统 | Ubuntu 22.04 |
| 中间件 | ROS 2 Humble, LCM, eCAL, Zenoh DDS |
| 编程语言 | Bash, Python 3, C++ (MuJoCo/MC) |
| 构建系统 | CMake, colcon (ROS 2) |
| 机器学习 | ONNX Runtime |
| 版本管理 | Git, GitHub Releases |
| 容器化 | Docker + NVIDIA Container Toolkit |
| 下载工具 | aria2, axel, wget, curl (多级降级) |
| 包管理 | apt + 自定义分块包系统 |
| 流式传输 | Pixel Streaming (WebSocket) |

---

## 二十一、延迟与性能考量

| 因素 | 影响 | 建议 |
|------|------|------|
| UDP 通信延迟 | 局域网 < 1ms，可忽略 | 使用 localhost |
| MuJoCo 仿真步长 | 通常 1ms，影响物理精度 | 根据机器人调整 |
| UE5 渲染帧率 | `t.MaxFPS 30`，约 33ms/帧 | 可调整上限 |
| 同步模式 | 开启后物理与渲染严格对齐，但降低吞吐量 | 研究需要时开启 |
| 传感器频率 | 默认 10Hz | 按需调整 `frequency` 字段 |
| 多机器人 | 每增加一个机器人增加 MuJoCo 和 UE5 负载 | 关闭次要机器人不必要的传感器 |

---

## 二十二、总结

MATRiX 仿真平台的核心设计理念是**松耦合的 UDP 通信架构**，这带来了四大优势：

1. **独立运行**：UE5 和 MuJoCo 可以独立运行，即使 MuJoCo 崩溃，UE5 仍能渲染
2. **灵活接入**：只要遵循 UDP 协议格式，任何程序都可以成为控制源
3. **多模式共存**：手柄控制、导航栈控制、外部算法控制可以并行运行
4. **渲染与物理分离**：MuJoCo 负责精确物理，UE5 负责高质量渲染，各司其职

**控制流核心结论：**
- 默认模式下，UE5 是交互入口，MC 将高层指令转为关节力矩，MuJoCo 执行物理仿真
- 可以通过 `mujoco_running=true` 让 MuJoCo 承担物理+控制，UE5 退化为纯渲染
- 外部算法可以通过 UDP 25002 直接向 MuJoCo 发送关节力矩，UE5 自动从 25001 获取状态并渲染——**无需任何额外配置，开箱即用**

---

## 二十三、UE5 如何驱动机械狗运动（基于运行日志的深度分析）

> **分析来源：** UE5 最新运行日志 `zsibot_mujoco_ue.log`（2782 行）  
> **运行环境：** Ubuntu 22.04, UE 5.5.4, NVIDIA RTX 4090, Vulkan SM6

### 23.1 UE5 的插件架构

日志揭示了 UE5 项目加载了 **10 个关键插件**：

| 插件名称 | 类型 | 功能 |
|---------|------|------|
| **MuJoCoUE** | 项目插件 | MuJoCo 物理模型解析与 UE5 内部重建 |
| **UDPWrapper** | 项目插件 | UDP 通信封装（接收 MuJoCo 状态数据） |
| **SensorSim** | 项目插件 | 传感器仿真框架（相机、LiDAR） |
| **rclUE** | 项目插件 | ROS 2 客户端库（发布传感器数据） |
| **Protobuf** | 项目插件 | Protocol Buffers 序列化支持 |
| **RealTimeImport** | 项目插件 | 实时 3D 资产导入 |
| **ThreeDGaussians** | 项目插件 | 3D 高斯溅射渲染 |
| **UnrealSplat** | 项目插件 | 3D 高斯溅射 UE 集成 |
| **SunPosition** | 项目插件 | 太阳位置模拟（光照） |
| **JsonBlueprintUtilities** | 项目插件 | JSON 配置读取 |

**关键发现：** MuJoCoUE 插件使 UE5 具备直接解析 MuJoCo XML 的能力，这是实现"渲染与物理同步"的核心。

### 23.2 UE5 加载 MuJoCo XML 模型

日志第 1669-1670 行显示：

```
LogTemp: Warning: Loading model from: model/xgb/scene_terrain_t10.xml
LogTemp: Warning: Loading MuJoCo model from: /home/qiyuan/.../Content/model/xgb/scene_terrain_t10.xml
```

**UE5 在运行时直接读取与 MuJoCo 相同的 XML 文件**，这意味着：
- UE5 不是被动等待 UDP 数据，而是主动构建物理模型的"镜像"
- XML 中定义的 body、mesh、joint 层级被完整解析

### 23.3 Body 层级树重建

日志第 1671-1685 行显示 UE5 创建了 **14 个 body**：

```
Creating body: world (ID 0)
Creating body: base_link (ID 1)
Creating body: FAR_ABAD_LINK (ID 2)
Creating body: FAR_HIP_LINK (ID 3)
Creating body: FAR_KNEE_LINK (ID 4)
Creating body: FBL_ABAD_LINK (ID 5)
Creating body: FBL_HIP_LINK (ID 6)
Creating body: FBL_KNEE_LINK (ID 7)
Creating body: RAR_ABAD_LINK (ID 8)
Creating body: RAR_HIP_LINK (ID 9)
Creating body: RAR_KNEE_LINK (ID 10)
Creating body: RBL_ABAD_LINK (ID 11)
Creating body: RBL_HIP_LINK (ID 12)
Creating body: RBL_KNEE_LINK (ID 13)
```

**层级结构：**
```
world (ID 0)
  └─ base_link (ID 1)  ← 机器人躯干
       ├─ FAR_ABAD_LINK (ID 2)  ← 右前腿 ABAD 关节
       │    └─ FAR_HIP_LINK (ID 3)
       │         └─ FAR_KNEE_LINK (ID 4)
       ├─ FBL_ABAD_LINK (ID 5)  ← 左前腿 ABAD 关节
       │    └─ FBL_HIP_LINK (ID 6)
       │         └─ FBL_KNEE_LINK (ID 7)
       ├─ RAR_ABAD_LINK (ID 8)  ← 右后腿 ABAD 关节
       │    └─ RAR_HIP_LINK (ID 9)
       │         └─ RAR_KNEE_LINK (ID 10)
       └─ RBL_ABAD_LINK (ID 11) ← 左后腿 ABAD 关节
            └─ RBL_HIP_LINK (ID 12)
                 └─ RBL_KNEE_LINK (ID 13)
```

**命名规则：**
- **F** = Front（前腿），**R** = Rear（后腿）
- **A** = Right（右侧），**B** = Left（左侧）
- **R** = Right（右侧），**L** = Left（左侧）
- 每条腿 3 个关节：**ABAD**（外展）、**HIP**（髋关节）、**KNEE**（膝关节）
- 总计 **12 个自由度**（4 腿 × 3 关节）

### 23.4 Mesh 组件与 CPU Read 标记

日志第 1579-1659 行显示 UE5 创建了 **~900 个 mesh**，包括：

**环境 mesh：**
- `floor`（地面）
- `curb1-9`（路沿）
- `ps_Cube1-541`（场景中的障碍物/装饰物）

**机器人 mesh（所有标记为 CPU read）：**
```
BASE_LINK, HEAD1_LINK, HEAD2_LINK
FL_hip_0_LINK, FL_thigh_0_LINK, FL_calf_0_LINK, FL_foot_LINK
FR_hip_0_LINK, FR_thigh_mirror_0_LINK, FR_calf_mirror_0_LINK, FR_foot_LINK
RL_hip_0_LINK, RL_thigh_0_LINK, RL_calf_0_LINK, RL_foot_LINK
RR_hip_mirror_0_LINK, RR_thigh_mirror_0_LINK, RR_calf_mirror_0_LINK, RR_foot_LINK
FAR_ABAD_LINK, FAR_HIP_LINK, FAR_KNEE_LINK, FAR_FOOT_LINK
FBL_ABAD_LINK, FBL_HIP_LINK, FBL_KNEE_LINK, FBL_FOOT_LINK
RAR_ABAD_LINK, RAR_HIP_LINK, RAR_KNEE_LINK, RAR_FOOT_LINK
RBL_ABAD_LINK, RBL_HIP_LINK, RBL_KNEE_LINK, RBL_FOOT_LINK
...
```

**"CPU read" 标记的意义：**
- 这些 mesh 的顶点数据存储在 GPU 显存中，但允许 CPU 访问
- 每帧从 MuJoCo 状态数据更新 mesh 的 transform（位置+旋转）
- 实现"物理驱动渲染"——MuJoCo 计算关节角度 → UE5 更新 mesh transform

### 23.5 UDP 通信与状态接收

日志第 1667-1668 行：

```
LogTemp: UUdpReceiverComponent: UDP receiver stopped and socket closed.
LogTemp: UUdpReceiverComponent: Started listening for UDP messages on port 9999, socket name: UnrealUdpReceiver.
```

**UDP 通信架构：**
- **端口 9999**：UE5 内部 UDP 接收器（可能用于调试或外部控制）
- **端口 25001**：MuJoCo → UE5 状态数据（关节角度、IMU、位姿）
- **端口 25002**：UE5/MC → MuJoCo 控制指令（关节力矩）

**数据流：**
```
MuJoCo (物理仿真)
    ↓ UDP 25001
UDPWrapper 插件 (UUdpReceiverComponent)
    ↓ 解析 Protobuf/JSON
MuJoCoUE 插件 (状态映射)
    ↓ 更新 body transform
UE5 渲染管线 (mesh transform 更新)
    ↓ Vulkan 渲染
屏幕输出
```

### 23.6 传感器仿真与 ROS 2 发布

日志第 2615-2630 行显示 UE5 初始化了 **3 个 ROS 2 节点**：

**1. RGB 相机（BP_RGB_Camera_C）**
```
LogTemp: Warning: URGBCaptureComponent:: Found camera Front!
LogTemp: Warning: Timer period > 0, capturing every 0.100000 seconds!
LogTemp: Published RGB CameraInfo: fx=960.00 fy=960.00 cx=960.00 cy=540.00
```
- 分辨率：1920×1080（fx=960, cx=960 表示焦距和光心）
- 捕获频率：10 Hz（每 0.1 秒一帧）
- 发布话题：`/camera/front/image_raw`, `/camera/front/camera_info`

**2. 深度相机（BP_Depth_Camera_C）**
```
LogTemp: Warning: Timer period > 0, capturing every 0.100000 seconds!
```
- 捕获频率：10 Hz
- 发布话题：`/camera/depth/image_raw`

**3. LiDAR（BP_Mid360_C）**
```
LogTemp: LiDAR Simulator for BP_Mid360_C_2147481406 started with a scan frequency of 10.000000 Hz.
```
- 型号：Mid-360（Livox 中型 3D LiDAR）
- 扫描频率：10 Hz
- 发布话题：`/lidar/point_cloud`

**传感器仿真的实现方式：**
- **SensorSim 插件**：提供相机/LiDAR 仿真框架
- **rclUE 插件**：通过 ROS 2 发布传感器数据
- **UE5 渲染管线**：相机从 UE5 场景渲染图像，LiDAR 基于场景几何生成点云

### 23.7 Game Mode 与运行时逻辑

日志第 1664 行：

```
LogLoad: Game class is 'GM_Test2_C'
```

**GM_Test2_C** 是 UE5 侧的核心 Blueprint 类，负责：
1. 加载 MuJoCo XML 模型（调用 MuJoCoUE 插件）
2. 创建 body 层级树和 mesh 映射
3. 初始化 UDP 接收器（UDPWrapper 插件）
4. 初始化传感器（SensorSim 插件）
5. 初始化 ROS 2 节点（rclUE 插件）
6. 每帧更新：接收 UDP 状态 → 更新 mesh transform → 渲染 → 发布传感器数据

### 23.8 完整的渲染驱动流程

综合日志分析，UE5 驱动机械狗运动的完整流程如下：

```
┌─────────────────────────────────────────────────────────────────┐
│                    启动阶段（一次性）                              │
├─────────────────────────────────────────────────────────────────┤
│ 1. UE5 启动 → 加载插件（MuJoCoUE, UDPWrapper, SensorSim, rclUE）│
│ 2. 加载地图 /Game/Maps/Town10World                               │
│ 3. GM_Test2_C 初始化                                             │
│ 4. 读取 MuJoCo XML → 创建 body 层级树（14 个 body）              │
│ 5. 创建 ~900 个 mesh（环境 + 机器人），标记 CPU read              │
│ 6. 启动 UDP 接收器（端口 9999/25001）                            │
│ 7. 初始化传感器（RGB 相机、深度相机、LiDAR）                     │
│ 8. 初始化 ROS 2 节点（3 个）                                     │
└─────────────────────────────────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────────┐
│                    运行时循环（每帧）                              │
├─────────────────────────────────────────────────────────────────┤
│ 9. MuJoCo 物理仿真（1ms 步长）                                   │
│     - 计算关节力矩 → 更新关节角度                                │
│     - 计算机器人位姿（base_link 位置+姿态）                      │
│    ↓ UDP 25001                                                  │
│ 10. UE5 UDPWrapper 接收状态数据                                  │
│     - 解析关节角度（12 个关节）                                  │
│     - 解析 IMU 数据（加速度+角速度）                             │
│     - 解析位姿（位置+四元数）                                    │
│    ↓                                                            │
│ 11. MuJoCoUE 插件更新 body transform                             │
│     - base_link → 更新躯干位置和朝向                             │
│     - 各关节角度 → 更新 leg mesh 的旋转                          │
│     - 通过 body 层级树传播变换                                   │
│    ↓                                                            │
│ 12. UE5 渲染管线（Vulkan）                                       │
│     - 更新 mesh 顶点位置（CPU read → GPU 上传）                  │
│     - 渲染场景（Nanite/Lumen）                                   │
│     - 输出到屏幕（30 FPS）                                       │
│    ↓                                                            │
│ 13. 传感器仿真                                                   │
│     - RGB 相机：从 UE5 场景渲染图像 → ROS 2 发布                 │
│     - 深度相机：渲染深度图 → ROS 2 发布                          │
│     - LiDAR：基于场景几何生成点云 → ROS 2 发布                   │
│    ↓                                                            │
│ 14. 回到步骤 9（继续下一帧）                                     │
└─────────────────────────────────────────────────────────────────┘
```

### 23.9 关键技术细节

**1. 为什么 UE5 要加载 MuJoCo XML？**
- 为了在 UE5 侧重建与 MuJoCo 完全一致的 body 层级树
- 确保 mesh 命名、ID、层级关系与 MuJoCo 完全匹配
- 这样 UDP 传来的状态数据才能正确映射到对应的 mesh

**2. CPU Read 的性能影响**
- 每帧需要从 GPU 回读 mesh 数据到 CPU（用于物理碰撞检测等）
- 会增加 GPU→CPU 带宽压力，但对于 12-DOF 机器人影响可控
- 日志显示 ~60 个机器人 link mesh 标记为 CPU read

**3. 同步机制**
- 日志显示 `t.MaxFPS 30`，UE5 以 30 FPS 渲染
- MuJoCo 以 1ms 步长运行（1000 Hz）
- 默认 `synchronous_mode: false`，两者异步运行
- UDP 状态数据按 MuJoCo 的可用频率发送（通常 10-100 Hz）

**4. 传感器数据来源**
- 相机/深度图：**UE5 渲染**（不是 MuJoCo 仿真）
- LiDAR 点云：**UE5 场景几何**（射线追踪）
- IMU 数据：**MuJoCo 物理仿真**（通过 UDP 25001 传来）
- 关节状态：**MuJoCo 物理仿真**（通过 UDP 25001 传来）

### 23.10 总结：UE5 驱动机械狗的核心机制

**UE5 不是简单的"渲染器"，而是"物理模型的视觉镜像"。**

核心机制：
1. **模型同步**：UE5 和 MuJoCo 读取同一份 XML，构建相同的 body 层级树
2. **状态驱动**：MuJoCo 计算物理状态 → UDP 传输 → UE5 更新 mesh transform
3. **传感器融合**：UE5 负责高质量视觉渲染（相机/LiDAR），MuJoCo 负责物理仿真（IMU/关节）
4. **松耦合架构**：通过 UDP 通信，两者可以独立运行，即使 MuJoCo 崩溃，UE5 仍能渲染

**优势：**
- ✅ 渲染质量高（UE5 Nanite/Lumen）
- ✅ 物理精度好（MuJoCo 接触力学）
- ✅ 扩展性强（外部算法可通过 UDP 直接控制）
- ✅ 传感器数据真实（UE5 渲染的相机/LiDAR 可直接用于算法测试）

**限制：**
- ⚠️ 异步模式下，渲染和物理可能有延迟（可通过同步模式解决）
- ⚠️ CPU read 标记会略微影响 GPU 性能
- ⚠️ 需要维护两份场景文件（UE5 map + MuJoCo XML），但通过启动脚本保持同步
