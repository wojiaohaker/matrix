# MATRiX 闭源组件分析与二次开发指南

> 最后更新：2026-07-23 | 基于实际文件系统分析

---

## 一、项目源码开放程度总览

| 组件 | 路径 | 开放程度 | 文件数 |
|------|------|:---:|:---:|
| 启动/构建脚本 | `scripts/` | ✅ 完全开源 | 22 |
| 配置文件 | `src/robot_mc/build/export/config/` | ✅ 完全开源 | 21 |
| 机器人模型（MuJoCo XML） | `src/robot_mujoco/zsibot_robots/` | ✅ 完全开源 | 122 |
| 文档 | `docs/` | ✅ 完全开源 | 21 |
| MC 运动控制器 | `src/robot_mc/build/export/mc/bin/` | ❌ 纯二进制 | 12 .so + 4 可执行文件 |
| RL 策略模型 | `src/robot_mc/build/export/onnx_model_crypto/` | ❌ 加密模型 | 100 |
| MuJoCo 仿真器 | `src/robot_mujoco/simulate/build/robot_mujoco` | ❌ 纯二进制 | 1 |
| UE5 渲染引擎 | `src/UeSim/Linux/` | ❌ 打包二进制 | 184 .so |
| sim_launcher | `bin/sim_launcher.bin` | ❌ 纯二进制 | 1 |
| robot_forward（TF 桥接） | `/opt/robot/robot-forward/` | ❌ .deb 安装 | 闭源 |

---

## 二、各闭源组件详细分析

### 2.1 MC 运动控制器（`mc_ctrl`）

**路径：** `src/robot_mc/build/export/mc/bin/mc_ctrl`

**作用：**
- 接收 UDP 速度命令（来自 RoamerX 导航栈的 `vel_cmd_udp_publisher`）
- 运行 RL 策略推理（ONNX 模型）
- 输出 12 个关节力矩/位置给 MuJoCo
- 管理运动模式切换（站立、行走、跳跃等）

**依赖库（均为闭源 .so）：**

| 库文件 | 推测功能 |
|--------|----------|
| `librobot.so` | 机器人运动学/动力学模型 |
| `libWBC.so` / `libWBC_CTRL.so` | 全身控制器（Whole-Body Control） |
| `libonnx_model.so` | ONNX Runtime 推理封装 |
| `libP2.so` | 未知（可能是规划器） |
| `libbiomimetics.so` | 仿生运动学 |
| `libGoldfarb_Optimizer.so` | 二次规划求解器 |
| `libqpOASES.so` | QP 优化器 |
| `libdynacore_param_handler.so` | 参数管理 |
| `libfilecrypto_shared.so` | 模型文件解密 |

**通信接口（已知）：**
- 输入：UDP 端口 43988，接收 `highLevelCmd` 结构体（vx, vy, yaw_rate, control_mode）
- 输出：关节命令 → MuJoCo（通过共享内存或内部接口）
- 状态反馈：50Hz 里程计 → `/odom/mujoco_odom`

---

### 2.2 RL 策略模型（加密 ONNX）

**路径：** `src/robot_mc/build/export/onnx_model_crypto/`

**内容（100 个加密模型文件）：**

| 机器人 | 策略类型 |
|--------|----------|
| `xg/` | policy_mlp, policy_ik, policy_soft_walk, policy_backflip, odom_gait, odom_tracking, odom_mix_sideflip 等 |
| `xg_wheel/` | 轮式策略 |
| `zg_wheels/` | ZG 轮式策略 |

**加密方式：** 使用 `file_crypto_cli` 工具加密，运行时由 `libfilecrypto_shared.so` 解密。

---

### 2.3 MuJoCo 仿真器（`robot_mujoco`）

**路径：** `src/robot_mujoco/simulate/build/robot_mujoco`

**作用：**
- 加载机器人 XML 模型和场景
- 执行物理仿真（动力学、碰撞检测）
- 与 `mc_ctrl` 交换关节状态
- 发布传感器数据（IMU、LiDAR）
- 与 UE5 同步状态

**已知接口：**
- 配置文件：`src/robot_mujoco/simulate/config.yaml`
- 机器人模型：`src/robot_mujoco/zsibot_robots/{go2,xgb,xgw,go2w,zgws}/`
- 仿真参数：`simulator-defaults.yaml`（dt=0.001s, high_level_dt=0.002s）

---

### 2.4 UE5 渲染引擎（`zsibot_mujoco_ue`）

**路径：** `src/UeSim/Linux/`

**作用：**
- 3D 环境渲染（地图、光照、材质）
- 传感器模拟（LiDAR 点云、相机图像）
- 接收 MuJoCo 状态 → 驱动 UE 中的机器人模型
- 发布 ROS 2 话题（`/livox/lidar`, `/livox/imu`）

**规模：** 184 个 .so 库文件，包含完整 UE5 引擎 + 自定义插件

---

### 2.5 sim_launcher

**路径：** `bin/sim_launcher.bin`

**作用：** 图形化启动器，选择机器人和地图，编排启动 MuJoCo + UE5 + MC

---

### 2.6 robot_forward（TF 桥接）

**安装路径：** `/opt/robot/robot-forward/`

**作用：**
- 订阅 `/odom/mujoco_odom`
- 发布 `odom → base_link` TF（50Hz）
- 发布 `map → odom` 静态 TF
- 包含 `ecal2ros` 组件（eCAL → ROS 2 桥接）

---

## 三、开源可用部分

### 3.1 完全可修改

| 内容 | 路径 | 用途 |
|------|------|------|
| 启动脚本 | `scripts/run_sim.sh` | 仿真启动流程 |
| 构建脚本 | `scripts/build.sh`, `build_mc.sh` | 编译流程 |
| 环境检查 | `scripts/check_env.sh` | 依赖验证 |
| SDK 配置 | `config/sdk_config.yaml` | UDP 通信端口 |
| 仿真参数 | `config/simulator-defaults.yaml` | 物理仿真参数 |
| 运动参数 | `config/*-motion_config.yaml` | 各机器人运动参数 |
| 用户参数 | `config/*-user-parameters.yaml` | 可调参数 |
| 机器人模型 | `zsibot_robots/*/` | MuJoCo XML + STL/OBJ 网格 |
| 场景配置 | `scene/scene.json` | 动态场景配置 |
| RViz 配置 | `rviz/matrix.rviz` | 可视化配置 |
| XML 校验 | `scripts/validate_xml_contract.py` | 模型格式验证 |

### 3.2 开源依赖（已安装 .deb）

| 依赖 | 版本 | 用途 |
|------|------|------|
| MuJoCo | 3.3.0 | 物理引擎（有头文件 `/usr/include/mujoco/`） |
| ONNX Runtime | 1.51.0 | 模型推理 |
| LCM | 1.5.1 | 进程间通信 |
| eCAL | 5.13.3 | 进程间通信 |
| zsibot_common | 0.5.9 | 公共库（闭源） |

---

## 四、二次开发路径分析

### 4.1 可行开发方向

#### 方向 A：自定义机器人模型（难度：低）

**官方支持：** ✅ 有完整教程（`docs/Custom_Robot_Tutorial.md`）

```
步骤：
1. 准备 MuJoCo XML 模型（含关节、执行器、IMU site）
2. 放到 src/robot_mujoco/robots/custom/custom.xml
3. 同步到 UE 端 Content/model/custom/custom.xml
4. sim_launcher 选择 custom 机器人
5. 使用自己的 MuJoCo API 控制器驱动
```

**限制：** 需要自己写控制器，`mc_ctrl` 的 RL 策略不适用于自定义机器人。

---

#### 方向 B：自定义控制器（难度：中）

**接口：** MuJoCo C API（`/usr/include/mujoco/`）

```c
// 可直接调用 MuJoCo API 控制机器人
#include <mujoco/mujoco.h>

mjModel* m = mj_loadXML("robot.xml", ...);
mjData* d = mj_makeData(m);

// 设置关节力矩
d->ctrl[0] = torque;
mj_step(m, d);
```

**开发方式：**
1. 编写自己的控制程序，直接操作 MuJoCo
2. 绕过 `mc_ctrl`，自己实现步态控制
3. 通过 UDP 43988 端口模拟 `highLevelCmd` 输入

---

#### 方向 C：自定义场景（难度：低）

**官方支持：** ✅ 有教程（`docs/Custom_Scene_Tutorial.md`）

- 修改 `scene/scene.json` 添加动态物体
- 修改 `scene_terrain_*.xml` 更换地形
- 支持 3DGS（3D Gaussian Splatting）场景

---

#### 方向 D：导航栈开发（难度：中）

**完全开源：** ✅ `genisom_roamerx_open` 全部源码可用

- 修改 MPPI 控制器参数
- 自定义 Behavior Tree 节点
- 调整 costmap 配置
- 开发新的规划算法

---

#### 方向 E：传感器扩展（难度：中-高）

**可行但受限：**
- MuJoCo 端：可在 XML 中添加传感器定义（camera, lidar, touch）
- UE5 端：需要 UE 源码才能添加新传感器插件（❌ 无源码）
- 替代方案：在 MuJoCo 中直接模拟传感器，绕过 UE5

---

#### 方向 F：替换渲染引擎（难度：高）

如果不需要 UE5 的高质量渲染：
- 使用 MuJoCo 自带渲染器（`mjv_*` API）
- 使用 Gazebo 替代整个仿真
- 使用 Isaac Sim / PyBullet 替代

---

### 4.2 不可行/极难开发方向

| 方向 | 原因 |
|------|------|
| 修改 mc_ctrl 内部逻辑 | 纯二进制，无源码 |
| 修改/训练 RL 策略 | 模型加密，无法导出 |
| 修改 UE5 渲染插件 | 打包二进制，无 .uproject |
| 修改 robot_forward | 闭源 .deb |
| 修改 sim_launcher | 纯二进制 |

---

## 五、开发环境搭建建议

### 5.1 最小开发环境（导航 + 仿真）

```bash
# 1. 已有：MATRiX 仿真（提供物理 + 渲染）
# 2. 已有：genisom_roamerx_open（导航栈，全开源）
# 3. 已有：MuJoCo 3.3.0 头文件（可编写自定义控制器）
# 4. 已有：ROS 2 Humble + Zenoh DDS

# 开发导航算法：
cd genisom_roamerx_open
# 修改 src/navigation/src/ 下的源码
./build.sh all

# 开发自定义控制器：
# 使用 MuJoCo C API，参考 /usr/include/mujoco/
```

### 5.2 自定义机器人开发

```bash
# 参考文档
cat docs/Custom_Robot_Tutorial.md

# 关键文件
src/robot_mujoco/robots/custom/custom.xml    # MuJoCo 模型
src/UeSim/Linux/zsibot_mujoco_ue/Content/model/custom/custom.xml  # UE 模型
config/config.json                            # 运行时配置
```

### 5.3 纯 MuJoCo 开发（绕过 UE5）

```bash
# 如果只需要物理仿真，不需要 UE5 渲染：
# 1. 使用 MuJoCo Python 绑定
pip install mujoco

# 2. 加载机器人模型
import mujoco
model = mujoco.MjModel.from_xml_path("src/robot_mujoco/zsibot_robots/xgb/xgb.xml")
data = mujoco.MjData(model)

# 3. 自己实现控制循环
mujoco.mj_step(model, data)
```

---

## 六、接口协议总结（可用于对接开发）

### 6.1 UDP 通信协议（RoamerX → MC）

| 字段 | 类型 | 说明 |
|------|------|------|
| `head` | uint16 | 帧头 0x5AA5 |
| `len` | uint16 | 数据长度 |
| `vx` | float | 前后速度 (m/s) |
| `vy` | float | 左右速度 (m/s) |
| `yaw_rate` | float | 旋转角速度 (rad/s) |
| `control_mode` | uint16 | 运动模式（18=K_RL_MIX） |
| `enable_control_mode` | uint16 | 模式使能 |
| `checksum` | uint16 | 校验和 |

- 目标端口：43988
- 发送频率：200Hz（5ms）

### 6.2 ROS 2 话题（仿真 → 导航）

| 话题 | 类型 | 频率 | 来源 |
|------|------|------|------|
| `/odom/mujoco_odom` | nav_msgs/Odometry | 50Hz | robot_mujoco |
| `/livox/lidar` | sensor_msgs/PointCloud2 | 10Hz | UE5 |
| `/livox/imu` | sensor_msgs/Imu | 200Hz | UE5 |
| `/cmd_vel` | geometry_msgs/Twist | 30Hz | 导航栈 |
| `/tf` | tf2_msgs/TFMessage | 50Hz | robot_forward |

### 6.3 仿真参数

| 参数 | 值 | 说明 |
|------|-----|------|
| `dynamics_dt` | 0.001s | 物理仿真步长 |
| `high_level_dt` | 0.002s | 高层控制周期 |
| `low_level_dt` | 0.0002s | 底层控制周期 |
| `floor_kp` | 500000 | 地面刚度 |
| `floor_kd` | 5000 | 地面阻尼 |

---

## 七、总结

### 你能做什么

| 开发内容 | 可行性 | 说明 |
|----------|:---:|------|
| 修改导航算法 | ✅ | genisom_roamerx_open 全开源 |
| 自定义机器人模型 | ✅ | 官方支持，有教程 |
| 自定义场景/地图 | ✅ | 官方支持 |
| 编写 MuJoCo 控制器 | ✅ | MuJoCo API 完全可用 |
| 修改通信协议 | ✅ | vel_cmd_udp_publisher 开源 |
| 修改启动流程 | ✅ | 脚本全开源 |
| 修改 MC 控制策略 | ❌ | 二进制 + 加密模型 |
| 修改 UE5 渲染 | ❌ | 无源码 |
| 修改 robot_forward | ❌ | 闭源 .deb |

### 核心结论

MATRiX 的**应用层**（导航、场景、模型）完全开源可开发，但**底层引擎**（运动控制、物理仿真、渲染）均为闭源二进制。二次开发应聚焦于：

1. **导航算法**（genisom_roamerx_open）
2. **自定义机器人/场景**（MuJoCo XML）
3. **上层应用逻辑**（ROS 2 节点开发）
4. **自定义 MuJoCo 控制器**（绕过 mc_ctrl）
