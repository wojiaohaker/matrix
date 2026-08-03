# MuJoCo + CarlaUnreal 联合仿真集成方案

> **日期:** 2026-07-31 (更新: 2026-07-16)
> **状态:** mujoco_sim 已完成，CarlaUnreal 侧待实现
> **目标:** 用 Mujoco330/mujoco_sim 替代 robot_mujoco，用 CarlaUnreal 替代 zsibot_mujoco_ue

---

## 一、背景与动机

### 1.1 Matrix 原始架构

Matrix 仿真平台由三个核心进程组成：

| 进程 | 二进制 | 职责 |
|------|--------|------|
| robot_mujoco | `src/robot_mujoco/simulate/build/robot_mujoco` | 独立物理仿真，广播状态 |
| zsibot_mujoco_ue | `src/UeSim/Linux/zsibot_mujoco_ue/` | UE5 渲染 + 内嵌物理 |
| mc_ctrl | `src/robot_mc/build/export/mc/bin/mc_ctrl` | RL 运动控制（"大脑"）|

Matrix 有两种运行模式：
- **mujoco_running=false（默认）:** UE 内嵌 MuJoCo 做物理 + 渲染，mc_ctrl(type=8) 发指令给 UE
- **mujoco_running=true:** standalone MuJoCo 做物理，UE 纯渲染，mc_ctrl(type=5) 发指令给 MuJoCo

### 1.2 我们的目标

用 **Mujoco330/mujoco_sim**（自研物理仿真服务器）替代 robot_mujoco，用 **CarlaUnreal** 替代 Matrix UE：

| Matrix 原始组件 | 替代方案 | 状态 |
|---|---|---|
| `src/robot_mujoco` (预编译, eCAL) | **Mujoco330/mujoco_sim** (MuJoCo 3.3.0, eCAL+UDP) | ✅ 已完成 |
| `src/UeSim/Linux` (zsibot_mujoco_ue) | **CarlaUE5/CarlaUnreal** | 🔧 待实现外部物理模式 |
| `src/robot_mc` (mc_ctrl) | 继续使用 Matrix 的 (type=5) | ✅ 无需修改 |

```
mujoco_sim (Mujoco330)             mc_ctrl (type=5)           CarlaUnreal
  [物理仿真 500Hz]               [RL运动控制]                [纯渲染]
      |                              |                          |
      |-- eCAL [mujoco_state] ----> | (接收状态,计算RL)        |
      |                              |                          |
      | <--- eCAL [mujoco_cmd] ---- | (发送关节PD目标)         |
      |                              |                          |
      |--- UDP 25001 RobotState -----------------------------> | (接收状态,更新mesh)
                                     |                          |
                                     | <--- 43997 HighLevel --- | (虚拟键盘脚本)
```

**关键发现:** robot_mujoco 与 mc_ctrl 之间使用 **eCAL 中间件**（非原始 UDP），topic 名称：
- `mujoco_state` (proto:robot_sdk.pb.RobotState) — 物理→控制
- `mujoco_cmd` (proto:robot_sdk.pb.RobotCmd) — 控制→物理

### 1.3 与 Matrix 默认模式的对比

| 组件 | Matrix 默认 (mujoco_running=false) | 我们的方案 |
|------|-------------------------------------|----------------------------------|
| 物理引擎 | UE 内嵌 MuJoCo | **mujoco_sim** (Mujoco330, 独立进程) |
| 渲染 | Matrix UE (zsibot_mujoco_ue) | **CarlaUnreal** |
| 控制 | mc_ctrl type=8 (UnrealCommandInterface) | mc_ctrl type=5 (MujocoCommandInterface) |
| 物理↔控制通信 | UDP 25001/25002 | **eCAL** (mujoco_state/mujoco_cmd) |
| 渲染端通信 | 内部 | **UDP 25001** (mujoco_sim → CarlaUnreal) |
| 按键 | sim_launcher X11 全局钩子 → mc_ctrl | 虚拟键盘脚本 → mc_ctrl port 43997 |

---

## 二、通信协议

### 2.1 通信架构

| 链路 | 协议 | 内容 | 大小 | 频率 |
|------|------|------|------|------|
| mujoco_sim → mc_ctrl | **eCAL** topic `mujoco_state` | RobotState | 316 bytes | 500Hz |
| mc_ctrl → mujoco_sim | **eCAL** topic `mujoco_cmd` | RobotCmd | 365 bytes | 500Hz |
| mujoco_sim → CarlaUnreal | **UDP** 127.0.0.1:25001 | RobotState | 316 bytes | 500Hz |
| 键盘脚本 → mc_ctrl | **UDP** port 43997 | HighLevelCmd | 可变 | 按需 |

### 2.2 Protobuf 消息定义

协议使用 `robot_sdk.pb` protobuf 格式，关节顺序为 **[FR, FL, RR, RL]**（右前、左前、右后、左后）。

#### RobotState (316 bytes) — mujoco_sim → mc_ctrl / CarlaUnreal

```protobuf
message RobotState {
    repeated float q_abad = 1;      // [4] 外展/内收关节角 (rad)
    repeated float q_hip = 2;       // [4] 髈关节角 (rad)
    repeated float q_knee = 3;      // [4] 膝关节角 (rad)
    repeated float q_foot = 4;      // [4] 足端关节 (xgb填0)
    repeated float qd_abad = 5;     // [4] 外展速度 (rad/s)
    repeated float qd_hip = 6;      // [4] 髈速度
    repeated float qd_knee = 7;     // [4] 膝速度
    repeated float qd_foot = 8;     // [4] 足端速度 (xgb填0)
    repeated float tau_abad_fb = 9; // [4] 力矩反馈
    repeated float tau_hip_fb = 10; // [4]
    repeated float tau_knee_fb = 11;// [4]
    repeated float tau_foot_fb = 12;// [4] 足端力矩 (xgb填0)
    repeated float quat = 13;       // [4] 四元数 [w, x, y, z]
    repeated float gyro = 14;       // [3] 陀螺仪 (rad/s)
    repeated float acc = 15;        // [3] 加速度计 (m/s²)
    uint64 time_stamp = 17;         // 纳秒时间戳
    repeated float rpy = 19;        // [3] 欧拉角
    repeated float position = 20;   // [3] 位置 (x,y,z)
    repeated float v_world = 21;    // [3] 世界系线速度
}
```

#### RobotCmd (365 bytes) — mc_ctrl → MuJoCo

```protobuf
message RobotCmd {
    repeated float q_des_abad = 1;  // [4] 目标关节角
    repeated float q_des_hip = 2;   // [4]
    repeated float q_des_knee = 3;  // [4]
    repeated float qd_des_abad = 4; // [4] 目标速度
    repeated float qd_des_hip = 5;  // [4]
    repeated float qd_des_knee = 6; // [4]
    repeated float tau_abad_ff = 7; // [4] 前馈力矩
    repeated float tau_hip_ff = 8;  // [4]
    repeated float kp_abad = 9;     // [4] PD 增益 Kp
    repeated float kp_hip = 10;     // [4]
    repeated float kp_knee = 11;    // [4]
    repeated float kd_abad = 12;    // [4] PD 增益 Kd
    repeated float kd_hip = 13;     // [4]
    repeated float kd_knee = 14;    // [4]
    repeated float kp_foot = 15;    // [4] 足端增益
    repeated float kd_foot = 16;    // [4]
    repeated float q_des_foot = 17; // [4]
    repeated float qd_des_foot = 18;// [4]
    repeated float tau_foot_fb = 19;// [4]
    repeated float tau_foot_ff = 20;// [4]
}
```

### 2.3 关节映射关系

CarlaUnreal MuJoCo 模型中的关节顺序（跳过 freejoint）：

| MuJoCo 索引 | 腿 | 关节 | 协议索引 |
|-------------|-----|------|----------|
| qpos[7] | FAR (右前) | ABAD | q_abad[0] |
| qpos[8] | FAR | HIP | q_hip[0] |
| qpos[9] | FAR | KNEE | q_knee[0] |
| qpos[10] | FBL (左前) | ABAD | q_abad[1] |
| qpos[11] | FBL | HIP | q_hip[1] |
| qpos[12] | FBL | KNEE | q_knee[1] |
| qpos[13] | RAR (右后) | ABAD | q_abad[2] |
| qpos[14] | RAR | HIP | q_hip[2] |
| qpos[15] | RAR | KNEE | q_knee[2] |
| qpos[16] | RBL (左后) | ABAD | q_abad[3] |
| qpos[17] | RBL | HIP | q_hip[3] |
| qpos[18] | RBL | KNEE | q_knee[3] |

**映射公式：**
- 发送: `protoIdx = jointType * 4 + leg` (jointType: 0=abad, 1=hip, 2=knee)
- 接收: `mData->qpos[7 + leg*3 + jointType] = proto[jointType*4 + leg]`

### 2.4 时间戳要求

**必须使用 `CLOCK_REALTIME`**（而非 `CLOCK_MONOTONIC`），mc_ctrl 内部用 `gettimeofday()` 校验时间戳新鲜度。

---

## 三、按键控制流程

### 3.1 Matrix 原始按键流程

```
按下 W 键
    │
    ▼
sim_launcher.bin (XGrabKey 全局拦截)
    ├── 转换为速度指令: vx=1.0, vy=0, vyaw=0
    └── 发送给 mc_ctrl (ROS2/zenoh 或 UDP port 43997)
         │
         ▼
mc_ctrl (RL 大脑)
    ├── 读取当前状态 (UDP 25001)
    ├── RL 策略推理: f(状态, 指令) → 12个关节目标角度
    └── UDP 25002 发送 RobotCmd 给 MuJoCo
         │
         ▼
robot_mujoco (物理仿真)
    ├── PD 控制: torque = Kp×(目标-当前) - Kd×速度
    ├── mj_step() 推进物理
    └── UDP 25001 广播新状态
         │
         ▼
CarlaUnreal (纯渲染)
    └── 接收状态 → 更新 mesh transform → 渲染画面
```

### 3.2 我们的按键方案

虚拟键盘脚本（自研）→ mc_ctrl highlevel port 43997，发送 HighLevelCmd：
- standUP / lieDown / passive
- move(vx, vy, yaw_rate)

---

## 四、CarlaUnreal 代码改造

### 4.1 添加外部物理模式开关

文件: `Plugins/MuJoCoUE/Source/MuJoCoUE/Public/MuJoCoSimulation.h`

```cpp
/** 外部物理模式: 不跑内部 mj_step, 从 UDP 接收 RobotState 驱动渲染 */
UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "UDP Control")
bool bExternalPhysicsMode = false;

/** 外部物理模式下的 UDP 状态接收器 (监听 port 25001) */
UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "UDP Control")
UUdpReceiverComponent* UdpStateReceiver;
```

### 4.2 BeginPlay() 逻辑

```
if (bExternalPhysicsMode):
    1. 仍加载 MuJoCo XML → 生成 mesh 和 body 层级 (复用现有 LoadModel)
    2. 创建 UdpStateReceiver, 绑定 port 25001, 模式=ParseRobotState
    3. 注册回调: 收到数据 → 解析 protobuf → 写入线程安全缓存
    4. bSimulationRunning = false (不跑内部物理)
    5. bUdpControlEnabled = false (不发状态, 不收 RobotCmd)
```

### 4.3 Tick() 逻辑

```
if (bExternalPhysicsMode):
    1. 从缓存读取最新 RobotState (加锁)
    2. 写入 mData->qpos:
       - qpos[0:2] = position (base xyz)
       - qpos[3:6] = quat (base wxyz)
       - qpos[7:18] = joint_pos (12个关节)
    3. mj_kinematics(mModel, mData)  // 仅计算 FK, 不推进物理
    4. ExtractCurrentState(_info)
    5. UpdateSimulationView(_info)   // 更新 mesh transform
    return  // 跳过所有内部物理/控制逻辑
```

### 4.4 禁用的逻辑（外部模式下）

| 函数 | 原因 |
|------|------|
| `SimulateMuJoCo()` | 物理由 standalone MuJoCo 负责 |
| `SendStateToMcCtrl()` | 状态由 MuJoCo 直接广播 |
| `ApplyUdpControl()` | mc_ctrl 直接控制 MuJoCo |
| `ApplyStandUpControl()` | 站立由 mc_ctrl RL 策略完成 |
| `UpdateGaitTargets()` | 步态由 mc_ctrl RL 生成 |
| 启动延迟 (`UDP_STARTUP_DELAY`) | 无需等待塌陷 |

### 4.5 UdpReceiverComponent 扩展

在现有 `UdpReceiverComponent` 中添加 RobotState 解析模式：

```cpp
// 新增: 解析模式枚举
UENUM()
enum class EUdpParseMode : uint8 {
    ParseRobotCmd,    // 现有: 解析 mc_ctrl 发来的 RobotCmd
    ParseRobotState   // 新增: 解析 MuJoCo 发来的 RobotState
};

UPROPERTY(EditAnywhere)
EUdpParseMode ParseMode = EUdpParseMode::ParseRobotCmd;

// 新增: RobotState 缓存 (ParseRobotState 模式下使用)
struct FExternalRobotState {
    bool bValid = false;
    double ReceiveTime = 0.0;
    float JointPos[12];    // [abad×4, hip×4, knee×4]
    float Quat[4];         // [w, x, y, z]
    float Position[3];     // [x, y, z]
    float Gyro[3];
    float Acc[3];
};
FExternalRobotState LastState;
FCriticalSection StateMutex;
```

解析逻辑（`SendStateToMcCtrl()` 的逆过程）：
```cpp
robot_sdk::pb::RobotState Msg;
Msg.ParseFromArray(Data.GetData(), Data.Num());
// joint_pos: q_abad[4] + q_hip[4] + q_knee[4] → JointPos[12]
for (int leg = 0; leg < 4; leg++) {
    JointPos[leg*3 + 0] = Msg.q_abad(leg);
    JointPos[leg*3 + 1] = Msg.q_hip(leg);
    JointPos[leg*3 + 2] = Msg.q_knee(leg);
}
// base pose
Quat[0..3] = Msg.quat(0..3);      // wxyz
Position[0..2] = Msg.position(0..2); // 注意: position 字段含义需验证
```

---

## 五、配置文件清单

### 5.1 mc_ctrl 配置

文件: `src/robot_mc/build/export/config/xg-user-parameters.yaml`

```yaml
motor_platform_type: 5    # MujocoCommandInterface (不是8!)
body_height: 0.375        # 必须匹配 mc_ctrl 内部 FK 值
```

文件: `src/robot_mc/build/export/config/robot-defaults.yaml`

```yaml
control_mode: 1           # PD_CTRL (不能是0)
motion_mode: 10
cur_control_mode: 1
cur_motion_mode: 10
```

### 5.2 mujoco_sim 配置

文件: `/home/qiyuan/Softwares/Mujoco330/mujoco_sim/config.yaml`

```yaml
robot: "xgb"
robot_scene: "scene_terrain_yard.xml"
enable_ecal: 1
enable_udp: 1
udp_target_ip: "127.0.0.1"
udp_target_port: 25001
sim_rate_hz: 500
publish_rate_hz: 500
```

模型路径: `/home/qiyuan/Softwares/Matrix/src/robot_mujoco/zsibot_robots/xgb/`

### 5.3 全局配置

文件: `config/config.json`

```json
{
    "robot": {
        "robot_type": "xgb",
        "mujoco_running": true,
        "state_port": 25001,
        "cmd_port": 25002,
        "EgoView": true
    }
}
```

### 5.4 CarlaUnreal 侧

| 参数 | 值 | 说明 |
|------|-----|------|
| bExternalPhysicsMode | true | 启用外部物理渲染模式 |
| bUdpControlEnabled | false | 禁用内部 UDP 控制回路 |
| XmlSourcePath | 与 robot_mujoco 相同的 XML | 确保 mesh 匹配 |

---

## 六、启动脚本

文件: `scripts/run_carlaunreal_sim.sh`

```bash
#!/usr/bin/env bash
set -euo pipefail

MUJOCO_SIM="/home/qiyuan/Softwares/Mujoco330/mujoco_sim/build/mujoco_sim"
MUJOCO_CONFIG="/home/qiyuan/Softwares/Mujoco330/mujoco_sim/config.yaml"
MUJOCO_LIB="/home/qiyuan/Softwares/Mujoco330/install/lib"
MC_CTRL="/home/qiyuan/Softwares/Matrix/src/robot_mc/build/export/mc/bin"

echo "=== MuJoCo + CarlaUnreal 联合仿真 ==="

# ① 启动 mujoco_sim (替代 robot_mujoco)
export LD_LIBRARY_PATH="${MUJOCO_LIB}:${LD_LIBRARY_PATH:-}"
${MUJOCO_SIM} ${MUJOCO_CONFIG} > /tmp/mujoco_sim.log 2>&1 &
MUJOCO_PID=$!
echo "[1/3] mujoco_sim started (PID=$MUJOCO_PID)"

# ② 等待初始化
sleep 2

# ③ 启动 CarlaUnreal (外部物理模式)
# TODO: 根据实际启动方式修改
echo "[2/3] CarlaUnreal starting... (请手动启动)"
sleep 7

# ④ 启动 mc_ctrl (type=5)
cd ${MC_CTRL}
export LD_LIBRARY_PATH="$(pwd):${LD_LIBRARY_PATH:-}"
export ROBOT_TYPE=XG
taskset -c 7 ./mc_ctrl r > /tmp/mc_ctrl.log 2>&1 &
MC_PID=$!
echo "[3/3] mc_ctrl started (PID=$MC_PID)"

echo ""
echo "=== 全部启动完成 ==="
echo "日志: /tmp/mujoco_sim.log, /tmp/mc_ctrl.log"
echo "按 Ctrl+C 停止所有进程"

trap "echo 'Stopping...'; kill $MUJOCO_PID $MC_PID 2>/dev/null; exit 0" EXIT SIGINT SIGTERM
wait
```

---

## 七、实施步骤

### Phase 1: mujoco_sim 物理仿真服务器 (✅ 已完成)

1. 分析 robot_mujoco 架构 (eCAL + protobuf + PD控制)
2. 创建 `/home/qiyuan/Softwares/Mujoco330/mujoco_sim/` 项目
3. 实现: 加载 xgb 模型 + mj_step 500Hz + eCAL pub/sub + UDP 25001
4. 验证: mc_ctrl 联调成功 (mujoco_state 316B@500Hz, mujoco_cmd 365B@500Hz)

### Phase 2: CarlaUnreal 外部物理模式 (3-4h)

1. `MuJoCoSimulation.h` 添加 `bExternalPhysicsMode` + `UdpStateReceiver`
2. `UdpReceiverComponent` 添加 `ParseRobotState` 模式
3. `MuJoCoSimulation.cpp` 实现:
   - BeginPlay: 初始化接收器
   - Tick: 接收 → FK → 渲染
   - 跳过内部物理/控制

### Phase 3: 坐标映射验证 (1h)

确认 mujoco_sim 发出的 joint_pos 与 CarlaUnreal mesh 映射一致。

### Phase 4: 虚拟键盘脚本适配 (0.5h)

确认键盘脚本发送到 mc_ctrl port 43997。

### Phase 5: 集成测试 (1-2h)

1. mujoco_sim + mc_ctrl + CarlaUnreal 三进程联调
2. U 键站立 → CarlaUnreal 同步显示
3. WASD 行走 → 渲染跟随

---

## 八、风险与备选方案

| 风险 | 影响 | 备选方案 |
|------|------|----------|
| RobotState protobuf 版本不一致 | 解析错误 | 对比 hexdump 验证字段 |
| 场景 XML 不一致 | mesh 与物理不匹配 | 确保加载同一份 XML |
| 帧率不同步 (MuJoCo 500Hz, UE 30fps) | 渲染抖动 | Tick 中取最新帧 / 插值 |
| mc_ctrl body_height 不匹配 | 无法进入 RL_MIX | 设为 0.375 |
| eCAL SHM 冲突 | 通信失败 | 清理 /tmp/ecal* 文件 |

---

## 九、关键文件索引

| 文件 | 作用 |
|------|------|
| `/home/qiyuan/Softwares/Mujoco330/mujoco_sim/mujoco_sim.h` | 物理仿真服务器头文件 |
| `/home/qiyuan/Softwares/Mujoco330/mujoco_sim/mujoco_sim.cpp` | 物理仿真实现 (eCAL+UDP+PD) |
| `/home/qiyuan/Softwares/Mujoco330/mujoco_sim/main.cpp` | 主入口 |
| `/home/qiyuan/Softwares/Mujoco330/mujoco_sim/config.yaml` | 仿真配置 |
| `CarlaUE5/.../MuJoCoSimulation.h` | CarlaUnreal 主 Actor 头文件 |
| `CarlaUE5/.../MuJoCoSimulation.cpp` | CarlaUnreal 主逻辑实现 |
| `CarlaUE5/.../UdpReceiverComponent.h` | UDP 接收组件 |
| `CarlaUE5/.../Proto/robot_sdk.pb.h` | Protobuf 定义 (CarlaUnreal侧) |
| `/usr/include/robot_sdk.pb.h` | Protobuf 定义 (系统侧, mujoco_sim用) |
| `Matrix/src/robot_mujoco/zsibot_robots/xgb/` | xgb 机器人模型 (XML+STL) |
| `Matrix/src/robot_mc/build/export/config/xg-user-parameters.yaml` | mc_ctrl 控制参数 |
| `Matrix/src/robot_mc/build/export/config/robot-defaults.yaml` | mc_ctrl 默认模式 |
