# 路线 B：从零训练 RL 策略实施计划

**目标**：使用 Isaac Lab + RSL_RL 从零训练 xgb 四足机器人 RL 策略，替代 robot_mc 的加密 ONNX 模型。

**模型基础**：`/home/qiyuan/Softwares/Matrix/src/robot_mujoco/zsibot_robots/xgb/xgb.xml`（12 DOF，STL 网格，含 URDF）

**操作系统**：Ubuntu 22.04

---

## 1. 系统概览

### 1.1 原系统（robot_mc）vs 目标系统

| 组件 | robot_mc（原） | 目标系统 |
|------|---------------|---------|
| 物理引擎 | MuJoCo | MuJoCo（训练）/ MuJoCo（部署） |
| RL 框架 | zsibot_sim（自研，闭源） | Isaac Lab + RSL_RL（开源） |
| 策略网络 | 加密 ONNX（policy + estimator） | 自训练 ONNX（policy） |
| 推理引擎 | libonnx_model.so（闭源） | ONNX Runtime C++（开源） |
| 控制架构 | mc_ctrl + libWBC（闭源 QP-WBC） | PD 控制 + FSM（自研） |
| 通信 | eCAL / UDP + Protobuf | UDP + Protobuf（复用现有） |
| 渲染 | UE5（zsibot_mujoco_ue） | CarlaUE5（复用现有） |

### 1.2 总体架构

```
训练阶段：
  Isaac Lab (GPU 并行 4096 环境)
    → RSL_RL (PPO)
    → PyTorch 策略网络
    → 导出 ONNX

部署阶段：
  Nav2 → /cmd_vel
    → FSM 状态机
    → ONNX Runtime C++ 推理（策略网络）
    → PD 控制 + 重力补偿
    → MuJoCo mj_step (500Hz)
    → UDP → CarlaUE5 渲染
```

---

## 2. xgb 机器人模型参数

### 2.1 关节配置（12 DOF）

| 腿 | Abad 关节 | Hip 关节 | Knee 关节 |
|----|----------|---------|----------|
| FR (右前) | FAR_ABAD_JOINT | FAR_HIP_JOINT | FAR_KNEE_JOINT |
| FL (左前) | FBL_ABAD_JOINT | FBL_HIP_JOINT | FBL_KNEE_JOINT |
| RR (右后) | RAR_ABAD_JOINT | RAR_HIP_JOINT | RAR_KNEE_JOINT |
| RL (左后) | RBL_ABAD_JOINT | RBL_HIP_JOINT | RBL_KNEE_JOINT |

### 2.2 关节限位与力矩

| 关节类型 | 限位范围 (rad) | 力矩限幅 (Nm) | 摩擦 |
|---------|---------------|--------------|------|
| Abad | ±0.4887 | ±28 | 0.2 |
| Hip | [-2.0, 3.491] | ±28 | 0.2 |
| Knee | [-2.723, -0.602] | ±28 | 0.2 |

### 2.3 身体参数

| 参数 | 值 |
|------|---|
| 基座质量 | 6.69 kg |
| 基座惯性 | diag(0.0285, 0.0937, 0.1121) |
| Abad 质量 | 0.42 kg |
| Hip 质量 | 1.31 kg |
| Knee 质量 | 0.18 kg |
| 大腿长 | 0.2 m |
| 小腿长 | 0.21366 m |

### 2.4 传感器

| 传感器 | 类型 | 位置 |
|--------|------|------|
| IMU | quat + gyro + acc | 基座中心 (0,0,0) |
| 关节位置 | 12 路 jointpos | 各关节 |
| 关节速度 | 12 路 jointvel | 各关节 |
| 关节力矩 | 12 路 jointactuatorfrc | 各关节 |

---

## 3. 实施阶段

### Phase 1：环境搭建（第 1-2 周）

#### 1.1 安装 Isaac Lab

```bash
# 前置要求
# - Ubuntu 22.04
# - NVIDIA Driver ≥ 525
# - CUDA 12.x
# - Python 3.10

git clone https://github.com/isaac-sim/IsaacLab.git
cd IsaacLab
./isaaclab.sh --install

# 验证
./isaaclab.sh -p source/standalone/tutorials/00_sim.py


# 2.3.2 需要 Python 3.11
conda create -n isaaclab python=3.11
conda activate isaaclab

# Isaac Sim 5.1.0
pip install 'isaacsim[all,extscache]==5.1.0' --extra-index-url https://pypi.nvidia.com

# Isaac Lab 2.3.2
git clone https://github.com/isaac-sim/IsaacLab.git
cd IsaacLab
git checkout v2.3.2
./isaaclab.sh --install rsl_rl
```

#### 1.2 安装 RSL_RL

```bash
pip install rsl-rl-lib
```

#### 1.3 安装辅助工具

```bash
pip install onnx onnxruntime-gpu tensorboard
```

#### 1.4 里程碑

- [ ] Isaac Lab demo 正常运行
- [ ] RSL_RL 可以 import
- [ ] GPU 并行环境可以启动

---

### Phase 2：模型导入（第 2-3 周）

#### 2.1 URDF → USD 转换

```bash
./isaaclab.sh -p source/tools/convert_urdf.py \
  --input_path /path/to/xg_b.urdf \
  --output_path data/Robots/xgb/xgb.usd \
  --merge_joints
```

#### 2.2 参数对齐验证

| 检查项 | 方法 | 标准 |
|--------|------|------|
| 关节限位 | Isaac Sim 中手动转动各关节 | 与 xgb.xml 一致 |
| 碰撞体 | 检查 hip/knee 碰撞盒 | 至少有 sphere 或 box |
| 惯性矩阵 | 对比 URDF 和 XML 中的质量/惯性 | 误差 < 5% |
| 关节方向 | 施加正力矩看旋转方向 | 与 XML 一致 |
| 地面接触 | 放在平面上看是否稳定 | 不穿透、不弹跳 |

#### 2.3 补充碰撞体（如 URDF 缺失）

xgb.xml 中 hip 和 knee 有 box 碰撞体，URDF 可能没有。需要在 URDF 中手动添加：

```xml
<!-- hip 碰撞体（参考 xgb.xml） -->
<collision>
  <origin xyz="-0.01 0 -0.06" rpy="0 0.1 0"/>
  <geometry><box size="0.04 0.02 0.18"/></geometry>
</collision>

<!-- knee 碰撞体 -->
<collision>
  <origin xyz="0.02 0 -0.09" rpy="0 -0.06 0"/>
  <geometry><box size="0.02 0.02 0.18"/></geometry>
</collision>

<!-- 足端球体 -->
<collision>
  <origin xyz="0 0 -0.21366"/>
  <geometry><sphere radius="0.03"/></geometry>
</collision>
```

#### 2.4 里程碑

- [ ] USD 在 Isaac Sim 中正确显示
- [ ] 关节限位与 xgb.xml 一致
- [ ] 放在平面上稳定不穿透

---

### Phase 3：训练环境配置（第 3-4 周）

#### 3.1 观测空间设计

对齐 zsibot_sim 的 `policy_obs` 结构：

| 观测项 | 维度 | 缩放 | 说明 |
|--------|------|------|------|
| 体坐标系线速度 | 3 | ×2.0 | `base_lin_vel` |
| 体坐标系角速度 | 3 | ×0.25 | `base_ang_vel` |
| 重力投影 | 3 | ×1.0 | `projected_gravity` |
| 速度命令 | 3 | ×1.0 | `(vx_cmd, vy_cmd, wz_cmd)` |
| 关节位置（相对默认） | 12 | ×1.0 | `q - q_default` |
| 关节速度 | 12 | ×0.05 | `qd` |
| 上一步动作 | 12 | ×1.0 | `a_prev` |
| **总计** | **48** | | |

> 注：zsibot_sim 使用 53 维观测（含 estimator 相关），此处简化为 48 维。

#### 3.2 动作空间设计

| 动作 | 维度 | 说明 |
|------|------|------|
| 关节目标位置偏移 | 12 | `q_target = q_default + action * scale` |

默认关节位置（站立姿态）：

```python
default_joint_pos = {
    "FAR_ABAD_JOINT": 0.0, "FAR_HIP_JOINT": 0.8, "FAR_KNEE_JOINT": -1.5,
    "FBL_ABAD_JOINT": 0.0, "FBL_HIP_JOINT": 0.8, "FBL_KNEE_JOINT": -1.5,
    "RAR_ABAD_JOINT": 0.0, "RAR_HIP_JOINT": 1.0, "RAR_KNEE_JOINT": -1.5,
    "RBL_ABAD_JOINT": 0.0, "RBL_HIP_JOINT": 1.0, "RBL_KNEE_JOINT": -1.5,
}
action_scale = 0.25  # 动作缩放
```

#### 3.3 速度命令空间

```python
command_ranges = {
    "lin_vel_x": (0.0, 1.0),     # 前进 0~1.0 m/s
    "lin_vel_y": (-0.3, 0.3),    # 横移 ±0.3 m/s
    "ang_vel_z": (-1.0, 1.0),    # 转弯 ±1.0 rad/s
}
```

#### 3.4 里程碑

- [ ] 观测空间维度正确
- [ ] 动作空间映射正确
- [ ] 速度命令可以随机采样

---

### Phase 4：奖励函数设计（第 4-6 周）

#### 4.1 奖励函数

```python
# === 跟踪奖励 ===
track_lin_vel_xy_exp:  weight= 1.0,  std=0.5    # 线速度跟踪
track_ang_vel_z_exp:   weight= 0.5,  std=0.5    # 角速度跟踪

# === 惩罚项 ===
lin_vel_z_l2:          weight=-2.0               # Z 方向速度
ang_vel_xy_l2:         weight=-0.05              # 非偏航旋转
torque_l2:             weight=-0.001             # 力矩能耗
dof_vel_l2:            weight=-0.001             # 关节速度
dof_acc_l2:            weight=-2.0e-7            # 关节加速度
action_rate_l2:        weight=-0.01              # 动作变化率
flat_orientation_l2:   weight=-1.0               # 基座倾斜
feet_air_time:         weight= 0.125             # 足端腾空（鼓励交替）
feet_stumble:          weight=-0.1               # 足端碰撞
feet_drag:             weight=-0.02              # 足端拖拽
collision:             weight=-5.0               # 基座碰撞
dof_pos_limit:         weight=-1.0               # 关节超限
```

#### 4.2 课程学习设计

| 阶段 | 地形 | 速度范围 | 域随机化 | 训练步数 |
|------|------|---------|---------|---------|
| Stage 1 | 平地 | vx∈[0, 0.5] | 无 | 500 迭代 |
| Stage 2 | 平地 | vx∈[0, 1.0], vy∈[-0.3, 0.3], wz∈[-1.0, 1.0] | 质量±10% | 1000 迭代 |
| Stage 3 | 斜坡(±15°) | 全范围 | 质量±20%, 摩擦±30% | 1000 迭代 |
| Stage 4 | 混合地形 | 全范围 | 全随机 + 外力扰动 | 2000 迭代 |

#### 4.3 域随机化参数

| 参数 | 范围 | 说明 |
|------|------|------|
| 基座质量 | ±20% | 模拟载荷变化 |
| 关节摩擦 | ±50% | 模拟磨损 |
| 地面摩擦 | 0.5~1.5 | 不同地面材质 |
| 电机延迟 | 0~2 步 | 通信延迟 |
| 外力扰动 | 0~10 N | 随机脉冲 |
| 关节零位偏移 | ±0.05 rad | 装配误差 |
| IMU 噪声 |  gyro: 0.01, acc: 0.1 | 传感器噪声 |

#### 4.4 里程碑

- [ ] Stage 1：机器人能站起来并走几步
- [ ] Stage 2：稳定行走 + 速度跟踪
- [ ] Stage 3：斜坡行走不摔倒
- [ ] Stage 4：混合地形鲁棒行走

---

### Phase 5：训练执行（第 6-8 周）

#### 5.1 训练命令

```bash
# 训练（headless 模式，4096 并行环境）
./isaaclab.sh -p source/standalone/train.py \
  --task Isaac-Velocity-Flat-Xgb-v0 \
  --num_envs 4096 \
  --headless \
  --seed 42 \
  --max_iterations 5000

# 可视化评估
./isaaclab.sh -p source/standalone/play.py \
  --task Isaac-Velocity-Flat-Xgb-v0 \
  --num_envs 64 \
  --checkpoint logs/xgb/nn_ppo/model_5000.pt
```

#### 5.2 PPO 超参数

```python
ppo_config = {
    "learning_rate": 3e-4,
    "gamma": 0.99,
    "lam": 0.95,
    "num_learning_epochs": 5,
    "num_mini_batches": 4,
    "clip_param": 0.2,
    "value_loss_coef": 1.0,
    "entropy_coef": 0.005,
    "max_grad_norm": 1.0,
}
```

#### 5.3 GPU 需求

| 并行环境数 | 最低显存 | 推荐 GPU |
|-----------|---------|---------|
| 2048 | 8 GB | RTX 3070 |
| 4096 | 12 GB | RTX 3090 |
| 8192 | 16 GB | RTX 4090 |

#### 5.4 里程碑

- [ ] 训练曲线收敛（reward 稳定上升）
- [ ] 可视化中行走流畅、不摔倒
- [ ] 速度命令跟踪误差 < 0.1 m/s
- [ ] 可应对 15° 斜坡

---

### Phase 6：模型导出与部署（第 8-10 周）

#### 6.1 PyTorch → ONNX

```python
import torch

model = policy_network.eval()
dummy_obs = torch.randn(1, 48).cuda()

torch.onnx.export(
    model, dummy_obs, "xgb_policy.onnx",
    input_names=["policy_obs"],
    output_names=["actions"],
    opset_version=17,
    dynamic_axes={"policy_obs": {0: "batch"}, "actions": {0: "batch"}}
)
```

#### 6.2 C++ 推理集成

```cpp
// ONNX Runtime 推理（参考 libonnx_model.so 接口设计）
class PolicyInference {
    Ort::Session session_;
    std::vector<float> obs_buffer_;   // 48 维
    std::vector<float> action_buffer_; // 12 维

public:
    void loadModel(const std::string& path);
    void updateObs(const std::vector<float>& obs);
    std::vector<float> infer();  // 返回 12 关节目标位置
};
```

#### 6.3 PD 控制 + 重力补偿

```cpp
// 推理输出 → 关节力矩
for (int i = 0; i < 12; i++) {
    float q_des = default_q[i] + action[i] * action_scale;
    float qd_des = 0.0;
    float tau = Kp * (q_des - q[i]) + Kd * (qd_des - qd[i]) + qfrc_bias[i];
    tau = clamp(tau, -28.0f, 28.0f);  // 力矩限幅
    mj_data->ctrl[i] = tau;
}
```

PD 参数参考：

| 参数 | 值 | 说明 |
|------|---|------|
| Kp | 80~150 | 根据训练效果调整 |
| Kd | 2.0~5.0 | 阻尼 |
| 力矩限幅 | ±28 Nm | 与 xgb.xml 一致 |

#### 6.4 FSM 状态机

```
PASSIVE → STANDUP → RL_WALK
  ↑                    ↓
  └────────────────────┘
       (趴下指令)

PASSIVE:  ctrl = 0, 关节自由
STANDUP:  PD 控制到默认关节位置（3 秒）
RL_WALK:  ONNX 推理 + PD 控制
```

#### 6.5 里程碑

- [ ] ONNX 模型导出成功
- [ ] C++ 推理延迟 < 1ms
- [ ] PD 控制 + 推理闭环运行
- [ ] FSM 状态切换正常

---

### Phase 7：联调与优化（第 10-12 周）

#### 7.1 CarlaUE5 集成

```
MuJoCo 物理仿真 (500Hz)
  → UDP → CarlaUE5 渲染
  → LiDAR → pointcloud_to_laserscan → /scan
  → Nav2 → /cmd_vel
  → FSM + ONNX 推理 → MuJoCo ctrl[]
```

#### 7.2 导航闭环测试

- [ ] 定点导航（goal_pose → 到达）
- [ ] 避障行走
- [ ] 长时间运行稳定性（> 30 分钟不摔倒）

#### 7.3 性能优化

- [ ] TensorRT 加速推理（可选，降低延迟）
- [ ] 训练策略微调（sim-to-real gap 补偿）
- [ ] 内存/CPU 占用优化

---

## 4. 风险与对策

| 风险 | 影响 | 对策 |
|------|------|------|
| URDF 碰撞体缺失 | 训练时足端穿透地面 | Phase 2 手动补充并验证 |
| 训练不收敛 | 无法行走 | 简化奖励函数，从最小可行环境开始 |
| sim-to-real gap 大 | 真机摔倒 | 充分域随机化 + 课程学习 |
| GPU 显存不足 | 无法并行 4096 环境 | 降到 2048 或使用 gradient accumulation |
| PD 参数不匹配 | 部署后行为异常 | 从保守参数开始，逐步增大 |

---

## 5. 交付物

| 交付物 | 说明 |
|--------|------|
| `xgb.usd` | Isaac Lab 可用的机器人模型 |
| `xgb_policy.onnx` | 训练好的策略网络 |
| 训练日志 | TensorBoard 日志 + 模型检查点 |
| C++ 推理代码 | ONNX Runtime 推理 + PD 控制 + FSM |
| 训练配置 | 奖励函数、课程学习、域随机化配置 |

---

## 6. 时间线总览

```
Week  1-2  ████████ Phase 1: 环境搭建
Week  2-3  ████     Phase 2: 模型导入
Week  3-4  ████████ Phase 3: 训练环境配置
Week  4-6  ████████████████ Phase 4: 奖励函数设计
Week  6-8  ████████████████ Phase 5: 训练执行
Week  8-10 ████████████████ Phase 6: 模型导出与部署
Week 10-12 ████████████████ Phase 7: 联调与优化
```

**总计：约 12 周（3 个月）**

---

**文档版本**：1.0
**创建日期**：2026-07-16
**参考**：[robot_mc_analysis.md](./robot_mc_analysis.md) 第 11 章
