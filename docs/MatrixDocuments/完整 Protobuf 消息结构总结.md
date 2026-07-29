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