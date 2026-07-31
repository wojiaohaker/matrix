# CarlaUnreal → mc_ctrl UDP 数据接收失败

## 问题现象

CarlaUnreal 启动后，mc_ctrl 日志中**完全没有** `Reciving data size: 303` 打印，
而 Matrix UE 正常运行时该日志有数千条。导致 mc_ctrl 的 FSM 卡在 STANDUP 状态，
body_height 始终为默认值 0.374875（FK 使用默认关节角），W 键无法前进。

## 排查过程

### 1. 确认端口匹配 ✓

- mc_ctrl 绑定 `0.0.0.0:25001` 接收 RobotState
- CarlaUnreal 发送到 `127.0.0.1:25001`
- 通过 `ss -ulnp` 确认 mc_ctrl 进程确实在监听

### 2. 确认 UE 端发送正常 ✓

在 UdpSenderComponent 添加诊断日志后确认：
```
[UDP-SEND-OK] MsgSize=316, BytesSent=316, TotalSent=1000, target=127.0.0.1:25001
```
UE 确实成功发送了 UDP 数据包，且 mc_ctrl 也在向 UE 发送命令（port 25002）。

### 3. 抓包对比 Matrix UE 的 303 字节包

使用 `SO_REUSEPORT` 共享端口抓取 Matrix UE 实际发送的数据，用 Python 解析 protobuf 结构：

```
Matrix UE (303 bytes) 包含 18 个字段:
  field  1 (q_abad)     : 4 floats
  field  2 (q_hip)      : 4 floats
  field  3 (q_knee)     : 4 floats
  field  4 (q_foot)     : 4 floats (全零)
  field  5 (qd_abad)    : 4 floats
  field  6 (qd_hip)     : 4 floats
  field  7 (qd_knee)    : 4 floats
  field  8 (qd_foot)    : 4 floats (全零)
  field  9 (tau_abad_fb): 4 floats
  field 10 (tau_hip_fb) : 4 floats
  field 11 (tau_knee_fb): 4 floats
  field 12 (tau_foot_fb): 4 floats (全零)
  field 13 (quat)       : 4 floats
  field 14 (gyro)       : 3 floats
  field 15 (acc)        : 3 floats
  field 17 (time_stamp) : varint = 1785468632919171000
  field 19 (position)   : 3 floats
  field 20 (v_world)    : 3 floats

注意: 没有 field 16 (rpy)！
```

## 根本原因（两个问题）

### 原因 1：时间戳基准错误（主因）

mc_ctrl 的 `reciveData()` 函数使用 **CLOCK_REALTIME**（epoch 以来的墙钟时间）
验证数据包新鲜度，要求 `|当前时间 - 包时间戳| < 10ms`。

| 时间源 | 值 | 与 mc_ctrl 期望的差距 |
|--------|------|------|
| CLOCK_REALTIME | 1,785,468,749 × 10⁹ ns | 0（正确）|
| CLOCK_MONOTONIC | 76,626 × 10⁹ ns | **20.6 天**（系统运行时长）|
| FPlatformTime::Seconds() | ~应用启动秒数 | **更大** |

CarlaUnreal 最初使用 `FPlatformTime::Seconds()`（自应用启动），后改为
`CLOCK_MONOTONIC`（自系统启动），两者都与 mc_ctrl 期望的 CLOCK_REALTIME 差距巨大，
导致**所有数据包被判定为过期而丢弃**。

### 原因 2：多余的 rpy 字段（次因）

CarlaUnreal 额外发送了 `rpy` 字段（field 16, 3 floats），导致消息大小为 316 字节
而非标准的 303 字节。虽然 protobuf 理论上可忽略未知字段，但为精确匹配 Matrix UE
的线上格式，应移除此字段。

## 修复方案

文件: `Unreal/CarlaUnreal/Plugins/MuJoCoUE/Source/MuJoCoUE/Private/UdpSenderComponent.cpp`

### 修复 1：时间戳改用 CLOCK_REALTIME

```cpp
#include <time.h>

// 在 UpdateState() 中:
struct timespec ts;
clock_gettime(CLOCK_REALTIME, &ts);
uint64 NowNs = static_cast<uint64>(ts.tv_sec) * 1000000000ULL
             + static_cast<uint64>(ts.tv_nsec);
StateMsg->set_time_stamp(NowNs);
```

### 修复 2：移除 rpy 字段

```cpp
// 注释掉 rpy 序列化代码:
// if (RPY.Num() >= 3) {
//     StateMsg->clear_rpy();
//     for (int32 i = 0; i < 3; i++)
//         StateMsg->add_rpy(RPY[i]);
// }
```

### 修复 3：禁用 bAutoSend 避免空包

```cpp
void UUdpSenderComponent::BeginPlay()
{
    Super::BeginPlay();
    bAutoSend = false;  // 由 MuJoCoSimulation 显式调用 UpdateState()+SendState()
    InitSocket();
}
```

## 验证方法

1. 重新编译 MuJoCoUE 插件
2. 启动 CarlaUnreal 编辑器 (L_Mujoco_Test_xgb 地图)
3. 启动 mc_ctrl: `taskset -c 7 ./mc_ctrl r 2>&1 | tee /tmp/mc_ctrl_carlaunreal.log`
4. 检查 mc_ctrl 日志出现 `Reciving data size: 303`
5. FSM 从 STANDUP 正常过渡后，W 键可前进

## 关键教训

- **跨进程通信的时间戳必须使用相同的时钟源**，不能想当然用 MONOTONIC
- 通过 `SO_REUSEPORT` 可以在不中断现有服务的情况下抓取 UDP 包内容
- 与已验证的二进制协议对接时，必须**逐字节匹配**线上格式，不能多发字段



太好了，终于成功了！让我把整个问题的原因和修复过程记录到文档中：


文档已创建：[carlaunreal_udp_timestamp_fix.md](file:///home/qiyuan/Softwares/Matrix/docs/issues/carlaunreal_udp_timestamp_fix.md)

记录了完整的排查过程和根因：

1. **主因**：时间戳用了 `CLOCK_MONOTONIC`（系统启动 21h ≈ 7.6×10¹³ ns），而 mc_ctrl 期望 `CLOCK_REALTIME`（epoch 以来 ≈ 1.78×10¹⁸ ns），差距 20.6 天，所有数据被判定过期丢弃
2. **次因**：多发了 `rpy` 字段导致 316 字节 ≠ 标准 303 字节

恭喜终于通了！这个问题从端口确认 → 发送确认 → 抓包对比 → 时钟源定位，跨了好几个会话，确实不容易。
