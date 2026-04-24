# MATRiX Chunk Packages 使用指南

## 📦 什么是Chunk Packages?

MATRiX现在支持模块化打包，将模拟器内容分为：
- **资源文件包**: 包含模拟器启动器和核心二进制文件（必需）
- **基础包**: 必需的核心文件和EmptyWorld地图（必需）
- **共享资源包**: 多个地图共享的资源（推荐安装）
- **地图包**: 各个独立的地图，可按需下载

这种设计让用户可以：
- ✅ 只下载需要的内容，节省存储空间
- ✅ 快速开始（只需下载基础包和资源包）
- ✅ 按需扩展（需要哪个地图再下载）

## 🚀 快速安装

### 自动安装（推荐）

```bash
bash scripts/release_manager/install_chunks.sh 0.2.2
```

### 手动安装

1. **准备目录**

   在项目根目录进入 `releases` 目录：
   ```bash
   cd releases
   ```

2. **下载包到 releases 目录**

   - **下载资源文件包**（必需）
     ```bash
     wget https://github.com/zsibot/matrix/releases/download/v0.2.2/assets-0.2.2.tar.gz
     ```

   - **下载基础包**（必需）
     ```bash
     wget https://github.com/zsibot/matrix/releases/download/v0.2.2/base-0.2.2.tar.gz
     ```

   - **下载共享资源包**（推荐）
     ```bash
     wget https://github.com/zsibot/matrix/releases/download/v0.2.2/shared-0.2.2.tar.gz
     ```

   - **下载地图包**（按需）
     ```bash
     wget https://github.com/zsibot/matrix/releases/download/v0.2.2/SceneWorld-0.2.2.tar.gz
     ```

3. **执行本地安装脚本**

   回到项目根目录并运行安装脚本：
   ```bash
   cd ..
   bash scripts/release_manager/install_chunks_local.sh 0.2.2
   ```

## 📋 包说明

### 资源文件包 (assets-0.2.2.tar.gz) - 必需
- **大小**: ~1020MB
- **内容**:
  - `bin/sim_launcher`: 模拟器启动器
  - `bin/sim_launcher.bin`: 原生启动器二进制
  - MuJoCo 运行时二进制和动态地图数据
  - MC 运行时二进制和共享库，包括 WBC 库
  - 已发布机器人需要的 ONNX 控制模型：`xg`、`xg_wheel`、`zg_wheels`
  - UE 运行时依赖，包括 OpenCV 共享库
- **必需**: ✅ 是

### 基础包 (base-0.2.2.tar.gz) - 必需
- **大小**: ~2.0GB
- **内容**:
  - EmptyWorld地图
  - 核心蓝图和系统文件
  - Chunk 0 (pakchunk0)
  - 已发布机器人模型目录：`xgb`、`xgw`、`zgws`、`go2`、`go2w`
  - 运行时模板目录：`Content/model/config` 和 `Content/model/SceneLoder`
- **不包含**:
  - `xxg` 和其他未发布机器人模型目录
  - `Content/model/dynamicmap`，MoonWorld 运行时会从 `dynamicmaps/moonworld.bin` 创建并拷贝
- **必需**: ✅ 是

### 共享资源包 (shared-0.2.2.tar.gz) - 推荐
- **大小**: ~3.3GB
- **内容**:
  - Fab/Carla共享资源
  - 多个地图共享的蓝图和资源
  - Chunk 1 (pakchunk1)
- **必需**: ⚠️ 否，但多个地图依赖，强烈建议安装

### 地图包 - 可选

| 地图包 | 大小 | Chunk ID | 说明 |
|--------|------|----------|------|
| 3DGSWorld | ~207MB | 25 | 3D 高斯地图 |
| ApartmentWorld | ~504MB | - | 公寓场景 |
| CaliWorld | ~16MB | - | 标定场景 |
| CrowdWorld | ~41MB | 14 | 人群场景 |
| CustomWorld | ~20MB | 24 | 自定义场景 |
| HouseWorld | ~385MB | 17 | 房屋场景 |
| IROSFlatWorld | ~300KB | 18 | IROS 平地场景 |
| IROSFlatWorld2025 | ~160KB | 21 | IROS 2025 平地场景 |
| IROSSlopedWorld | ~251MB | 19 | IROS 斜坡场景 |
| IROSSloppedWorld2025 | ~160KB | 22 | IROS 2025 斜坡场景 |
| MeetRoomWorld | ~151MB | - | 会议室场景 |
| MoonWorld | ~605MB | 26 | 月球环境 |
| OfficeWorld | ~414MB | 23 | 办公室场景 |
| RunningWorld | ~36MB | 16 | 跑步场景 |
| SceneWorld | ~381MB | 11 | 仓库场景 |
| Town10World | ~1.1GB | 12 | 城镇场景（大） |
| Town10Zombie | ~631MB | 20 | 僵尸场景（大） |
| VeniceWorld | ~329MB | 15 | 威尼斯场景 |
| YardWorld | ~656MB | 13 | 庭院场景 |

## 🔍 验证安装

安装后检查：

```bash
# 1. 检查启动器资源
ls -lh bin/sim_launcher bin/sim_launcher.bin

# 2. 检查仅发布的机器人模型
find src/UeSim/Linux/zsibot_mujoco_ue/Content/model -mindepth 1 -maxdepth 1 -type d -printf '%f\n' | sort
find src/robot_mujoco/zsibot_robots -mindepth 1 -maxdepth 1 -type d -printf '%f\n' | sort

# 3. 检查已发布的 ONNX 模型目录
find src/robot_mc/build/export/onnx_model_crypto -mindepth 1 -maxdepth 1 -type d -printf '%f\n' | sort

# 4. 检查 PAK 文件
cd src/UeSim/Linux/zsibot_mujoco_ue/Content/Paks
ls -lh pakchunk*.pak
```

应该看到：
- `pakchunk0-Linux.pak` - 基础包（必需）
- `pakchunk1-Linux.pak` - 共享资源包（如果已安装）
- `pakchunk11-Linux.pak` 等 - 地图包（如果已安装）
- UeSim 机器人模型目录：`SceneLoder`、`config`、`go2`、`go2w`、`xgb`、`xgw`、`zgws`
- MuJoCo 机器人镜像目录：`go2`、`go2w`、`xgb`、`xgw`、`zgws`
- ONNX 模型目录：`xg`、`xg_wheel`、`zg_wheels`

## 🎮 使用

安装完成后，运行模拟器：

```bash
# 已在 matrix 根目录
./scripts/run_sim.sh 1 0  # XGB机器人，CustomWorld地图
./scripts/run_sim.sh 1 1  # XGB机器人，Warehouse地图（需要SceneWorld地图包）
```

## ❓ 常见问题

**Q: 我只想运行EmptyWorld，需要下载哪些包？**
A: 需要资源文件包（assets）和基础包（base）。

**Q: 为什么共享资源包是推荐的？**
A: 因为多个地图都依赖共享资源包中的资源，如果不安装，这些地图可能无法正常加载。

**Q: 我可以只下载部分地图包吗？**
A: 可以！你可以根据需要只下载要使用的地图包。

**Q: 如何更新到新版本？**
A: 下载新版本的包，解压覆盖旧文件即可。建议先备份。

## 📚 更多信息

- [主 README](../README.md) - 项目主文档
- [中文文档](../docs/README_CN.md) - 中文使用指南
