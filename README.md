# MATRiX

<div align="center">
  <a href="#">
    <img alt="Forest" src="demo_gif/Forest.png" width="800" height="450"/>
  </a>
</div>

<div align="center">

[![English](https://img.shields.io/badge/Language-English-blue)](README.md)
[![中文](https://img.shields.io/badge/语言-中文-red)](docs/README_CN.md)

</div>

> **Last Updated:** 2026-01-06

MATRiX is an advanced simulation platform that integrates **MuJoCo**, **Unreal Engine 5**, and **CARLA** to provide high-fidelity, interactive environments for quadruped robot research. Its software-in-the-loop architecture enables realistic physics, immersive visuals, and optimized sim-to-real transfer for robotics development and deployment.

## 🚀 Quick Start

### 1. Environment Dependencies
- **OS:** Ubuntu 22.04
- **GPU:** NVIDIA RTX 4060 or above (Driver >= 535)
- **Tools:** GCC/G++ ≥ C++11, CMake ≥ 3.16
- **ROS:** `ROS_humble`

### 2. Installation
```bash
# Install LCM
sudo apt update && sudo apt install -y cmake-qt-gui gcc g++ libglib2.0-dev python3-pip
# (Download and install LCM from source: https://github.com/lcm-proj/lcm/releases)

# Clone & Build
git clone https://github.com/zsibot/matrix.git
cd matrix
./scripts/build.sh

# Install Assets (Modular)
bash scripts/release_manager/install_chunks.sh 0.2.2
```
*See [Chunk Packages Guide](docs/CHUNK_PACKAGES_GUIDE.md) for offline/manual installation.*

### 3. Run Simulation
```bash
./bin/sim_launcher
```
*(Select your robot and map in the launcher interface.)*

<div align="center">
  <img src="demo_gif/Launcher.png" alt="Simulation Running Example" width="640" height="360"/>
</div>

## 📚 Documentation Directory

To keep this README concise, detailed guides have been organized into the `docs/` folder:

**Basics & Setup**
- [📦 Chunk Packages Guide](docs/CHUNK_PACKAGES_GUIDE.md) - Modular package deployment & offline install
- [🎮 Controller Guide](docs/Controller_Guide.md) - Gamepad & Keyboard control mappings
- [🛠️ Scripts Guide](docs/Scripts_Guide.md) - Detailed CLI scripts usage

**Simulation & Customization**
- [🤖 Robot Types & Maps](docs/Robots_and_Maps.md) - IDs and visual previews of all robots and maps
- [⚙️ Sensor Configuration](docs/Sensor_Config_Tutorial.md) - Adjusting cameras, LiDAR, and RViz visualization
- [🌍 Custom Scene Guide](docs/Custom_Scene_Tutorial.md) - Building custom JSON-based environments
- [🐕 Custom Robot Tutorial](docs/Custom_Robot_Tutorial.md) - Importing your own MuJoCo URDF models

**Advanced Features**
- [🌐 Multi-Robot Tutorial](docs/Multi_Robot_Tutorial.md) - Simulating multiple robots simultaneously
- [🐳 Docker Tutorial](docs/Docker_Tutorial.md) - Running MATRiX in a container
- [📡 RoamerX Lite Integration](docs/RoamerX_Lite_Integration.md) - ROS2 Nav2 stack integration
- [🎥 Pixel Streaming](docs/pixelstreaming_tutorial.md) - Web browser streaming

## 💬 Community

**Join our WeChat group for MATRiX simulation discussions:**

<div align="center">
  <img src="demo_gif/wechat.png" alt="WeChat Group QR Code" style="height: 280px; width: auto; margin: 0 12px;"/>
  <p><em>Scan to join MATRiX simulation community</em></p>
</div>

## 🙏 Acknowledgements

This project builds upon the incredible work of the following open-source projects:

- [MuJoCo-Unreal-Engine-Plugin](https://github.com/oneclicklabs/MuJoCo-Unreal-Engine-Plugin)
- [MuJoCo](https://github.com/google-deepmind/mujoco)
- [Unreal Engine](https://github.com/EpicGames/UnrealEngine)
- [CARLA](https://carla.org/)