# MATRiX Chunk Packages Guide

## 📦 What are Chunk Packages?

MATRiX now supports modular packaging, splitting simulator content into:
- **Assets Package**: Contains simulator launcher and core binaries (Required).
- **Base Package**: Essential core files and EmptyWorld map (Required).
- **Shared Resources Package**: Resources shared across multiple maps (Recommended).
- **Map Packages**: Individual maps that can be downloaded on demand.

This design allows users to:
- ✅ Download only what is needed, saving storage space.
- ✅ Quick start (only Assets and Base Packages required).
- ✅ Expand on demand (download specific maps as needed).

## 🚀 Quick Installation

### Automatic Installation (Recommended)

```bash
bash scripts/release_manager/install_chunks.sh 0.2.2
```

### Manual Installation

1. **Prepare Directory**

   Enter `releases` directory in the project root:
   ```bash
   cd releases
   ```

2. **Download Packages to releases Directory**

   - **Download Assets Package** (Required)
     ```bash
     wget https://github.com/zsibot/matrix/releases/download/v0.2.2/assets-0.2.2.tar.gz
     ```

   - **Download Base Package** (Required)
     ```bash
     wget https://github.com/zsibot/matrix/releases/download/v0.2.2/base-0.2.2.tar.gz
     ```

   - **Download Shared Resources Package** (Recommended)
     ```bash
     wget https://github.com/zsibot/matrix/releases/download/v0.2.2/shared-0.2.2.tar.gz
     ```

   - **Download Map Packages** (On Demand)
     ```bash
     wget https://github.com/zsibot/matrix/releases/download/v0.2.2/SceneWorld-0.2.2.tar.gz
     ```

3. **Run Local Installation Script**

   Return to the project root and run the installation script:
   ```bash
   cd ..
   bash scripts/release_manager/install_chunks_local.sh 0.2.2
   ```

## 📋 Package Description

### Assets Package (assets-0.2.2.tar.gz) - Required
- **Size**: ~1020MB
- **Contents**:
  - `bin/sim_launcher`: Simulator launcher
  - `bin/sim_launcher.bin`: Native launcher binary
  - MuJoCo runtime binary and dynamic map payloads
  - MC runtime binaries and shared libraries, including WBC libraries
  - ONNX controller models for the published robots: `xg`, `xg_wheel`, `zg_wheels`
  - UE runtime dependencies, including OpenCV shared libraries
- **Required**: ✅ Yes

### Base Package (base-0.2.2.tar.gz) - Required
- **Size**: ~2.0GB
- **Contents**:
  - EmptyWorld Map
  - Core Blueprints and System Files
  - Chunk 0 (pakchunk0)
  - Published robot model directories: `xgb`, `xgw`, `zgws`, `go2`, `go2w`
  - Runtime template directories: `Content/model/config` and `Content/model/SceneLoder`
- **Not included**:
  - `xxg` and other unpublished robot model directories
  - `Content/model/dynamicmap`, which is created at runtime for MoonWorld from `dynamicmaps/moonworld.bin`
- **Required**: ✅ Yes

### Shared Resources Package (shared-0.2.2.tar.gz) - Recommended
- **Size**: ~3.3GB
- **Contents**:
  - Fab/Carla Shared Resources
  - Blueprints and Resources shared by multiple maps
  - Chunk 1 (pakchunk1)
- **Required**: ⚠️ No, but many maps depend on it. Strongly recommended.

### Map Packages - Optional

| Package Name | Size | Chunk ID | Description |
|--------------|------|----------|-------------|
| 3DGSWorld | ~207MB | 25 | 3D Gaussian Splatting Map |
| ApartmentWorld | ~504MB | - | Apartment Scene |
| CaliWorld | ~16MB | - | Calibration Scene |
| CrowdWorld | ~41MB | 14 | Crowd Scene |
| CustomWorld | ~20MB | 24 | Custom Scene |
| HouseWorld | ~385MB | 17 | House Scene |
| IROSFlatWorld | ~300KB | 18 | IROS Flat Terrain |
| IROSFlatWorld2025 | ~160KB | 21 | IROS 2025 Flat Terrain |
| IROSSlopedWorld | ~251MB | 19 | IROS Sloped Terrain |
| IROSSloppedWorld2025 | ~160KB | 22 | IROS 2025 Sloped Terrain |
| MeetRoomWorld | ~151MB | - | Meeting Room Scene |
| MoonWorld | ~605MB | 26 | Moon Environment |
| OfficeWorld | ~414MB | 23 | Office Scene |
| RunningWorld | ~36MB | 16 | Running Game Scene |
| SceneWorld | ~381MB | 11 | Warehouse Scene |
| Town10World | ~1.1GB | 12 | Large Town Scene |
| Town10Zombie | ~631MB | 20 | Zombie Scene (Large) |
| VeniceWorld | ~329MB | 15 | Venice Scene |
| YardWorld | ~656MB | 13 | Courtyard Scene |

## 🔍 Verify Installation

After installation, check:

```bash
# 1. Check launcher assets
ls -lh bin/sim_launcher bin/sim_launcher.bin

# 2. Check published robot models only
find src/UeSim/Linux/zsibot_mujoco_ue/Content/model -mindepth 1 -maxdepth 1 -type d -printf '%f\n' | sort
find src/robot_mujoco/zsibot_robots -mindepth 1 -maxdepth 1 -type d -printf '%f\n' | sort

# 3. Check released ONNX model directories
find src/robot_mc/build/export/onnx_model_crypto -mindepth 1 -maxdepth 1 -type d -printf '%f\n' | sort

# 4. Check PAK files
cd src/UeSim/Linux/zsibot_mujoco_ue/Content/Paks
ls -lh pakchunk*.pak
```

You should see:
- `pakchunk0-Linux.pak` - Base Package (Required)
- `pakchunk1-Linux.pak` - Shared Resources Package (if installed)
- `pakchunk11-Linux.pak` etc. - Map Packages (if installed)
- UeSim robot model directories: `SceneLoder`, `config`, `go2`, `go2w`, `xgb`, `xgw`, `zgws`
- MuJoCo robot mirror directories: `go2`, `go2w`, `xgb`, `xgw`, `zgws`
- ONNX model directories: `xg`, `xg_wheel`, `zg_wheels`

## 🎮 Usage

After installation, run the simulator:

```bash
# In matrix root directory
./scripts/run_sim.sh 1 0  # XGB Robot, CustomWorld Map
./scripts/run_sim.sh 1 1  # XGB Robot, Warehouse Map (Requires SceneWorld package)
```

## ❓ FAQ

**Q: I only want to run EmptyWorld. Which packages do I need?**
A: You need the Assets Package (assets) and the Base Package (base).

**Q: Why is the Shared Resources Package recommended?**
A: Because many maps depend on assets in the Shared Resources Package. Without it, those maps may not load correctly.

**Q: Can I download only specific map packages?**
A: Yes! You can download only the map packages you need.

**Q: How do I update to a new version?**
A: Download the new version packages and extract them, overwriting the old files. It is recommended to backup first.

## 📚 More Information

- [Main README](../README.md) - Project Main Documentation
- [Chinese Documentation](README_CN.md) - User Guide in Chinese
