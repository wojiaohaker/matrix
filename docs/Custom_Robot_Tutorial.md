# Custom Robot Dog Tutorial

This document explains how to integrate your own quadruped MuJoCo robot model into MATRiX, run your controller in MuJoCo mode, and synchronize the robot state from MuJoCo back to Unreal Engine (UE) to complete the full simulation workflow.

## 1. What This Feature Does

MATRiX currently provides a built-in `custom` robot type to support user-defined quadruped robots.

The full workflow can be summarized as follows:

1. Prepare your custom URDF/XML and mesh assets.
2. Update the IMU and other sensor definitions according to your robot structure.
3. Let the custom import wrapper generate and synchronize the UE-side and MuJoCo-side runtime XML.
4. Select the `custom` robot in `sim_launcher` and enable MuJoCo mode.
5. Use your own MuJoCo API controller to drive the robot.

In simple terms:

- MuJoCo handles robot dynamics and control execution.
- UE handles environment rendering, sensor simulation, and overall visualization.
- MATRiX connects these two parts together.

## 2. Key File Locations

The custom quadruped workflow mainly involves the following runtime files. These paths are created by the custom URDF import flow when the launcher delegates to `scripts/run_custom_urdf.sh`; they are not shipped as prebuilt `custom` model content in the base package.

- Custom robot model on the UE side:
  - `src/UeSim/Linux/zsibot_mujoco_ue/Content/model/custom/current.xml`
- Custom robot model on the MuJoCo side:
  - `src/robot_mujoco/zsibot_robots/custom/current.xml`
- Custom robot scene entry file:
  - `src/robot_mujoco/zsibot_robots/custom/scene_terrain_custom.xml`
- Runtime robot configuration file:
  - `config/config.json`

Please pay special attention to the following points:

- `scene_terrain_custom.xml` directly includes the active custom XML.
- The import flow creates and synchronizes the `assets/` folder.
- Your robot mesh file paths must match the filenames referenced in the imported XML.

## 3. Default Model Description

The released base package does not include a prebuilt `custom` robot directory. A custom robot is generated at runtime from your URDF/XML input and cached under `src/robot_mujoco/zsibot_robots/custom/_cache/` and `src/UeSim/Linux/zsibot_mujoco_ue/Content/model/custom/_cache/`.

This template already includes:

- The robot rigid-body structure
- Joint and actuator definitions
- IMU-related sites and sensors
- Additional sensor pose definitions such as `livox_imu` and `camera_imu`

For controller-compatible URDFs, the import flow may reuse one of the published reference layouts: `xgb`, `xgw`, `zgws`, `go2`, or `go2w`. The `xxg` reference profile is intentionally not part of the v0.2.2 public release package.

## 4. Step 1: Import Your Custom Model

First, prepare your URDF/XML and mesh assets. When launching from `sim_launcher`, select the `custom` robot and provide the custom model file so `scripts/run_custom_urdf.sh` can import it.

If you are debugging manually, the active generated files are:

- `src/UeSim/Linux/zsibot_mujoco_ue/Content/model/custom/current.xml`
- `src/robot_mujoco/zsibot_robots/custom/current.xml`

Do not rely on a pre-existing `Content/model/custom` directory after a clean install; it is runtime-generated.

In most cases, your model should include at least the following parts:

- `<asset>`: mesh assets and file references
- `<worldbody>`: robot rigid-body tree structure
- `<joint>`: joint definitions
- `<actuator>`: motor or actuator definitions
- `<sensor>`: sensor definitions

When replacing the model, check the following carefully:

- Whether link names and joint names are consistent throughout the file
- Whether mesh filenames match the files in `assets/`
- Whether the robot base body can be loaded correctly as the root rigid body
- Whether the model can be parsed by MuJoCo without XML errors

If your robot uses new mesh files, place them next to your input URDF under an `assets/` or `meshes/` directory. The import wrapper copies them into:

- `src/UeSim/Linux/zsibot_mujoco_ue/Content/model/custom/assets/`
- `src/robot_mujoco/zsibot_robots/custom/assets/`

Also make sure the filenames match the mesh references exactly.

## 5. Step 2: Update the IMU and Other Sensors

After preparing the robot body, you also need to update the sensor-related parts in your source model.

This is very important because sensor poses are robot-structure-specific, for example:

- `imu`
- `livox_imu`
- `camera_imu`
- `this_camera`

If your robot dimensions, body structure, or sensor mounting positions are different, these parameters must be updated accordingly.

Typical updates include:

- The position and orientation of each `site`
- The mounting position of the main IMU
- The mounting position of the camera
- The position of LiDAR-related IMUs or auxiliary sites
- The site names referenced in the `<sensor>` section

For example, if you rename a site in `<worldbody>`, then the corresponding entries in `<sensor>` must also be updated, including:

- `framequat`
- `gyro`
- `accelerometer`
- `framepos`
- `framelinvel`

## 6. Step 3: Synchronize the Runtime Copies

The import wrapper synchronizes the UE-side and MuJoCo-side copies automatically:

- UE side: `src/UeSim/Linux/zsibot_mujoco_ue/Content/model/custom/current.xml`
- MuJoCo side: `src/robot_mujoco/zsibot_robots/custom/current.xml`

These two files should stay equivalent for the active custom robot.

The reason is:

- The UE-side path is used for simulator content resource loading.
- The MuJoCo-side path is used when loading the MuJoCo scene under `src/robot_mujoco/zsibot_robots/custom/`.

If you change the source URDF or mesh resources, re-import the custom robot from the launcher. For a forced refresh, set `SIM_LAUNCHER_FORCE_REIMPORT_CUSTOM_URDF=1`.

## 7. Step 4: Select the `custom` Robot

The runtime robot type is controlled by `config/config.json`.

For a custom robot dog, it should be set to:

- `"robot_type": "custom"`

In the current repository, `config/config.json` is already configured to use `custom` by default, so if you are following the custom quadruped workflow, this setting is usually already correct.

### Important: Select `CustomWorld` in the Launcher

When you import and launch a URDF-based custom robot from `sim_launcher`, the map selection must be:

- Map: `CustomWorld`

This is not just a recommendation. The current custom URDF runtime uses the fixed scene entry:

- `custom/scene_terrain_custom.xml`

If you select another map such as `YardWorld`, the launcher may still hand off the custom robot to the fixed `custom/scene_terrain_custom.xml` path while other parts of the runtime continue to expect a map-specific scene such as `scene_terrain_yard.xml`. That mismatch can lead to startup failures or inconsistent MuJoCo/UE behavior.

For the current workflow, treat the following combination as required:

- Robot type: `custom`
- Map: `CustomWorld`
- MuJoCo mode: enabled

## 8. Step 5: Enable MuJoCo Mode

The core execution path for a custom robot dog is MuJoCo dynamics simulation, so you need to enable MuJoCo mode in `sim_launcher`.

At the configuration level, this means:

- Set `mujoco_running` to `true` in `config/config.json`

In actual use, you usually do not need to edit this manually, because `run_sim.sh` automatically rewrites it based on the launcher selection:

- If MuJoCo mode is enabled in the launcher, it will automatically be written as `"mujoco_running": true`
- If it is not enabled, it will be reset to `false`

For a custom robot dog, the following combination is recommended at all times:

- Robot type: `custom`
- MuJoCo mode: enabled

Otherwise, your custom MuJoCo robot model will not enter the intended dynamics-control workflow.

## 9. Step 6: Understand How the Custom Robot Scene Is Loaded

The MuJoCo scene entry used by the custom robot dog is:

- `src/robot_mujoco/zsibot_robots/custom/scene_terrain_custom.xml`

This scene file directly includes:

- the active custom XML generated by the import wrapper

In other words, the robot body ultimately loaded by MuJoCo is the active generated custom XML.

At the same time, `run_sim.sh` writes the following file:

- `src/robot_mujoco/simulate/config.yaml`

The key fields will become:

- `robot: "custom"`
- `robot_scene: "scene_terrain_custom.xml"`

This is why the generated active custom XML is the core file in the entire custom quadruped workflow.

## 10. Step 7: Integrate Your Own MuJoCo API Controller

Once your custom robot model is ready, the next step is to use your own controller to drive the robot through the MuJoCo API.

Typically, your controller needs to do the following:

- Read the robot state from MuJoCo
- Compute target joint commands
- Write torques, positions, or other control signals back to MuJoCo
- Implement your own gait, balance, navigation, or policy logic

You can understand MATRiX custom robot support as providing:

- An entry point for custom robot models
- A MuJoCo runtime environment and scene entry
- UE-side display and environment simulation capabilities

How the robot actually moves and how it is controlled are handled by your MuJoCo controller.

## 11. Step 8: Synchronize MuJoCo State Back to UE

When the custom robot dog runs in MuJoCo mode:

- MuJoCo computes robot dynamics and updates the state
- MATRiX synchronizes the robot state back to UE
- UE displays the robot motion inside the full scene

This allows you to get both:

- More realistic robot dynamics simulation on the MuJoCo side
- Rich environment rendering and full-scene simulation on the UE side

This is the core value of the custom robot workflow.

## 12. Recommended Workflow

To reduce problems, it is recommended to follow the steps below:

1. Prepare your own MuJoCo robot XML and mesh assets.
2. Import it through `sim_launcher` with robot type `custom`.
3. Update the IMU, camera, and other sensor definitions.
4. Let the import wrapper synchronize generated XML and `assets/` into both runtime locations.
5. Select the `custom` robot in the launcher.
6. Enable MuJoCo mode.
7. Start the simulation.
8. Run your own MuJoCo controller.
9. Check whether the robot motion shown in UE matches the MuJoCo execution result.

## 13. Common Issues

### Issue 1: The Robot Model Cannot Be Loaded

Possible reasons:

- The source URDF/XML contains syntax errors
- Referenced mesh files are missing from `assets/`
- Mesh filenames do not match the references in the XML
- Body or joint naming is inconsistent

### Issue 2: The Robot Loads, but IMU or Camera Data Is Incorrect

Possible reasons:

- The `site` positions still use parameters from another robot layout
- The `<sensor>` section still references old site names
- The camera pose was not updated for the new robot structure

### Issue 3: MuJoCo-Side and UE-Side Behavior Are Inconsistent

Possible reasons:

- The runtime XML was edited manually on only one side
- Resource files were imported to only one side
- MuJoCo mode was not enabled in the launcher

### Issue 4: The Robot Does Not Move as Expected

Possible reasons:

- Your controller is not sending commands correctly
- The actuator definitions do not match the controller assumptions
- The joint order or naming does not match the control code

## 14. Pre-Launch Checklist

Before starting the simulation, confirm the following:

- Your robot model is valid MuJoCo XML
- The generated UE-side and MuJoCo-side runtime XML are equivalent
- Both `assets/` directories contain all referenced mesh files
- The IMU and other sensor definitions match the robot structure
- `robot_type` is set to `custom`
- MuJoCo mode is enabled in the launcher
- Your MuJoCo controller is ready

## 15. Summary

Integrating a custom robot dog into MATRiX is not complicated. The key steps are:

- Prepare your custom URDF/XML and mesh assets
- Update the IMU and other sensors according to their real mounting positions
- Let the import wrapper keep runtime XML and related assets synchronized
- Select `custom` and enable MuJoCo mode in `sim_launcher`
- Drive the robot using your own MuJoCo API controller

Once this pipeline is working end to end, you can run your own quadruped model in MATRiX and use the combined MuJoCo + UE simulation workflow for development and validation.
