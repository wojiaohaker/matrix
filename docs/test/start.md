CarlaUnreal + mujoco

一、UE

启动项目

```
cd /home/qiyuan/UnrealEngine/CarlaUE5

/home/qiyuan/UnrealEngine/UnrealEngine5_carla/Engine/Binaries/Linux/UnrealEditor /home/qiyuan/UnrealEngine/CarlaUE5/Unreal/CarlaUnreal/CarlaUnreal.uproject
```

二、启动键盘脚本

```
cd /home/qiyuan/UnrealEngine/CarlaUE5

sudo python3 /home/qiyuan/UnrealEngine/CarlaUE5/Unreal/CarlaUnreal/Plugins/MuJoCoUE/Scripts/keyboard_control.py
```

三、mc_ctrl

```
cd /home/qiyuan/Softwares/Matrix/src/robot_mc/build/export/mc/bin

# 必须！否则 bind 失败走备用通道
export SDK_CLIENT_IP=127.0.0.1
export ROBOT_TYPE=XG
export LD_LIBRARY_PATH="$(pwd)/build/export/mc/bin:${LD_LIBRARY_PATH:-}"

taskset -c 7 ./mc_ctrl r 2>&1 | tee /tmp/mc_ctrl_carlaunreal.log
```



Matrix

一、sim_launcher

```
cd /home/qiyuan/Softwares/Matrix

./bin/sim_launcher
```

二、ue

```
cd /home/qiyuan/Softwares/Matrix/src/UeSim/Linux

# 必须！
export LD_LIBRARY_PATH="$(pwd)/zsibot_mujoco_ue/Binaries/Linux:$(pwd)/Engine/Binaries/Linux:$(pwd)/Engine/Plugins/Runtime/OpenCV/Binaries/ThirdParty/Linux:${LD_LIBRARY_PATH:-}"

./zsibot_mujoco_ue.sh -game /Game/Maps/YardWorld -ExecCmds="t.MaxFPS 30"
```

三、mc_ctrl

```
cd /home/qiyuan/Softwares/Matrix/src/robot_mc/build/export/mc/bin

export ROBOT_TYPE=XG
export SDK_CLIENT_IP=127.0.0.1
export LD_LIBRARY_PATH="$(pwd)/build/export/mc/bin:${LD_LIBRARY_PATH:-}"

taskset -c 7 ./mc_ctrl r 2>&1 | tee /tmp/mc_ctrl_matrix.log
```

