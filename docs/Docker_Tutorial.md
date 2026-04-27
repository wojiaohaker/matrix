# Docker Tutorial

This guide explains how to build and run MATRiX in Docker on Linux with GPU acceleration and X11 display forwarding.

## What This Tutorial Does

Using the provided Docker setup, you can:

- build a ready-to-use development image;
- start a container with NVIDIA GPU passthrough;
- display GUI applications from the container on the host;
- mount the current repository into the container at `/workspace`.

The recommended launch script is `scripts/docker_run_gpu.sh`.

## Prerequisites

Before you begin, make sure the following are installed on the host machine:

- **Docker**: install it from the official guide: https://docs.docker.com/engine/install/
- **Docker post-install configuration**: follow https://docs.docker.com/engine/install/linux-postinstall/ if you want to run Docker without `sudo`
- **NVIDIA Container Toolkit**: install it from https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#installation-guide so Docker containers can access the GPU
- **A Linux desktop session with X11**: the provided script forwards the host display into the container

## Important Notes Before Running

- Run all commands from the **repository root**.
- The run script mounts `$(pwd)` to `/workspace`, so if you start it from another directory, the wrong path will be mounted.
- The script is designed for Linux hosts with NVIDIA GPUs.
- If you are using Docker without `sudo`, simply remove `sudo` from the examples below.

## Step 1: Build the Docker Image

> **💡 对于国内用户的提示 (For users in China):**
>
> 在构建 Docker 镜像期间，为了加速 `apt-get`、`pip` 或 `ros` 依赖的下载，`Dockerfile` 中配置了本地的 HTTP/HTTPS 代理：
> ```dockerfile
> # Configure bashrc for root (Proxy + PS1 + ROS)
> RUN echo "export http_proxy=http://127.0.0.1:7897" >> /root/.bashrc && \
>     echo "export https_proxy=http://127.0.0.1:7897" >> /root/.bashrc
> ```
> 如果你有自己的代理软件（如 Clash、V2Ray 等），请**务必在构建前修改 `Dockerfile` 中对应的端口号**（如这里是 `7897`），并确保代理软件允许局域网连接。
> 同样地，请检查并修改文件中为 `matrix_user` 设置的代理端口。
> 如果你不需要代理，可以直接将 `Dockerfile` 中涉及 `http_proxy` 和 `https_proxy` 的行用 `#` 注释掉或删除。

From the repository root, build the image:

```bash
bash docker/docker_build_image.sh
```

### What This Command Does

- builds an image named `zsibot/matrix:latest`;
- uses the repository root `.` as the build context;
- installs system dependencies, ROS 2 Humble, and project dependencies;
- runs `build.sh` during image creation.

### Expected Result

If the build succeeds, Docker will finish without errors and you will have a local image named `zsibot/matrix:latest`.

You can verify it with:

```bash
sudo docker images | grep zsibot/matrix
```

## Step 2: Start the Container

Launch the provided GPU-enabled container (must be run on the host machine):

```bash
bash docker/docker_run_gpu.sh
```

## Step 3: Join the Container

Once the container is running in the background, you can enter it using:

```bash
bash docker/docker_join.sh
```

## What the Launch Script Does

The script `docker/docker_run_gpu.sh` automatically:

- removes any existing container named `MATRiX`;
- enables local X11 access with `xhost +local:root`;
- starts the container with `--gpus all` in detached mode;
- mounts the current repository to `/workspace`;
- mounts X11 and Vulkan-related host paths for GUI rendering;
- adds necessary hardware permissions to `matrix_user`;
- uses host networking.

## Inside the Container

After using `docker_join.sh` to enter the container.

Key behavior:

- the default user is `matrix_user`;
- the repository is available at `/workspace`;
- ROS 2 Humble is sourced automatically;
- if `/workspace/install/setup.bash` exists, it is sourced automatically too.


## Quick Start Summary

```bash
cd /path/to/matrix
bash docker/docker_build_image.sh
bash docker/docker_run_gpu.sh
bash docker/docker_join.sh
```
