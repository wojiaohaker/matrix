#!/bin/bash
set -e

# Get script directory and project root
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
cd "$PROJECT_ROOT"

echo "Installing system dependencies "

DEPS_DIR="deps"

if [ ! -d "$DEPS_DIR" ]; then
    echo "ERROR: Dependencies directory not found: $DEPS_DIR"
    exit 1
fi

install_deb_glob() {
    local matched=0
    local pkg

    for pkg in "$@"; do
        if [ -f "$pkg" ]; then
            sudo dpkg -i "$pkg"
            matched=1
        fi
    done

    if [ "$matched" -eq 0 ]; then
        echo "ERROR: No package matched: $*"
        exit 1
    fi
}

sudo apt-get install protobuf-compiler -y
sudo apt-get install libspdlog-dev -y
sudo apt install libglfw3-dev libxinerama-dev libxcursor-dev libxi-dev libyaml-cpp-dev -y
sudo apt-get install libglib2.0-dev mesa-common-dev freeglut3-dev coinor-libipopt-dev libblas-dev liblapack-dev gfortran liblapack-dev libboost-all-dev libeigen3-dev -y
sudo apt install libhdf5-dev -y
sudo apt install libqt5core5a -y
sudo apt install libqt5gui5 -y
sudo apt install libqt5svg5-dev -y
sudo apt install libqt5widgets5 -y
sudo apt install curl -y
sudo apt install cmake-qt-gui -y
sudo apt install g++ gcc -y
sudo apt install libopencv-dev -y
sudo apt install jq -y
sudo apt install ros-humble-image-transport* -y
sudo apt install qtcreator -y
sudo apt install qtquickcontrols2-5-dev -y
sudo apt install qml-module-qtquick-controls2 -y
sudo apt install libqt5x11extras5-dev

install_deb_glob "$DEPS_DIR"/lcm_*.deb
install_deb_glob "$DEPS_DIR"/zsibot_common_*.deb
install_deb_glob "$DEPS_DIR"/robot-forward_*.deb
install_deb_glob "$DEPS_DIR"/ecal_*.deb
install_deb_glob "$DEPS_DIR"/mujoco_*.deb
install_deb_glob "$DEPS_DIR"/onnx_*.deb

sudo apt install -y qtbase5-dev qtchooser qt5-qmake qtbase5-dev-tools \
    qml-module-qtquick-controls qml-module-qtquick-controls2 \
    qml-module-qtquick-layouts qml-module-qtgraphicaleffects \
    qml-module-qtqml-models2 qml-module-qtqml qml-module-qtquick-window2
sudo apt install fonts-noto-color-emoji -y

echo "System dependencies installed."
