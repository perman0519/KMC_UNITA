#!/usr/bin/env bash
set -e

echo "==== ROS2 Package Build Script ===="

# Check if we're in a ROS2 workspace by looking for the src directory structure
CURRENT_DIR=$(pwd)
if [[ "$CURRENT_DIR" =~ /src/ ]]; then
    echo "Detected ROS2 workspace structure"
    # Extract workspace root (everything before /src/)
    WORKSPACE_ROOT="${CURRENT_DIR%%/src/*}"
    echo "Workspace root: $WORKSPACE_ROOT"
else
    echo "WARNING: Not in a standard ROS2 workspace structure"
    echo "Expected path like: ~/ros2_ws/src/simulator"
    echo "Current path: $CURRENT_DIR"
    echo ""
fi

echo "==== Installing required Ubuntu packages ===="
sudo apt-get update
sudo apt-get install -y \
    build-essential cmake git pkg-config \
    libx11-dev libxext-dev libxrandr-dev \
    libxcursor-dev libxfixes-dev libxi-dev \
    libudev-dev \
    libgl1-mesa-dev
    
echo "==== Checking for SDL3 ===="
if pkg-config --exists sdl3 2>/dev/null; then
    SDL3_VERSION=$(pkg-config --modversion sdl3)
    echo "SDL3 is already installed (version $SDL3_VERSION)"
elif [ -f "/usr/local/lib/cmake/SDL3/SDL3Config.cmake" ]; then
    echo "SDL3 is already installed (found CMake config in /usr/local)"
else
    echo "==== SDL3 not found — installing from source ===="
    SDL3_TMP_DIR="$HOME/sdl3_build"
    SDL3_REPO="https://github.com/libsdl-org/SDL.git"
    rm -rf "$SDL3_TMP_DIR"
    mkdir -p "$SDL3_TMP_DIR"
    echo "Cloning SDL3..."
    git clone --branch release-3.2.2 --depth 1 "$SDL3_REPO" "$SDL3_TMP_DIR"
    echo "Building SDL3..."
    cd "$SDL3_TMP_DIR"
    cmake -S . -B build \
        -DCMAKE_BUILD_TYPE=Release \
        -DSDL_STATIC=OFF \
        -DSDL_SHARED=ON \
        -DSDL_TESTS=OFF
    cmake --build build --config Release -j$(nproc)
    echo "Installing SDL3..."
    sudo cmake --install build
    echo "Running ldconfig..."
    if [ -d "/usr/local/lib" ]; then
        sudo ldconfig /usr/local/lib
    else
        sudo ldconfig
    fi
    echo "SDL3 installed"
    cd "$CURRENT_DIR"
fi

echo "==== Detecting ROS2 distribution ===="
ROS_DISTRO_PATH=""
if [ -f "/opt/ros/humble/setup.bash" ]; then
    ROS_DISTRO_PATH="/opt/ros/humble"
    ROS_DISTRO_NAME="humble"
elif [ -f "/opt/ros/foxy/setup.bash" ]; then
    ROS_DISTRO_PATH="/opt/ros/foxy"
    ROS_DISTRO_NAME="foxy"
elif [ -f "/opt/ros/iron/setup.bash" ]; then
    ROS_DISTRO_PATH="/opt/ros/iron"
    ROS_DISTRO_NAME="iron"
elif [ -f "/opt/ros/jazzy/setup.bash" ]; then
    ROS_DISTRO_PATH="/opt/ros/jazzy"
    ROS_DISTRO_NAME="jazzy"
else
    echo "ERROR: No ROS2 distribution found!"
    echo "Please install ROS2. For Ubuntu 22.04, install Humble:"
    echo "  sudo apt update"
    echo "  sudo apt install ros-humble-desktop"
    echo "  sudo apt install ros-humble-rclcpp ros-humble-std-msgs ros-humble-geometry-msgs"
    exit 1
fi

echo "Found ROS2 $ROS_DISTRO_NAME at $ROS_DISTRO_PATH"
source "$ROS_DISTRO_PATH/setup.bash"

echo "==== Installing ROS2 dependencies ===="
sudo apt-get install -y \
    ros-${ROS_DISTRO_NAME}-rclcpp \
    ros-${ROS_DISTRO_NAME}-std-msgs \
    ros-${ROS_DISTRO_NAME}-geometry-msgs \
    python3-colcon-common-extensions

# Navigate to workspace root
if [ -z "$WORKSPACE_ROOT" ]; then
    # If not detected, assume we're in src/simulator
    WORKSPACE_ROOT="../.."
fi

echo "==== Building with colcon ===="
cd "$WORKSPACE_ROOT"
colcon build --packages-select simulator --cmake-args -DCMAKE_BUILD_TYPE=Release

echo ""
echo "==== Build complete ===="
echo ""
echo "To use the package:"
echo "  source $WORKSPACE_ROOT/install/setup.bash"
echo "  ros2 run simulator simulation_node"
echo ""
echo "To check the custom message:"
echo "  ros2 interface show simulator/msg/CustomMessage"
