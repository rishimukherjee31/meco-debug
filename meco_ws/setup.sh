#!/bin/bash
# Quick Setup Script for MeCO Menu ROS2 Package

set -e

echo "=========================================="
echo "MeCO Menu - ROS2 Package Setup"
echo "=========================================="
echo ""

# Check if you're in the right directory
if [ ! -d "src/meco_menu" ]; then
    echo "ERROR: Please run this script from the meco_ws directory"
    echo "Current directory: $(pwd)"
    exit 1
fi

# Locate ROS2 distribution
if [ -f "/opt/ros/jazzy/setup.bash" ]; then
    ROS_DISTRO="jazzy"
elif [ -f "/opt/ros/humble/setup.bash" ]; then
    ROS_DISTRO="humble"
else
    echo "ERROR: No ROS2 installation found (looking for Jazzy or Humble)"
    exit 1
fi

echo "Detected ROS2 distribution: $ROS_DISTRO"
echo ""

# Source 
echo "Sourcing ROS2 $ROS_DISTRO..."
source /opt/ros/$ROS_DISTRO/setup.bash

# Install dependencies
echo ""
echo "Installing dependencies..."
rosdep install --from-paths src --ignore-src -r -y

# Remove previous build
echo ""
echo "Cleaning previous build..."
rm -rf build/ install/ log/

# colcon Build
echo ""
echo "Building meco_menu package..."
colcon build --packages-select meco_menu

# Check build success
if [ $? -eq 0 ]; then
    echo ""
    echo "------------------------------------------"
    echo "  Build successful "
    echo "------------------------------------------"
    echo ""
    echo "To use the package, source the workspace:"
    echo "  source install/setup.bash"
    echo ""
    echo "Then run:"
    echo "  ros2 run meco_menu menu_node        # Normal mode"
    echo "  ros2 run meco_menu menu_node_debug  # Debug mode (TUI)"
    echo ""
    echo "Or use the launch file:"
    echo "  ros2 launch meco_menu menu.launch.py"
    echo ""
else
    echo ""
    echo "------------------------------------------"
    echo "  Build failed "
    echo "------------------------------------------"
    echo ""
    echo "Please check the error messages above."
    exit 1
fi
