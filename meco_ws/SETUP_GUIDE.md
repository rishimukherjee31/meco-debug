# MeCO Menu - Complete Setup Guide

## Quick Start

```bash
# Extract the workspace
tar -xzf meco_ws.tar.gz
cd meco_ws

# Run setup script
./setup.sh

# Source workspace
source install/setup.bash

# Test in debug mode
ros2 run meco_menu menu_node_debug
```

## Detailed Setup Instructions

### Step 1: Extract the Workspace

```bash
# Extract the tarball
tar -xzf meco_ws.tar.gz

# Navigate to workspace
cd meco_ws
```

The workspace structure:
```
meco_ws/
├── setup.sh               # Quick setup script
└── src/
    └── meco_menu/        # The ROS2 package
        ├── config/
        │   └── menu.yaml
        ├── launch/
        │   └── menu.launch.py
        ├── meco_menu/
        │   ├── __init__.py
        │   ├── menu.py
        │   ├── processes.py
        │   ├── menu_node.py
        │   └── menu_node_debug.py
        ├── resource/
        │   └── meco_menu
        ├── package.xml
        ├── setup.py
        ├── setup.cfg
        └── README.md
```

### Step 2: Build the Package

#### Option A: Use the Setup Script (Recommended)

```bash
cd meco_ws
./setup.sh
```

This script will:
- Detect your ROS2 distribution (Jazzy or Humble)
- Source ROS2
- Install dependencies
- Clean previous builds
- Build the package
- Show you next steps

#### Option B: Manual Build

```bash
cd meco_ws

# Source ROS2 (use jazzy or humble depending on your system)
source /opt/ros/jazzy/setup.bash    # On dev machine
# OR
source /opt/ros/humble/setup.bash   # On meco robot

# Install dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build
colcon build --packages-select meco_menu

# Source the workspace
source install/setup.bash
```

### Step 3: Test the Installation

#### Test 1: Check Package Installation

```bash
# Source the workspace
source install/setup.bash

# List ROS2 packages (should include meco_menu)
ros2 pkg list | grep meco_menu

# Check executables
ros2 pkg executables meco_menu
```

Expected output:
```
meco_menu menu_node
meco_menu menu_node_debug
```

#### Test 2: Run Debug Node

```bash
# Run the debug node with terminal UI
ros2 run meco_menu menu_node_debug
```

You should see a terminal-based UI with the menu system. Use keyboard controls:
- `w` or `↑` - Navigate up
- `s` or `↓` - Navigate down
- `1-4` - Quick jump to items
- `ENTER` - Select item
- `b` - Go back
- `k` - Kill all processes
- `q` - Quit

#### Test 3: Run Production Node

```bash
# In terminal 1 - Run the menu node
ros2 run meco_menu menu_node
```

```bash
# In terminal 2 - Monitor topics
ros2 topic echo /meco/oled_menu

# Send test commands
ros2 topic pub --once /menu_navigate std_msgs/Int8 "data: 2"  # Navigate down
ros2 topic pub --once /menu_select std_msgs/Int8 "data: 1"     # Select
```

### Step 4: Integration with MeCO

If you're integrating this into the existing meco workspace:

```bash
# On your development machine
cd /path/to/your/existing/meco_workspace/src

# Copy just the package
cp -r /path/to/meco_ws/src/meco_menu .

# Rebuild the entire meco workspace
cd ..
colcon build

# Source
source install/setup.bash
```

## For MeCO Robot (ROS2 Humble)

### Transfer to Robot

```bash
# On development machine
cd /path/to/meco_ws
tar -czf meco_menu_package.tar.gz src/meco_menu

# Copy to robot (adjust username and hostname)
scp meco_menu_package.tar.gz user@meco-robot:~/

# SSH to robot
ssh user@meco-robot
```

### Build on Robot

```bash
# On the robot
cd ~/meco_ws/src  # Or wherever your meco workspace is
tar -xzf ~/meco_menu_package.tar.gz

# Build
cd ~/meco_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select meco_menu

# Source
source install/setup.bash

# Test
ros2 run meco_menu menu_node_debug
```

## Customizing the Menu

### Edit Configuration

```bash
# Edit the menu configuration
cd meco_ws/src/meco_menu/config
nano menu.yaml

# After editing, rebuild to install the new config
cd ~/meco_ws
colcon build --packages-select meco_menu
source install/setup.bash
```

### Menu Configuration Examples

See the included `src/meco_menu/config/menu.yaml` for examples of:
- Submenus
- Docker processes
- ROS2 nodes
- ROS2 commands
- Shell commands

## Troubleshooting

### "Package not found" Error

```bash
# Make sure you sourced the workspace
source install/setup.bash

# Verify package is installed
ros2 pkg list | grep meco_menu
```

### Build Errors

```bash
# Clean build
rm -rf build/ install/ log/

# Rebuild
colcon build --packages-select meco_menu --symlink-install

# Check for missing dependencies
rosdep install --from-paths src --ignore-src -r -y
```

### Config File Not Found

```bash
# Check config file location
ls install/meco_menu/share/meco_menu/config/menu.yaml

# If missing, rebuild
colcon build --packages-select meco_menu
```

### Import Errors

```bash
# Verify Python files are installed
ls install/meco_menu/lib/python3.*/site-packages/meco_menu/

# Rebuild with symlink for development
colcon build --packages-select meco_menu --symlink-install
```

## Development Tips

### Fast Development Cycle

```bash
# Use symlink install for Python development
colcon build --packages-select meco_menu --symlink-install

# Now Python changes don't require rebuild, just source:
source install/setup.bash
ros2 run meco_menu menu_node_debug
```

### Adding New Process Types

1. Edit `src/meco_menu/meco_menu/processes.py`
2. Add new process class
3. Update `ProcessFactory.create()` method
4. Rebuild: `colcon build --packages-select meco_menu`

### Changing Menu Structure

1. Edit `src/meco_menu/config/menu.yaml`
2. Rebuild to install: `colcon build --packages-select meco_menu`
3. Test: `ros2 run meco_menu menu_node_debug`

## Next Steps

1. **Test the Menu** - Run `menu_node_debug` to familiarize yourself with navigation
2. **Customize Configuration** - Edit `config/menu.yaml` for your specific needs
3. **Integration Testing** - Test with actual ROS2 nodes and Docker containers
4. **Deploy to Robot** - Transfer and build on the meco robot

## Additional Resources

- Full documentation: `src/meco_menu/README.md`
- Debug node guide: See `DEBUG_NODE_README.md`
- Example configuration: `src/meco_menu/config/menu.yaml`

## Support

If you encounter issues:
1. Check the troubleshooting section above
2. Verify all dependencies are installed
3. Check ROS2 installation and sourcing
4. Review error messages carefully

## Summary

The MeCO menu system is now ready to use! The package structure follows standard ROS2 conventions and can be easily integrated into your existing meco workspace. Use the debug mode for development and testing, then deploy the production node to your robot.
