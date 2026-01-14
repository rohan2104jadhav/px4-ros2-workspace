# PX4 ROS2 Workspace

A comprehensive development workspace integrating PX4 Autopilot with ROS2 for drone simulation, control, and mission planning.

## Project Structure

```
px4/
├── PX4-Autopilot/              # PX4 flight stack
├── PX4-gazebo-models/          # Gazebo simulation models
├── microros_ws/                # Micro-ROS workspace
├── px4_ros_uxrce_dds_ws/       # XRCE-DDS middleware workspace
└── ros2_ws/                    # ROS2 workspace with custom packages
    └── src/
        ├── com_bridge/         # Communication bridge package
        ├── px4_msgs/           # PX4 message definitions
        └── translation_node/   # Message translation utilities
```

## Prerequisites

- Ubuntu 20.04 or 22.04
- ROS2 (Humble/Iron)
- PX4 development environment
- Gazebo simulator
- Python 3.8+
- Git

## Installation & Setup

### 1. Micro-ROS Workspace Setup
```bash
cd /home/rohan/px4/microros_ws

# Source the setup script
source install/setup.bash
```

### 2. ROS2 Workspace Setup
```bash
cd /home/rohan/px4/ros2_ws

# Source the local setup
source install/setup.bash

# Build the workspace (if needed)
colcon build
```

### 3. PX4-Autopilot Setup
```bash
cd /home/rohan/px4/PX4-Autopilot

# Build PX4 SITL (Software In The Loop)
make px4_sitl_default gz_x500
```

### 4. Gazebo Models Setup
```bash
cd /home/rohan/px4/PX4-gazebo-models

# Run simulation with baylands world
python3 simulation-gazebo --world=baylands
```

## Running the System

### Terminal 1: Micro-ROS Setup
```bash
cd /home/rohan/px4/microros_ws
source install/setup.bash
# Ready for micro-ROS agent communication
```

### Terminal 2: ROS2 Communication Bridge
```bash
cd /home/rohan/px4/ros2_ws
source install/setup.bash
ros2 run com_bridge mission
```

### Terminal 3: PX4 Simulation
```bash
cd /home/rohan/px4/PX4-Autopilot
make px4_sitl_default gz_x500
```

### Terminal 4: Gazebo Simulation
```bash
cd /home/rohan/px4/PX4-gazebo-models
python3 simulation-gazebo --world=baylands
```

### Terminal 5: QGroundControl
```bash
cd /home/rohan/Downloads
./QGroundControl.AppImage
```

## Key Components

### com_bridge Package
Communication bridge for translating PX4 telemetry and commands via ROS2.

**Location:** `/home/rohan/px4/ros2_ws/src/com_bridge`

**Run command:**
```bash
cd /home/rohan/px4/ros2_ws
ros2 run com_bridge mission
```

### translation_node Package
Handles message format translations between different versions and protocols.

**Location:** `/home/rohan/px4/ros2_ws/src/translation_node`

### px4_msgs Package
Custom ROS2 message definitions for PX4 autopilot.

**Location:** `/home/rohan/px4/ros2_ws/src/px4_msgs`

## Common Workflows

### Full System Startup (Recommended Sequence)

1. **Terminal 1 - Micro-ROS:**
```bash
cd /home/rohan/px4/microros_ws
source install/setup.bash
```

2. **Terminal 2 - ROS2 Bridge:**
```bash
cd /home/rohan/px4/ros2_ws
source install/setup.bash
ros2 run com_bridge mission
```

3. **Terminal 3 - PX4 SITL:**
```bash
cd /home/rohan/px4/PX4-Autopilot
make px4_sitl_default gazebo-iris
```

4. **Terminal 4 - Gazebo Simulation:**
```bash
cd /home/rohan/px4/PX4-gazebo-models
python3 simulation-gazebo --world=warehouse
```

5. **Terminal 5 - QGroundControl (GUI):**
```bash
cd /home/rohan/Downloads
./QGroundControl.AppImage
```

## Building and Compiling

### Rebuild ROS2 Workspace
```bash
cd /home/rohan/px4/ros2_ws
colcon build
```

### Rebuild Micro-ROS Workspace
```bash
cd /home/rohan/px4/microros_ws
colcon build
```

### Clean Build (Remove build artifacts)
```bash
cd /home/rohan/px4/ros2_ws
rm -rf build install log
colcon build
```

## Useful ROS2 Commands

### Check ROS2 Network
```bash
# List all nodes
ros2 node list

# List all topics
ros2 topic list

# Listen to a specific topic
ros2 topic echo /topic_name

# List all services
ros2 service list

# Call a service
ros2 service call /service_name service_type "args"
```

### Monitor System
```bash
# Check system information
ros2 doctor

# Monitor resource usage
# Run from any ROS2 workspace directory
```

## Troubleshooting

### ROS2 Setup Issues
- Ensure you source the correct setup file: `source install/setup.bash`
- Check that ROS2 installation is correct: `echo $ROS_DISTRO`

### Build Failures
- Clean and rebuild: `rm -rf build install log && colcon build`
- Check dependencies: `rosdep check --all`

### Gazebo Simulation Issues
- Ensure Gazebo is installed: `gazebo --version`
- Check PX4 build is successful before running simulation

### Communication Issues
- Verify all nodes are running: `ros2 node list`
- Check topic connections: `ros2 topic info /topic_name`
- Monitor network: `ros2 node info /node_name`

## Git Repository

Repository: https://github.com/rohan2104jadhav/px4-ros2-workspace

Track your changes:
```bash
cd /home/rohan/px4
git add .
git commit -m "Your message"
git push origin main
```

## License

See individual package licenses in their respective directories.

## References

- [PX4 Documentation](https://docs.px4.io/)
- [ROS2 Documentation](https://docs.ros.org/en/humble/)
- [Gazebo Simulation](https://gazebosim.org/)
- [QGroundControl](http://qgroundcontrol.com/)

---

**Last Updated:** January 14, 2026
