# a750_ros2

ROS2 support for the A-750 robotic arm.

To use this package, make sure you have have ROS2 installed.

## ROS2 installation and set up

[ROS 2](https://www.ros.org/) (Robot Operating System 2) is a set of open-source software libraries and tools for building robot applications.

Select a [ROS 2 distribution](https://www.ros.org/blog/getting-started/) to install. Jazzy Jalisco is recommended.

Follow the installation guide at https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html

The packages you need to install vary based on the tools you use, but `ros-jazzy-desktop` is recommended.

```bash
sudo apt install -y ros-jazzy-desktop # or ros-jazzy-ros-base - Minimal setup (no GUI tools)
```

To use the [a750_ros2](https://github.com/adob/a750_ros2) repository, you'll also need the following packages:

```bash
sudo apt install -y \
  ros-jazzy-joint-state-publisher-gui
  ros-jazzy-controller-manager
  ros-jazzy-hardware-interface \
  ros-jazzy-xacro \
  ros-jazzy-moveit \
  ros-jazzy-joint-trajectory-controller \
  ros-jazzy-joint-state-broadcaster \
  ros-dev-tools \
  g++-14
```

Source ROS 2 Environment:

```bash
source /opt/ros/jazzy/setup.bash
```

Install development tools at https://docs.ros.org/en/jazzy/Installation/Alternatives/Ubuntu-Development-Setup.html#install-development-tools-and-ros-tools.

### Verify Installation

Try running:

```bash
ros2 -h
ros2 topic list
```

### Workspace setup

```bash
# Source your ros2 distro
source /opt/ros/jazzy/setup.bash

export ROS_WS=~/ros2_ws   # Customize workspace path

# Create the workspace
mkdir -p $ROS_WS/src


# Set colcon defaults
cd $ROS_WS
cat > colcon_defaults.yaml <<EOF
build:
  cmake-args:
    - "-DCMAKE_EXPORT_COMPILE_COMMANDS=ON"
    - "--no-warn-unused-cli"
    - "-DCMAKE_C_COMPILER=/usr/bin/gcc-14"
    - "-DCMAKE_CXX_COMPILER=/usr/bin/g++-14"
EOF


# Head to the workspace src directory
cd $ROS_WS/src

# Clone packages
git clone https://github.com/adob/a750_description.git
git clone --recurse-submodules https://github.com/adob/a750_ros2.git

# Build the workspace
cd $ROS_WS
colcon build

# Source the workspace
source $ROS_WS/install/setup.bash
```

### Configure udev
<!--```bash
This step grants the currently logged-in user access serial USB devices.

sudo tee /etc/udev/rules.d/90-usb-serial.rules > /dev/null <<'EOF'
KERNEL=="ttyUSB[0-9]*", SUBSYSTEM=="tty", TAG+="uaccess"
KERNEL=="ttyACM[0-9]*", SUBSYSTEM=="tty", TAG+="uaccess"
EOF
```-->

Add yourself to the dialout group for tty/serial permissions.
```
sudo usermod -a -G dialout $USER
```
Log out and log back in for this to take effect.

## Visualization
To display the robot in RViz with a simple joint state GUI:

```bash
ros2 launch a750_description visualize_a750.launch.py hwrev:=1
```


## MoveIt
Control the robot with MoveIt:
```bash
ros2 launch a750_moveit_config demo.launch.py device_path:=/dev/ttyACM0
```