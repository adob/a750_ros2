# A-750 Bringup

This package provides launch files to bring up the A-750 robotic arm.

## Quick Start

Launch the A-750 at hardware revision 1 and fake hardware:

```bash
ros2 launch a750_bringup a750.launch.py hwrev:=1 hardware_type:=fake
```

## Launch Files

- `a750.launch.py` - standard arm configuration

## Key Parameters

- `hwrev` - Hardware revision (default: 1)
- `hardware_type` - Use real/mock/mujoco hardware (default: real)
- `can_interface` - device interface to use (default: /dev/ttyACM0)
- `robot_controller` - Controller type: `joint_trajectory_controller` or `forward_position_controller`

## What Gets Launched

- Robot state publisher
- Controller manager with ros2_control
- Joint state broadcaster
- Robot controller (joint trajectory or forward position)
- Gripper controller
- RViz2 visualization