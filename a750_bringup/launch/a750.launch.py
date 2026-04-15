import os
import xacro
import json

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, TimerAction, OpaqueFunction
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_robot_description(context: LaunchContext, description_package, hwrev, use_fake_hardware, device_path):
    """Generate robot description."""

    # Substitute launch configuration values
    description_package_str = context.perform_substitution(description_package)
    hwrev_str = context.perform_substitution(hwrev)
    use_fake_hardware_str = context.perform_substitution(use_fake_hardware)
    device_path_str = context.perform_substitution(device_path)

    plugin = "a750_hardware/A750System"
    if json.loads(use_fake_hardware_str):
        plugin = "mock_components/GenericSystem"

    robot_description_path = os.path.join(
            get_package_share_directory(description_package_str), "urdf", f"a750_rev{hwrev_str}.urdf")

    robot_description = xacro.process_file(
        robot_description_path,
        mappings={
            "plugin": plugin,
            "use_fake_hardware": use_fake_hardware_str,
            "device_path": device_path_str,
        }
    ).toprettyxml(indent="  ")
    #print("robot description", robot_description)
    #print("use_fake_hardware_str", use_fake_hardware_str)
    return robot_description


def robot_nodes_spawner(context: LaunchContext, description_package, hwrev, controllers_file,
                        use_fake_hardware, device_path):
    """Spawn both robot state publisher and control nodes with shared robot description."""

    # Generate robot description once
    robot_description = generate_robot_description(
        context, description_package, hwrev, use_fake_hardware, device_path)

    # Get controllers file path
    controllers_file_str = context.perform_substitution(controllers_file)
    robot_description_param = {"robot_description": robot_description}

    # Robot state publisher node
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[robot_description_param],
    )

    # Control node
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="both",
        emulate_tty=True,
        parameters=[robot_description_param, controllers_file_str],
    )

    return [robot_state_pub_node, control_node]


def generate_launch_description():
    """Generate launch description for A-750."""

    # Declare launch arguments
    declared_arguments = [
        DeclareLaunchArgument(
            "description_package",
            default_value="a750_description",
            description="Description package with robot URDF files.",
        ),
        DeclareLaunchArgument(
            "hwrev",
            default_value = "1",
            description="Hardware revision (e.g., 1)"
        ),
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="false",
            description="Use fake hardware instead of real hardware.",
        ),
        DeclareLaunchArgument(
            "robot_controller",
            default_value="joint_trajectory_controller",
            choices=["forward_position_controller",
                     "joint_trajectory_controller"],
            description="Robot controller to start.",
        ),
        DeclareLaunchArgument(
            "runtime_config_package",
            default_value="a750_bringup",
            description="Package with the controller's configuration in config folder.",
        ),
        DeclareLaunchArgument(
            "controllers_file",
            default_value="a750_controllers.yaml",
            description="Controllers file(s) to use. Can be a single file or comma-separated list of files.",
        ),
        DeclareLaunchArgument(
            "device_path",
            default_value="/dev/ttyACM0",
            description="dev file to use.",
        ),
    ]

    # Initialize launch configurations
    description_package = LaunchConfiguration("description_package")
    hwrev = LaunchConfiguration("hwrev")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    robot_controller = LaunchConfiguration("robot_controller")
    runtime_config_package = LaunchConfiguration("runtime_config_package")
    controllers_file = LaunchConfiguration("controllers_file")
    controllers_file = PathJoinSubstitution(
        [FindPackageShare(runtime_config_package), "config", controllers_file]
    )
    device_path = LaunchConfiguration("device_path")

    # Robot nodes spawner (both state publisher and control)
    robot_nodes_spawner_func = OpaqueFunction(
        function=robot_nodes_spawner,
        args=[description_package, hwrev, controllers_file, use_fake_hardware, device_path]
    )
    # RViz configuration
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(description_package), "rviz",
         "arm.rviz"]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    # Joint state broadcaster spawner
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster",
                   "--controller-manager", "/controller_manager"],
    )

    # Controller spawners
    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[robot_controller, "--controller-manager", "/controller_manager"],
    )

    # gripper_controller_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=["gripper_controller", "--controller-manager", "/controller_manager"],
    # )

    # Timing and sequencing
    delayed_joint_state_broadcaster = TimerAction(
        period=1.0,
        actions=[joint_state_broadcaster_spawner],
    )

    delayed_robot_controller = TimerAction(
        period=1.0,
        actions=[robot_controller_spawner],
    )
    # delayed_gripper_controller = TimerAction(
    #     period=1.0,
    #     actions=[gripper_controller_spawner],
    # )

    return LaunchDescription(
        declared_arguments + [
            robot_nodes_spawner_func,
            rviz_node,
        ] +
        [
            delayed_joint_state_broadcaster,
            delayed_robot_controller,
            # delayed_gripper_controller,
        ]
    )