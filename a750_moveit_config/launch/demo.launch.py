import os
import xacro
import json
from ament_index_python.packages import (
    get_package_share_directory,
)
from launch import LaunchDescription, LaunchContext
from launch.actions import (
    DeclareLaunchArgument,
    TimerAction,
    OpaqueFunction,
)
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


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
    return robot_description


def robot_nodes_spawner(
    context: LaunchContext,
    description_package,
    hwrev,
    use_fake_hardware,
    controllers_file,
    device_path
):
    robot_description = generate_robot_description(
        context,
        description_package,
        hwrev,
        use_fake_hardware,
        device_path
    )

    controllers_file_str = context.perform_substitution(controllers_file)
    robot_description_param = {"robot_description": robot_description}

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[robot_description_param],
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="both",
        parameters=[robot_description_param, controllers_file_str],
    )

    return [robot_state_pub_node, control_node]


def controller_spawner(context: LaunchContext, robot_controller):
    robot_controller_str = context.perform_substitution(robot_controller)

    if robot_controller_str == "forward_position_controller":
        controller = "a750_forward_position_controller"
    elif robot_controller_str == "joint_trajectory_controller":
        controller = "a750_joint_trajectory_controller"
    else:
        raise ValueError(f"Unknown robot_controller: {robot_controller_str}")

    return [
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[controller, "-c", "/controller_manager"],
        )
    ]


def generate_launch_description():
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
        DeclareLaunchArgument("use_fake_hardware", default_value="false"),
        DeclareLaunchArgument(
            "robot_controller",
            default_value="joint_trajectory_controller",
            choices=["forward_position_controller",
                     "joint_trajectory_controller"],
        ),
        DeclareLaunchArgument(
            "runtime_config_package", default_value="a750_bringup"
        ),
        DeclareLaunchArgument(
            "controllers_file",
            default_value="a750_controllers.yaml",
        ),
        DeclareLaunchArgument(
            "device_path",
            default_value="/dev/ttyACM0",
            description="dev file to use.",
        ),
    ]

    description_package = LaunchConfiguration("description_package")
    hwrev = LaunchConfiguration("hwrev")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    robot_controller = LaunchConfiguration("robot_controller")
    runtime_config_package = LaunchConfiguration("runtime_config_package")
    controllers_file = LaunchConfiguration("controllers_file")
    device_path = LaunchConfiguration("device_path")

    controllers_file = PathJoinSubstitution(
        [FindPackageShare(runtime_config_package), "config",
         controllers_file]
    )

    robot_nodes_spawner_func = OpaqueFunction(
        function=robot_nodes_spawner,
        args=[
            description_package,
            hwrev,
            use_fake_hardware,
            controllers_file,
            device_path,
        ],
    )

    jsb_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["a750_joint_state_broadcaster",
                   "--controller-manager", "/controller_manager"],
    )

    controller_spawner_func = OpaqueFunction(
        function=controller_spawner, args=[robot_controller])

    # gripper_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=["left_gripper_controller",
    #                "right_gripper_controller", "-c", "/controller_manager"],
    # )

    delayed_jsb = TimerAction(period=2.0, actions=[jsb_spawner])
    delayed_arm_ctrl = TimerAction(
        period=1.0, actions=[controller_spawner_func])
    # delayed_gripper = TimerAction(period=1.0, actions=[gripper_spawner])

    moveit_config = MoveItConfigsBuilder(
        "a750", package_name="a750_moveit_config"
    ).to_moveit_configs()

    moveit_params = moveit_config.to_dict()

    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_params],
    )

    rviz_cfg = os.path.join(
        get_package_share_directory(
            "a750_moveit_config"), "config", "moveit.rviz"
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_cfg],
        parameters=[moveit_params],
    )

    return LaunchDescription(
        declared_arguments
        + [
            robot_nodes_spawner_func,
            delayed_jsb,
            delayed_arm_ctrl,
            #delayed_gripper,
            run_move_group_node,
            rviz_node,
        ]
    )