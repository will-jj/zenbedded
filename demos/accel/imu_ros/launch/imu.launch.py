from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.substitutions import Command, PathSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = PathSubstitution(FindPackageShare("zenbedded_imu"))
    controllers_yaml = pkg_share / "config" / "controllers.yaml"
    urdf_path = pkg_share / "urdf" / "imu.urdf"

    robot_description = ParameterValue(Command(["xacro ", urdf_path]), value_type=str)

    robot_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[controllers_yaml],
        output="screen",
    )

    spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["imu_sensor_broadcaster", "--param-file", controllers_yaml],
    )

    return LaunchDescription(
        [
            robot_state_pub,
            control_node,
            RegisterEventHandler(
                OnProcessStart(target_action=control_node, on_start=[spawner]),
            ),
        ]
    )