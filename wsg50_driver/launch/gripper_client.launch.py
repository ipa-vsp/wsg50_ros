import os

from pkg_resources import declare_namespace

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declare_arguments = []

    declare_arguments.append(
        DeclareLaunchArgument(
            "gripper_ip",
            default_value="172.31.1.160",
            description="Gripper IP address",
        ),
    )

    declare_arguments.append(
        DeclareLaunchArgument(
            "port",
            default_value="1501",
            description="Gripper Port Number",
        ),
    )

    declare_arguments.append(
        DeclareLaunchArgument(
            "grasp_force",
            default_value="40",
            description="Gripper grasp force",
        ),
    )

    declare_arguments.append(
        DeclareLaunchArgument(
            "grasp_speed",
            default_value="10.0",
            description="Gripper grasp Speed",
        ),
    )

    gripper_ip = LaunchConfiguration("gripper_ip")
    port = LaunchConfiguration("port")
    grasp_force = LaunchConfiguration("grasp_force")
    grasp_speed = LaunchConfiguration("grasp_speed")

    gripper_server_launch_file = PathJoinSubstitution([FindPackageShare('wsg50_driver'), "launch" , "gripper.launch.py"])
    gripper_server_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gripper_server_launch_file]),
        launch_arguments={
            'gripper_ip': gripper_ip,
            'port': port,
            'grasp_force': grasp_force,
            'grasp_speed': grasp_speed,
        }.items(),
    )

    node_client = Node(
        package='wsg50_driver',
        executable='gripper_client_node',
        name=['wsg50_gripper_driver'],
    )

    nodes = [node_client]

    return LaunchDescription(declare_arguments + nodes)
