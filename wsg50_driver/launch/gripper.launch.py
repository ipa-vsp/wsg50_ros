import os

from pkg_resources import declare_namespace

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    declare_arguments = []

    declare_arguments.append(
        DeclareLaunchArgument(
            "gripper_ip",
            default_value="192.168.1.160",
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

    node = Node(
        package='wsg50_driver',
        executable='gripper_server_node',
        name=['wsg50_gripper_driver'],
        parameters=[{'gripper_ip': gripper_ip, 'port': port, 'grasp_force': grasp_force, 'grasp_speed': grasp_speed}],
    )

    return LaunchDescription(declare_arguments + [node])
