import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument

from launch.substitutions import LaunchConfiguration
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node



def generate_launch_description():

    config_file = 'config.yaml'

    params_file = os.path.join(
        get_package_share_directory("pnd_controller_nodes"),
        "config",
        config_file
    )

    param_file_launch_arg = DeclareLaunchArgument(
        'pnd_controller_param_file',
        default_value=params_file,
        description='pnd_controller_param'
    )

    pnd_controller_node = Node(
        package="pnd_controller_nodes",
        executable="pnd_controller_nodes_exe",
        output={
            "stderr": "screen",
            "stdout": "screen"
        },
        emulate_tty=True,
        parameters=[
            LaunchConfiguration('pnd_controller_param_file'),
        ],
        remappings=[
            # Origin
            # ("in_bestpos", "novatel_bottom/bestgnsspos"), 
        ]
    )

    return LaunchDescription(
        [
            param_file_launch_arg,
            pnd_controller_node
        ])
