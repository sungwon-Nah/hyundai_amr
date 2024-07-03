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
        get_package_share_directory("nif_ecef_converter_nodes"),
        "config",
        config_file
    )

    param_file_launch_arg = DeclareLaunchArgument(
        'nif_ecef_converter_param_file',
        default_value=params_file,
        description='nif_ecef_converter_param'
    )

    nif_ecef_converter_node = Node(
        package="nif_ecef_converter_nodes",
        executable="nif_ecef_converter_nodes_exe",
        output={
            "stderr": "screen",
            "stdout": "screen"
        },
        emulate_tty=True,
        parameters=[
            LaunchConfiguration('nif_ecef_converter_param_file'),
        ],
        remappings=[
            # Origin
            # ("in_bestpos", "novatel_bottom/bestgnsspos"), 
        ]
    )

    return LaunchDescription(
        [
            param_file_launch_arg,
            nif_ecef_converter_node
        ])
