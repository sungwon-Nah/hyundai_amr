import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


IMS = 0
LOR = 1
IMS_SIM = 2
LVMS = 3
LVMS_SIM = 4
LVMS_SIM_AC = 5
TMS = 6
MONZA = 7
MONZA_SIM = 8
track = None

# get which track we are at
track_id = os.environ.get('TRACK').strip()

if track_id == "IMS" or track_id == "ims":
    track = IMS
elif track_id == "LOR" or track_id == "lor":
    track = LOR
elif track_id == "IMS_SIM" or track_id == "ims_sim":
    track = IMS_SIM
elif track_id == "LVMS" or track_id == "lvms":
    track = LVMS
elif track_id == "LVMS_SIM" or track_id == "lvms_sim":
    track = LVMS_SIM
elif track_id == "LVMS_SIM_AC" or track_id == "lvms_sim_ac":
    track = LVMS_SIM_AC
elif track_id == "TMS" or track_id == "tms":
    track = TMS    
elif track_id == "MONZA" or track_id == "monza":
    track = MONZA    
elif track_id == "MONZA_SIM" or track_id == "monza_sim":
    track = MONZA_SIM    
else:
    raise RuntimeError("ERROR: Invalid track {}".format(track_id))

def get_share_file(package_name, file_name):
    return os.path.join(get_package_share_directory(package_name), file_name)

def generate_launch_description():
  
    config_file = None
    if track == LOR:
        subdir = "LOR"
    elif track == IMS:
        subdir = "IMS"
    elif track == LVMS:
        subdir = "LVMS"
    elif track == LVMS_SIM:
        subdir = "LVMS_SIM"
    elif track == LVMS_SIM_AC:
        subdir = "LVMS_SIM_AC"
    elif track == TMS:
        subdir = "TMS"
    elif track == MONZA:
        subdir = "MONZA"
    elif track == MONZA_SIM:
        subdir = "MONZA_SIM"
    else:
        raise RuntimeError("ERROR: invalid track provided: {}".format(track))

    prediction_node = Node(
        package='waypoint_viz',
        executable='csv_viz_tool_exe',
        output='screen',
        parameters=[
            {
                'track_name': subdir       
            }
        ]
    )

    return LaunchDescription([
        prediction_node
    ])
