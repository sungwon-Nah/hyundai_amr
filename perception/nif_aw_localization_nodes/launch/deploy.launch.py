import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument

from launch.substitutions import LaunchConfiguration
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node


IMS = 0
LOR = 1
IMS_SIM = 2
LVMS = 3
LVMS_SIM = 4
TMS = 5
MONZA = 6
MONZA_SIM = 7
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
elif track_id == "TMS" or track_id == "tms":
    track = TMS
elif track_id == "MONZA" or track_id == "monza":
    track = MONZA
elif track_id == "MONZA_SIM" or track_id == "monza_sim":
    track = MONZA_SIM
else:
    raise RuntimeError("ERROR: Invalid track {}".format(track_id))


def generate_launch_description():

    config_file = None

    if track == LOR:
        config_file = 'config_lor.yaml'
    elif track == IMS:
        config_file = 'config_ims.yaml'
    elif track == IMS_SIM:
        config_file = 'config_lgsim.yaml'
    elif track == LVMS:
        config_file = 'config_lvms.yaml'
    elif track == LVMS_SIM:
        config_file = 'config_lvms_sim.yaml'
    elif track == TMS:
        config_file = 'config_tms.yaml'
    elif track == MONZA:
        config_file = 'config_monza.yaml'
    elif track == MONZA_SIM:
        config_file = 'config_monza_sim.yaml'
    else:
        raise RuntimeError("ERROR: invalid track provided: {}".format(track))

    params_file = os.path.join(
        get_package_share_directory("nif_aw_localization_nodes"),
        "config",
        config_file
    )

    param_file_launch_arg = DeclareLaunchArgument(
        'nif_aw_localization_param_file',
        default_value=params_file,
        description='nif_aw_localization_param'
    )

    referemce_error_map = os.path.join(
        get_package_share_directory("nif_aw_localization_nodes"),
        "map",
        "gps_error_reference_thres10.pcd",
    )   

    aw_localization_node = Node(
        package="nif_aw_localization_nodes",
        executable="nif_aw_localization_nodes_exe",
        output={
            "stderr": "screen",
            "stdout": "screen"
        },
        emulate_tty=True,
        parameters=[
            LaunchConfiguration('nif_aw_localization_param_file'),
            {
                "bestvel_heading_update_velocity_thres": 3.0,  # 3.0,
                "heading_initial_guess_enabled": True,
                # "heading_initial_guess_deg"     : 275.0,#LVMS
                "heading_initial_guess_deg"     : 255.0,#LVMS
                # "heading_initial_guess_deg"     : 197.0,#TMS
                # "heading_initial_guess_deg": 5.0,  # MONZA_SIM
                # "heading_initial_guess_deg": 308.0,  # MONZA_GARAGE
                # "heading_initial_guess_deg"     : -225.0,#LOR
                # ros2 topic echo /novatel_bottom/heading2 -> add heading bias
                "heading_heading2_offset_deg": 0.0,
                # "heading_heading2_offset_deg"   : -90.0 # For MIT-PITT data
                "use_reference_information" : False,
                "publish_reference_points" : False,
                "reference_information_filename" : referemce_error_map,

            }
        ],
        remappings=[
            # Current set : Bottom INS Disabled // Top INS Enabled
            # /novatel_bottom/bestvel is used to back-up solution when novatel_top/inspva heading is not published.

            # Origin
            ("in_bestpos", "novatel_bottom/bestgnsspos"),  # POSE (X,Y)
            ("in_top_bestpos", "novatel_top/bestgnsspos"),  # POSE (X,Y)
            ("in_imu", "novatel_bottom/rawimux"),  # YAW RATE
            ("in_top_imu", "novatel_top/rawimux"),  # YAW RATE
            ("in_vn_gps1", "vectornav/raw/gps"),  # VN_ANTENNA1
            ("in_vn_gps2", "vectornav/raw/gps2"),  # VN_ANTENNA2
            ("in_vn_ins", "vectornav/raw/ins"),  # VN_INS can be replaced with vectornav/raw/common
            ("in_vn_common", "vectornav/raw/common"),  # ROLL
            ("in_vn_attitude", "vectornav/raw/attitude"),  # ROLL
            # HEADING PRIORITY 3(BACK UP SOLUTION)
            ("in_bestvel", "novatel_bottom/bestgnssvel"),
            # HEADING PRIORITY 3(BACK UP SOLUTION)
            ("in_bestvel_top", "novatel_top/bestgnssvel"),
            # ("in_bestvel", "novatel_bottom/bestvel"), #HEADING PRIORITY 3(BACK UP SOLUTION)  #for LVMS bag
            # ("in_bestvel_top", "novatel_top/bestvel"), #HEADING PRIORITY 3(BACK UP SOLUTION) #for LVMS bag
            # HEADING PRIORITY 3(BACK UP SOLUTION)
            ("in_heading2", "novatel_bottom/heading2"),
            # HEADING PRIORITY 3(BACK UP SOLUTION)
            ("in_top_heading2", "novatel_top/heading2"),


            # For Sensor Disconnection Fail Test
            # ("in_bestpos", "novatel_bottom/bestgnsspos_test"), # POSE (X,Y)
            # ("in_top_bestpos", "novatel_top/bestgnsspos_test"), # POSE (X,Y)
            # ("in_imu", "novatel_bottom/rawimux_test"), # YAW RATE
            # ("in_top_imu", "novatel_top/rawimux_test"), # YAW RATE
            # ("in_vn_gps1", "vectornav/raw/gps"), # VN_ANTENNA1
            # ("in_vn_gps2", "vectornav/raw/gps2"), # VN_ANTENNA2
            # ("in_vn_gps", "vectornav/raw/gps2_test"), # ROLL
            # ("in_vn_common", "vectornav/raw/common_test"), # ROLL
            # ("in_bestvel", "novatel_bottom/bestgnssvel_test"), #HEADING PRIORITY 3(BACK UP SOLUTION)
            # ("in_bestvel_top", "novatel_top/bestgnssvel_test"), #HEADING PRIORITY 3(BACK UP SOLUTION)
            # ("in_heading2", "novatel_bottom/heading2_test"), #HEADING PRIORITY 3(BACK UP SOLUTION)
            # ("in_top_heading2", "novatel_top/heading2_test"), #HEADING PRIORITY 3(BACK UP SOLUTION)


            # For TUM dataset
            # ("in_bestpos", "novatel_top/bestpos"), # POSE (X,Y)
            # ("in_top_bestpos", "novatel_top/bestpos"), # POSE (X,Y)
            # ("in_imu2", "novatel_top/rawimu"), # YAW RATE
            # ("in_top_imu", "novatel_top/rawimux"), # YAW RATE
            # ("in_vn_imu", "vectornav/raw/imu"), # YAW RATE
            # ("in_vn_gps", "vectornav/raw/gps2"), # ROLL
            # ("in_bestvel", "novatel_top/bestvel"), #HEADING PRIORITY 3(BACK UP SOLUTION)
            # ("in_bestvel_top", "novatel_top/bestvel"), #HEADING PRIORITY 3(BACK UP SOLUTION)
            # ("in_heading2", "novatel_bottom/heading2"), #HEADING PRIORITY 3(BACK UP SOLUTION)
            # ("in_top_heading2", "novatel_top/heading2"), #HEADING PRIORITY 3(BACK UP SOLUTION)

            # For KAIST prev
            # ("in_bestpos", "novatel_bottom/bestgnsspos"), # POSE (X,Y)
            # ("in_top_bestpos", "novatel_bottom/bestgnsspos"), # POSE (X,Y)
            # ("in_imu", "novatel_bottom/rawimux"), # YAW RATE
            # ("in_top_imu", "novatel_bottom/rawimux"), # YAW RATE
            # ("in_vn_imu", "vectornav/raw/imu"), # YAW RATE
            # ("in_vn_gps", "vectornav/raw/gps2"), # ROLL
            # ("in_bestvel", "novatel_top/bestvel"), #HEADING PRIORITY 3(BACK UP SOLUTION)
            # ("in_bestvel_top", "novatel_top/bestvel"), #HEADING PRIORITY 3(BACK UP SOLUTION)
            # ("in_heading2", "novatel_bottom/heading2"), #HEADING PRIORITY 3(BACK UP SOLUTION)
            # ("in_top_heading2", "novatel_top/heading2"), #HEADING PRIORITY 3(BACK UP SOLUTION)

            # For ublox f9p gps
            # ("in_ublox_pos", "/fix"),

            # For Xsens
            ("in_xsens_pos", "/xsens/gnss"),
            ("in_xsens_imu", "/xsens/imu/data"),

            # HEADING PRIORITY 1 #Remaping
            ("in_inspva", "novatel_bottom/inspva2"),
            # HEADING PRIORITY 2 #Remaping
            ("in_top_inspva", "novatel_top/inspva2"),
            # INS STANDARD DEVIATION
            ("in_insstdev", "novatel_bottom/insstdev2"),
            # INS STANDARD DEVIATION
            ("in_top_insstdev", "novatel_top/insstdev2"),
            ("in_wheel_speed_report",
             "raptor_dbw_interface/wheel_speed_report"),  # WHEEL SPEED

            # ("in_wheel_speed_report", "autoverse/vehicle_data"), # WHEEL SPEED

            ("out_odometry_ekf_estimated", "/aw_localization/ekf/odom"),
            ("out_odometry_bestpos", "/aw_localization/ekf/odom_bestpos"),
            ("out_top_odometry_bestpos", "/aw_localization/ekf/top_bestpos"),
            ("out_odometry_bottom_ins", "/aw_localization/ekf/odom_bottom_inspva"),
            ("out_odometry_top_ins", "/aw_localization/ekf/odom_top_inspva"),
            ("out_ubx_odom", "/aw_localization/ekf/ubx_bestpos"),
            ("out_vn_odom1", "/aw_localization/ekf/vn_odom1"),
            ("out_vn_odom2", "/aw_localization/ekf/vn_odom2"),
            ("out_vn_ins_odom", "/aw_localization/ekf/vn_ins_odom"),
            ("out_xsens_odom", "/aw_localization/ekf/xsens_bestpos"),
            ('out_localization_error', '/aw_localization/ekf/error'),
            ('out_localization_status', '/aw_localization/ekf/status'),
            ('/debug', '/aw_localization/debug'),
            ('/debug/measured_pose', '/aw_localization/debug/measured_pose'),
            ('/estimated_yaw_bias', '/aw_localization/estimated_yaw_bias'),
        ]
    )

    return LaunchDescription(
        [
            param_file_launch_arg,
            aw_localization_node
        ])
