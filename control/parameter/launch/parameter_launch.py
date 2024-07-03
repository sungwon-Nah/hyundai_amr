from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Node(
        #     package='parameter',
        #     executable='yaw_rate_kalman_node',
        #     name='yaw_rate_kalman_node',
        #     output='screen',
        #     parameters=[
        #         {'R': 0.001},       # Measurement noise covariance
        #         {'x0': 0.0},        # Initial estimate of the state
        #         {'P0': 1.2},        # Initial estimate error covariance
        #         {'Q': 1e-5}         # Processs noise covariance
        #     ],
        # ),
        Node(
            package='parameter',
            executable='yaw_accel_node',
            name='yaw_accel_node',
            output='screen',
            parameters=[],
        ),
        # Node(
        #     package='parameter',
        #     executable='yaw_accel_butterworth_node',
        #     name='yaw_accel_butterworth_node',
        #     output='screen',
        #     parameters=[
        #         {'filter_order': 2},
        #         {'butterworth_cutoff_frequency': 5.0},
        #         {'butterworth_sampling_rate': 100.0}
        #     ],
        # ),
        # Node(
        #     package='parameter',
        #     executable='yaw_accel_average_node',
        #     name='yaw_accel_average_node',
        #     output='screen',
        #     parameters=[
        #         {'window_size': 50}
        #     ],
        # ),
        # Node(
        #     package='parameter',
        #     executable='yaw_accel_combined_node',
        #     name='yaw_accel_combined_node',
        #     output='screen',
        #     parameters=[
        #         {'filter_order': 2},
        #         {'butterworth_cutoff_frequency': 5.0},
        #         {'butterworth_sampling_rate': 100.0},
        #         {'window_size': 100}
        #     ],
        # ),
        Node(
            package='parameter',
            executable='yaw_accel_kalman_node',
            name='yaw_accel_kalman_node',
            output='screen',
            parameters=[
                {'R': 0.00003},     # Measurement noise covariance
                {'x0': 0.0},        # Initial estimate of the state
                {'P0': 5.0},        # Initial estimate error covariance
                {'Q': 1e-6}         # Process noise covariance
            ],
        )
        # Node(
        #     package='parameter',
        #     executable='yaw_accel_kalman_poly_node',
        #     name='yaw_accel_kalman_poly_node',
        #     output='screen',
        #     parameters=[
        #         {'R': 0.00003},       # Measurement noise covariance
        #         {'x0': 0.0},        # Initial estimate of the state
        #         {'P0': 5.0},        # Initial estimate error covariance
        #         {'Q': 1e-6},         # Process noise covariance
        #         {'polynomial_order': 30}
        #     ],
        # )
    ])
