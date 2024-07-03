# Assetto Corsa car telemetry
# Author : Andrea Finazzi @ KAIST
# Date : 2022.03.21

std_msgs/Header header
uint8 cid

# 3D vectors
geometry_msgs/Point world_position
geometry_msgs/Vector3 linear_velocity_vector

#geolinear_velocity_vector_local
#angular_velocity_vector_local
geometry_msgs/Twist twist_local
geometry_msgs/Vector3 linear_acceleration

geometry_msgs/PoseStamped odometry

geometry_msgs/Vector3 tyre_fy_fl
geometry_msgs/Vector3 tyre_fy_fr
geometry_msgs/Vector3 tyre_fy_rl
geometry_msgs/Vector3 tyre_fy_rr

# # 4D vectors
float64[] wheel_angular_speed
float64[] slip_angle
float64[] slip_ratio
float64[] tyre_slip

# # Scalar
float64 linear_speed_mps
float64 control_steer_angle
float64 control_gas_norm
float64 control_brake_norm
float64 control_clutch_norm
int8 control_gear

float64 norm_spline_position
uint16 lap_count
uint64 current_lap_time
uint64 last_lap_time
uint64 best_lap_time

uint8 leaderboard_position

# # Bool {0, 1}
bool is_lap_invalidated false  
bool is_car_in_pitlane  false   
bool is_car_in_pit      false       
bool is_ai_controlled   false    
bool is_race_finished   false
