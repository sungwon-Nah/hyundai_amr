cmake_minimum_required(VERSION 3.8)
project(parameter)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
# find_package(Eigen3 REQUIRED)
# find_package(Ceres REQUIRED)

# # include directory settings
# include_directories(
#   ${EIGEN3_INCLUDE_DIRS}
#   ${CERES_INCLUDE_DIRS}
# )

# link_directories(${EIGEN3_LIBRARY_DIRS})

# # Yaw Rate using Kalman Filter Node
# add_executable(yaw_rate_kalman_node src/yaw_rate_kalman_filter.cpp)
# ament_target_dependencies(yaw_rate_kalman_node
#   rclcpp
#   geometry_msgs
#   sensor_msgs
#   std_msgs
# )

# Yaw Acceleration Node
add_executable(yaw_accel_node src/yaw_accel.cpp)
ament_target_dependencies(yaw_accel_node
  rclcpp
  geometry_msgs
  sensor_msgs
  std_msgs
)

# # Yaw Acceleration (Butterworth filter) Node
# add_executable(yaw_accel_butterworth_node src/yaw_accel_butterworth_filter.cpp)
# ament_target_dependencies(yaw_accel_butterworth_node
#   rclcpp
#   geometry_msgs
#   sensor_msgs
#   std_msgs
# )

# # Yaw Acceleration (Average filter) Node
# add_executable(yaw_accel_average_node src/yaw_accel_average_filter.cpp)
# ament_target_dependencies(yaw_accel_average_node
#   rclcpp
#   geometry_msgs
#   sensor_msgs
#   std_msgs
# )

# # Yaw Acceleration (Combined filter) Node
# add_executable(yaw_accel_combined_node src/yaw_accel_combined_filter.cpp)
# ament_target_dependencies(yaw_accel_combined_node
#   rclcpp
#   geometry_msgs
#   sensor_msgs
#   std_msgs
# )

# Yaw Acceleration (Kalman filter) Node
add_executable(yaw_accel_kalman_node src/yaw_accel_kalman_filter.cpp)
ament_target_dependencies(yaw_accel_kalman_node
  rclcpp
  geometry_msgs
  sensor_msgs
  std_msgs
)

# # Yaw Acceleration (Kalman filter & Polynomial regression) Node -> make some delay
# add_executable(yaw_accel_kalman_poly_node src/yaw_accel_kalman_poly.cpp)
# ament_target_dependencies(yaw_accel_kalman_poly_node
#   rclcpp
#   geometry_msgs
#   sensor_msgs
#   std_msgs
#   Eigen3
#   Ceres
# )

# target_link_libraries(yaw_accel_kalman_poly_node ${CERES_LIBRARIES})

# Slip Angle Calculator Node
add_executable(slip_angle_node src/slip_angle.cpp)
ament_target_dependencies(slip_angle_node
  rclcpp
  geometry_msgs
  sensor_msgs
  std_msgs
)

install(TARGETS
  # yaw_rate_kalman_node
  yaw_accel_node
  # yaw_accel_butterworth_node
  # yaw_accel_average_node
  # yaw_accel_combined_node
  yaw_accel_kalman_node
  # yaw_accel_kalman_poly_node
  slip_angle_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()



### 원본 ###

# cmake_minimum_required(VERSION 3.8)
# project(parameter)

# # Default to C++14
# if(NOT CMAKE_CXX_STANDARD)
#   set(CMAKE_CXX_STANDARD 14)
# endif()

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

# # find dependencies
# find_package(ament_cmake REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(geometry_msgs REQUIRED)
# find_package(sensor_msgs REQUIRED)
# find_package(std_msgs REQUIRED)

# # Yaw Acceleration Node
# add_executable(yaw_accel_node src/yaw_accel.cpp)
# ament_target_dependencies(yaw_accel_node
#   rclcpp
#   geometry_msgs
#   sensor_msgs
#   std_msgs
# )

# # Slip Angle Calculator Node
# add_executable(slip_angle_node src/slip_angle.cpp)
# ament_target_dependencies(slip_angle_node
#   rclcpp
#   geometry_msgs
#   sensor_msgs
#   std_msgs
# )

# install(TARGETS
#   yaw_accel_node
#   slip_angle_node
#   DESTINATION lib/${PROJECT_NAME}
# )

# install(
#   DIRECTORY launch
#   DESTINATION share/${PROJECT_NAME}
# )

# ament_package()
