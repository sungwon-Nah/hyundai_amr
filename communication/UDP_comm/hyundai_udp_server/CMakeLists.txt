cmake_minimum_required(VERSION 2.8.3)
project(hyundai_udp_server)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  ackermann_msgs
)


catkin_package(
  CATKIN_DEPENDS roscpp std_msgs
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(hyundai_udp_server src/hyundai_udp_server.cpp)

target_link_libraries(
  hyundai_udp_server
        ${catkin_LIBRARIES}
  )
    