cmake_minimum_required(VERSION 3.5)
project(imu_vn_100)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(
  include
  vncpplib/include
)

add_library(imu_vn_100
  vncpplib/src/arch/linux/vncp_services.c
  vncpplib/src/vndevice.c
  vncpplib/src/vn100.c
  src/imu_vn_100.cpp
)
ament_target_dependencies(imu_vn_100
  geometry_msgs
  rclcpp
  sensor_msgs
)

add_executable(imu_vn_100_node
  src/imu_vn_100_node.cpp)
ament_target_dependencies(imu_vn_100_node
  rclcpp
)
target_link_libraries(imu_vn_100_node
  imu_vn_100
)

install(TARGETS imu_vn_100
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

install(TARGETS imu_vn_100_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
