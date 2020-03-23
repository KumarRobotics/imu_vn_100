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
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Threads REQUIRED)

include_directories(
  include
  vncpplib/include
)

add_library(imu_vn_100_cpplib SHARED
  vncpplib/src/arch/linux/vncp_services.c
  vncpplib/src/vndevice.c
  vncpplib/src/vn100.c
)
target_link_libraries(imu_vn_100_cpplib
  ${CMAKE_THREAD_LIBS_INIT}
)

add_library(imu_vn_100 SHARED
  src/imu_vn_100.cpp
)
ament_target_dependencies(imu_vn_100
  geometry_msgs
  rclcpp
  rclcpp_components
  sensor_msgs
)
target_link_libraries(imu_vn_100
  imu_vn_100_cpplib
)

rclcpp_components_register_nodes(imu_vn_100
  "imu_vn_100::ImuVn100")

add_executable(imu_vn_100_node
  src/imu_vn_100_node.cpp)
ament_target_dependencies(imu_vn_100_node
  rclcpp
)
target_link_libraries(imu_vn_100_node
  imu_vn_100
  imu_vn_100_cpplib
)

add_executable(imu_vn_100_flash_baud_rate
  src/imu_vn_100_flash_baud_rate.cpp
)
target_link_libraries(imu_vn_100_flash_baud_rate
  imu_vn_100_cpplib
)

install(TARGETS imu_vn_100 imu_vn_100_cpplib
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

install(TARGETS imu_vn_100_node imu_vn_100_flash_baud_rate
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

ament_package()
