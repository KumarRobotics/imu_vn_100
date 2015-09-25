cmake_minimum_required(VERSION 2.8.3)
project(imu_vn_100)

find_package(catkin REQUIRED COMPONENTS
  diagnostic_updater
  geometry_msgs
  roscpp
  sensor_msgs
)
find_package(Boost REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
  CATKIN_DEPENDS diagnostic_updater geometry_msgs roscpp sensor_msgs
  DEPENDS boost
)

include_directories(
  include
  include/imu_vn_100/vncpplib/include
  ${catkin_INCLUDE_DIRS}
)


add_library(imu_vn_100
  include/imu_vn_100/vncpplib/src/arch/linux/vncp_services.c
  include/imu_vn_100/vncpplib/src/vndevice.c
  include/imu_vn_100/vncpplib/src/vn100.c
  src/imu_ros_base.cpp
)
target_link_libraries(imu_vn_100
  ${catkin_LIBRARIES}
)
add_dependencies(imu_vn_100
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

add_executable(imu_vn_100_cont_node
  src/imu_vn_100_cont.cpp)
target_link_libraries(imu_vn_100_cont_node
  ${catkin_LIBRARIES}
  imu_vn_100
)
add_dependencies(imu_vn_100_cont_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)



