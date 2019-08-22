cmake_minimum_required(VERSION 3.10)
project(imu_vn_100)

set(CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs diagnostic_updater)

catkin_package(CATKIN_DEPENDS
               diagnostic_updater
               roscpp
               sensor_msgs)

add_executable(${PROJECT_NAME}
               src/imu_vn_100.cpp
               vncpplib/src/arch/linux/vncp_services.c
               vncpplib/src/vn100.c
               vncpplib/src/vndevice.c)
target_include_directories(${PROJECT_NAME}
                           PUBLIC src vncpplib/include ${catkin_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} PUBLIC ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING
        PATTERN "*.h**")

install(FILES vncpplib/include/vn100.h
              vncpplib/include/vncp_services.h
              vncpplib/include/vndevice.h
              vncpplib/include/vn_errorCodes.h
              vncpplib/include/vn_kinematics.h
              vncpplib/include/vn_linearAlgebra.h
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
