#include <ros/ros.h>
#include <ros/node_handle.h>
#include <imu_vn_100/imu_ros_base.h>

using namespace imu_vn_100;

int main(int argc, char **argv) {
  ros::init(argc, argv, "imu_vn_100");
  ros::NodeHandle nh("~");

  // new instance of the IMU
  ImuRosBase imu(nh);

  // Initialize the device
  if (!imu.initialize()) {
    ROS_ERROR("Cannot initialize the device");
    return -1;
  }

  // Enable the continuous streaming
  imu.enableIMUStream(true);

  ros::spin();

  // Disconnect the device
  imu.disconnect();

  return 0;
}
