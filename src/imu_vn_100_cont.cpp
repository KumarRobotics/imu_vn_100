/*
 * Copyright [2015] [Ke Sun]
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

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
