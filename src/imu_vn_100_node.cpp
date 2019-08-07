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

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <imu_vn_100/imu_vn_100.h>

using namespace imu_vn_100;

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  // Force flush of the stdout buffer, which ensures a sync of all prints
  // even from a launch file.
  setvbuf(stdout, nullptr, _IONBF, BUFSIZ);

  auto imu = std::make_shared<imu_vn_100::ImuVn100>();
  imu->Stream(true);

  rclcpp::spin(imu);

  rclcpp::shutdown();

  return 0;
}
