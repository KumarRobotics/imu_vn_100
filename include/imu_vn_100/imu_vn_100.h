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

#ifndef IMU_VN_100_ROS_H_
#define IMU_VN_100_ROS_H_

#include <memory>
#include <string>

#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <sensor_msgs/msg/temperature.hpp>

#include "vn100.h"

namespace imu_vn_100 {

/**
 * @brief ImuVn100 The class is a ros wrapper for the Imu class
 * @author Ke Sun
 */
class ImuVn100 final : public rclcpp::Node {
 public:
  static constexpr int kBaseImuRate = 800;
  static constexpr int kDefaultImuRate = 100;
  static constexpr int kDefaultSyncOutRate = 20;

  ImuVn100();
  ImuVn100(const ImuVn100&) = delete;
  ImuVn100& operator=(const ImuVn100&) = delete;
  ~ImuVn100();

  void Initialize();

  void Stream(bool async = true);

  void PublishData(const VnDeviceCompositeData& data);

  void RequestOnce();

  void Idle(bool need_reply = true);

  void Resume(bool need_reply = true);

  void Disconnect();

  void Configure();

  struct SyncInfo {
    unsigned count = 0;
    rclcpp::Time time;

    int rate = -1;
    double rate_double = -1;
    int pulse_width_us = 1000;
    int skip_count = 0;

    void Update(const unsigned sync_count, const rclcpp::Time& sync_time);
    void FixSyncRate();
    bool SyncEnabled() const;
  };

  const SyncInfo sync_info() const { return sync_info_; }

 private:
  Vn100 imu_;

  // Settings
  std::string port_;
  uint32_t baudrate_;
  uint32_t initial_baudrate_;
  int imu_rate_;
  double imu_rate_double_;
  std::string frame_id_;

  bool enable_mag_;
  bool enable_pres_;
  bool enable_temp_;
  bool enable_rpy_;

  bool binary_output_;
  int binary_async_mode_;

  bool imu_compensated_;

  bool vpe_enable_;
  int vpe_heading_mode_;
  int vpe_filtering_mode_;
  int vpe_tuning_mode_;
  VnVector3 vpe_mag_base_tuning_;
  VnVector3 vpe_mag_adaptive_tuning_;
  VnVector3 vpe_mag_adaptive_filtering_;
  VnVector3 vpe_accel_base_tuning_;
  VnVector3 vpe_accel_adaptive_tuning_;
  VnVector3 vpe_accel_adaptive_filtering_;

  SyncInfo sync_info_;

  uint64_t device_time_zero_;
  rclcpp::Time ros_time_zero_;
  bool has_time_zero_{false};

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pd_imu_;
  rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr pd_mag_;
  rclcpp::Publisher<sensor_msgs::msg::FluidPressure>::SharedPtr pd_pres_;
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr pd_temp_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr pd_rpy_;

  void FixImuRate();
  void LoadParameters();
  void CreatePublishers();

  // Just don't like type that is ALL CAP
  using VnErrorCode = VN_ERROR_CODE;
  void VnEnsure(const VnErrorCode& error_code);
};

}  // namespace imu_vn_100

#endif  // IMU_VN_100_ROS_H_
