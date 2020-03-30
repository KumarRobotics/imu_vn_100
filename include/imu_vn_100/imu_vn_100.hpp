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

#ifndef IMU_VN_100_HPP_
#define IMU_VN_100_HPP_

#include <string>

#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
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

  explicit ImuVn100(const rclcpp::NodeOptions& options);
  ImuVn100(const ImuVn100&) = delete;
  ImuVn100& operator=(const ImuVn100&) = delete;
  ~ImuVn100() override;

  void Initialize();

  void Stream(bool async);

  void PublishData(const VnDeviceCompositeData& data);

  void RequestOnce();

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

 private:
  Vn100 imu_{};

  // Settings
  std::string port_;
  uint32_t baudrate_{0};
  uint32_t initial_baudrate_{0};
  int imu_rate_{0};
  double imu_rate_double_{0.0};
  std::string frame_id_;
  enum class AxesConvention {
    NED,
    ENU,
  };
  AxesConvention axes_convention_;

  double linear_acceleration_variance_{0.0};
  double angular_velocity_variance_{0.0};
  double magnetic_field_variance_{0.0};

  bool enable_mag_{false};
  bool enable_pres_{false};
  bool enable_temp_{false};
  bool enable_rpy_{false};

  bool binary_output_{false};
  int binary_async_mode_{0};

  bool imu_compensated_{false};

  bool vpe_enable_{false};
  int vpe_heading_mode_{0};
  int vpe_filtering_mode_{0};
  int vpe_tuning_mode_{0};
  VnVector3 vpe_mag_base_tuning_{};
  VnVector3 vpe_mag_adaptive_tuning_{};
  VnVector3 vpe_mag_adaptive_filtering_{};
  VnVector3 vpe_accel_base_tuning_{};
  VnVector3 vpe_accel_adaptive_tuning_{};
  VnVector3 vpe_accel_adaptive_filtering_{};

  int hsi_mode_;
  int hsi_output_;
  int hsi_converge_rate_;

  bool ref_use_models_;
  uint32_t ref_recalc_threshold_m_;

  SyncInfo sync_info_;

  rclcpp::Time last_cb_time_;
  bool synchronize_timestamps_{true};
  rclcpp::Time ros_time_zero_;
  int64_t cb_delta_epsilon_ns_{0};
  uint64_t data_time_zero_ns_{0};
  bool can_publish_{false};
  int64_t time_resync_interval_ns_{0};
  int64_t data_interval_ns_{0};
  uint64_t last_ros_stamp_ns_{0};

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pd_imu_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pd_imu_raw_;
  rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr pd_mag_;
  rclcpp::Publisher<sensor_msgs::msg::FluidPressure>::SharedPtr pd_pres_;
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr pd_temp_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr pd_rpy_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub_gps_fix_;

  void HandleGPSFix(std::unique_ptr<sensor_msgs::msg::NavSatFix> msg);

  void FixImuRate();
  void LoadParameters();
  void CreateSubscribers();
  void CreatePublishers();

  // Just don't like type that is ALL CAP
  using VnErrorCode = VN_ERROR_CODE;
  static void VnEnsure(const VnErrorCode& error_code);
};

}  // namespace imu_vn_100

#endif  // IMU_VN_100_HPP_
