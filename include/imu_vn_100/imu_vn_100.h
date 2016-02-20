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

#include <ros/ros.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/FluidPressure.h>
#include <sensor_msgs/Temperature.h>

#include <vn100.h>

namespace imu_vn_100 {

using uuid = long long int;

struct SyncInfo {
  uuid sync_count_;
  ros::Time sync_time_;

  SyncInfo() : sync_count_(-1), sync_time_(ros::Time::now()) {}

  uuid sync_count() { return sync_count_; }

  ros::Time sync_time() { return sync_time_; }

  void set_sync_count(const uuid& sync_count) { sync_count_ = sync_count; }

  void set_sync_time(const ros::Time& sync_time) { sync_time_ = sync_time; }
};

/**
 * @brief ImuRosBase The class is a ros wrapper for the Imu class
 * @author Ke Sun
 */
class ImuVn100 {
 public:
  explicit ImuVn100(const ros::NodeHandle& pnh);
  ImuVn100(const ImuVn100&) = delete;
  ImuVn100& operator=(const ImuVn100&) = delete;
  ~ImuVn100();

  void Initialize();
  /**
   * @brief enableIMUStream Enable or disable IMU stream
   * @param enabled If ture, the continuous stream is enabled
   */
  void Stream(bool async = true);

  void PublishData(const VnDeviceCompositeData& data);

  void RequestOnce();

  void Idle(bool need_reply = true);

  void Resume(bool need_reply = true);

  void Disconnect();

  void Configure();

  int sync_out_rate() const;

  uuid sync_count() const;

  const ros::Time sync_time() const;

 private:
  static constexpr int kBaseImuRate = 800;
  static constexpr int kDefaultImuRate = 100;
  static constexpr int kDefaultSyncOutRate = 20;

  ros::NodeHandle pnh_;
  Vn100 imu_;

  // Settings
  std::string port_;
  int baudrate_ = 921600;
  int imu_rate_ = kDefaultImuRate;
  double imu_rate_update_ = kDefaultImuRate;
  std::string frame_id_;

  bool enable_mag_ = true;
  bool enable_pres_ = true;
  bool enable_temp_ = true;
  bool use_binary_output_ = true;

  int sync_out_rate_ = kDefaultSyncOutRate;
  int sync_out_pulse_width_us_;
  int sync_out_skip_cnt_;

  // Tracking the triggering signal
  SyncInfo sync_info;

  // Publishers
  ros::Publisher pub_imu_;
  ros::Publisher pub_mag_;
  ros::Publisher pub_pres_;
  ros::Publisher pub_temp_;

  // diagnostic_updater resources
  //  boost::shared_ptr<diagnostic_updater::Updater> updater;
  //  boost::shared_ptr<diagnostic_updater::TopicDiagnostic> imu_diag;
  //  boost::shared_ptr<diagnostic_updater::TopicDiagnostic> mag_diag;
  //  boost::shared_ptr<diagnostic_updater::TopicDiagnostic> pres_diag;
  //  boost::shared_ptr<diagnostic_updater::TopicDiagnostic> temp_diag;

  void FixImuRate();
  void FixSyncOutRate();

  void LoadParameters();
  void CreatePublishers();

  //  void UpdateDiagnosticInfo(diagnostic_updater::DiagnosticStatusWrapper&
  //  stat);
};

// Just don't like type that is ALL CAP
using VnErrorCode = VN_ERROR_CODE;
void VnEnsure(const VnErrorCode& error_code);

}  // namespace imu_vn_100

#endif  // IMU_VN_100_ROS_H_
