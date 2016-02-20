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

#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

#include <ros/ros.h>
#include <ros/node_handle.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/FluidPressure.h>
#include <sensor_msgs/Temperature.h>

#include <vn100.h>

namespace imu_vn_100 {

using uuid = long long int;
/**
 * @brief SyncInfo Contains the data for reporting
 *    synchronization status. And it handles the
 *    thread-safe reading and writing.
 */
struct SyncInfo {
  uuid sync_count_;
  ros::Time sync_time_;
  boost::shared_mutex mtx;

  SyncInfo() : sync_count_(-1), sync_time_(ros::Time::now()) {}

  uuid sync_count() {
    boost::shared_lock<boost::shared_mutex> read_lock(mtx);
    return sync_count_;
  }

  ros::Time sync_time() {
    boost::shared_lock<boost::shared_mutex> read_lock(mtx);
    return sync_time_;
  }

  void setSyncCount(const uuid& new_count) {
    boost::unique_lock<boost::shared_mutex> write_lock(mtx);
    sync_count_ = new_count;
  }

  void setSyncTime(const ros::Time& new_time) {
    boost::unique_lock<boost::shared_mutex> write_lock(mtx);
    sync_time_ = new_time;
  }
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

  bool Initialize();
  /**
   * @brief enableIMUStream Enable or disable IMU stream
   * @param enabled If ture, the continuous stream is enabled
   */
  void Stream(bool enabled);

  void RequestOnce();

  void Idle(bool need_reply = true);

  void Resume(bool need_reply = true);

  void Disconnect();

  float getSyncRate() { return act_sync_out_rate; }

  uuid getSyncCount() { return sync_info.sync_count(); }

  ros::Time getSyncTime() { return sync_info.sync_time(); }

 private:
  // Settings
  std::string port_;
  int baudrate_;
  int imu_rate_;
  std::string frame_id_;

  bool enable_mag;
  bool enable_pres;
  bool enable_temp;
  bool use_binary_output;

  bool enable_sync_out;
  int sync_out_pulse_width;
  int sync_out_rate;
  float act_sync_out_rate;
  int sync_out_skip_count;

  ros::NodeHandle pnh_;
  Vn100 imu_;

  // Tracking the triggering signal
  SyncInfo sync_info;

  // Publishers
  ros::Publisher pub_imu_;
  ros::Publisher pub_mag_;
  ros::Publisher pub_pres_;
  ros::Publisher pub_temp_;

  // diagnostic_updater resources
  double update_rate;
  boost::shared_ptr<diagnostic_updater::Updater> updater;
  boost::shared_ptr<diagnostic_updater::TopicDiagnostic> imu_diag;
  boost::shared_ptr<diagnostic_updater::TopicDiagnostic> mag_diag;
  boost::shared_ptr<diagnostic_updater::TopicDiagnostic> pres_diag;
  boost::shared_ptr<diagnostic_updater::TopicDiagnostic> temp_diag;

  bool LoadParameters();
  void CreatePublishers();
  void ErrorCodeParser(const VN_ERROR_CODE& error_code);

  // Publish IMU msgs
  void PublishData();
  // Callback function for adding meta info in the diag msgs
  void updateDiagnosticInfo(diagnostic_updater::DiagnosticStatusWrapper& stat);
};

}  // namespace imu_vn_100

#endif  // IMU_VN_100_ROS_H_
