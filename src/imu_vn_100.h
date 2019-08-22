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
#pragma once

#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>
#include <ros/ros.h>

#include "vn100.h"

namespace imu_vn_100 {

namespace du = diagnostic_updater;

struct DiagnosedPublisher {
  template <typename T>
  void Advertise(ros::NodeHandle& nh, const std::string& topic,
                 du::Updater& updater, double& rate) {
    pub = nh.advertise<T>(topic, 1);
    diag.reset(new du::TopicDiagnostic(topic, updater, {&rate, &rate, 0.01, 10},
                                       {-1 / rate, 1 / rate}));
  }

  template <typename T>
  void Publish(const T& message) {
    diag->tick(message.header.stamp);
    pub.publish(message);
  }

  operator bool() const { return diag != nullptr; }

  ros::Publisher pub;
  boost::shared_ptr<du::TopicDiagnostic> diag;
};

static constexpr int kBaseImuRate = 800;
static constexpr int kDefaultImuRate = 100;
static constexpr int kDefaultSyncOutRate = 20;

/**
 * @brief ImuVn100 The class is a ros wrapper for the Imu class
 * @author Ke Sun
 */
class ImuVn100 {
 public:
  explicit ImuVn100(const ros::NodeHandle& pnh);
  ImuVn100(const ImuVn100&) = delete;
  ImuVn100& operator=(const ImuVn100&) = delete;
  ~ImuVn100();

  void Initialize();

  void Stream(bool async = true);

  void PublishData(const VnDeviceCompositeData& data);

  void Disconnect();

  struct SyncInfo {
    unsigned count = 0;
    ros::Time time;

    int rate = -1;
    double rate_double = -1;
    int pulse_width_us = 1000;
    int skip_count = 0;

    void Update(const unsigned sync_count, const ros::Time& sync_time);
    void FixSyncRate();
    bool SyncEnabled() const;
  };

  const SyncInfo sync_info() const { return sync_info_; }

 private:
  void FixImuRate();
  void LoadParameters();
  void CreatePublishers();

  ros::NodeHandle pnh_;
  Vn100 imu_;

  // Settings
  std::string port_;
  int baudrate_ = 921600;
  int imu_rate_ = kDefaultImuRate;
  double imu_rate_double_ = kDefaultImuRate;
  std::string frame_id_;

  ros::Time ros_time_last_;       ///< previous time stamp
  ros::Time ros_time_zero_;       ///< ros time of first data
  uint64_t device_time_zero_{0};  ///< device time of first data, ns
  double time_alpha_{0.0};        ///< t1 = dnow * a + ddev * (1-a) + t0

  bool binary_output_ = true;
  int binary_async_mode_ = BINARY_ASYNC_MODE_SERIAL_2;
  bool compensated_ = false;

  bool vpe_enable_ = true;
  int vpe_heading_mode_ = 1;
  int vpe_filtering_mode_ = 1;
  int vpe_tuning_mode_ = 1;

  VnVector3 vpe_mag_base_tuning_;
  VnVector3 vpe_mag_adaptive_tuning_;
  VnVector3 vpe_mag_adaptive_filtering_;
  VnVector3 vpe_accel_base_tuning_;
  VnVector3 vpe_accel_adaptive_tuning_;
  VnVector3 vpe_accel_adaptive_filtering_;

  SyncInfo sync_info_;

  ros::Publisher pub_dt_;    ///< publish time between curr and last
  ros::Publisher pub_dnow_;  ///< publish time between curr and now
  du::Updater updater_;
  DiagnosedPublisher pub_imu_, pub_mag_, pub_pres_, pub_temp_, pub_ypr_;
};

}  // namespace imu_vn_100
