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

#include "imu_vn_100.h"

#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/FluidPressure.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/Temperature.h>
#include <std_msgs/Float64.h>

namespace imu_vn_100 {

ImuVn100* imu_vn_100_ptr;

using namespace geometry_msgs;
using namespace sensor_msgs;

using VnErrorCode = VN_ERROR_CODE;
void VnEnsure(const VnErrorCode& error_code);

std::ostream& operator<<(std::ostream& os, const VnVector3& v) {
  os << "(" << v.c0 << ", " << v.c1 << ", " << v.c2 << ")";
  return os;
}

std::string VectorToString(const std::vector<std::string>& vec) {
  std::string str;
  for (const auto& v : vec) {
    str += v + " | ";
  }
  return str;
}

Vector3 RosVecFromVnVec(const VnVector3& u) {
  Vector3 v;
  v.x = u.c0;
  v.y = u.c1;
  v.z = u.c2;
  return v;
}

Quaternion RosQuatFromVnQuat(const VnQuaternion& p) {
  Quaternion q;
  q.x = p.x;
  q.y = p.y;
  q.z = p.z;
  q.w = p.w;
  return q;
}

void AsyncListener(void* sender, VnDeviceCompositeData* data) {
  imu_vn_100_ptr->PublishData(*data);
}

void ImuVn100::SyncInfo::Update(const unsigned sync_count,
                                const ros::Time& sync_time) {
  if (rate <= 0) return;

  if (count != sync_count) {
    count = sync_count;
    time = sync_time;
  }
}

bool ImuVn100::SyncInfo::SyncEnabled() const { return rate > 0; }

void ImuVn100::SyncInfo::FixSyncRate() {
  // Check the sync out rate
  if (SyncEnabled()) {
    if (kBaseImuRate % rate != 0) {
      rate = kBaseImuRate / (kBaseImuRate / rate);
      ROS_INFO("Set SYNC_OUT_RATE to %d", rate);
    }
    skip_count =
        (std::floor(kBaseImuRate / static_cast<double>(rate) + 0.5f)) - 1;

    if (pulse_width_us > 10000) {
      ROS_INFO("Sync out pulse with is over 10ms. Reset to 1ms");
      pulse_width_us = 1000;
    }
    rate_double = rate;
  }

  ROS_INFO("Sync out rate: %d", rate);
}

ImuVn100::ImuVn100(const ros::NodeHandle& pnh)
    : pnh_(pnh),
      port_(std::string("/dev/ttyUSB0")),
      baudrate_(921600),
      frame_id_(std::string("imu")) {
  Initialize();
  imu_vn_100_ptr = this;
  ROS_INFO_STREAM("Diagnostic period: " << updater_.getPeriod() << " s");
}

ImuVn100::~ImuVn100() { Disconnect(); }

void ImuVn100::FixImuRate() {
  if (imu_rate_ <= 0) {
    ROS_WARN("Imu rate %d is < 0. Set to %d", imu_rate_, kDefaultImuRate);
    imu_rate_ = kDefaultImuRate;
  }

  if (kBaseImuRate % imu_rate_ != 0) {
    int imu_rate_old = imu_rate_;
    imu_rate_ = kBaseImuRate / (kBaseImuRate / imu_rate_old);
    ROS_WARN("Imu rate %d cannot evenly decimate base rate %d, reset to %d",
             imu_rate_old, kBaseImuRate, imu_rate_);
  }
}

void ImuVn100::LoadParameters() {
  pnh_.param<std::string>("port", port_, std::string("/dev/ttyUSB0"));
  pnh_.param<std::string>("frame_id", frame_id_, pnh_.getNamespace());
  pnh_.param("baudrate", baudrate_, 115200);
  pnh_.param("imu_rate", imu_rate_, kDefaultImuRate);

  pnh_.param("sync_rate", sync_info_.rate, kDefaultSyncOutRate);
  pnh_.param("sync_pulse_width_us", sync_info_.pulse_width_us, 1000);

  pnh_.param("binary_output", binary_output_, true);
  pnh_.param("binary_async_mode", binary_async_mode_,
             BINARY_ASYNC_MODE_SERIAL_2);

  pnh_.param("compensated", compensated_, false);
  pnh_.param("time_alpha", time_alpha_, 0.0);
  ROS_INFO("time_alpha: %f", time_alpha_);
  time_alpha_ = std::max(0.0, std::min(time_alpha_, 1.0));

  pnh_.param("vpe/enable", vpe_enable_, true);
  pnh_.param("vpe/heading_mode", vpe_heading_mode_, 1);
  pnh_.param("vpe/filtering_mode", vpe_filtering_mode_, 1);
  pnh_.param("vpe/tuning_mode", vpe_tuning_mode_, 1);

  pnh_.param("vpe/mag_tuning/base_tuning/x", vpe_mag_base_tuning_.c0, 4.0);
  pnh_.param("vpe/mag_tuning/base_tuning/y", vpe_mag_base_tuning_.c1, 4.0);
  pnh_.param("vpe/mag_tuning/base_tuning/z", vpe_mag_base_tuning_.c2, 4.0);
  pnh_.param("vpe/mag_tuning/adaptive_tuning/x", vpe_mag_adaptive_tuning_.c0,
             5.0);
  pnh_.param("vpe/mag_tuning/adaptive_tuning/y", vpe_mag_adaptive_tuning_.c1,
             5.0);
  pnh_.param("vpe/mag_tuning/adaptive_tuning/z", vpe_mag_adaptive_tuning_.c2,
             5.0);
  pnh_.param("vpe/mag_tuning/adaptive_filtering/x",
             vpe_mag_adaptive_filtering_.c0, 5.5);
  pnh_.param("vpe/mag_tuning/adaptive_filtering/y",
             vpe_mag_adaptive_filtering_.c1, 5.5);
  pnh_.param("vpe/mag_tuning/adaptive_filtering/z",
             vpe_mag_adaptive_filtering_.c2, 5.5);

  pnh_.param("vpe/accel_tuning/base_tuning/x", vpe_accel_base_tuning_.c0, 5.0);
  pnh_.param("vpe/accel_tuning/base_tuning/y", vpe_accel_base_tuning_.c1, 5.0);
  pnh_.param("vpe/accel_tuning/base_tuning/z", vpe_accel_base_tuning_.c2, 5.0);
  pnh_.param("vpe/accel_tuning/adaptive_tuning/x",
             vpe_accel_adaptive_tuning_.c0, 3.0);
  pnh_.param("vpe/accel_tuning/adaptive_tuning/y",
             vpe_accel_adaptive_tuning_.c1, 3.0);
  pnh_.param("vpe/accel_tuning/adaptive_tuning/z",
             vpe_accel_adaptive_tuning_.c2, 3.0);
  pnh_.param("vpe/accel_tuning/adaptive_filtering/x",
             vpe_accel_adaptive_filtering_.c0, 4.0);
  pnh_.param("vpe/accel_tuning/adaptive_filtering/y",
             vpe_accel_adaptive_filtering_.c1, 4.0);
  pnh_.param("vpe/accel_tuning/adaptive_filtering/z",
             vpe_accel_adaptive_filtering_.c2, 4.0);

  FixImuRate();
  sync_info_.FixSyncRate();
}

void ImuVn100::CreatePublishers() {
  imu_rate_double_ = static_cast<double>(imu_rate_);

  pub_dt_ = pnh_.advertise<std_msgs::Float64>("dt", 1);
  pub_dnow_ = pnh_.advertise<std_msgs::Float64>("dnow", 1);

  pub_imu_.Advertise<Imu>(pnh_, "imu", updater_, imu_rate_double_);
  ROS_INFO("Publish imu to %s", pub_imu_.pub.getTopic().c_str());

  if (pnh_.param("enable_mag_pres", true)) {
    pub_mag_.Advertise<MagneticField>(pnh_, "magnetic_field", updater_,
                                      imu_rate_double_);
    ROS_INFO("Publish magnetic filed to %s", pub_mag_.pub.getTopic().c_str());
    pub_pres_.Advertise<FluidPressure>(pnh_, "fluid_pressure", updater_,
                                       imu_rate_double_);
    ROS_INFO("Publish pressure to %s", pub_pres_.pub.getTopic().c_str());
    pub_temp_.Advertise<Temperature>(pnh_, "temperature", updater_,
                                     imu_rate_double_);
    ROS_INFO("Publish temperature to %s", pub_temp_.pub.getTopic().c_str());
  }
}

void ImuVn100::Initialize() {
  LoadParameters();

  ROS_DEBUG("Connecting to device");
  VnEnsure(vn100_connect(&imu_, port_.c_str(), 115200));
  ros::Duration(0.5).sleep();
  ROS_INFO("Connected to device at %s", port_.c_str());

  unsigned int old_baudrate;
  VnEnsure(vn100_getSerialBaudRate(&imu_, &old_baudrate));
  ROS_INFO("Default serial baudrate: %u", old_baudrate);

  ROS_INFO("Set serial baudrate to %d", baudrate_);
  VnEnsure(vn100_setSerialBaudRate(&imu_, baudrate_, true));

  ROS_DEBUG("Disconnecting the device");
  vn100_disconnect(&imu_);
  ros::Duration(0.5).sleep();

  ROS_DEBUG("Reconnecting to device");
  VnEnsure(vn100_connect(&imu_, port_.c_str(), baudrate_));
  ros::Duration(0.5).sleep();
  ROS_INFO("Connected to device at %s", port_.c_str());

  VnEnsure(vn100_getSerialBaudRate(&imu_, &old_baudrate));
  ROS_INFO("New serial baudrate: %u", old_baudrate);

  // Idle the device for intialization
  VnEnsure(vn100_pauseAsyncOutputs(&imu_, true));

  ROS_INFO("Fetching device info.");
  char model_number_buffer[30] = {0};
  int hardware_revision = 0;
  char serial_number_buffer[30] = {0};
  char firmware_version_buffer[30] = {0};

  VnEnsure(vn100_getModelNumber(&imu_, model_number_buffer, 30));
  ROS_INFO("Model number: %s", model_number_buffer);
  VnEnsure(vn100_getHardwareRevision(&imu_, &hardware_revision));
  ROS_INFO("Hardware revision: %d", hardware_revision);
  VnEnsure(vn100_getSerialNumber(&imu_, serial_number_buffer, 30));
  ROS_INFO("Serial number: %s", serial_number_buffer);
  VnEnsure(vn100_getFirmwareVersion(&imu_, firmware_version_buffer, 30));
  ROS_INFO("Firmware version: %s", firmware_version_buffer);

  if (sync_info_.SyncEnabled()) {
    ROS_INFO("Set Synchronization Control Register (id:32).");
    VnEnsure(vn100_setSynchronizationControl(
        &imu_, SYNCINMODE_COUNT, SYNCINEDGE_RISING, 0, SYNCOUTMODE_IMU_START,
        SYNCOUTPOLARITY_POSITIVE, sync_info_.skip_count,
        sync_info_.pulse_width_us * 1000, true));

    if (!binary_output_) {
      ROS_INFO("Set Communication Protocol Control Register (id:30).");
      VnEnsure(vn100_setCommunicationProtocolControl(
          &imu_, SERIALCOUNT_SYNCOUT_COUNT, SERIALSTATUS_OFF, SPICOUNT_NONE,
          SPISTATUS_OFF, SERIALCHECKSUM_8BIT, SPICHECKSUM_8BIT, ERRORMODE_SEND,
          true));
    }
  }

  uint8_t vpe_enable;
  uint8_t vpe_heading_mode;
  uint8_t vpe_filtering_mode;
  uint8_t vpe_tuning_mode;
  VnEnsure(vn100_getVpeControl(&imu_, &vpe_enable, &vpe_heading_mode,
                               &vpe_filtering_mode, &vpe_tuning_mode));
  ROS_INFO(
      "Default VPE enable: %hhu, heading mode: %hhu, filtering mode: %hhu, "
      "tuning mode: %hhu",
      vpe_enable, vpe_heading_mode, vpe_filtering_mode, vpe_tuning_mode);

  if (vpe_enable != vpe_enable_ || vpe_heading_mode != vpe_heading_mode_ ||
      vpe_filtering_mode != vpe_filtering_mode_ ||
      vpe_tuning_mode != vpe_tuning_mode_) {
    vpe_enable = vpe_enable_;
    vpe_heading_mode = vpe_heading_mode_;
    vpe_filtering_mode = vpe_filtering_mode_;
    vpe_tuning_mode = vpe_tuning_mode_;
    ROS_INFO(
        "Setting VPE enable: %hhu, heading mode: %hhu, filtering mode: %hhu, "
        "tuning mode: %hhu",
        vpe_enable, vpe_heading_mode, vpe_filtering_mode, vpe_tuning_mode);
    VnEnsure(vn100_setVpeControl(&imu_, vpe_enable, vpe_heading_mode,
                                 vpe_filtering_mode, vpe_tuning_mode, true));
  }

  if (vpe_enable_) {
    ROS_INFO_STREAM("Setting VPE MagnetometerBasicTuning BaseTuning "
                    << vpe_mag_base_tuning_);
    ROS_INFO_STREAM("Setting VPE MagnetometerBasicTuning AdaptiveTuning "
                    << vpe_mag_adaptive_tuning_);
    ROS_INFO_STREAM("Setting VPE MagnetometerBasicTuning AdaptiveFiltering "
                    << vpe_mag_adaptive_filtering_);

    VnEnsure(vn100_setVpeMagnetometerBasicTuning(
        &imu_, vpe_mag_base_tuning_, vpe_mag_adaptive_tuning_,
        vpe_mag_adaptive_filtering_, true));

    ROS_INFO_STREAM("Setting VPE AccelerometerBasicTuning BaseTuning "
                    << vpe_accel_base_tuning_);
    ROS_INFO_STREAM("Setting VPE AccelerometerBasicTuning AdaptiveTuning "
                    << vpe_accel_adaptive_tuning_);
    ROS_INFO_STREAM("Setting VPE AccelerometerBasicTuning AdaptiveFiltering "
                    << vpe_accel_adaptive_filtering_);

    VnEnsure(vn100_setVpeAccelerometerBasicTuning(
        &imu_, vpe_accel_base_tuning_, vpe_accel_adaptive_tuning_,
        vpe_accel_adaptive_filtering_, true));
  }

  CreatePublishers();

  const auto hardware_id = std::string("vn100-") +
                           std::string(model_number_buffer) +
                           std::string(serial_number_buffer);
  updater_.setHardwareID(hardware_id);
}

void ImuVn100::Stream(bool async) {
  // Pause the device first
  VnEnsure(vn100_pauseAsyncOutputs(&imu_, true));

  if (async) {
    VnEnsure(vn100_setAsynchronousDataOutputType(&imu_, VNASYNC_OFF, true));

    if (binary_output_) {
      // Set the binary output data type and data rate
      // Add BG1_TIME_STARTUP to have more accurate time stamp
      uint16_t grp1 = BG1_QTN | BG1_SYNC_IN_CNT | BG1_TIME_STARTUP;
      std::vector<std::string> sgrp1 = {"BG1_QTN", "BG1_SYNC_IN_CNT",
                                        "BG1_TIME_STARTUP"};

      if (compensated_) {
        grp1 |= BG1_ACCEL | BG1_ANGULAR_RATE;
        sgrp1.push_back("BG1_ACCEL");
        sgrp1.push_back("BG1_ANGULAR_RATE");
      } else {
        grp1 |= BG1_IMU;
        sgrp1.push_back("BG1_IMU");
      }

      if (pub_mag_) {
        grp1 |= BG1_MAG_PRES;
        sgrp1.push_back("BG1_MAG_PRES");
      }

      ROS_INFO("Streaming #1: %s", VectorToString(sgrp1).c_str());

      VnEnsure(vn100_setBinaryOutput1Configuration(
          &imu_, binary_async_mode_, kBaseImuRate / imu_rate_, grp1, BG3_NONE,
          BG5_NONE, true));
    } else {
      // Set the ASCII output data type and data rate
      // ROS_INFO("Configure the output data type and frequency (id: 6 & 7)");
      VnEnsure(vn100_setAsynchronousDataOutputType(&imu_, VNASYNC_VNIMU, true));
    }

    // Add a callback function for new data event
    VnEnsure(vn100_registerAsyncDataReceivedListener(&imu_, &AsyncListener));

    ROS_INFO("Setting IMU rate to %d", imu_rate_);
    VnEnsure(vn100_setAsynchronousDataOutputFrequency(&imu_, imu_rate_, true));
  } else {
    // Mute the stream
    ROS_DEBUG("Mute the device");
    VnEnsure(vn100_setAsynchronousDataOutputType(&imu_, VNASYNC_OFF, true));
    // Remove the callback function for new data event
    VnEnsure(vn100_unregisterAsyncDataReceivedListener(&imu_, &AsyncListener));
  }

  // Resume the device
  VnEnsure(vn100_resumeAsyncOutputs(&imu_, true));
}

void ImuVn100::Disconnect() {
  ROS_INFO("Reset and disconnect from imu");
  vn100_reset(&imu_);
  vn100_disconnect(&imu_);
}

void ImuVn100::PublishData(const VnDeviceCompositeData& data) {
  // Handle timestamp
  const auto ros_time_now = ros::Time::now();
  if (dev_time_last_ == 0) {
    ros_time_last_ = ros_time_now;
    stamp_last_ = ros_time_last_;
    dev_time_last_ = data.timeStartup;
    //    ROS_INFO_STREAM("Set device time zero to " << dev_time_last_);
    //    ROS_INFO_STREAM("Set ros time zero to " << ros_time_zero_.toSec());
  }

  // delta time from device
  const int64_t dt_dev = data.timeStartup - dev_time_last_;
  // delta time from ros
  const int64_t dt_ros = (ros_time_now - ros_time_last_).toNSec();
  // filtered delta time
  const int64_t dt_filtered = dt_ros * time_alpha_ + dt_dev * (1 - time_alpha_);
  ROS_DEBUG_STREAM("dt_dev: " << dt_dev << ", dt_ros: " << dt_ros
                              << ", dt_filtered: " << dt_filtered);

  ros::Duration dt;
  dt.fromNSec(dt_filtered);
  const ros::Time stamp = stamp_last_ + dt;

  if (dt_dev < 0) {
    // This should never happen, but we check for it anyway
    ROS_WARN_STREAM("dt: " << dt_dev << " ns, startup: " << data.timeStartup
                           << ", zero: " << dev_time_last_);
  }

  {
    std_msgs::Float64 msg;
    msg.data = dt.toSec();
    pub_dt_.publish(msg);
  }
  {
    std_msgs::Float64 msg;
    msg.data = (ros_time_now - stamp).toSec();
    pub_dnow_.publish(msg);
  }

  ros_time_last_ = ros_time_now;
  dev_time_last_ = data.timeStartup;
  stamp_last_ = stamp;

  // Fill in data
  Imu imu_msg;
  imu_msg.header.stamp = stamp;
  imu_msg.header.frame_id = frame_id_;

  if (compensated_) {
    imu_msg.linear_acceleration = RosVecFromVnVec(data.acceleration);
    imu_msg.angular_velocity = RosVecFromVnVec(data.angularRate);
  } else {
    // NOTE: The IMU angular velocity and linear acceleration outputs are
    // swapped. And also why are they different?
    imu_msg.angular_velocity = RosVecFromVnVec(data.accelerationUncompensated);
    imu_msg.linear_acceleration =
        RosVecFromVnVec(data.angularRateUncompensated);
  }

  if (binary_output_) {
    imu_msg.orientation = RosQuatFromVnQuat(data.quaternion);
  }

  pub_imu_.Publish(imu_msg);

  if (pub_mag_) {
    {
      MagneticField msg;
      msg.header = imu_msg.header;
      msg.magnetic_field = RosVecFromVnVec(data.magnetic);  // compensated
      pub_mag_.Publish(msg);
    }
    {
      FluidPressure msg;
      msg.header = imu_msg.header;
      msg.fluid_pressure = data.pressure;
      pub_pres_.Publish(msg);
    }
    {
      Temperature msg;
      msg.header = imu_msg.header;
      msg.temperature = data.temperature;
      pub_temp_.Publish(msg);
    }
  }

  sync_info_.Update(data.syncInCnt, imu_msg.header.stamp);
  //  updater_.update();
}

void VnEnsure(const VnErrorCode& error_code) {
  if (error_code == VNERR_NO_ERROR) return;

  switch (error_code) {
    case VNERR_UNKNOWN_ERROR:
      throw std::runtime_error("VN: Unknown error");
    case VNERR_NOT_IMPLEMENTED:
      throw std::runtime_error("VN: Not implemented");
    case VNERR_TIMEOUT:
      ROS_WARN("Opertation time out");
      break;
    case VNERR_SENSOR_INVALID_PARAMETER:
      ROS_WARN("VN: Sensor invalid paramter");
      break;
    case VNERR_INVALID_VALUE:
      ROS_WARN("VN: Invalid value");
      break;
    case VNERR_FILE_NOT_FOUND:
      ROS_WARN("VN: File not found");
      break;
    case VNERR_NOT_CONNECTED:
      throw std::runtime_error("VN: not connected");
    case VNERR_PERMISSION_DENIED:
      throw std::runtime_error("VN: Permission denied");
    default:
      ROS_WARN("Unhandled error type");
  }
}

}  //  namespace imu_vn_100

int main(int argc, char** argv) {
  ros::init(argc, argv, "imu_vn_100");
  ros::NodeHandle pnh("~");

  imu_vn_100::ImuVn100 imu(pnh);
  imu.Stream(true);
  ros::spin();
}
