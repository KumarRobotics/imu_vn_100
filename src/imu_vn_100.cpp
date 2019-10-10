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

#include <algorithm>
#include <cmath>
#include <iterator>
#include <list>
#include <sstream>
#include <stdexcept>
#include <string>

#include <imu_vn_100/imu_vn_100.h>

#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <sensor_msgs/msg/temperature.hpp>

namespace imu_vn_100 {

// LESS HACK IS STILL HACK
ImuVn100* imu_vn_100_ptr;

void RosVector3FromVnVector3(geometry_msgs::msg::Vector3& ros_vec3,
                             const VnVector3& vn_vec3);
void RosQuaternionFromVnQuaternion(geometry_msgs::msg::Quaternion& ros_quat,
                                   const VnQuaternion& vn_quat);

void AsyncListener(void* sender, VnDeviceCompositeData* data) {
  (void)sender;
  imu_vn_100_ptr->PublishData(*data);
}

constexpr int ImuVn100::kBaseImuRate;
constexpr int ImuVn100::kDefaultImuRate;
constexpr int ImuVn100::kDefaultSyncOutRate;

void ImuVn100::SyncInfo::Update(const unsigned sync_count,
                                const rclcpp::Time& sync_time) {
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
    if (ImuVn100::kBaseImuRate % rate != 0) {
      rate = ImuVn100::kBaseImuRate / (ImuVn100::kBaseImuRate / rate);
    }
    skip_count =
        (std::floor(ImuVn100::kBaseImuRate / static_cast<double>(rate) +
                    0.5f)) -
        1;

    if (pulse_width_us > 10000) {
      pulse_width_us = 1000;
    }
    rate_double = rate;
  }
}

ImuVn100::ImuVn100(const rclcpp::NodeOptions& options) : rclcpp::Node("imu_vn_100", options)
{
  Initialize();
  imu_vn_100_ptr = this;
  this->Stream(true);
}

ImuVn100::~ImuVn100() { Disconnect(); }

void ImuVn100::FixImuRate() {
  if (imu_rate_ <= 0) {
    RCLCPP_WARN(get_logger(), "Imu rate %d is < 0. Set to %d", imu_rate_, kDefaultImuRate);
    imu_rate_ = kDefaultImuRate;
  }

  if (kBaseImuRate % imu_rate_ != 0) {
    int imu_rate_old = imu_rate_;
    imu_rate_ = kBaseImuRate / (kBaseImuRate / imu_rate_old);
    RCLCPP_WARN(get_logger(), "Imu rate %d cannot evenly decimate base rate %d, reset to %d",
                imu_rate_old, kBaseImuRate, imu_rate_);
  }
}

void ImuVn100::LoadParameters() {
  port_ = declare_parameter("port", std::string("/dev/ttyUSB0"));
  frame_id_ = declare_parameter("frame_id", std::string("imu_link"));
  imu_rate_ = declare_parameter("imu_rate", kDefaultImuRate);
  initial_baudrate_ = declare_parameter("initial_baudrate", 115200);
  baudrate_ = declare_parameter("baudrate", 921600);

  // From the datasheet at https://www.vectornav.com/products/vn-100/specifications,
  // we see that the accelerometer noise density is 0.14 mg/sqrt(Hz) and the bandwidth
  // is 260 Hz.  We can convert that to RMS (standard deviation) using the following:
  //
  // RMS = noise_density * sqrt(bandwidth)
  // RMS = 0.14 mg/sqrt(Hz) * sqrt(260)
  // RMS = 2.25743217 mg
  //
  // And converting to g:
  //
  // RMS = 2.25743217 mg * 1g/1000.0mg
  // RMS = 0.002257432 g
  double linear_acceleration_stddev = declare_parameter("linear_acceleration_stddev", 0.002257432);
  linear_acceleration_covariance_ = linear_acceleration_stddev * linear_acceleration_stddev;

  // From the datasheet, the gyroscope noise density is 0.0035 degree/second sqrt(Hz),
  // and the bandwidth is 256 Hz.  Using the formula above:
  //
  // RMS = 0.0035 degree/second sqrt(Hz) * sqrt(256)
  // RMS = 0.056 degree/second
  //
  // And converting to radians/sec:
  //
  // RMS = 0.056 degree/second * pi/180
  // RMS = 0.000977384 rad/second
  double angular_velocity_stddev = declare_parameter("angular_velocity_stddev", 0.000977384);
  angular_velocity_covariance_ = angular_velocity_stddev * angular_velocity_stddev;

  // From the datasheet, the magnetic field noise density is 140 uG/sqrt(Hz)
  // and the bandwidth is 200 Hz.  Using the formula above:
  //
  // RMS = 140 uG/sqrt(Hz) * sqrt(200)
  // RMS = 1979.898987322 uG
  //
  // Converting to G:
  //
  // RMS = 1979.898987322 uG * 1mG/1000.0uG * 1G/1000.0mG
  // RMS = 0.001979899 G
  //
  // Converting to Tesla:
  //
  // RMS = 0.001979899 G * 1T/10000.0G
  // RMS = 0.000000198 T
  double magnetic_field_stddev = declare_parameter("magnetic_field_stddev", 0.000000198);
  magnetic_field_covariance_ = magnetic_field_stddev * magnetic_field_stddev;

  enable_mag_ = declare_parameter("enable_mag", true);
  enable_pres_ = declare_parameter("enable_pres", true);
  enable_temp_ = declare_parameter("enable_temp", true);
  enable_rpy_ = declare_parameter("enable_rpy", false);

  sync_info_.rate = declare_parameter("sync_rate", kDefaultSyncOutRate);
  sync_info_.pulse_width_us = declare_parameter("sync_pulse_width_us", 1000);

  binary_output_ = declare_parameter("binary_output", true);
  binary_async_mode_ = declare_parameter("binary_async_mode", BINARY_ASYNC_MODE_SERIAL_2);

  imu_compensated_ = declare_parameter("imu_compensated", false);

  vpe_enable_ = declare_parameter("vpe.enable", true);

  vpe_heading_mode_ = declare_parameter("vpe.heading_mode", 1);
  vpe_filtering_mode_ = declare_parameter("vpe.filtering_mode", 1);
  vpe_tuning_mode_ = declare_parameter("vpe.tuning_mode", 1);

  vpe_mag_base_tuning_.c0 = declare_parameter("vpe.mag_tuning.base_tuning.x", 4.0);
  vpe_mag_base_tuning_.c1 = declare_parameter("vpe.mag_tuning.base_tuning.y", 4.0);
  vpe_mag_base_tuning_.c2 = declare_parameter("vpe.mag_tuning.base_tuning.z", 4.0);
  vpe_mag_adaptive_tuning_.c0 = declare_parameter("vpe.mag_tuning.adaptive_tuning.x", 5.0);
  vpe_mag_adaptive_tuning_.c1 = declare_parameter("vpe.mag_tuning.adaptive_tuning.y", 5.0);
  vpe_mag_adaptive_tuning_.c2 = declare_parameter("vpe.mag_tuning.adaptive_tuning.z", 5.0);
  vpe_mag_adaptive_filtering_.c0 = declare_parameter("vpe.mag_tuning.adaptive_filtering.x", 5.5);
  vpe_mag_adaptive_filtering_.c1 = declare_parameter("vpe.mag_tuning.adaptive_filtering.y", 5.5);
  vpe_mag_adaptive_filtering_.c2 = declare_parameter("vpe.mag_tuning.adaptive_filtering.z", 5.5);

  vpe_accel_base_tuning_.c0 = declare_parameter("vpe.accel_tuning.base_tuning.x", 5.0);
  vpe_accel_base_tuning_.c1 = declare_parameter("vpe.accel_tuning.base_tuning.y", 5.0);
  vpe_accel_base_tuning_.c2 = declare_parameter("vpe.accel_tuning.base_tuning.z", 5.0);
  vpe_accel_adaptive_tuning_.c0 = declare_parameter("vpe.accel_tuning.adaptive_tuning.x", 3.0);
  vpe_accel_adaptive_tuning_.c1 = declare_parameter("vpe.accel_tuning.adaptive_tuning.y", 3.0);
  vpe_accel_adaptive_tuning_.c2 = declare_parameter("vpe.accel_tuning.adaptive_tuning.z", 3.0);
  vpe_accel_adaptive_filtering_.c0 = declare_parameter("vpe.accel_tuning.adaptive_filtering.x", 4.0);
  vpe_accel_adaptive_filtering_.c1 = declare_parameter("vpe.accel_tuning.adaptive_filtering.y", 4.0);
  vpe_accel_adaptive_filtering_.c2 = declare_parameter("vpe.accel_tuning.adaptive_filtering.z", 4.0);

  int time_resync_ms = this->declare_parameter("time_resynchronization_interval_ms", 5000);
  time_resync_interval_ns_ = static_cast<int64_t>(time_resync_ms) * 1000 * 1000;

  int cb_delta_epsilon_ms = this->declare_parameter("callback_delta_epsilon_ms", 1);
  cb_delta_epsilon_ns_ = cb_delta_epsilon_ms * 1000 * 1000;

  int data_interval_ms = 1000 / imu_rate_;
  if (cb_delta_epsilon_ms >= data_interval_ms) {
    throw std::runtime_error("Callback epsilon is larger than the data interval; "
                             "this can never work");
  }
  data_interval_ns_ = data_interval_ms * 1000 * 1000;

  FixImuRate();
  sync_info_.FixSyncRate();
  RCLCPP_INFO(get_logger(), "Sync out rate: %d", sync_info_.rate);
}

void ImuVn100::CreatePublishers() {
  imu_rate_double_ = imu_rate_;
  pd_imu_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data", 10);
  pd_imu_raw_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", 10);
  if (enable_mag_) {
    pd_mag_ = this->create_publisher<sensor_msgs::msg::MagneticField>("imu/mag", 10);
  }
  if (enable_pres_) {
    pd_pres_ = this->create_publisher<sensor_msgs::msg::FluidPressure>("imu/fluid_pressure", 10);
  }
  if (enable_temp_) {
    pd_temp_ = this->create_publisher<sensor_msgs::msg::Temperature>("imu/temperature", 10);
  }
  if (enable_rpy_) {
    pd_rpy_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("imu/rpy", 10);
  }
}

void ImuVn100::Initialize() {
  LoadParameters();

  RCLCPP_INFO(get_logger(), "Connecting to device %s at baudrate %u", port_.c_str(), initial_baudrate_);
  VnEnsure(vn100_connect(&imu_, port_.c_str(), initial_baudrate_));
  rclcpp::sleep_for(std::chrono::milliseconds(500));

  uint32_t old_baudrate;
  VnEnsure(vn100_getSerialBaudRate(&imu_, &old_baudrate));
  // We don't know if we've successfully connected until an operation succeeds,
  // so only print this once `getSerialBaudRate` has succeeded.
  RCLCPP_INFO(get_logger(), "Connected to device %s at baudrate %u", port_.c_str(), old_baudrate);

  if (initial_baudrate_ != baudrate_) {
    RCLCPP_INFO(get_logger(), "Set serial baudrate to %u", baudrate_);
    VnEnsure(vn100_setSerialBaudRate(&imu_, baudrate_, true));

    RCLCPP_DEBUG(get_logger(), "Disconnecting the device");
    vn100_disconnect(&imu_);
    rclcpp::sleep_for(std::chrono::milliseconds(500));

    RCLCPP_DEBUG(get_logger(), "Reconnecting to device");
    VnEnsure(vn100_connect(&imu_, port_.c_str(), baudrate_));
    rclcpp::sleep_for(std::chrono::milliseconds(500));
    RCLCPP_INFO(get_logger(), "Connected to device at %s", port_.c_str());
  }

  VnEnsure(vn100_getSerialBaudRate(&imu_, &old_baudrate));
  RCLCPP_INFO(get_logger(), "New serial baudrate: %u", old_baudrate);

  // Idle the device for initialization
  VnEnsure(vn100_pauseAsyncOutputs(&imu_, true));

  RCLCPP_INFO(get_logger(), "Fetching device info.");
  char model_number_buffer[30] = {0};
  int hardware_revision = 0;
  char serial_number_buffer[30] = {0};
  char firmware_version_buffer[30] = {0};

  VnEnsure(vn100_getModelNumber(&imu_, model_number_buffer, 30));
  RCLCPP_INFO(get_logger(), "Model number: %s", model_number_buffer);
  VnEnsure(vn100_getHardwareRevision(&imu_, &hardware_revision));
  RCLCPP_INFO(get_logger(), "Hardware revision: %d", hardware_revision);
  VnEnsure(vn100_getSerialNumber(&imu_, serial_number_buffer, 30));
  RCLCPP_INFO(get_logger(), "Serial number: %s", serial_number_buffer);
  VnEnsure(vn100_getFirmwareVersion(&imu_, firmware_version_buffer, 30));
  RCLCPP_INFO(get_logger(), "Firmware version: %s", firmware_version_buffer);

  if (sync_info_.SyncEnabled()) {
    RCLCPP_INFO(get_logger(), "Set Synchronization Control Register (id:32).");
    VnEnsure(vn100_setSynchronizationControl(
        &imu_, SYNCINMODE_COUNT, SYNCINEDGE_RISING, 0, SYNCOUTMODE_IMU_START,
        SYNCOUTPOLARITY_POSITIVE, sync_info_.skip_count,
        sync_info_.pulse_width_us * 1000, true));

    if (!binary_output_) {
      RCLCPP_INFO(get_logger(), "Set Communication Protocol Control Register (id:30).");
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
  RCLCPP_INFO(get_logger(), "Default VPE enable: %hhu", vpe_enable);
  RCLCPP_INFO(get_logger(), "Default VPE heading mode: %hhu", vpe_heading_mode);
  RCLCPP_INFO(get_logger(), "Default VPE filtering mode: %hhu", vpe_filtering_mode);
  RCLCPP_INFO(get_logger(), "Default VPE tuning mode: %hhu", vpe_tuning_mode);
  if (vpe_enable != vpe_enable_ ||
      vpe_heading_mode != vpe_heading_mode_ ||
      vpe_filtering_mode != vpe_filtering_mode_ ||
      vpe_tuning_mode != vpe_tuning_mode_) {
      vpe_enable = vpe_enable_;
      vpe_heading_mode = vpe_heading_mode_;
      vpe_filtering_mode = vpe_filtering_mode_;
      vpe_tuning_mode = vpe_tuning_mode_;
      RCLCPP_INFO(get_logger(), "Setting VPE enable: %hhu", vpe_enable);
      RCLCPP_INFO(get_logger(), "Setting VPE heading mode: %hhu", vpe_heading_mode);
      RCLCPP_INFO(get_logger(), "Setting VPE filtering mode: %hhu", vpe_filtering_mode);
      RCLCPP_INFO(get_logger(), "Setting VPE tuning mode: %hhu", vpe_tuning_mode);
      VnEnsure(vn100_setVpeControl(
        &imu_,
        vpe_enable,
        vpe_heading_mode,
        vpe_filtering_mode,
        vpe_tuning_mode,
        true));
  }

  if (vpe_enable_) {
      RCLCPP_INFO(get_logger(),
         "Setting VPE MagnetometerBasicTuning BaseTuning (%f, %f, %f)",
         vpe_mag_base_tuning_.c0,
         vpe_mag_base_tuning_.c1,
         vpe_mag_base_tuning_.c2);
      RCLCPP_INFO(get_logger(),
        "Setting VPE MagnetometerBasicTuning AdaptiveTuning (%f, %f, %f)",
        vpe_mag_adaptive_tuning_.c0,
        vpe_mag_adaptive_tuning_.c1,
        vpe_mag_adaptive_tuning_.c2);
      RCLCPP_INFO(get_logger(),
        "Setting VPE MagnetometerBasicTuning AdaptiveFiltering (%f, %f, %f)",
        vpe_mag_adaptive_filtering_.c0,
        vpe_mag_adaptive_filtering_.c1,
        vpe_mag_adaptive_filtering_.c2);
      VnEnsure(vn100_setVpeMagnetometerBasicTuning(
        &imu_,
        vpe_mag_base_tuning_,
        vpe_mag_adaptive_tuning_,
        vpe_mag_adaptive_filtering_,
        true));

      RCLCPP_INFO(get_logger(),
        "Setting VPE AccelerometerBasicTuning BaseTuning (%f, %f, %f)",
        vpe_accel_base_tuning_.c0,
        vpe_accel_base_tuning_.c1,
        vpe_accel_base_tuning_.c2);
      RCLCPP_INFO(get_logger(),
        "Setting VPE AccelerometerBasicTuning AdaptiveTuning (%f, %f, %f)",
        vpe_accel_adaptive_tuning_.c0,
        vpe_accel_adaptive_tuning_.c1,
        vpe_accel_adaptive_tuning_.c2);
      RCLCPP_INFO(get_logger(),
        "Setting VPE AccelerometerBasicTuning AdaptiveFiltering (%f, %f, %f)",
        vpe_accel_adaptive_filtering_.c0,
        vpe_accel_adaptive_filtering_.c1,
        vpe_accel_adaptive_filtering_.c2);
      VnEnsure(vn100_setVpeAccelerometerBasicTuning(
        &imu_,
        vpe_accel_base_tuning_,
        vpe_accel_adaptive_tuning_,
        vpe_accel_adaptive_filtering_,
        true));
  }

  CreatePublishers();

  auto hardware_id = std::string("vn100-") + std::string(model_number_buffer) +
                     std::string(serial_number_buffer);
}

void ImuVn100::Stream(bool async) {
  // Pause the device first
  VnEnsure(vn100_pauseAsyncOutputs(&imu_, true));

  if (async) {
    VnEnsure(vn100_setAsynchronousDataOutputType(&imu_, VNASYNC_OFF, true));

    if (binary_output_) {
      // Set the binary output data type and data rate
      uint16_t grp1 = BG1_QTN | BG1_SYNC_IN_CNT | BG1_TIME_STARTUP;
      std::list<std::string> sgrp1 = {"BG1_QTN", "BG1_SYNC_IN_CNT", "BG1_TIME_STARTUP"};
      if (enable_rpy_) {
        grp1 |= BG1_YPR;
        sgrp1.push_back("BG1_YPR");
      }
      uint16_t grp3 = BG3_NONE;
      std::list<std::string> sgrp3;
      uint16_t grp5 = BG5_NONE;
      std::list<std::string> sgrp5;
      if (imu_compensated_) {
        grp1 |=  BG1_ACCEL | BG1_ANGULAR_RATE;
        sgrp1.push_back("BG1_ACCEL");
        sgrp1.push_back("BG1_ANGULAR_RATE");
        if (enable_mag_) {
          grp3 |= BG3_MAG;
          sgrp3.push_back("BG3_MAG");
        }
      } else {
        grp1 |=  BG1_IMU;
        sgrp1.push_back("BG1_IMU");
        if (enable_mag_) {
          grp3 |= BG3_UNCOMP_MAG;
          sgrp3.push_back("BG3_UNCOMP_MAG");
        }
      }
      if (enable_temp_) {
          grp3 |= BG3_TEMP;
          sgrp3.push_back("BG3_TEMP");
      }
      if (enable_pres_) {
          grp3 |= BG3_PRES;
          sgrp3.push_back("BG3_PRES");
      }
      if (!sgrp1.empty()) {
        std::stringstream ss;
        std::copy(
          sgrp1.begin(),
          sgrp1.end(),
          std::ostream_iterator<std::string>(ss, "|")
        );
        std::string s = ss.str();
        s.pop_back();
        RCLCPP_INFO(get_logger(), "Streaming #1: %s", s.c_str());
      }
      if (!sgrp3.empty()) {
        std::stringstream ss;
        std::copy(
          sgrp3.begin(),
          sgrp3.end(),
          std::ostream_iterator<std::string>(ss, "|")
        );
        std::string s = ss.str();
        s.pop_back();
        RCLCPP_INFO(get_logger(), "Streaming #3: %s", s.c_str());
      }
      if (!sgrp5.empty()) {
        std::stringstream ss;
        std::copy(
          sgrp5.begin(),
          sgrp5.end(),
          std::ostream_iterator<std::string>(ss, "|")
        );
        std::string s = ss.str();
        s.pop_back();
        RCLCPP_INFO(get_logger(), "Streaming #5: %s", s.c_str());
      }
      VnEnsure(vn100_setBinaryOutput1Configuration(
        &imu_,
        binary_async_mode_,
        kBaseImuRate / imu_rate_,
        grp1, grp3, grp5,
        true
      ));
    } else {
      // Set the ASCII output data type and data rate
      // RCLCPP_INFO(get_logger(), "Configure the output data type and frequency (id: 6 & 7)");
      VnEnsure(vn100_setAsynchronousDataOutputType(&imu_, VNASYNC_VNIMU, true));
    }

    // Add a callback function for new data event
    VnEnsure(vn100_registerAsyncDataReceivedListener(&imu_, &AsyncListener));

    RCLCPP_INFO(get_logger(), "Setting IMU rate to %d", imu_rate_);
    VnEnsure(vn100_setAsynchronousDataOutputFrequency(&imu_, imu_rate_, true));
  } else {
    // Mute the stream
    RCLCPP_DEBUG(get_logger(), "Mute the device");
    VnEnsure(vn100_setAsynchronousDataOutputType(&imu_, VNASYNC_OFF, true));
    // Remove the callback function for new data event
    VnEnsure(vn100_unregisterAsyncDataReceivedListener(&imu_, &AsyncListener));
  }

  // Resume the device
  VnEnsure(vn100_resumeAsyncOutputs(&imu_, true));
}

void ImuVn100::Resume(bool need_reply) {
  vn100_resumeAsyncOutputs(&imu_, need_reply);
}

void ImuVn100::Idle(bool need_reply) {
  vn100_pauseAsyncOutputs(&imu_, need_reply);
}

void ImuVn100::Disconnect() {
  // Reset the device so that the baud rate goes back to the default 115200 and
  // subsequent runs of this node will be able to connect.
  vn100_reset(&imu_);
  vn100_disconnect(&imu_);
}

void ImuVn100::PublishData(const VnDeviceCompositeData& data) {
  // When publishing the message on the ROS network, we want to publish the
  // time that the data was acquired in seconds/nanoseconds since the Unix
  // epoch.  The data we have to work with is the time that the callback
  // happened (on the local processor, in Unix epoch seconds/nanoseconds), and
  // the timestamp that the IMU gives us on the callback (from the processor on
  // the IMU, in nanoseconds since some arbitrary starting point).
  //
  // At a first approximation, we can apply the timestamp from the device to
  // Unix epoch seconds/nanoseconds by taking a common starting point on the IMU
  // and the local processor, then applying the delta between this IMU timestamp
  // and the "zero" IMU timestamp to the local processor starting point.
  //
  // There are several complications with the simple scheme above.  The first
  // is finding a proper "zero" point where the IMU timestamp and the local
  // timestamp line up.  Due to potential delays in servicing this process,
  // along with USB delays, the delta timestamp from the IMU and the time when
  // this callback gets called can be wildly different.  Since we want the
  // initial zero for both the IMU and the local time to be in the same time
  // "window", we throw away data at the beginning until we see that the delta
  // callback and delta timestamp are within reasonable bounds of each other.
  //
  // The second complication is that the time on the IMU drifts away from the
  // time on the local processor.  Taking the "zero" time once at the
  // beginning isn't sufficient, and we have to periodically re-synchronize
  // the times given the constraints above.  Because we still have the
  // arbitrary delays present as described above, it can take us several
  // callbacks to successfully synchronize.  We continue publishing data using
  // the old "zero" time until we successfully resynchronize, at which point
  // we switch to the new zero point.

  rclcpp::Time now = this->now();

  // At the beginning of time, need to initialize last_cb_time for later use;
  // last_cb_time is used to figure out the time between callbacks
  if (last_cb_time_.nanoseconds() == 0) {
    last_cb_time_ = now;
    // We need to initialize the ros_time_zero since rclcpp::Duration
    // below won't let us subtract an essentially uninitialized
    // rclcpp::Time from another one.  However, we'll still do an initial
    // synchronization since the default value of synchronize_timestamp
    // is true.
    ros_time_zero_ = now;
    return;
  }

  rclcpp::Duration time_since_last_cb = now - last_cb_time_;
  uint64_t this_ts_ns = data.timeStartup;

  if (synchronize_timestamps_) {
    // The only time it's safe to sync time between IMU and ROS Node is when
    // the data that came in is within the data interval that data is
    // expected. It's possible for data to come late because of USB issues
    // or swapping, etc and we don't want to sync with data that was
    // actually published before this time interval, so we wait until we get
    // data that is within the data interval +/- an epsilon since we will
    // have taken some time to process and/or a short delay (maybe USB
    // comms) may have happened
    if (time_since_last_cb.nanoseconds() >=
          (data_interval_ns_ - cb_delta_epsilon_ns_) &&
        time_since_last_cb.nanoseconds() <=
          (data_interval_ns_ + cb_delta_epsilon_ns_)) {
      ros_time_zero_ = now;
      data_time_zero_ns_ = this_ts_ns;
      synchronize_timestamps_ = false;
      can_publish_ = true;
    } else {
      RCLCPP_DEBUG(get_logger(),
                   "Data not within acceptable window for synchronization: "
                   "expected between %ld and %ld, saw %ld",
                   data_interval_ns_ - cb_delta_epsilon_ns_,
                   data_interval_ns_ + cb_delta_epsilon_ns_,
                   time_since_last_cb.nanoseconds());
    }
  }

  if (can_publish_) {  // Cannot publish data until IMU/ROS timestamps have been
                       // synchronized at least once

    uint64_t imu_diff_in_ns = this_ts_ns - data_time_zero_ns_;
    uint64_t time_in_ns = ros_time_zero_.nanoseconds() + imu_diff_in_ns;

    if (time_in_ns < last_ros_stamp_ns_) {
      RCLCPP_WARN(get_logger(), "Time went backwards (%lu < %lu)!",
                  time_in_ns, last_ros_stamp_ns_);
    }

    last_ros_stamp_ns_ = time_in_ns;

    rclcpp::Time ros_time = rclcpp::Time(time_in_ns);

    if (binary_output_) {
      auto imu_msg = std::make_unique<sensor_msgs::msg::Imu>();
      imu_msg->header.stamp = ros_time;
      imu_msg->header.frame_id = frame_id_;

      if (imu_compensated_) {
        RosVector3FromVnVector3(imu_msg->linear_acceleration, data.acceleration);
        RosVector3FromVnVector3(imu_msg->angular_velocity, data.angularRate);
      } else {
        // NOTE: The IMU angular velocity and linear acceleration outputs are
        // swapped. And also why are they different?
        RosVector3FromVnVector3(imu_msg->angular_velocity,
                                data.accelerationUncompensated);
        RosVector3FromVnVector3(imu_msg->linear_acceleration,
                                data.angularRateUncompensated);
      }
      RosQuaternionFromVnQuaternion(imu_msg->orientation, data.quaternion);

      imu_msg->angular_velocity_covariance[0] = angular_velocity_covariance_;
      imu_msg->angular_velocity_covariance[4] = angular_velocity_covariance_;
      imu_msg->angular_velocity_covariance[8] = angular_velocity_covariance_;

      imu_msg->linear_acceleration_covariance[0] = linear_acceleration_covariance_;
      imu_msg->linear_acceleration_covariance[4] = linear_acceleration_covariance_;
      imu_msg->linear_acceleration_covariance[8] = linear_acceleration_covariance_;

      pd_imu_->publish(std::move(imu_msg));
    }

    auto imu_raw_msg = std::make_unique<sensor_msgs::msg::Imu>();
    imu_raw_msg->header.stamp = ros_time;
    imu_raw_msg->header.frame_id = frame_id_;

    if (imu_compensated_) {
      RosVector3FromVnVector3(imu_raw_msg->linear_acceleration, data.acceleration);
      RosVector3FromVnVector3(imu_raw_msg->angular_velocity, data.angularRate);
    } else {
      // NOTE: The IMU angular velocity and linear acceleration outputs are
      // swapped. And also why are they different?
      RosVector3FromVnVector3(imu_raw_msg->angular_velocity,
                              data.accelerationUncompensated);
      RosVector3FromVnVector3(imu_raw_msg->linear_acceleration,
                              data.angularRateUncompensated);
    }
    imu_raw_msg->angular_velocity_covariance[0] = angular_velocity_covariance_;
    imu_raw_msg->angular_velocity_covariance[4] = angular_velocity_covariance_;
    imu_raw_msg->angular_velocity_covariance[8] = angular_velocity_covariance_;

    imu_raw_msg->linear_acceleration_covariance[0] = linear_acceleration_covariance_;
    imu_raw_msg->linear_acceleration_covariance[4] = linear_acceleration_covariance_;
    imu_raw_msg->linear_acceleration_covariance[8] = linear_acceleration_covariance_;

    pd_imu_raw_->publish(std::move(imu_raw_msg));

    if (enable_rpy_) {
      auto rpy_msg = std::make_unique<geometry_msgs::msg::Vector3Stamped>();
      rpy_msg->header.stamp = ros_time;
      rpy_msg->header.frame_id = frame_id_;
      rpy_msg->vector.z = data.ypr.yaw * M_PI/180.0;
      rpy_msg->vector.y = data.ypr.pitch * M_PI/180.0;
      rpy_msg->vector.x = data.ypr.roll * M_PI/180.0;
      pd_rpy_->publish(std::move(rpy_msg));
    }

    if (enable_mag_) {
      auto mag_msg = std::make_unique<sensor_msgs::msg::MagneticField>();
      mag_msg->header.stamp = ros_time;
      mag_msg->header.frame_id = frame_id_;
      if (imu_compensated_) {
        RosVector3FromVnVector3(mag_msg->magnetic_field, data.magnetic);
      } else {
        RosVector3FromVnVector3(mag_msg->magnetic_field, data.magneticUncompensated);
      }

      // The device reports in Gauss but REP 145 specifies that we report in Tesla.
      mag_msg->magnetic_field.x /= 10000.0;
      mag_msg->magnetic_field.y /= 10000.0;
      mag_msg->magnetic_field.z /= 10000.0;

      mag_msg->magnetic_field_covariance[0] = magnetic_field_covariance_;
      mag_msg->magnetic_field_covariance[4] = magnetic_field_covariance_;
      mag_msg->magnetic_field_covariance[8] = magnetic_field_covariance_;

      pd_mag_->publish(std::move(mag_msg));
    }

    if (enable_pres_) {
      auto pres_msg = std::make_unique<sensor_msgs::msg::FluidPressure>();
      pres_msg->header.stamp = ros_time;
      pres_msg->header.frame_id = frame_id_;
      pres_msg->fluid_pressure = data.pressure;
      pd_pres_->publish(std::move(pres_msg));
    }

    if (enable_temp_) {
      auto temp_msg = std::make_unique<sensor_msgs::msg::Temperature>();
      temp_msg->header.stamp = ros_time;
      temp_msg->header.frame_id = frame_id_;
      temp_msg->temperature = data.temperature;
      pd_temp_->publish(std::move(temp_msg));
    }

    sync_info_.Update(data.syncInCnt, ros_time);
  }

  // Determine if we need to resynchronize - time between IMU and ROS Node can
  // drift, periodically resync to deal with this issue
  rclcpp::Duration diff = now - ros_time_zero_;
  if (time_resync_interval_ns_ > 0 &&
      diff.nanoseconds() >= time_resync_interval_ns_) {
    synchronize_timestamps_ = true;
  }

  last_cb_time_ = now;
}

void ImuVn100::VnEnsure(const VnErrorCode& error_code) {
  if (error_code == VNERR_NO_ERROR) return;

  switch (error_code) {
    case VNERR_UNKNOWN_ERROR:
      throw std::runtime_error("VN: Unknown error");
    case VNERR_NOT_IMPLEMENTED:
      throw std::runtime_error("VN: Not implemented");
    case VNERR_TIMEOUT:
      throw std::runtime_error("VN: Operation timed out");
    case VNERR_SENSOR_INVALID_PARAMETER:
      throw std::runtime_error("VN: Sensor invalid paramter");
    case VNERR_INVALID_VALUE:
      throw std::runtime_error("VN: Invalid value");
    case VNERR_FILE_NOT_FOUND:
      throw std::runtime_error("VN: File not found");
    case VNERR_NOT_CONNECTED:
      throw std::runtime_error("VN: not connected");
    case VNERR_PERMISSION_DENIED:
      throw std::runtime_error("VN: Permission denied");
    default:
      throw std::runtime_error("VN: Unhandled error type");
  }
}

void RosVector3FromVnVector3(geometry_msgs::msg::Vector3& ros_vec3,
                             const VnVector3& vn_vec3) {
  ros_vec3.x = vn_vec3.c0;
  ros_vec3.y = vn_vec3.c1;
  ros_vec3.z = vn_vec3.c2;
}

void RosQuaternionFromVnQuaternion(geometry_msgs::msg::Quaternion& ros_quat,
                                   const VnQuaternion& vn_quat) {
  ros_quat.x = vn_quat.x;
  ros_quat.y = vn_quat.y;
  ros_quat.z = vn_quat.z;
  ros_quat.w = vn_quat.w;
}

}  //  namespace imu_vn_100

RCLCPP_COMPONENTS_REGISTER_NODE(imu_vn_100::ImuVn100)
