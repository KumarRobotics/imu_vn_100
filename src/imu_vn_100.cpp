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

#include <imu_vn_100/imu_vn_100.h>
#include <geometry_msgs/Vector3Stamped.h>

namespace imu_vn_100 {

// LESS HACK IS STILL HACK
ImuVn100* imu_vn_100_ptr;

using geometry_msgs::Vector3Stamped;

using sensor_msgs::Imu;
using sensor_msgs::MagneticField;
using sensor_msgs::FluidPressure;
using sensor_msgs::Temperature;

void RosVector3FromVnVector3(geometry_msgs::Vector3& ros_vec3,
                             const VnVector3& vn_vec3);
void RosQuaternionFromVnQuaternion(geometry_msgs::Quaternion& ros_quat,
                                   const VnQuaternion& vn_quat);

void AsyncListener(void* sender, VnDeviceCompositeData* data) {
  imu_vn_100_ptr->PublishData(*data);
}

constexpr int ImuVn100::kBaseImuRate;
constexpr int ImuVn100::kDefaultImuRate;
constexpr int ImuVn100::kDefaultSyncOutRate;

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
    if (ImuVn100::kBaseImuRate % rate != 0) {
      rate = ImuVn100::kBaseImuRate / (ImuVn100::kBaseImuRate / rate);
      ROS_INFO("Set SYNC_OUT_RATE to %d", rate);
    }
    skip_count =
        (std::floor(ImuVn100::kBaseImuRate / static_cast<double>(rate) +
                    0.5f)) -
        1;

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

  pnh_.param("enable_mag", enable_mag_, true);
  pnh_.param("enable_pres", enable_pres_, true);
  pnh_.param("enable_temp", enable_temp_, true);
  pnh_.param("enable_rpy", enable_rpy_, false);

  pnh_.param("sync_rate", sync_info_.rate, kDefaultSyncOutRate);
  pnh_.param("sync_pulse_width_us", sync_info_.pulse_width_us, 1000);

  pnh_.param("binary_output", binary_output_, true);
  pnh_.param("binary_async_mode", binary_async_mode_,
             BINARY_ASYNC_MODE_SERIAL_2);
  pnh_.param("imu_compensated", imu_compensated_, false);	
  pnh_.param("vpe/enable", vpe_enable_, true);
  pnh_.param("vpe/heading_mode", vpe_heading_mode_, 1);
  pnh_.param("vpe/filtering_mode", vpe_filtering_mode_, 1);
  pnh_.param("vpe/tuning_mode", vpe_tuning_mode_, 1);

  pnh_.param("vpe/mag_tuning/base_tuning/x", vpe_mag_base_tuning_.c0, 4.0);
  pnh_.param("vpe/mag_tuning/base_tuning/y", vpe_mag_base_tuning_.c1, 4.0);
  pnh_.param("vpe/mag_tuning/base_tuning/z", vpe_mag_base_tuning_.c2, 4.0);
  pnh_.param("vpe/mag_tuning/adaptive_tuning/x", vpe_mag_adaptive_tuning_.c0, 5.0);
  pnh_.param("vpe/mag_tuning/adaptive_tuning/y", vpe_mag_adaptive_tuning_.c1, 5.0);
  pnh_.param("vpe/mag_tuning/adaptive_tuning/z", vpe_mag_adaptive_tuning_.c2, 5.0);
  pnh_.param("vpe/mag_tuning/adaptive_filtering/x", vpe_mag_adaptive_filtering_.c0, 5.5);
  pnh_.param("vpe/mag_tuning/adaptive_filtering/y", vpe_mag_adaptive_filtering_.c1, 5.5);
  pnh_.param("vpe/mag_tuning/adaptive_filtering/z", vpe_mag_adaptive_filtering_.c2, 5.5);

  pnh_.param("vpe/accel_tuning/base_tuning/x", vpe_accel_base_tuning_.c0, 5.0);
  pnh_.param("vpe/accel_tuning/base_tuning/y", vpe_accel_base_tuning_.c1, 5.0);
  pnh_.param("vpe/accel_tuning/base_tuning/z", vpe_accel_base_tuning_.c2, 5.0);
  pnh_.param("vpe/accel_tuning/adaptive_tuning/x", vpe_accel_adaptive_tuning_.c0, 3.0);
  pnh_.param("vpe/accel_tuning/adaptive_tuning/y", vpe_accel_adaptive_tuning_.c1, 3.0);
  pnh_.param("vpe/accel_tuning/adaptive_tuning/z", vpe_accel_adaptive_tuning_.c2, 3.0);
  pnh_.param("vpe/accel_tuning/adaptive_filtering/x", vpe_accel_adaptive_filtering_.c0, 4.0);
  pnh_.param("vpe/accel_tuning/adaptive_filtering/y", vpe_accel_adaptive_filtering_.c1, 4.0);
  pnh_.param("vpe/accel_tuning/adaptive_filtering/z", vpe_accel_adaptive_filtering_.c2, 4.0);

  FixImuRate();
  sync_info_.FixSyncRate();
}

void ImuVn100::CreateDiagnosedPublishers() {
  imu_rate_double_ = imu_rate_;
  pd_imu_.Create<Imu>(pnh_, "imu", updater_, imu_rate_double_);
  if (enable_mag_) {
    pd_mag_.Create<MagneticField>(pnh_, "magnetic_field", updater_,
                                  imu_rate_double_);
  }
  if (enable_pres_) {
    pd_pres_.Create<FluidPressure>(pnh_, "fluid_pressure", updater_,
                                   imu_rate_double_);
  }
  if (enable_temp_) {
    pd_temp_.Create<Temperature>(pnh_, "temperature", updater_,
                                 imu_rate_double_);
  }
  if (enable_rpy_) {
      pd_rpy_.Create<Vector3Stamped>(pnh_, "rpy", updater_, imu_rate_double_);
  }
  if(sync_info_.SyncEnabled())  {
      pd_sync_trigger.Create<imu_vn_100::sync_trigger>(pnh_,"sync_trigger",updater_,imu_rate_double_);
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
  ROS_INFO("Default VPE enable: %hhu", vpe_enable);
  ROS_INFO("Default VPE heading mode: %hhu", vpe_heading_mode);
  ROS_INFO("Default VPE filtering mode: %hhu", vpe_filtering_mode);
  ROS_INFO("Default VPE tuning mode: %hhu", vpe_tuning_mode);
  if (vpe_enable != vpe_enable_ ||
      vpe_heading_mode != vpe_heading_mode_ ||
      vpe_filtering_mode != vpe_filtering_mode_ ||
      vpe_tuning_mode != vpe_tuning_mode_) {
      vpe_enable = vpe_enable_;
      vpe_heading_mode = vpe_heading_mode_;
      vpe_filtering_mode = vpe_filtering_mode_;
      vpe_tuning_mode = vpe_tuning_mode_;
      ROS_INFO("Setting VPE enable: %hhu", vpe_enable);
      ROS_INFO("Setting VPE heading mode: %hhu", vpe_heading_mode);
      ROS_INFO("Setting VPE filtering mode: %hhu", vpe_filtering_mode);
      ROS_INFO("Setting VPE tuning mode: %hhu", vpe_tuning_mode);
      VnEnsure(vn100_setVpeControl(
        &imu_,
        vpe_enable,
        vpe_heading_mode,
        vpe_filtering_mode,
        vpe_tuning_mode,
        true));
  }

  if (vpe_enable_) {
      ROS_INFO(
         "Setting VPE MagnetometerBasicTuning BaseTuning (%f, %f, %f)",
         vpe_mag_base_tuning_.c0,
         vpe_mag_base_tuning_.c1,
         vpe_mag_base_tuning_.c2);
      ROS_INFO(
        "Setting VPE MagnetometerBasicTuning AdaptiveTuning (%f, %f, %f)",
        vpe_mag_adaptive_tuning_.c0,
        vpe_mag_adaptive_tuning_.c1,
        vpe_mag_adaptive_tuning_.c2);
      ROS_INFO(
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

      ROS_INFO(
       "Setting VPE AccelerometerBasicTuning BaseTuning (%f, %f, %f)",
       vpe_accel_base_tuning_.c0,
       vpe_accel_base_tuning_.c1,
       vpe_accel_base_tuning_.c2);
    ROS_INFO(
      "Setting VPE AccelerometerBasicTuning AdaptiveTuning (%f, %f, %f)",
      vpe_accel_adaptive_tuning_.c0,
      vpe_accel_adaptive_tuning_.c1,
      vpe_accel_adaptive_tuning_.c2);
    ROS_INFO(
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

  CreateDiagnosedPublishers();

  auto hardware_id = std::string("vn100-") + std::string(model_number_buffer) +
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
      uint16_t grp1 = BG1_QTN | BG1_SYNC_IN_CNT;
      std::list<std::string> sgrp1 = {"BG1_QTN", "BG1_SYNC_IN_CNT"};
      if (enable_rpy_) {
        grp1 |= BG1_YPR;
        sgrp1.push_back("BG1_YPR");
      }
      // Set the binary output data type for group 2
      uint16_t grp2 = BG2_NONE;
      if (sync_info_.SyncEnabled())
		{
			grp2 |= BG2_SYNC_OUT_CNT;
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
        ROS_INFO("Streaming #1: %s", s.c_str());
      }
      if(sync_info_.SyncEnabled())
      {
        ROS_INFO("Streaming #2:  BG2_SYNC_OUT_CNT");
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
        ROS_INFO("Streaming #3: %s", s.c_str());
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
        ROS_INFO("Streaming #5: %s", s.c_str());
      }
      VnEnsure(vn100_setBinaryOutput1Configuration_withgrp2(
        &imu_,
        binary_async_mode_,
        kBaseImuRate / imu_rate_,
        grp1, grp2, grp3, grp5,
        true
      ));
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

void ImuVn100::Resume(bool need_reply) {
  vn100_resumeAsyncOutputs(&imu_, need_reply);
}

void ImuVn100::Idle(bool need_reply) {
  vn100_pauseAsyncOutputs(&imu_, need_reply);
}

void ImuVn100::Disconnect() {
  // TODO: why reset the device?
  vn100_reset(&imu_);
  vn100_disconnect(&imu_);
}

void ImuVn100::PublishData(const VnDeviceCompositeData& data) {
  sensor_msgs::Imu imu_msg;
  imu_msg.header.stamp = ros::Time::now();
  imu_msg.header.frame_id = frame_id_;
  if (imu_compensated_) {
    RosVector3FromVnVector3(imu_msg.linear_acceleration, data.acceleration);
    RosVector3FromVnVector3(imu_msg.angular_velocity, data.angularRate);
  } else {
    // NOTE: The IMU angular velocity and linear acceleration outputs are
    // swapped. And also why are they different?
    RosVector3FromVnVector3(imu_msg.angular_velocity,
                            data.accelerationUncompensated);
    RosVector3FromVnVector3(imu_msg.linear_acceleration,
                            data.angularRateUncompensated);
  }
  if (binary_output_) {
    RosQuaternionFromVnQuaternion(imu_msg.orientation, data.quaternion);
  }
  pd_imu_.Publish(imu_msg);

  if (enable_rpy_) {
    Vector3Stamped rpy_msg;
    rpy_msg.header= imu_msg.header;
    rpy_msg.vector.z = data.ypr.yaw * M_PI/180.0;
    rpy_msg.vector.y = data.ypr.pitch * M_PI/180.0;
    rpy_msg.vector.x = data.ypr.roll * M_PI/180.0;
    pd_rpy_.Publish(rpy_msg);
  }

  if (enable_mag_) {
    sensor_msgs::MagneticField mag_msg;
    mag_msg.header = imu_msg.header;
    RosVector3FromVnVector3(mag_msg.magnetic_field, data.magnetic);
    pd_mag_.Publish(mag_msg);
  }

  if (enable_pres_) {
    sensor_msgs::FluidPressure pres_msg;
    pres_msg.header = imu_msg.header;
    pres_msg.fluid_pressure = data.pressure;
    pd_pres_.Publish(pres_msg);
  }

  if (enable_temp_) {
    sensor_msgs::Temperature temp_msg;
    temp_msg.header = imu_msg.header;
    temp_msg.temperature = data.temperature;
    pd_temp_.Publish(temp_msg);
  }
  sync_info_.Update(data.syncInCnt, imu_msg.header.stamp);
  if (sync_info_.SyncEnabled()){
    imu_vn_100::sync_trigger sync_trigger_msg;
    syncOutCnt = data.syncOutCnt;
        if (syncOutCnt != syncOutCnt_old)
        {
            sync_trigger_msg.header = imu_msg.header;
            sync_trigger_msg.data = syncOutCnt;
            pd_sync_trigger.Publish(sync_trigger_msg);
        }
    syncOutCnt_old = syncOutCnt;
  }

  sync_info_.Update(data.syncInCnt, imu_msg.header.stamp);
  updater_.update();
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

void RosVector3FromVnVector3(geometry_msgs::Vector3& ros_vec3,
                             const VnVector3& vn_vec3) {
  ros_vec3.x = vn_vec3.c0;
  ros_vec3.y = vn_vec3.c1;
  ros_vec3.z = vn_vec3.c2;
}

void RosQuaternionFromVnQuaternion(geometry_msgs::Quaternion& ros_quat,
                                   const VnQuaternion& vn_quat) {
  ros_quat.x = vn_quat.x;
  ros_quat.y = vn_quat.y;
  ros_quat.z = vn_quat.z;
  ros_quat.w = vn_quat.w;
}

}  //  namespace imu_vn_100
