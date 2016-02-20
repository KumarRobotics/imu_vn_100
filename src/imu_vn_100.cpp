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

namespace imu_vn_100 {

// TODO: This is hacky!
//    The official VN100 driver requires a
//    plain C callback function, which cannot
//    be a class member function. So, some of
//    the class members are made transparent
//    here.
std::string* frame_id_ptr;
bool* enable_mag_ptr;
bool* enable_pres_ptr;
bool* enable_temp_ptr;
int* enable_sync_out_ptr;
bool* use_binary_output_ptr;

SyncInfo* sync_info_ptr;

ros::Publisher* pub_imu_ptr;
ros::Publisher* pub_mag_ptr;
ros::Publisher* pub_pres_ptr;
ros::Publisher* pub_temp_ptr;

boost::shared_ptr<diagnostic_updater::Updater> updater_ptr;
boost::shared_ptr<diagnostic_updater::TopicDiagnostic> imu_diag_ptr;
boost::shared_ptr<diagnostic_updater::TopicDiagnostic> mag_diag_ptr;
boost::shared_ptr<diagnostic_updater::TopicDiagnostic> pres_diag_ptr;
boost::shared_ptr<diagnostic_updater::TopicDiagnostic> temp_diag_ptr;

// Callback function for new data event
// in the continous stream mode
void asyncDataListener(void* sender, VnDeviceCompositeData* data) {
  sensor_msgs::Imu imu;
  imu.header.stamp = ros::Time::now();
  imu.header.frame_id = *frame_id_ptr;

  // TODO: get the covariance for the estimated attitude
  // imu.orientation.x = data->quaternion.x;
  // imu.orientation.y = data->quaternion.y;
  // imu.orientation.z = data->quaternion.z;
  // imu.orientation.w = data->quaternion.w;

  if (*use_binary_output_ptr) {
    imu.orientation.x = data->quaternion.x;
    imu.orientation.y = data->quaternion.y;
    imu.orientation.z = data->quaternion.z;
    imu.orientation.w = data->quaternion.w;
    // NOTE: The IMU angular velocity and linear acceleration outputs are
    // swapped
    imu.angular_velocity.x = data->accelerationUncompensated.c0;
    imu.angular_velocity.y = data->accelerationUncompensated.c1;
    imu.angular_velocity.z = data->accelerationUncompensated.c2;
    imu.linear_acceleration.x = data->angularRateUncompensated.c0;
    imu.linear_acceleration.y = data->angularRateUncompensated.c1;
    imu.linear_acceleration.z = data->angularRateUncompensated.c2;
  } else {
    imu.linear_acceleration.x = data->acceleration.c0;
    imu.linear_acceleration.y = data->acceleration.c1;
    imu.linear_acceleration.z = data->acceleration.c2;
    imu.angular_velocity.x = data->angularRate.c0;
    imu.angular_velocity.y = data->angularRate.c1;
    imu.angular_velocity.z = data->angularRate.c2;
  }
  pub_imu_ptr->publish(imu);
  imu_diag_ptr->tick(imu.header.stamp);

  if (*enable_mag_ptr) {
    sensor_msgs::MagneticField field;
    field.header.stamp = imu.header.stamp;
    field.header.frame_id = *frame_id_ptr;
    field.magnetic_field.x = data->magnetic.c0;
    field.magnetic_field.y = data->magnetic.c1;
    field.magnetic_field.z = data->magnetic.c2;
    pub_mag_ptr->publish(field);
    mag_diag_ptr->tick(imu.header.stamp);
  }

  if (*enable_pres_ptr) {
    sensor_msgs::FluidPressure pressure;
    pressure.header.stamp = imu.header.stamp;
    pressure.header.frame_id = *frame_id_ptr;
    pressure.fluid_pressure = data->pressure;
    pub_pres_ptr->publish(pressure);
    pres_diag_ptr->tick(imu.header.stamp);
  }

  if (*enable_temp_ptr) {
    sensor_msgs::Temperature temperature;
    temperature.header.stamp = imu.header.stamp;
    temperature.header.frame_id = *frame_id_ptr;
    temperature.temperature = data->temperature;
    pub_temp_ptr->publish(temperature);
    temp_diag_ptr->tick(imu.header.stamp);
  }

  if (*enable_sync_out_ptr > 0) {
    if (sync_info_ptr->sync_count() == -1) {
      // Initialize the count if never set
      sync_info_ptr->setSyncCount(data->syncInCnt);
    } else {
      // Record the time when the sync counter increases
      if (sync_info_ptr->sync_count() != data->syncInCnt) {
        sync_info_ptr->setSyncTime(imu.header.stamp);
        sync_info_ptr->setSyncCount(data->syncInCnt);
      }
    }
  }
  // ROS_INFO("Sync Count:    %lld", sync_info_ptr->getSyncCount());
  // ROS_INFO("Sync Time :    %f", sync_info_ptr->getSyncTime().toSec());

  // Update diagnostic info
  updater_ptr->update();

  return;
}

constexpr int ImuVn100::kBaseImuRate;
constexpr int ImuVn100::kDefaultImuRate;
constexpr int ImuVn100::kDefaultSyncOutRate;

ImuVn100::ImuVn100(const ros::NodeHandle& pnh)
    : pnh_(pnh),
      port_(std::string("/dev/ttyUSB0")),
      baudrate_(921600),
      frame_id_(std::string("imu")),
      sync_out_pulse_width_us(500000) {
  Initialize();
}

ImuVn100::~ImuVn100() { Disconnect(); }

void ImuVn100::FixImuRate() {
  if (imu_rate_ <= 0) {
    ROS_WARN("Imu rate %d is < 0. Set to %d", imu_rate_, kDefaultImuRate);
    imu_rate_ = kDefaultImuRate;
  }

  if (kBaseImuRate % imu_rate_ != 0) {
    int imu_rate_old = imu_rate_;
    // TODO: THIS DOENS'T DO WHAT'S INTENDED
    imu_rate_ = kBaseImuRate / (kBaseImuRate / imu_rate_old);
    ROS_WARN("Imu rate %d cannot evenly decimate base rate %d, reset to %d",
             imu_rate_old, kBaseImuRate, imu_rate_);
  }
}

void ImuVn100::FixSyncOutRate() {
  // Check the sync out rate
  if (sync_out_rate_ > 0) {
    act_sync_out_rate = sync_out_rate_;
    if (kBaseImuRate % sync_out_rate_ != 0) {
      act_sync_out_rate = 800.0 / (800 / sync_out_rate_);
      ROS_INFO("Set SYNC_OUT_RATE to %f", act_sync_out_rate);
    }
    sync_out_skip_count =
        static_cast<int>(floor(800.0 / act_sync_out_rate + 0.5f)) - 1;

    if (sync_out_pulse_width_us > 10000000) {
      ROS_INFO("Sync out pulse with is over 10ms. Reset to 0.5ms");
      sync_out_pulse_width_us = 500000;
    }
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

  pnh_.param("sync_out_rate", sync_out_rate_, kDefaultSyncOutRate);
  pnh_.param("sync_out_pulse_width", sync_out_pulse_width_us, 500000);

  pnh_.param("use_binary_output", use_binary_output_, true);

  frame_id_ptr = &frame_id_;
  enable_mag_ptr = &enable_mag_;
  enable_pres_ptr = &enable_pres_;
  enable_temp_ptr = &enable_temp_;
  enable_sync_out_ptr = &sync_out_rate_;
  use_binary_output_ptr = &use_binary_output_;
  sync_info_ptr = &sync_info;

  FixImuRate();
  FixSyncOutRate();
}

void ImuVn100::CreatePublishers() {
  // IMU data publisher
  pub_imu_ = pnh_.advertise<sensor_msgs::Imu>("imu", 1);
  pub_imu_ptr = &pub_imu_;
  // Magnetic field data publisher
  if (enable_mag_) {
    pub_mag_ = pnh_.advertise<sensor_msgs::MagneticField>("magnetic_field", 1);
    pub_mag_ptr = &pub_mag_;
  }
  // Pressure data publisher
  if (enable_pres_) {
    pub_pres_ = pnh_.advertise<sensor_msgs::FluidPressure>("pressure", 1);
    pub_pres_ptr = &pub_pres_;
  }
  // Temperature data publisher
  if (enable_temp_) {
    pub_temp_ = pnh_.advertise<sensor_msgs::Temperature>("temperature", 1);
    pub_temp_ptr = &pub_temp_;
  }
}

void ImuVn100::Resume(bool need_reply) {
  vn100_resumeAsyncOutputs(&imu_, need_reply);
}

void ImuVn100::Idle(bool need_reply) {
  vn100_pauseAsyncOutputs(&imu_, need_reply);
}

void ImuVn100::Disconnect() {
  vn100_reset(&imu_);
  vn100_disconnect(&imu_);
}

void ImuVn100::ErrorCodeParser(const VN_ERROR_CODE& error_code) {
  // We only parse a fraction of the error code here.
  // All of the error codes with VNERR_SENSOR* as prefix
  // are omitted.
  // For the detailed definition of VN_ERROR_CODE,
  // please refer to include/vn_errorCodes.h within
  // the official driver for the device
  switch (error_code) {
    case VNERR_NO_ERROR:
      break;
    case VNERR_UNKNOWN_ERROR:
      ROS_ERROR("Unknown error happened with the device");
      break;
    case VNERR_NOT_IMPLEMENTED:
      ROS_ERROR("The operation is not implemented");
      break;
    case VNERR_TIMEOUT:
      ROS_WARN("Opertation time out");
      break;
    case VNERR_SENSOR_INVALID_PARAMETER:
      ROS_WARN("Invalid parameter was provided");
      break;
    case VNERR_INVALID_VALUE:
      ROS_WARN("Invalid value was provided");
      break;
    case VNERR_FILE_NOT_FOUND:
      ROS_WARN("The file was not found");
      break;
    case VNERR_NOT_CONNECTED:
      ROS_ERROR("The device is not connected");
      break;
    case VNERR_PERMISSION_DENIED:
      ROS_ERROR("Permission denied");
      break;
    default:
      ROS_ERROR("Sensor type error happened");
      break;
  }
  return;
}

bool ImuVn100::Initialize() {
  LoadParameters();
  CreatePublishers();

  VN_ERROR_CODE error_code;

  // Connect to the device
  ROS_INFO("Connecting to device");
  error_code = vn100_connect(&imu_, port_.c_str(), 115200);
  ErrorCodeParser(error_code);
  ros::Duration(0.5).sleep();
  ROS_INFO("Connected to device at %s", port_.c_str());

  // Get the old serial baudrate
  unsigned int old_baudrate;
  error_code = vn100_getSerialBaudRate(&imu_, &old_baudrate);
  ROS_INFO("Default serial baudrate: %u", old_baudrate);

  // Set the new serial baudrate
  ROS_INFO("Set serial baudrate to %d", baudrate_);
  error_code = vn100_setSerialBaudRate(&imu_, baudrate_, true);
  ErrorCodeParser(error_code);

  // Disconnect the device
  ROS_INFO("Disconnecting the device");
  vn100_disconnect(&imu_);
  ros::Duration(0.5).sleep();

  // Reconnect to the device
  ROS_INFO("Reconnecting to device");
  error_code = vn100_connect(&imu_, port_.c_str(), baudrate_);
  ErrorCodeParser(error_code);
  ros::Duration(0.5).sleep();
  ROS_INFO("Connected to device at %s", port_.c_str());

  // Check the new baudrate
  error_code = vn100_getSerialBaudRate(&imu_, &old_baudrate);
  ROS_INFO("New serial baudrate: %u", old_baudrate);

  // Idle the device for intialization
  error_code = vn100_pauseAsyncOutputs(&imu_, true);
  ErrorCodeParser(error_code);

  // Get device info
  ROS_INFO("Fetching device info.");
  char model_number_buffer[30] = {0};
  int hardware_revision = 0;
  char serial_number_buffer[30] = {0};
  char firmware_version_buffer[30] = {0};

  error_code = vn100_getModelNumber(&imu_, model_number_buffer, 30);
  ErrorCodeParser(error_code);
  ROS_INFO("Model number: %s", model_number_buffer);
  error_code = vn100_getHardwareRevision(&imu_, &hardware_revision);
  ErrorCodeParser(error_code);
  ROS_INFO("Hardware revision: %d", hardware_revision);
  error_code = vn100_getSerialNumber(&imu_, serial_number_buffer, 30);
  ErrorCodeParser(error_code);
  ROS_INFO("Serial number: %s", serial_number_buffer);
  error_code = vn100_getFirmwareVersion(&imu_, firmware_version_buffer, 30);
  ErrorCodeParser(error_code);
  ROS_INFO("Firmware version: %s", firmware_version_buffer);

  if (sync_out_rate_ > 0) {
    // Configure the synchronization control register
    ROS_INFO("Set Synchronization Control Register (id:32).");
    error_code = vn100_setSynchronizationControl(
        &imu_, SYNCINMODE_COUNT, SYNCINEDGE_RISING, 0, SYNCOUTMODE_IMU_START,
        SYNCOUTPOLARITY_POSITIVE, sync_out_skip_count, sync_out_pulse_width_us,
        true);
    ErrorCodeParser(error_code);

    if (!use_binary_output_) {
      // Configure the communication protocal control register
      ROS_INFO("Set Communication Protocal Control Register (id:30).");
      error_code = vn100_setCommunicationProtocolControl(
          &imu_, SERIALCOUNT_SYNCOUT_COUNT, SERIALSTATUS_OFF, SPICOUNT_NONE,
          SPISTATUS_OFF, SERIALCHECKSUM_8BIT, SPICHECKSUM_8BIT, ERRORMODE_SEND,
          true);
    }
  }

  // Resume the device
  // ROS_INFO("Resume the device");
  // error_code = vn100_resumeAsyncOutputs(&imu, true);
  // errorCodeParser(error_code);

  // configure diagnostic updater
  if (!pnh_.hasParam("diagnostic_period")) {
    pnh_.setParam("diagnostic_period", 0.2);
  }

  updater.reset(new diagnostic_updater::Updater());
  std::string hw_id =
      std::string("vn100") + '-' + std::string(model_number_buffer);
  updater->setHardwareID(hw_id);
  // updater->add("diagnostic_info", this,
  //    &ImuRosBase::updateDiagnosticInfo);

  diagnostic_updater::FrequencyStatusParam freqParam(
      &imu_rate_update_, &imu_rate_update_, 0.01, 10);
  diagnostic_updater::TimeStampStatusParam timeParam(0, 0.5 / imu_rate_);
  imu_diag.reset(new diagnostic_updater::TopicDiagnostic("imu", *updater,
                                                         freqParam, timeParam));
  if (enable_mag_)
    mag_diag.reset(new diagnostic_updater::TopicDiagnostic(
        "magnetic_field", *updater, freqParam, timeParam));
  if (enable_pres_)
    pres_diag.reset(new diagnostic_updater::TopicDiagnostic(
        "pressure", *updater, freqParam, timeParam));
  if (enable_temp_)
    temp_diag.reset(new diagnostic_updater::TopicDiagnostic(
        "temperature", *updater, freqParam, timeParam));

  updater_ptr = updater;
  imu_diag_ptr = imu_diag;
  mag_diag_ptr = mag_diag;
  pres_diag_ptr = pres_diag;
  temp_diag_ptr = temp_diag;

  return true;
}

void ImuVn100::Stream(bool async) {
  VN_ERROR_CODE error_code;

  // Pause the device first
  error_code = vn100_pauseAsyncOutputs(&imu_, true);
  ErrorCodeParser(error_code);

  if (async) {
    error_code = vn100_setAsynchronousDataOutputType(&imu_, VNASYNC_OFF, true);
    ErrorCodeParser(error_code);

    if (use_binary_output_) {
      // Set the binary output data type and data rate
      error_code = vn100_setBinaryOutput1Configuration(
          &imu_, BINARY_ASYNC_MODE_SERIAL_2, static_cast<int>(800 / imu_rate_),
          BG1_QTN | BG1_IMU | BG1_MAG_PRES | BG1_SYNC_IN_CNT,
          // BG1_IMU,
          BG3_NONE, BG5_NONE, true);
      ErrorCodeParser(error_code);
      // unsigned int base_freq = 0;
      // error_code = vn100_getAsynchronousDataOutputFrequency(
      //  &imu, &base_freq);
      // errorCodeParser(error_code);
    } else {
      // Set the ASCII output data type and data rate
      // ROS_INFO("Configure the output data type and frequency (id: 6 & 7)");
      error_code =
          vn100_setAsynchronousDataOutputType(&imu_, VNASYNC_VNIMU, true);
      ErrorCodeParser(error_code);
    }

    // Add a callback function for new data event
    error_code =
        vn100_registerAsyncDataReceivedListener(&imu_, &asyncDataListener);
    ErrorCodeParser(error_code);

    ROS_INFO("Setting IMU rate to %d", imu_rate_);
    error_code =
        vn100_setAsynchronousDataOutputFrequency(&imu_, imu_rate_, true);
    ErrorCodeParser(error_code);
  } else {
    // Mute the stream
    ROS_INFO("Mute the device");
    error_code = vn100_setAsynchronousDataOutputType(&imu_, VNASYNC_OFF, true);
    ErrorCodeParser(error_code);
    // Remove the callback function for new data event
    error_code =
        vn100_unregisterAsyncDataReceivedListener(&imu_, &asyncDataListener);
    ErrorCodeParser(error_code);
  }

  // Resume the device
  error_code = vn100_resumeAsyncOutputs(&imu_, true);
  ErrorCodeParser(error_code);
}

void ImuVn100::UpdateDiagnosticInfo(
    diagnostic_updater::DiagnosticStatusWrapper& stat) {
  // TODO: add diagnostic info
}

}  // End namespace imu_vn_100
