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

#include <imu_vn_100/imu_ros_base.h>


using namespace std;
using namespace ros;

namespace imu_vn_100 {

// TODO: This is hacky!
//    The official VN100 driver requires a
//    plain C callback function, which cannot
//    be a class member function. So, some of
//    the class members are made transparent
//    here.
boost::shared_ptr<string> frame_id_ptr;
boost::shared_ptr<ros::Publisher> pub_imu_ptr;
boost::shared_ptr<ros::Publisher> pub_mag_ptr;
boost::shared_ptr<diagnostic_updater::Updater> updater_ptr;
boost::shared_ptr<diagnostic_updater::TopicDiagnostic> imu_diag_ptr;


// Callback function for new data event
// in the continous stream mode
void asyncDataListener(void* sender,
    VnDeviceCompositeData* data) {

  sensor_msgs::Imu imu;
  sensor_msgs::MagneticField field;

  imu.header.stamp = ros::Time::now();
  imu.header.frame_id = *frame_id_ptr;
  field.header.stamp = imu.header.stamp;
  field.header.frame_id = *frame_id_ptr;

  // TODO: get the covariance for the estimated attitude
  imu.orientation.x = data->quaternion.x;
  imu.orientation.y = data->quaternion.y;
  imu.orientation.z = data->quaternion.z;
  imu.orientation.w = data->quaternion.w;

  imu.linear_acceleration.x = data->angularRateUncompensated.c0;
  imu.linear_acceleration.y = data->angularRateUncompensated.c1;
  imu.linear_acceleration.z = data->angularRateUncompensated.c2;
  //imu.linear_acceleration.x = data->acceleration.c0;
  //imu.linear_acceleration.y = data->acceleration.c1;
  //imu.linear_acceleration.z = data->acceleration.c2;
  //imu.linear_acceleration.x = data->linearAccelBody.c0;
  //imu.linear_acceleration.y = data->linearAccelBody.c1;
  //imu.linear_acceleration.z = data->linearAccelBody.c2;

  imu.angular_velocity.x = data->accelerationUncompensated.c0;
  imu.angular_velocity.y = data->accelerationUncompensated.c1;
  imu.angular_velocity.z = data->accelerationUncompensated.c2;
  //imu.angular_velocity.x = data->angularRate.c0;
  //imu.angular_velocity.y = data->angularRate.c1;
  //imu.angular_velocity.z = data->angularRate.c2;

  field.magnetic_field.x = data->magnetic.c0;
  field.magnetic_field.y = data->magnetic.c1;
  field.magnetic_field.z = data->magnetic.c2;

  pub_imu_ptr->publish(imu);
  pub_mag_ptr->publish(field);

  // Update diagnostic info
  imu_diag_ptr->tick(imu.header.stamp);
  updater_ptr->update();

  //unsigned int sync_in_count = 10;
  //unsigned int sync_in_time = 10;
  //unsigned int sync_out_count = 10;
  //VN_ERROR_CODE error_code = vndevice_getSynchronizationStatus(
  //  static_cast<VnDevice*>(sender),
  //  &sync_in_count,
  //  &sync_in_time,
  //  &sync_out_count);

  //printf("sync info: %d %d %d %d\n",
  //    sync_in_count, sync_in_time, sync_out_count, error_code);

  return;
}

ImuRosBase::ImuRosBase(const NodeHandle& n):
  nh(n) {
  return;
}

bool ImuRosBase::loadParameters() {

  nh.param<string>("port", port, std::string("/dev/ttyUSB0"));
  nh.param<int>("baudrate", baudrate, 115200);
  nh.param<string>("frame_id", frame_id, std::string("imu"));
  nh.param<int>("imu_rate", imu_rate, 100);
  frame_id_ptr = boost::shared_ptr<string>(&frame_id);

  if (imu_rate < 0) {
    imu_rate = 100;
    ROS_WARN("IMU_RATE is invalid. Reset to %d", imu_rate);
  }

  if (800%imu_rate != 0) {
    imu_rate = 800 / (800/imu_rate);
    ROS_WARN("IMU_RATE is invalid. Reset to %d", imu_rate);
  }
  return true;
}

void ImuRosBase::createPublishers(){
  pub_imu = nh.advertise<sensor_msgs::Imu>("imu", 1);
  pub_mag = nh.advertise<sensor_msgs::MagneticField>("magnetic_field", 1);
  pub_imu_ptr = boost::shared_ptr<ros::Publisher>(&pub_imu);
  pub_mag_ptr = boost::shared_ptr<ros::Publisher>(&pub_mag);
  return;
}

void ImuRosBase::errorCodeParser(const VN_ERROR_CODE& error_code) {
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

bool ImuRosBase::initialize() {
  // load parameters
  if(!loadParameters()) return false;

  // Create publishers
  createPublishers();

	VN_ERROR_CODE error_code;

  // Connect to the device
  ROS_INFO("Connecting to device");
	error_code = vn100_connect(&imu, port.c_str(), 115200);
  errorCodeParser(error_code);
  ros::Duration(0.5).sleep();
  ROS_INFO("Connected to device at %s", port.c_str());

  // Get the old serial baudrate
  unsigned int  old_baudrate;
	error_code = vn100_getSerialBaudRate(&imu, &old_baudrate);
  ROS_INFO("Default serial baudrate: %u", old_baudrate);

  // Set the new serial baudrate
  ROS_INFO("Set serial baudrate to %d", baudrate);
	error_code = vn100_setSerialBaudRate(&imu, baudrate, true);
  errorCodeParser(error_code);

  ROS_INFO("Disconnecting the device");
  vn100_disconnect(&imu);
  ros::Duration(0.5).sleep();

  ROS_INFO("Reconnecting to device");
	error_code = vn100_connect(&imu, port.c_str(), baudrate);
  errorCodeParser(error_code);
  ros::Duration(0.5).sleep();
  ROS_INFO("Connected to device at %s", port.c_str());

	error_code = vn100_getSerialBaudRate(&imu, &old_baudrate);
  ROS_INFO("New serial baudrate: %u", old_baudrate);

  // Idle the device for intialization
  ROS_INFO("Pause the device");
  error_code = vn100_pauseAsyncOutputs(&imu, true);
  errorCodeParser(error_code);

  // Get device info
  ROS_INFO("Fetching device info.");
  char model_number_buffer[30] = {0};
  int hardware_revision = 0;
  char serial_number_buffer[30] = {0};
  char firmware_version_buffer[30] = {0};

  error_code = vn100_getModelNumber(&imu, model_number_buffer, 30);
  errorCodeParser(error_code);
  ROS_INFO("Model number: %s", model_number_buffer);
  error_code = vn100_getHardwareRevision(&imu, &hardware_revision);
  errorCodeParser(error_code);
  ROS_INFO("Hardware revision: %d", hardware_revision);
  error_code = vn100_getSerialNumber(&imu, serial_number_buffer, 30);
  errorCodeParser(error_code);
  ROS_INFO("Serial number: %s", serial_number_buffer);
  error_code = vn100_getFirmwareVersion(&imu, firmware_version_buffer, 30);
  errorCodeParser(error_code);
  ROS_INFO("Firmware version: %s", firmware_version_buffer);

  // Resume the device
  ROS_INFO("Resume the device");
  error_code = vn100_resumeAsyncOutputs(&imu, true);
  errorCodeParser(error_code);

  // configure diagnostic updater
  if (!nh.hasParam("diagnostic_period")) {
    nh.setParam("diagnostic_period", 0.2);
  }

  updater.reset(new diagnostic_updater::Updater());
  string hw_id = string("vn100") + '-' + string(model_number_buffer);
  updater->setHardwareID(hw_id);
  //updater->add("diagnostic_info", this,
  //    &ImuRosBase::updateDiagnosticInfo);

  double imu_rate_float = static_cast<double>(imu_rate);
  diagnostic_updater::FrequencyStatusParam freqParam(
      &imu_rate_float, &imu_rate_float, 0.01, 10);
  diagnostic_updater::TimeStampStatusParam timeParam(
      0, 0.5/imu_rate_float);
  imu_diag.reset(new diagnostic_updater::TopicDiagnostic("imu",
        *updater, freqParam, timeParam));

  updater_ptr = updater;
  imu_diag_ptr = imu_diag;

  return true;
}

void ImuRosBase::enableIMUStream(bool enabled){

	VN_ERROR_CODE error_code;

  // Pause the device first
  error_code = vn100_pauseAsyncOutputs(&imu, true);
  errorCodeParser(error_code);

  if (enabled) {
    // Set output data type and data rate
    //error_code = vn100_setBinaryOutput1Configuration(
    //  &imu, BINARY_ASYNC_MODE_SERIAL_1,
    //  static_cast<int>(800/imu_rate),
    //  BG1_QTN | BG1_IMU | BG1_MAG_PRES,
    //  //BG1_IMU,
    //  BG3_NONE, BG5_NONE, true);
    //unsigned int base_freq = 0;
    //error_code = vn100_getAsynchronousDataOutputFrequency(
    //  &imu, &base_freq);
    //ROS_INFO("Base frequency: %d", base_freq);
    error_code = vn100_setAsynchronousDataOutputType(
          &imu, VNASYNC_OFF, true);
    error_code = vn100_setBinaryOutput1Configuration(
      &imu,
      BINARY_ASYNC_MODE_SERIAL_1,
      80,
      BG1_QTN | BG1_IMU | BG1_MAG_PRES,
      BG3_NONE,
      BG5_NONE,
      true);
    errorCodeParser(error_code);
    //error_code = vn100_setAsynchronousDataOutputType(
    //    &imu, VNASYNC_VNQAR, true);
    //errorCodeParser(error_code);
    //error_code  = vn100_setAsynchronousDataOutputFrequency(
    //    &imu, 100, true);
    //errorCodeParser(error_code);
    ROS_INFO("Configure the device");
    // Add a callback function for new data event
    error_code = vn100_registerAsyncDataReceivedListener(
        &imu, &asyncDataListener);
    errorCodeParser(error_code);
  } else {
    // Mute the stream
    error_code = vn100_setAsynchronousDataOutputType(
          &imu, VNASYNC_OFF, true);
    errorCodeParser(error_code);
    ROS_INFO("Mute the device");
    // Remove the callback function for new data event
    error_code = vn100_unregisterAsyncDataReceivedListener(
        &imu, &asyncDataListener);
    errorCodeParser(error_code);
  }

  // Resume the device
  error_code = vn100_resumeAsyncOutputs(&imu, true);
  errorCodeParser(error_code);
  return;
}

void ImuRosBase::requestIMUOnce() {
	VN_ERROR_CODE error_code;
  VnQuaternion att;
  VnVector3 ang, acc, mag;
  ros::Time time = ros::Time::now();

  // Note that this function blocks the program until the data is received
  error_code  = vn100_getQuaternionMagneticAccelerationAngularRate(
      &imu, &att, &mag, &acc, &ang);
  errorCodeParser(error_code);
  publishIMUData(time, att, ang, acc, mag);
  return;
}

void ImuRosBase::publishIMUData(const ros::Time& time,
    const VnQuaternion& att, const VnVector3& ang,
    const VnVector3& acc, const VnVector3& mag){
  sensor_msgs::Imu imu;
  sensor_msgs::MagneticField field;

  imu.header.stamp = time;
  imu.header.frame_id = frame_id;
  field.header.stamp = time;
  field.header.frame_id = frame_id;

  // TODO: get the covariance for the estimated attitude
  imu.orientation.x = att.x;
  imu.orientation.y = att.y;
  imu.orientation.z = att.z;
  imu.orientation.w = att.w;

  imu.linear_acceleration.x = acc.c0;
  imu.linear_acceleration.y = acc.c1;
  imu.linear_acceleration.z = acc.c2;

  imu.angular_velocity.x = ang.c0;
  imu.angular_velocity.y = ang.c1;
  imu.angular_velocity.z = ang.c2;

  field.magnetic_field.x = mag.c0;
  field.magnetic_field.y = mag.c1;
  field.magnetic_field.z = mag.c2;

  pub_imu.publish(imu);
  pub_mag.publish(field);

  // Update diagnostic info
  imu_diag->tick(imu.header.stamp);
  updater->update();
  return;
}

void ImuRosBase::updateDiagnosticInfo(
    diagnostic_updater::DiagnosticStatusWrapper& stat) {
  // TODO: add diagnostic info
  return;
}

}// End namespace imu_vn_100
