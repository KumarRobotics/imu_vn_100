/*
 * Copyright 2016
 * Authors: [Ke Sun]
 *          Andre Phu-Van Nguyen <andre-phu-van.nguyen@polymtl.ca>
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
#include "vn/sensors/sensors.h"
#include "vn/math/math.h"
#include "vn/math/conversions.h"
#include "vn/math/kinematics.h"

namespace imu_vn_100 {

/**
 * @brief RosVector3FromVnVector3
 * @param ros_vec3
 * @param vn_vec3
 */
void RosVector3FromVnVector3(geometry_msgs::Vector3& ros_vec3,
                             const vn::math::vec3f& vn_vec3);

/**
 * @brief RosQuaternionFromVnQuaternion
 * @param ros_quat
 * @param vn_vec4
 */
void RosQuaternionFromVnVector4(geometry_msgs::Quaternion& ros_quat,
                                const vn::math::vec4f& vn_vec4);

/**
 * @brief FillImuMessage
 * @param imu_msg
 * @param p
 * @param binary_output
 */
void FillImuMessage(sensor_msgs::Imu& imu_msg,
                    vn::protocol::uart::Packet& p, bool binary_output);

/**
 * @brief asciiOrBinaryAsyncMessageReceived
 * @param userData
 * @param p
 * @param index
 */
void asciiOrBinaryAsyncMessageReceived(void* userData,
                                       vn::protocol::uart::Packet& p,
                                       size_t index);

/**
 * @brief errorMessageReceived
 * @param userData
 * @param p
 * @param index
 */
void errorMessageReceived(void* userData, vn::protocol::uart::Packet& p,
                          size_t index);

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
    :   pnh_(pnh),
      port_(std::string("/dev/ttyUSB0")),
      baudrate_(921600),
      frame_id_(std::string("imu")) {
    Initialize();
    //TODO pointer hack
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

    pnh_.param("sync_rate", sync_info_.rate, kDefaultSyncOutRate);
    pnh_.param("sync_pulse_width_us", sync_info_.pulse_width_us, 1000);

    pnh_.param("binary_output", binary_output_, true);
    int vn_serial_output_tmp = 4; //init on invalid number
    pnh_.param("vn_serial_output", vn_serial_output_tmp, 1);
    switch(vn_serial_output_tmp){
    case 0:
        vn_serial_output_ = vn::protocol::uart::ASYNCMODE_NONE;
        break;
    case 1:
        vn_serial_output_ = vn::protocol::uart::ASYNCMODE_PORT1;
        break;
    case 2:
        vn_serial_output_ = vn::protocol::uart::ASYNCMODE_PORT2;
        break;
    case 3:
        vn_serial_output_ = vn::protocol::uart::ASYNCMODE_BOTH;
        break;
    default:
        ROS_ERROR("Incorrect VN serial port chosen.");
        break;
    }

    FixImuRate();
    sync_info_.FixSyncRate();
}

void ImuVn100::CreateDiagnosedPublishers() {
    imu_rate_double_ = imu_rate_;
    pd_imu_.Create<sensor_msgs::Imu>(pnh_, "imu", updater_, imu_rate_double_);
    if (enable_mag_) {
        pd_mag_.Create<sensor_msgs::MagneticField>(pnh_, "magnetic_field", updater_,
                                                   imu_rate_double_);
    }
    if (enable_pres_) {
        pd_pres_.Create<sensor_msgs::FluidPressure>(pnh_, "fluid_pressure", updater_,
                                                    imu_rate_double_);
    }
    if (enable_temp_) {
        pd_temp_.Create<sensor_msgs::Temperature>(pnh_, "temperature", updater_,
                                                  imu_rate_double_);
    }
}

void ImuVn100::Initialize() {
    LoadParameters();
    unsigned int old_baudrate;
    // Try initial opening
    try{
        ROS_INFO("Connecting to device");
        imu_.connect(port_, 115200);
        ros::Duration(1).sleep();
        ROS_INFO("Connected to device at %s", port_.c_str());

        old_baudrate = imu_.readSerialBaudRate();
        ROS_INFO("Default serial baudrate: %u", old_baudrate);

        ROS_INFO("Set serial baudrate to %d", baudrate_);
        imu_.writeSerialBaudRate(baudrate_, true);

        ROS_DEBUG("Disconnecting the device");
        imu_.disconnect();
        ros::Duration(0.5).sleep();
    } catch (std::exception except){
        ROS_DEBUG("Failed to open device with default baudrate with exception: %s",
                  except.what());
    }

    // Open with the desired baud rate
    ROS_DEBUG("Reconnecting to device");
    imu_.connect(port_, baudrate_);
    ros::Duration(0.5).sleep();
    ROS_INFO("Connected to device at %s", port_.c_str());

    old_baudrate = imu_.readSerialBaudRate();
    ROS_INFO("New serial baudrate: %u", old_baudrate);

    ROS_INFO("Fetching device info.");
    std::string model_num = imu_.readModelNumber();
    ROS_INFO("Model number: %s", model_num.c_str());
    unsigned int hardw_rev = imu_.readHardwareRevision();
    ROS_INFO("Hardware revision: %d", hardw_rev);
    unsigned int serial_num = imu_.readSerialNumber();
    ROS_INFO("Serial number: %d", serial_num);
    std::string firmw_rev = imu_.readFirmwareVersion();
    ROS_INFO("Firmware version: %s", firmw_rev.c_str());

    if (sync_info_.SyncEnabled()) {
        ROS_INFO("Set Synchronization Control Register.");
        imu_.writeSynchronizationControl(
                    vn::protocol::uart::SYNCINMODE_COUNT,
                    vn::protocol::uart::SYNCINEDGE_RISING,
                    0,
                    vn::protocol::uart::SYNCOUTMODE_ITEMSTART,
                    vn::protocol::uart::SYNCOUTPOLARITY_POSITIVE,
                    sync_info_.skip_count,
                    sync_info_.pulse_width_us * 1000,
                    true);

        if (!binary_output_) {
            ROS_INFO("Set Communication Protocol Control Register (id:30).");
            imu_.writeCommunicationProtocolControl(
                        vn::protocol::uart::COUNTMODE_SYNCOUTCOUNTER,
                        vn::protocol::uart::STATUSMODE_OFF,
                        vn::protocol::uart::COUNTMODE_NONE, // SPI
                        vn::protocol::uart::STATUSMODE_OFF, // SPI
                        vn::protocol::uart::CHECKSUMMODE_CHECKSUM, // serial checksum is 8bit
                        vn::protocol::uart::CHECKSUMMODE_CHECKSUM, // SPI
                        vn::protocol::uart::ERRORMODE_SEND,
                        true);
        }
    }

    CreateDiagnosedPublishers();

    auto hardware_id = std::string("vn100-") + model_num +
                        std::to_string(serial_num);
    updater_.setHardwareID(hardware_id);
}

void ImuVn100::Stream(bool async){
    // TODO There isn't a pause function in the new lib, what do here?
    using namespace vn::protocol::uart;

    if (async)
    {
        imu_.writeAsyncDataOutputType(VNOFF, true);

        if (binary_output_) {
            // Set the binary output data type and data rate
            vn::sensors::BinaryOutputRegister bor(
                        vn_serial_output_,
                        kBaseImuRate / imu_rate_,
                        (COMMONGROUP_QUATERNION | COMMONGROUP_IMU |
                         COMMONGROUP_MAGPRES | COMMONGROUP_SYNCINCNT),
                        TIMEGROUP_NONE,
                        IMUGROUP_NONE,
                        GPSGROUP_NONE,
                        ATTITUDEGROUP_NONE,
                        INSGROUP_NONE);

            imu_.writeBinaryOutput1(bor, true);
        } else {
            // Set the ASCII output data type and data rate
            imu_.writeAsyncDataOutputType(VNIMU, true);
        }

        // add a callback function for new data event
        imu_.registerAsyncPacketReceivedHandler(this, asciiOrBinaryAsyncMessageReceived);

        ROS_INFO("Setting IMU rate to %d", imu_rate_);
        imu_.writeAsyncDataOutputFrequency(imu_rate_, true);
    } else {
        // Mute the stream
        ROS_DEBUG("Mute the device");
        imu_.writeAsyncDataOutputType(VNOFF, true);
        // Remove the callback function for new data event
        try {
            imu_.unregisterAsyncPacketReceivedHandler();
        } catch (std::exception except) {
            ROS_WARN("Unable to unregister async packet handler: %s",
                     except.what());
        }
    }
}

void ImuVn100::Resume(bool need_reply) {

}

void ImuVn100::Idle(bool need_reply) {

}

void ImuVn100::Disconnect() {

}

void ImuVn100::PublishData(vn::protocol::uart::Packet& p) {
    sensor_msgs::Imu imu_msg;
    imu_msg.header.stamp = ros::Time::now();
    imu_msg.header.frame_id = frame_id_;

    FillImuMessage(imu_msg, p, binary_output_);
    pd_imu_.Publish(imu_msg);

    if (enable_mag_) {
        sensor_msgs::MagneticField mag_msg;
        mag_msg.header = imu_msg.header;
        vn::math::vec3f magnetometer = p.extractVec3f();// COMMONGROUP_MAGPRES
        RosVector3FromVnVector3(mag_msg.magnetic_field, magnetometer);
        pd_mag_.Publish(mag_msg);
    }

    if (enable_temp_) {
        sensor_msgs::Temperature temp_msg;
        temp_msg.header = imu_msg.header;
        float temperature = p.extractFloat();           // COMMONGROUP_MAGPRES
        temp_msg.temperature = temperature;
        pd_temp_.Publish(temp_msg);
    }

    if (enable_pres_) {
        sensor_msgs::FluidPressure pres_msg;
        pres_msg.header = imu_msg.header;
        float pressure = p.extractFloat();              // COMMONGROUP_MAGPRES
        pres_msg.fluid_pressure = pressure;
        pd_pres_.Publish(pres_msg);
    }

    unsigned int syncInCnt = p.extractUint32();
    sync_info_.Update(syncInCnt, imu_msg.header.stamp);

    updater_.update();
}

void RosVector3FromVnVector3(geometry_msgs::Vector3& ros_vec3,
                             const vn::math::vec3f& vn_vec3){
    ros_vec3.x = vn_vec3[0];
    ros_vec3.y = vn_vec3[1];
    ros_vec3.z = vn_vec3[2];
}

void RosQuaternionFromVnVector4(geometry_msgs::Quaternion& ros_quat,
                                const vn::math::vec4f& vn_vec4) {
    ros_quat.x = vn_vec4[0];    //see quaternion application note
    ros_quat.y = vn_vec4[1];
    ros_quat.z = vn_vec4[2];
    ros_quat.w = vn_vec4[3];
}


void FillImuMessage(sensor_msgs::Imu& imu_msg,
                    vn::protocol::uart::Packet& p, bool binary_output){
    if (binary_output) {
        // Note: With this library, we are responsible for extracting the data
        // in the appropriate order! Need to refer to manual and to how we
        // configured the common output group
        vn::math::vec4f quaternion = p.extractVec4f();  // COMMONGROUP_QUATERNION
        // NOTE: The IMU angular velocity and linear acceleration outputs are
        // swapped. And also why are they different?
        vn::math::vec3f linear_accel = p.extractVec3f();// COMMONGROUP_IMU
        vn::math::vec3f angular_rate = p.extractVec3f();// COMMONGROUP_IMU

        RosQuaternionFromVnVector4(imu_msg.orientation, quaternion);
        RosVector3FromVnVector3(imu_msg.angular_velocity, angular_rate);
        RosVector3FromVnVector3(imu_msg.linear_acceleration, linear_accel);
    } else {
        // TODO
    }
}

void asciiOrBinaryAsyncMessageReceived(void* userData, vn::protocol::uart::Packet& p, size_t index){
    using namespace vn::protocol::uart;
    ImuVn100* imu = (ImuVn100*) userData;
    if (!p.isCompatible(
        (COMMONGROUP_QUATERNION | COMMONGROUP_IMU |
         COMMONGROUP_MAGPRES | COMMONGROUP_SYNCINCNT),
        TIMEGROUP_NONE,
        IMUGROUP_NONE,
        GPSGROUP_NONE,
        ATTITUDEGROUP_NONE,
        INSGROUP_NONE))
        // Not the type of binary packet we are expecting.
        return;

    imu->PublishData(p);
}

void errorMessageReceived(void* userData, vn::protocol::uart::Packet& p,
                          size_t index) {
    ImuVn100* imu = (ImuVn100*) userData;
}

} // end namespace

