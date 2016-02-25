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
 * @param vn_quat
 */
void RosQuaternionFromVnQuaternion(geometry_msgs::Quaternion& ros_quat,
                                   const vn::math::kinematics::quat<float>& vn_quat);

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

ImuVn100::ImuVn100(const ros::NodeHandle& pnh){

}

ImuVn100::~ImuVn100() {
    Disconnect();
}

void ImuVn100::FixImuRate() {

}

void ImuVn100::LoadParameters() {

}

void ImuVn100::CreateDiagnosedPublishers() {

}

void ImuVn100::Initialize() {

}

void ImuVn100::Stream(bool async){

}

void ImuVn100::Resume(bool need_reply) {

}

void ImuVn100::Idle(bool need_reply) {

}

void ImuVn100::Disconnect() {

}
/*
void ImuVn100::PublishData(const VnDeviceCompositeData& data) {

}*/

}
