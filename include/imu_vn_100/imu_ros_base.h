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

#ifndef IMU_VN_100_ROS_H
#define IMU_VN_100_ROS_H

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

#include <imu_vn_100/vncpplib/include/vn100.h>

namespace imu_vn_100 {

  /**
   * @brief SyncInfo Contains the data for reporting
   *    synchronization status. And it handles the
   *    thread-safe reading and writing.
   */
  struct SyncInfo {
    long long int sync_count;
    ros::Time sync_time;
    boost::shared_mutex mtx;

    SyncInfo():
      sync_count(-1),
      sync_time(ros::Time::now()) {
      return;
    }
    ~SyncInfo() {return;}

    long long int getSyncCount() {
      boost::shared_lock<boost::shared_mutex> read_lock(mtx);
      return sync_count;
    }
    ros::Time getSyncTime() {
      boost::shared_lock<boost::shared_mutex> read_lock(mtx);
      return sync_time;
    }
    void setSyncCount(const long long int& new_count) {
      boost::unique_lock<boost::shared_mutex> write_lock(mtx);
      sync_count = new_count;
      return;
    }
    void setSyncTime(const ros::Time& new_time) {
      boost::unique_lock<boost::shared_mutex> write_lock(mtx);
      sync_time = new_time;
    }
  };

  /**
   * @brief ImuRosBase The class is a ros wrapper for the Imu class
   * @author Ke Sun
   */
  class ImuRosBase {
  public:
    /**
     * @brief Constructor of the class
     */
    ImuRosBase(const ros::NodeHandle& nh);
    /**
     * @brief Destructor of the class
     * @note Disconnect the device automatically
     */
    ~ImuRosBase() {
      disconnect();
    }
    /**
     * @brief initialize Initialize IMU according to the settings
     */
    bool initialize();
    /**
     * @brief enableIMUStream Enable or disable IMU stream
     * @param enabled If ture, the continuous stream is enabled
     */
    void enableIMUStream(bool enabled);

    /**
     * @brief requestIMUOnce Request IMU data for a single time
     */
    void requestIMUOnce();

    /**
     * @brief idle Set the device to idle mode
     * @need_reply If ture, a ack package is expected
     */
    void idle(bool need_reply = true) {
      vn100_pauseAsyncOutputs(&imu, need_reply);
      return;
    }

    /**
     * @brief resume Resume data streaming
     * @need_reply If ture, a ack package is expected
     */
    void resume(bool need_reply = true) {
      vn100_resumeAsyncOutputs(&imu, need_reply);
      return;
    }

    /**
     * @brief disconnect Disconnect the device
     */
    void disconnect() {
      vn100_reset(&imu);
      vn100_disconnect(&imu);
    }

    /**
     * @brief getSyncCount Resturns the count for sync trigger
     */
    long long int getSyncCount() {
      return sync_info.getSyncCount();
    }

    /**
     * @brief getSyncTime Resturns the ros time stamp for the
     *    latest trigger
     */
    ros::Time getSyncTime() {
      return sync_info.getSyncTime();
    }

  private:

    // Settings
    std::string port;
    int baudrate;
    int imu_rate;
    std::string frame_id;

    bool enable_mag;
    bool enable_pres;
    bool enable_temp;

    bool enable_sync_out;
    int sync_out_pulse_width;
    int sync_out_rate;
    int sync_out_skip_count;

    ros::NodeHandle nh;
    Vn100 imu;

    // Tracking the triggering signal
    SyncInfo sync_info;

    // Publishers
    ros::Publisher pub_imu;
    ros::Publisher pub_mag;
    ros::Publisher pub_pres;
    ros::Publisher pub_temp;

    // diagnostic_updater resources
    double update_rate;
    boost::shared_ptr<diagnostic_updater::Updater> updater;
    boost::shared_ptr<diagnostic_updater::TopicDiagnostic> imu_diag;
    boost::shared_ptr<diagnostic_updater::TopicDiagnostic> mag_diag;
    boost::shared_ptr<diagnostic_updater::TopicDiagnostic> pres_diag;
    boost::shared_ptr<diagnostic_updater::TopicDiagnostic> temp_diag;

    // Disable copy constructor and assign operator
    ImuRosBase(const ImuRosBase&);
    ImuRosBase& operator=(const ImuRosBase&);

    bool loadParameters();
    void createPublishers();
    void errorCodeParser(const VN_ERROR_CODE& error_code);

    // Publish IMU msgs
    void publishIMUData();
    // Callback function for adding meta info in the diag msgs
    void updateDiagnosticInfo(
        diagnostic_updater::DiagnosticStatusWrapper& stat);
  };

}

#endif

