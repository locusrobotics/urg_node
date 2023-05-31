/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2023, Locus Robotics
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#ifndef UAM_UAM_DRIVER_ROS_H
#define UAM_UAM_DRIVER_ROS_H

#include <dynamic_reconfigure/server.h>
#include <ros/callback_queue.h>
#include <std_srvs/Trigger.h>
#include <uam/uam_driver.h>
#include <uam/uam_driver_ros_params.h>
#include <urg_node/URGConfig.h>

#include <atomic>
#include <mutex>
#include <string>

namespace uam
{
class UamROS
{
public:
  /**
   * @brief Constructor
   *
   * The class will immediately connect to the lidar and start publishing scans.
   *
   * @param[in] node_handle - A node handle in the global namespace, used for advertising topics
   * @param[in] params - A populated parameters structure containing the node configuration
   */
  UamROS(const ros::NodeHandle& nh, const ros::NodeHandle& nh_prv, const UamROSParams& params);

private:
  /**
   * @brief Attempt to reconnect to the lidar if no scan are received
   */
  void scanWatchdogTimerCallback(const ros::TimerEvent& event);

  /**
   * @brief Connect and configure
   *
   * @return true if successfully connected
   */
  bool configure();

  /**
   * @brief Attempt to connect to and configure the lidar in response to a timer event
   *
   * @param[in] event - Timer event
   */
  void configureTimerCallback(const ros::TimerEvent& event);

  /**
   * @brief Dynamic reconfigure callback
   *
   * @param[in] config - Configuration
   * @param[in] level - Level
   */
  bool dynamicReconfigureCallback(urg_node::URGConfig& config, int level);

  /**
   * @brief Fill laser scan ros message
   *
   * @param[in] scan_packet - Incoming scan packet
   * @param[in] range_offset - Range in meters to be added to each range reading
   * @param[in/out] scan - scan ros message
   */
  template <typename T>
  void fillScanMessageData(const T& scan_packet, const double range_offset, sensor_msgs::LaserScan& scan) const
  {
    ROS_WARN_STREAM("Handler not implemented!");
  }

  /**
   * @brief Scan callback packet callback
   *
   * @param[in] reply - Reply message
   * @param[in] wall_time - wall time
   */
  template <typename T>
  void scanCallback(const T& reply, const ros::Time& wall_time)
  {
    {
      std::lock_guard<std::mutex> lock(watchdog_mutex_);
      scan_stamp_ = ros::Time::now();
    }
    ros::Duration time_offset;
    double range_offset = 0;
    {
      std::lock_guard<std::mutex> lock(reconfigure_mutex_);
      range_offset = params_.range_offset;
      time_offset = params_.time_offset;
      if (params_changed_)
      {
        scan_params_.setAngleLimits(params_.angle_min, params_.angle_max);
        params_changed_ = false;
      }
    }
    // Fill scan metadata and header
    sensor_msgs::LaserScan msg;
    msg.header.frame_id = params_.frame_id;
    msg.angle_min = scan_params_.getAngleMin();
    msg.angle_max = scan_params_.getAngleMax();
    msg.angle_increment = scan_params_.getAngleIncrement();
    msg.scan_time = scan_params_.getScanPeriod();
    msg.time_increment = scan_params_.getTimeIncrement();
    msg.range_min = scan_params_.getRangeMin();
    msg.range_max = scan_params_.getRangeMax();

    msg.header.stamp = wall_time + time_offset + ros::Duration(scan_params_.getAngularTimeOffset());

    // Read the right fields
    fillScanMessageData(reply, range_offset, msg);
    // Update status
    updateStatus(reply.sensing_data);
    scan_publisher_.publish(msg);
  }

  /**
   * @brief Request status service callback
   *
   * @param[in] req - request
   * @param[in] res - response
   * @return always true
   */
  bool statusCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);

  /**
   * @brief Trigger reconfigure routine
   */
  void triggerReconfigure();

  /**
   * @brief Update Reconfigure limits (as in urg_node_driver)
   */
  void updateReconfigureLimits();

  /**
   * @brief Update status
   *
   * @param[in] sensing_data - Last received sensing data
   * @param[in] override_check - Boolean to override comparison between last status and sensing_data
   */
  void updateStatus(const protocol::sensing_data::SensingDataHeader& sensing_data, const bool override_check = false);

private:
  /**
   * \defgroup Lidar communication Section
   * @{
   */

  /**
   * @brief Flag indicating the lidar is configured
   */
  std::atomic_bool configured_;

  /**
   * @brief Counter of configure attempts
   */
  uint32_t configure_attempts_;

  /**
   * @brief Object to interface with the lidar
   */
  UamDriver lidar_;

  /**@}*/

  /**
   * \defgroup Dynamic reconfigure details section
   * @{
   */

  /**
   * @brief UAM configuration parameters
   */
  UamROSParams params_;

  /**
   * @brief Synchronization guard for watchdog variables accessed by ROS and lidar callbacks
   */
  std::mutex reconfigure_mutex_;

  /**
   * @brief Flag pointing that reconfigure was requested
   */
  bool params_changed_;

  /**@}*/

  /**
   * \defgroup Watchdog section
   * @{
   */

  /**
   * @brief Synchronization guard for watchdog variables accessed by ROS and lidar callbacks
   */
  std::mutex watchdog_mutex_;

  /**
   * @brief Watchdog timer to reconnect if no sectors are received
   */
  ros::Timer scan_watchdog_timer_;

  /**
   * @brief The last received scan sector timestamp
   */
  ros::Time scan_stamp_;

  /**
   * @brief The last time the lidar was configured
   */
  ros::Time configured_stamp_;

  /**@}*/

  /**
   * \defgroup ROS members
   * @{
   */

  /**
   * @brief Node Handler
   */
  ros::NodeHandle node_handle_;
  /**
   * @brief Node Handler
   */
  ros::NodeHandle private_node_handle_;

  /**
   * @brief Configure Timer
   */
  ros::Timer configure_timer_;

  /**
   * @brief Dynamic reconfigure server
   */
  boost::shared_ptr<dynamic_reconfigure::Server<urg_node::URGConfig>> srv_;

  /**
   * @brief Scan Parameters
   */
  ScanParameters scan_params_;

  /**
   * @brief Scan Publisher
   */
  ros::Publisher scan_publisher_;

  /**
   * @brief Status Publisher
   * 
   * Publishes lidar status on request
   */
  ros::Publisher status_on_request_publisher_;

  /**
   * @brief Status Publisher
   * 
   * Publishes lidar status on update
   */
  ros::Publisher status_on_update_publisher_;

  /**
   * @brief Status Service
   */
  ros::ServiceServer request_status_service_;

  /**
   * @brief Last received sensing data status
   */
  protocol::sensing_data::SensingDataHeader last_received_status_;

  /**
   * @brief Flag the receiver thread to publish the status no mater if it
   * is equal to the last published one
   */
  std::atomic_bool publish_status_requested_;

  /**@}*/
};

}  // namespace uam

#endif  // UAM_UAM_DRIVER_ROS_H
