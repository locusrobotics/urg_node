/**
Software License Agreement (proprietary)
\file      uam_driver_ros.h
\authors   Carlos Mendes <cribeiromendes@locusrobotics.com>
\copyright Copyright (c) (2023,), Locus Robotics Corp., All rights reserved.
Unauthorized copying of this file, via any medium, is strictly prohibited.
Proprietary and confidential.
**/

#ifndef INCLUDE_UAM_UAM_DRIVER_ROS_H_
#define INCLUDE_UAM_UAM_DRIVER_ROS_H_

#include "uam/uam_driver.h"
#include <atomic>
#include <mutex>
#include <ros/callback_queue.h>
#include <string>
#include <uam/uam_driver_ros_params.h>

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
  UamROS(const ros::NodeHandle& node_handle, const UamROSParams& params);

private:
  /**
   * @brief Attempt to reconnect to the lidar if no scan sectors are received
   */
  void scanSectorWatchdogTimerCallback(const ros::TimerEvent& event);

  /**
   * @brief Attempt to connect to and configure the lidar in response to a timer event
   */
  void configureTimerCallback(const ros::TimerEvent& event);

  /**
   * @brief Trigger reconfigure routine
   */
  void triggerReconfigure();

  /**
   * @brief
   *
   * @return
   */
  bool configure();

  /**
   * @brief
   *
   * @param scan_sector
   */
  void scanCallback(const protocol::AR00CommandReply& scan_sector);
  /**
   * @brief
   *
   * @param scan_sector
   */
  void scanCallback(const protocol::AR01CommandReply& scan_sector);

  /**
   * @brief
   *
   * @param scan_sector
   */
  void scanCallback(const protocol::AR06CommandReply& scan_sector);


private:
  /**
   * \defgroup
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

  /**
   * @brief UAM configuration parameters
   */
  UamROSParams params_;

  /**
   * @brief Synchronization guard for watchdog variables accessed by ROS and lidar callbacks
   */
  std::mutex watchdog_mutex_;

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
   * @brief Configure Timer
   */
  ros::Timer configure_timer_;

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
   */
  ros::Publisher status_publisher_;

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
};

}  // namespace uam

#endif  // INCLUDE_UAM_UAM_DRIVER_ROS_H_
