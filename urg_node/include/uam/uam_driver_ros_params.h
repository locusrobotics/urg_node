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

#ifndef UAM_UAM_DRIVER_ROS_PARAMS_H
#define UAM_UAM_DRIVER_ROS_PARAMS_H

#include <ros/node_handle.h>
#include <uam/protocol_types/uam_protocol_types.h>

#include <optional>
#include <string>

namespace uam
{
/**
 * @brief UAM driver ros parameters
 */
struct UamROSParams
{
public:
  /**
   * @brief Method for loading parameter values from ROS.
   *
   * @param[in] nh - The ROS node handle with which to load parameters
   */
  static UamROSParams loadFromROS(const ros::NodeHandle& nh);

  /**
   * @brief Controls (filters) the angle of the first range measurement in radians.
   */
  double angle_min { -2.36 };

  /**
   * @brief Controls (filters) the angle of the last range measurement in radians
   */
  double angle_max { 2.36 };

  /**
   * @brief The amount of seconds the client waits for a reply to the requested command
   */
  ros::Duration command_reply_timeout { 2.0 };

  /**
   * @brief The frame id attached to the LaserScan message header
   */
  std::string frame_id { "laser" };

  /**
   * @brief The IP Address of the lidar server
   */
  std::string ip_address { "localhost" };

  /**
   * @brief The tcp client port number the client/robot/NUC is using to communicate with laser
   */
  unsigned int ip_port { 10940 };

  /**
   * @brief Flag to log safety area CRC
   */
  bool log_safety_areas_crc { false };

  /**
   * @brief The user defined maximum range of the lidar
   */
  double max_range { 40.0 };

  /**
   * @brief The user defined minimum range of the lidar
   */
  double min_range { 0.0 };

  /**
   * @brief If the update_laser_status service is to be provided
   */
  bool provide_laser_status_service { false };

  /**
   * @brief User range Offset
   */
  double range_offset {0.f};

  /**
   * @brief Period between lidar reconfiguration attempts
   */
  ros::Duration reconfiguration_timeout { 5.0 };

  /**
   * @brief Period between lidar reconnect attempts
   */
  ros::Duration reconnect_timeout { 5.0 };


  /**
   * @brief Update Laser Status service name
   */
  std::string request_status_service { "update_laser_status" };

  /**
   * @brief If no scan sectors have been received after this many seconds, reconnect to the lidar
   */
  ros::Duration scan_timeout { 5.0 };

  /**
   * @brief The topic name where LaserScan messages will be published
   */
  std::string scan_topic { "scan" };

  /**
   * @brief The topic name where LaserScan messages will be published
   */
  std::string status_topic { "laser_status" };

  /**
   * @brief The topic name where the area violating points will be published
   * as sensor_msgs::PointCloud2
   */
  std::string points_in_safety_area_topic { "points_in_safety_area" };

  /**
   * @brief The service name to request the lidar to power cycle
   */
  std::string lidar_hard_reset_service { "set_restarting_lasers_flag" };

  /**
    * @brief The minimum time between lidar power cycles
  */
  ros::Duration lidar_power_cycle_interval { 30 };

  /**
   * @brief If points are detected within this area, these will be published
   * on the points_in_safety_area topic
   */
  std::optional<uam::protocol::EYRAreaType> reference_safety_area {std::nullopt};

  /**
   * @brief Additional time offset to add to the lidar timestamps to compensate for unmeasured delays
   */
  ros::Duration time_offset { 0. };

  /**
   * @brief Enable or disable intensity readings
   */
  bool use_intensity { true };

  /**
   * @brief Enable or disable high resolution (multi-echo).
   *
   * This setting cannot be set to true if intensity is to be used
   */
  bool use_multi_echo { false };
};
}  // namespace uam

#endif  // UAM_UAM_DRIVER_ROS_PARAMS_H
