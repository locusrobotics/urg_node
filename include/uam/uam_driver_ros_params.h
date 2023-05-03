/**
Software License Agreement (proprietary)
\file      uam_ros_params.h
\authors   Carlos Mendes <cribeiromendes@locusrobotics.com>
\copyright Copyright (c) (2023,), Locus Robotics Corp., All rights reserved.
Unauthorized copying of this file, via any medium, is strictly prohibited.
Proprietary and confidential.
**/

#ifndef INCLUDE_UAM_UAM_DRIVER_ROS_PARAMS_H_
#define INCLUDE_UAM_UAM_DRIVER_ROS_PARAMS_H_


#include <ros/node_handle.h>

namespace uam
{
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
  float angle_min { -2.36 } ;

  /**
   * @brief Controls (filters) the angle of the last range measurement in radians
   */
  float angle_max {2.36};

  /**
   * @brief Period between lidar reconnect attempts
   */
  ros::Duration reconnect_timeout { 5.0 };

  /**
   * @brief If no scan sectors have been received after this many seconds, reconnect to the lidar
   */
  ros::Duration scan_timeout { 10.0 };

  /**
   * @brief Enable or disable intensity readings
   */
  bool use_intensity { true };

  /**
   * @brief Enable or disable high resolution (multi-echo).
   *
   * This setting cannot be set to true if intensity is to be used
   */
  bool use_multi_echo {false};


  /**
   * @brief The frame id attached to the LaserScan message header
   */
  std::string frame_id { "laser" };

  /**
   * @brief Enable or disable hardware timestamping of received UDP packets by the ethernet adapter
   *
   * The hardwaree timestamp is captured by the ethernet adapter itself, either at the MAC or PHY layer. Support for
   * hardware timestamping is dependent on the ethernet adapter hardware and kernel driver implementation; not all
   * ethernet adapters support hardware timestamping. Additionally, hardware timestamping must be enabled for each
   * ethernet adapter at the kernel level. The easiest way to accomplish this is by running
   * `sudo hwstamp_ctl -i eth0 -r 1` from the `linuxptp` debian package before connecting to the lidar. Further, the
   * hardware timestamps will use the clock in the ethernet adapter. The ethernet clock and main system clock should
   * be synchronized before enabling hardware timestamps. The easiest method of synchronizing the clocks is running
   * `sudo phc2sys -r -r -s CLOCK_REALTIME -c eth0 -O 0`, also from the `linuxptp` debian package.
   */
  bool hardware_timestamps { false };

  /**
   * @brief The tcp client port number the client/robot/NUC is using to communicate with laser
   */
  unsigned int ip_port { 10940 };

  /**
   * @brief The amount of seconds the client waits for a reply to the requested command
   */
  ros::Duration command_reply_timeout { 60.0 };

  /**
   * @brief Async read timeout - Amount of seconds the client waits for the continuous
   * message to arrive (usually this should be set to twice the frame rate so ~ 60 ms
   */
//  ros::Duration scan_timeout { 60.0 };

  /**
   * @brief The IP Address of the lidar server
   */
  std::string ip_address { "localhost" };

  /**
   * @brief The maximum range of the lidar
   */
  double max_range { 40.0 };

  /**
   * @brief The minimum range of the lidar
   */
  double min_range { 0.0 };

  /**
   * @brief Period between lidar reconfiguration attempts
   */
  ros::Duration reconfiguration_timeout { 5.0 };

  /**
   * @brief Additional time offset to add to the lidar timestamps to compensate for unmeasured delays
   */
  ros::Duration time_offset { -0.0218 };

  /**
   * @brief The topic name where LaserScan messages will be published
   */
  std::string scan_topic { "scan" };

  /**
   * @brief The topic name where LaserScan messages will be published
   */
  std::string status_topic { "status" };
};
}

#endif  // INCLUDE_UAM_UAM_DRIVER_ROS_PARAMS_H_
