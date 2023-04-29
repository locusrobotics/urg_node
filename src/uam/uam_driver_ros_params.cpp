/*
 * uam_driver_ros_params.cpp
 *
 *  Created on: 21/04/2023
 *      Author: cribeiromendes
 */

#include <uam/uam_driver_ros_params.h>

namespace uam
{
UamROSParams UamROSParams::loadFromROS(const ros::NodeHandle& nh)
{
  UamROSParams params;
  nh.getParam("angle_min", params.angle_min);
  nh.getParam("angle_max", params.angle_max);
  nh.getParam("publish_intensity", params.use_intensity);
  nh.getParam("publish_multiecho", params.use_multi_echo);
  if (params.use_multi_echo && params.use_intensity)
  {
    ROS_WARN_STREAM("Multiecho and intensity are not supported. Publishing only intensity.");
    params.use_multi_echo = false;
  }
  nh.getParam("frame_id", params.frame_id);
  nh.getParam("hardware_timestamps", params.hardware_timestamps);
  nh.getParam("ip_address", params.ip_address);
  auto port = nh.param("ip_port", static_cast<int>(params.ip_port));
  if (params.ip_port < 0)
  {
    ROS_WARN_STREAM("Invalid port! Using: " << params.ip_port);
  }
  else
  {
    params.ip_port = static_cast<unsigned int>(port);
  }
  nh.getParam("max_range", params.max_range);
  nh.getParam("min_range", params.max_range);

  double reconfig_timeout = params.reconfiguration_timeout.toSec();
  if (nh.param("reconfiguration_timeout", reconfig_timeout, reconfig_timeout))
  {
    params.reconfiguration_timeout = ros::Duration(reconfig_timeout);
  }
  nh.getParam("topic", params.scan_topic);
  return params;
}

}  // namespace uam
