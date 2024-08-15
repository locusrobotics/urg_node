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
  double time_offset_sec = 0;
  nh.getParam("time_offset", time_offset_sec);
  params.time_offset = ros::Duration(time_offset_sec);

  double reconfig_timeout = params.reconfiguration_timeout.toSec();
  if (nh.param("reconfiguration_timeout", reconfig_timeout, reconfig_timeout))
  {
    params.reconfiguration_timeout = ros::Duration(reconfig_timeout);
  }
  nh.getParam("topic", params.scan_topic);
  nh.getParam("restart_counter_topic", params.restart_counter_topic);

  nh.getParam("provide_laser_status_service", params.provide_laser_status_service);
  nh.getParam("request_status_service", params.request_status_service);
  nh.getParam("range_offset", params.range_offset);
  nh.getParam("log_safety_areas_crc", params.log_safety_areas_crc);

  int reference_area;
  if (nh.getParam("reference_safety_area_type", reference_area))
  {
    if (
      reference_area < static_cast<int>(protocol::EYRAreaType::protection_1) ||
      reference_area > static_cast<int>(protocol::EYRAreaType::warning_2))
    {
      ROS_WARN_STREAM(
        "Invalid Reference Area Type Detected, supported options are: "
        "protection_1 = 0, protection_2 = 1, warning_1 = 2, warning_2 = 3. Detected: "
        << reference_area);
    }
    else
    {
      params.reference_safety_area = static_cast<protocol::EYRAreaType>(reference_area);
    }
  }

  nh.param("configure_attempts", params.configure_attempts, params.configure_attempts);

  return params;
}

}  // namespace uam
