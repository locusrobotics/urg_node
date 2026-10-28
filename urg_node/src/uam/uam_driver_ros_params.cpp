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

#include <uam/uam_driver_ros_params.hpp>

namespace uam
{
UamROSParams UamROSParams::loadFromROS(rclcpp::Node* node)
{
  UamROSParams params;
  params.angle_min = node->declare_parameter<double>("angle_min", params.angle_min);
  params.angle_max = node->declare_parameter<double>("angle_max", params.angle_max);
  params.use_intensity = node->declare_parameter<bool>("publish_intensity", params.use_intensity);
  params.use_multi_echo = node->declare_parameter<bool>("publish_multiecho", params.use_multi_echo);
  if (params.use_multi_echo && params.use_intensity)
  {
    RCLCPP_WARN_STREAM(node->get_logger(), "Multiecho and intensity are not supported. Publishing only intensity.");
    params.use_multi_echo = false;
  }
  params.frame_id = node->declare_parameter<std::string>("frame_id", params.frame_id);
  params.ip_address = node->declare_parameter<std::string>("ip_address", params.ip_address);
  auto port = node->declare_parameter<int>("ip_port", static_cast<int>(params.ip_port));
  if (port < 0)
  {
    RCLCPP_WARN_STREAM(node->get_logger(), "Invalid port! Using: " << params.ip_port);
  }
  else
  {
    params.ip_port = static_cast<unsigned int>(port);
  }
  params.max_range = node->declare_parameter<double>("max_range", params.max_range);
  params.max_range = node->declare_parameter<double>("min_range", params.max_range);
  double time_offset_sec = node->declare_parameter<double>("time_offset", params.time_offset.seconds());
  params.time_offset = rclcpp::Duration::from_seconds(time_offset_sec);

  double reconfig_timeout = node->declare_parameter<double>(
    "reconfiguration_timeout", params.reconfiguration_timeout.seconds());
  params.reconfiguration_timeout = rclcpp::Duration::from_seconds(reconfig_timeout);

  params.scan_topic = node->declare_parameter<std::string>("topic", params.scan_topic);

  params.provide_laser_status_service = node->declare_parameter<bool>(
    "provide_laser_status_service", params.provide_laser_status_service);
  params.request_status_service = node->declare_parameter<std::string>(
    "request_status_service", params.request_status_service);
  params.range_offset = node->declare_parameter<double>("range_offset", params.range_offset);
  params.log_safety_areas_crc = node->declare_parameter<bool>(
    "log_safety_areas_crc", params.log_safety_areas_crc);
  params.discard_startup_data_s = node->declare_parameter<double>(
    "discard_startup_data_s", params.discard_startup_data_s);

  int reference_area = node->declare_parameter<int>("reference_safety_area_type", -1);
  if (reference_area != -1)
  {
    if (
      reference_area < static_cast<int>(protocol::EYRAreaType::protection_1) ||
      reference_area > static_cast<int>(protocol::EYRAreaType::warning_2))
    {
      RCLCPP_WARN_STREAM(
        node->get_logger(),
        "Invalid Reference Area Type Detected, supported options are: "
        "protection_1 = 0, protection_2 = 1, warning_1 = 2, warning_2 = 3. Detected: "
        << reference_area);
    }
    else
    {
      params.reference_safety_area = static_cast<protocol::EYRAreaType>(reference_area);
    }
  }

  params.configure_attempts = node->declare_parameter<int>("configure_attempts", params.configure_attempts);

  return params;
}

}  // namespace uam
