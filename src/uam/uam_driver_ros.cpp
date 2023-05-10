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

#include <ros/callback_queue.h>
#include <uam/uam_driver_ros.h>

#include <urg_node/URGConfig.h>

#include <urg_node/Status.h>

#include <algorithm>
#include <atomic>
#include <limits>
#include <mutex>
#include <string>

namespace uam
{
UamROS::UamROS(const ros::NodeHandle& nh, const ros::NodeHandle& nh_prv, const UamROSParams& params) :
  configured_(false),
  configure_attempts_(0),
  lidar_(),
  params_(params),
  node_handle_(nh),
  private_node_handle_(nh_prv),
  publish_status_requested_(false),
  params_changed_(false)
{
  scan_publisher_ = node_handle_.advertise<sensor_msgs::LaserScan>(params.scan_topic, 1);
  status_publisher_ = node_handle_.advertise<urg_node::Status>(params.status_topic, 1, true);
  if (params_.provide_laser_status_service)
    request_status_service_ =
      node_handle_.advertiseService(params.request_status_service, &UamROS::statusCallback, this);

  configure_timer_ =
    node_handle_.createTimer(params_.reconfiguration_timeout, &UamROS::configureTimerCallback, this, true, false);

  // Clear the dynamic reconfigure server
  srv_.reset(new dynamic_reconfigure::Server<urg_node::URGConfig>(private_node_handle_));
  srv_->setCallback(boost::bind(&UamROS::dynamicReconfigureCallback, this, _1, _2));

  scan_watchdog_timer_ = node_handle_.createTimer(params_.scan_timeout, &UamROS::scanWatchdogTimerCallback, this);

  lidar_.registerCallback<AR01Worker>(std::bind(
    static_cast<void (UamROS::*)(const protocol::AR01CommandReply&, const ros::Time&)>(
      &UamROS::scanCallback<AR01Worker::Reply>),
    this,
    std::placeholders::_1,
    std::placeholders::_2));
  lidar_.registerCallback<AR06Worker>(std::bind(
    static_cast<void (UamROS::*)(const protocol::AR06CommandReply&, const ros::Time&)>(
      &UamROS::scanCallback<AR06Worker::Reply>),
    this,
    std::placeholders::_1,
    std::placeholders::_2));
  lidar_.registerCallback<AR00Worker>(std::bind(
    static_cast<void (UamROS::*)(const protocol::AR00CommandReply&, const ros::Time&)>(
      &UamROS::scanCallback<AR00Worker::Reply>),
    this,
    std::placeholders::_1,
    std::placeholders::_2));

  // Configure the lidar. On failure, start a timer to try again later.
  if (!configure())
  {
    configure_timer_.start();
  }
}

void UamROS::scanWatchdogTimerCallback(const ros::TimerEvent& event)
{
  // If the timeout has expired, try to reconnect to the lidar
  std::lock_guard<std::mutex> lock(watchdog_mutex_);
  bool should_reset_lidar = false;
  auto elapsed_time = event.current_real - std::max(configured_stamp_, scan_stamp_);
  if (configured_ && elapsed_time > params_.scan_timeout)
  {
    ROS_WARN_STREAM(
      "No scan sector messages have been received in the last " << std::setprecision(3) << elapsed_time.toSec()
                                                                << " seconds (since " << scan_stamp_
                                                                << "). Resetting the lidar.");
    should_reset_lidar = true;
    ROS_WARN_STREAM("Scan sector watchdog found an issue. Trying to reconnect to lidar.");
    triggerReconfigure();
  }
}

bool UamROS::statusCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
  publish_status_requested_ = true;
  res.success = true;
  res.message = "Status update requested";
  return true;
}

void UamROS::configureTimerCallback(const ros::TimerEvent& event)
{
  // Stop the timer so that it may be restarted later
  configure_timer_.stop();

  // Try to configure the lidar again
  if (!configured_ && !configure())
  {
    // Configuration failed. Restart the timer to try again.
    configure_timer_.start();
  }
}

void UamROS::triggerReconfigure()
{
  lidar_.disconnect();
  configured_ = false;
  configure_timer_.start();
}

void UamROS::updateReconfigureLimits()
{
  urg_node::URGConfig min, max;
  srv_->getConfigMin(min);
  srv_->getConfigMax(max);

  min.angle_min = scan_params_.getAngleMinLimit();
  min.angle_max = min.angle_min;
  max.angle_max = scan_params_.getAngleMaxLimit();
  max.angle_min = max.angle_max;

  srv_->setConfigMin(min);
  srv_->setConfigMax(max);
}

bool UamROS::configure()
{
  try
  {
    lidar_.connect(params_.ip_address, params_.ip_port);
    ROS_INFO_STREAM("Connected to Uam lidar.");

    // TODO(cribeiromendes): make scip commands work seamlessly. Right now we
    // need to ask this before starting continuous async reads
    scan_params_ = lidar_.getScanDetails();
    // Set the reconfigure limits after fetching scan details
    updateReconfigureLimits();
    auto version_details = lidar_.getVersionDetails();
    ROS_INFO_STREAM("Sensor details: " << version_details);
    updateStatus(lidar_.getSensorStatus(), true);

    lidar_.startStreaming(params_.use_intensity, params_.use_multi_echo);
    {
      std::lock_guard<std::mutex> lock(watchdog_mutex_);
      configured_stamp_ = ros::Time::now();
    }
    configure_attempts_ = 0;
    configured_ = true;
  }
  catch (const std::exception& e)
  {
    ROS_ERROR_STREAM("Error while configuring the lidar: " << e.what());
    lidar_.disconnect();
    configure_attempts_++;
    configured_ = false;
  }
  return configured_;
}

template <>
void UamROS::fillScanMessageData(
  const protocol::AR01CommandReply& reply,
  const double range_offset,
  sensor_msgs::LaserScan& scan) const
{
  scan.ranges.resize(reply.ranges.size());
  scan.intensities.resize(reply.intensities.size());

  // Filter out steps
  auto first_step = scan_params_.getFirstStep();
  auto last_step = scan_params_.getLastStep();

  if (reply.ranges.size() <= last_step)
  {
    ROS_ERROR_STREAM("Unexpected outcome: " << reply.ranges.size() << " and last_step is " << last_step);
    return;
  }

  for (size_t i = first_step; i <= last_step; i++)
  {
    auto& range = reply.ranges[i];
    auto& intensity = reply.intensities[i];

    // According to the doc:
    // 1. Values more than 40000 are error code (0xFFFF).
    // 2. If object is not detected value will be 65534 (0xFFFE)
    // 3. If object is at a very close range the value will be 65533 (0xFFFD).
    // 4. When the device is in laser off state the value will be 65532 (0xFFFC)
    if (range != 0 && range < 0xFFFC)
    {
      scan.ranges[i] = range_offset + static_cast<float>(reply.ranges[i]) / 1000.0f;
      scan.intensities[i] = reply.intensities[i];
    }
    else
    {
      scan.ranges[i] = std::numeric_limits<float>::quiet_NaN();
    }
  }
}

template <>
void UamROS::fillScanMessageData(
  const protocol::AR00CommandReply& reply,
  const double range_offset,
  sensor_msgs::LaserScan& scan) const
{
  scan.ranges.resize(reply.ranges.size());

  // Filter out steps
  auto first_step = scan_params_.getFirstStep();
  auto last_step = scan_params_.getLastStep();

  if (reply.ranges.size() <= last_step)
  {
    ROS_ERROR_STREAM("Unexpected outcome: " << reply.ranges.size() << " and last_step is " << last_step);
    return;
  }

  for (size_t i = first_step; i <= last_step; i++)
  {
    auto& range = reply.ranges[i];

    // According to the doc:
    // 1. Values more than 40000 are error code (0xFFFF).
    // 2. If object is not detected value will be 65534 (0xFFFE)
    // 3. If object is at a very close range the value will be 65533 (0xFFFD).
    // 4. When the device is in laser off state the value will be 65532 (0xFFFC)
    if (range != 0 && range < 0xFFFC)
    {
      scan.ranges[i] = range_offset + static_cast<float>(reply.ranges[i]) / 1000.0f;
    }
    else
    {
      scan.ranges[i] = std::numeric_limits<float>::quiet_NaN();
    }
  }
}

bool UamROS::dynamicReconfigureCallback(urg_node::URGConfig& config, int level)
{
  if (level < 0)
  {
    config.angle_max = params_.angle_max;
    config.angle_min = params_.angle_min;
    config.time_offset = params_.time_offset.toSec();
    config.range_offset = params_.range_offset;
    return true;
  }

  std::lock_guard<std::mutex> lock(reconfigure_mutex_);
  params_.angle_max = config.angle_max;
  params_.angle_min = config.angle_min;
  params_.time_offset = ros::Duration(config.time_offset);
  params_.range_offset = config.range_offset;
  params_changed_ = true;
  return true;
}

void UamROS::updateStatus(const protocol::sensing_data::SensingDataHeader& sensing_data, const bool override_check)
{
  // We only want to compare a subset of members, so operator overload should not
  // be done as it would not do the expected/
  const auto equal = [](
                       const protocol::sensing_data::SensingDataHeader& lhs,
                       const protocol::sensing_data::SensingDataHeader& rhs) -> bool
  {
    return lhs.area_number == rhs.area_number && lhs.error_code == rhs.error_code &&
           lhs.error_state == rhs.error_state && lhs.lockout_state == rhs.lockout_state &&
           lhs.operating_mode == rhs.operating_mode &&
           lhs.optical_window_contaminated == rhs.optical_window_contaminated && lhs.ossd1_state == rhs.ossd1_state &&
           lhs.ossd2_state == rhs.ossd2_state && lhs.warning1_state == rhs.warning1_state &&
           lhs.warning2_state == rhs.warning2_state;
  };

  if (publish_status_requested_.load() || override_check || !equal(last_received_status_, sensing_data))
  {
    last_received_status_ = sensing_data;
    urg_node::Status msg;
    msg.operating_mode = sensing_data.operating_mode;
    msg.error_status = sensing_data.error_state;
    msg.error_code = sensing_data.error_code;
    msg.lockout_status = sensing_data.lockout_state;
    msg.area_number = sensing_data.area_number;
    msg.ossd1_state = sensing_data.ossd1_state;
    msg.ossd2_state = sensing_data.ossd2_state;
    msg.warning1_state = sensing_data.warning1_state;
    msg.warning2_state = sensing_data.warning2_state;
    msg.optical_window_contaminated = sensing_data.optical_window_contaminated;
    status_publisher_.publish(msg);
    publish_status_requested_ = false;
  }
}

}  // namespace uam
