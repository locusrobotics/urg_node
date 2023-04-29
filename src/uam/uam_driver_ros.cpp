/**
Software License Agreement (proprietary)
\file      uam_driver_ros.cpp
\authors   Carlos Mendes <cribeiromendes@locusrobotics.com>
\copyright Copyright (c) (2023,), Locus Robotics Corp., All rights reserved.
Unauthorized copying of this file, via any medium, is strictly prohibited.
Proprietary and confidential.
**/

#include <ros/callback_queue.h>
#include <uam/uam_driver_ros.h>

#include <atomic>
#include <mutex>
#include <string>
namespace uam
{
UamROS::UamROS(const ros::NodeHandle& node_handle, const uam::UamROSParams& params) :
  configured_(false),
  configure_attempts_(0),
  lidar_(),
  params_(params),
  node_handle_(node_handle)
{
  scan_publisher_ = node_handle_.advertise<sensor_msgs::LaserScan>(params.scan_topic, 1);

  configure_timer_ =
    node_handle.createTimer(params_.reconfiguration_timeout, &UamROS::configureTimerCallback, this, true, false);

  scan_watchdog_timer_ = node_handle.createTimer(params_.scan_timeout, &UamROS::scanSectorWatchdogTimerCallback, this);
  // Configure the lidar. On failure, start a timer to try again later.
  if (!configure())
  {
    configure_timer_.start();
  }
}

void UamROS::scanSectorWatchdogTimerCallback(const ros::TimerEvent& event)
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

/**
 * @brief Attempt to connect to and configure the lidar in response to a timer event
 */
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

/**
 * @brief Trigger reconfigure routine
 */
void UamROS::triggerReconfigure()
{
  lidar_.disconnect();
  configured_ = false;
  configure_timer_.start();
}

bool UamROS::configure()
{
  try
  {
    lidar_.connect(params_.ip_address, params_.ip_port);
    ROS_INFO_STREAM("Connected to Uam lidar.");
    auto version_details = lidar_.getVersionDetails();
    auto sensor_status = lidar_.getSensorStatus();
    ROS_INFO_STREAM("Connected to Uam lidar.");
    ROS_INFO_STREAM("Sensor details: " << version_details);
    scan_params_ = lidar_.getScanDetails();

    lidar_.registerCallback<AR01Worker>(std::bind(
      static_cast<void (UamROS::*)(const protocol::AR01CommandReply&)>(&UamROS::scanCallback),
      this,
      std::placeholders::_1));
    lidar_.registerCallback<AR06Worker>(std::bind(
      static_cast<void (UamROS::*)(const protocol::AR06CommandReply&)>(&UamROS::scanCallback),
      this,
      std::placeholders::_1));
    lidar_.registerCallback<AR00Worker>(std::bind(
      static_cast<void (UamROS::*)(const protocol::AR00CommandReply&)>(&UamROS::scanCallback),
      this,
      std::placeholders::_1));

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

void UamROS::scanCallback(const protocol::AR01CommandReply& reply)
{
  sensor_msgs::LaserScan msg;
  msg.header.frame_id = scan_params_.frame_id;
  msg.angle_min = scan_params_.angle_min;
  msg.angle_max = scan_params_.angle_max;
  msg.angle_increment = scan_params_.angle_increment;
  msg.scan_time = scan_params_.scan_period;
  msg.time_increment = scan_params_.time_increment;
  msg.range_min = scan_params_.range_min;
  msg.range_max = scan_params_.range_max;

  // Grab scan
  long time_stamp = 0;
  unsigned long long system_time_stamp = 0;

  // Fill scan
  if (true)
  {
    msg.header.stamp = ros::Time::now();
  }
  else
  {
    msg.header.stamp.fromNSec((uint64_t)system_time_stamp);
  }
  // msg.header.stamp = msg.header.stamp + system_latency_ + user_latency_ + getAngularTimeOffset();
  msg.ranges.resize(reply.ranges.size());
  msg.intensities.resize(reply.intensities.size());

  for (size_t i = 0; i < reply.ranges.size(); i++)
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
      msg.ranges[i] = /*range_offset_ +*/ static_cast<float>(reply.ranges[i]) / 1000.0f;
      msg.intensities[i] = reply.intensities[i];
    }
    else
    {
      msg.ranges[i] = std::numeric_limits<float>::quiet_NaN();
      continue;
    }
  }
  scan_publisher_.publish(msg);
}

void UamROS::scanCallback(const protocol::AR06CommandReply& scan_sector) {}

void UamROS::scanCallback(const protocol::AR00CommandReply& scan_sector) {}

}  // namespace uam
