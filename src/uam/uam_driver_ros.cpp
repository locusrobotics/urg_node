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

#include <urg_node/Status.h>

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
  status_publisher_ = node_handle_.advertise<urg_node::Status>(params.status_topic, 1, true);

  configure_timer_ =
    node_handle.createTimer(params_.reconfiguration_timeout, &UamROS::configureTimerCallback, this, true, false);

  scan_watchdog_timer_ = node_handle.createTimer(params_.scan_timeout, &UamROS::scanWatchdogTimerCallback, this);
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

bool UamROS::configure()
{
  try
  {
    lidar_.connect(params_.ip_address, params_.ip_port);
    ROS_INFO_STREAM("Connected to Uam lidar.");

    //TODO (cribeiromendes): make scip commands work seamlessly. Right now we
    // need to ask this before starting continuous async reads
    scan_params_ = lidar_.getScanDetails();
    std::this_thread::sleep_for(std::chrono::seconds(1));
    auto version_details = lidar_.getVersionDetails();
    ROS_INFO_STREAM("Sensor details: " << version_details);
    updateStatus(lidar_.getSensorStatus(), true);

    lidar_.registerCallback<AR01Worker>(std::bind(
      static_cast<void (UamROS::*)(const protocol::AR01CommandReply&, const ros::Time&)>(&UamROS::scanCallback),
      this,
      std::placeholders::_1,
      std::placeholders::_2));
    lidar_.registerCallback<AR06Worker>(std::bind(
      static_cast<void (UamROS::*)(const protocol::AR06CommandReply&, const ros::Time&)>(&UamROS::scanCallback),
      this,
      std::placeholders::_1,
      std::placeholders::_2));
    lidar_.registerCallback<AR00Worker>(std::bind(
      static_cast<void (UamROS::*)(const protocol::AR00CommandReply&, const ros::Time&)>(&UamROS::scanCallback),
      this,
      std::placeholders::_1,
      std::placeholders::_2));

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


void UamROS::scanCallback(const protocol::AR01CommandReply& reply, const ros::Time& wall_time)
{
  {
    std::lock_guard<std::mutex> lock(watchdog_mutex_);
    scan_stamp_ = ros::Time::now();
  }
  sensor_msgs::LaserScan msg;
  msg.header.frame_id = params_.frame_id;
  msg.angle_min = scan_params_.getAngleMin();
  msg.angle_max = scan_params_.getAngleMax();
  msg.angle_increment = scan_params_.getAngleIncrement();
  msg.scan_time = scan_params_.getScanPeriod();
  msg.time_increment = scan_params_.getTimeIncrement();
  msg.range_min = scan_params_.getRangeMin();
  msg.range_max = scan_params_.getRangeMax();

  // Grab scan
  msg.header.stamp = wall_time;
  msg.header.stamp = msg.header.stamp + params_.time_offset + ros::Duration(scan_params_.getAngularTimeOffset());

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
  updateStatus(reply.sensing_data);
  scan_publisher_.publish(msg);
}

void UamROS::scanCallback(const protocol::AR06CommandReply& scan_sector, const ros::Time& wall_time) {}

void UamROS::scanCallback(const protocol::AR00CommandReply& scan_sector, const ros::Time& wall_time) {}

void UamROS::updateStatus(const protocol::sensing_data::SensingDataHeader& sensing_data, const bool override_check)
{
  // We have POD structures, not sure if we should add operator overload to them.
  // Since we are making sure that the structure does not contain any padding bytes (pragma pack(1))
  // we can use memcmp to compare value
  if (override_check ||
    (0 != std::memcmp(&last_received_status_, &sensing_data, sizeof(protocol::sensing_data::SensingDataHeader))))
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
  }
}

}  // namespace uam
