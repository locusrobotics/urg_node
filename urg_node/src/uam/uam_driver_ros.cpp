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

#include <uam/uam_driver_ros.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <urg_node_msgs/msg/status.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <algorithm>
#include <atomic>
#include <iomanip>
#include <limits>
#include <mutex>
#include <string>
#include <vector>
#include <utility>

namespace uam
{
UamROS::UamROS(const rclcpp::NodeOptions & node_options) :
  Node("uam_driver_node", node_options),
  configured_(false),
  configure_attempts_(0),
  lidar_(this->get_logger()),
  params_(UamROSParams::loadFromROS(this)),
  scan_stamp_(0, 0, RCL_ROS_TIME),
  configured_stamp_(0, 0, RCL_ROS_TIME),
  discard_data_until_(0, 0, RCL_ROS_TIME),
  publish_status_requested_(false),
  logger_(this->get_logger())
{
  // Latched (transient-local) QoS, to emulate the ROS1 latched topics for status/safety area publishers.
  const auto latched_qos = rclcpp::QoS(1).transient_local();

  scan_publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>(params_.scan_topic, 1);
  status_on_request_publisher_ =
    this->create_publisher<urg_node_msgs::msg::Status>(params_.status_topic, latched_qos);
  status_on_update_publisher_ =
    this->create_publisher<urg_node_msgs::msg::Status>(params_.status_topic + "_update", latched_qos);
  if (params_.provide_laser_status_service)
  {
    request_status_service_ = this->create_service<std_srvs::srv::Trigger>(
      params_.request_status_service,
      std::bind(
        &UamROS::statusCallback, this,
        std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  }

  configure_timer_ = this->create_wall_timer(
    std::chrono::duration<double>(params_.reconfiguration_timeout.seconds()),
    std::bind(&UamROS::configureTimerCallback, this),
    nullptr,
    false);

  points_in_area_pub_ =
    this->create_publisher<sensor_msgs::msg::PointCloud2>(params_.points_in_safety_area_topic, 10);
  points_in_area_markers_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
    params_.points_in_safety_area_topic + "_markers", 10);

  // lidar_hard_reset_client_ uses its own reentrant callback group so its
  // response can be processed while lidarHardReset() blocks waiting for it
  // from a timer callback. This node must be spun with a
  // rclcpp::executors::MultiThreadedExecutor for that to work.
  lidar_hard_reset_callback_group_ =
    this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  lidar_hard_reset_client_ = this->create_client<std_srvs::srv::Trigger>(
    params_.lidar_hard_reset_service, rclcpp::ServicesQoS(), lidar_hard_reset_callback_group_);
  lidar_hard_reset_client_->wait_for_service();

  // Advertise safety area publisher
  advertiseSafetyAreaPublishers();

  scan_watchdog_timer_ = this->create_wall_timer(
    std::chrono::duration<double>(params_.scan_timeout.seconds()),
    std::bind(&UamROS::scanWatchdogTimerCallback, this));

  lidar_.registerCallback<AR01Worker>(std::bind(
    static_cast<void (UamROS::*)(const protocol::AR01CommandReply&, const rclcpp::Time&)>(
      &UamROS::scanCallback<AR01Worker::Reply>),
    this,
    std::placeholders::_1,
    std::placeholders::_2));
  lidar_.registerCallback<AR06Worker>(std::bind(
    static_cast<void (UamROS::*)(const protocol::AR06CommandReply&, const rclcpp::Time&)>(
      &UamROS::scanCallback<AR06Worker::Reply>),
    this,
    std::placeholders::_1,
    std::placeholders::_2));
  lidar_.registerCallback<AR00Worker>(std::bind(
    static_cast<void (UamROS::*)(const protocol::AR00CommandReply&, const rclcpp::Time&)>(
      &UamROS::scanCallback<AR00Worker::Reply>),
    this,
    std::placeholders::_1,
    std::placeholders::_2));

  // Configure the lidar. On failure, start a timer to try again later.
  if (!configure())
  {
    configure_timer_->reset();
  }
}

void UamROS::advertiseSafetyAreaPublishers()
{
  const auto latched_qos = rclcpp::QoS(1).transient_local();
  safety_area_publishers_[protocol::EYRAreaType::protection_1] =
    this->create_publisher<sensor_msgs::msg::LaserScan>("safety_area/protection_1", latched_qos);
  safety_area_publishers_[protocol::EYRAreaType::protection_2] =
    this->create_publisher<sensor_msgs::msg::LaserScan>("safety_area/protection_2", latched_qos);
  safety_area_publishers_[protocol::EYRAreaType::warning_1] =
    this->create_publisher<sensor_msgs::msg::LaserScan>("safety_area/warning_1", latched_qos);
  safety_area_publishers_[protocol::EYRAreaType::warning_2] =
    this->create_publisher<sensor_msgs::msg::LaserScan>("safety_area/warning_2", latched_qos);
  safety_area_publishers_[protocol::EYRAreaType::muting_1] =
    this->create_publisher<sensor_msgs::msg::LaserScan>("safety_area/muting_1", latched_qos);
  safety_area_publishers_[protocol::EYRAreaType::muting_2] =
    this->create_publisher<sensor_msgs::msg::LaserScan>("safety_area/muting_2", latched_qos);
  safety_area_publishers_[protocol::EYRAreaType::reference_center] =
    this->create_publisher<sensor_msgs::msg::LaserScan>("safety_area/reference_center", latched_qos);
  safety_area_publishers_[protocol::EYRAreaType::reference_max] =
    this->create_publisher<sensor_msgs::msg::LaserScan>("safety_area/reference_max", latched_qos);
  safety_area_publishers_[protocol::EYRAreaType::reference_min] =
    this->create_publisher<sensor_msgs::msg::LaserScan>("safety_area/reference_min", latched_qos);
}

void UamROS::publishSafetyArea(const size_t area_number)
{
  if (safety_areas_.empty())
    return;
  else if (!safety_areas_.count(area_number))
  {
    LOCUS_LOG_ERROR_STREAM(logger_, "Cannot find requested area number in the database.");
    return;
  }

  for (uint16_t area_type = static_cast<uint16_t>(protocol::EYRAreaType::protection_1);
       area_type < static_cast<uint16_t>(uam::protocol::EYRAreaType::MAX);
       area_type++)
  {
    auto message = safety_areas_.at(area_number).find(static_cast<protocol::EYRAreaType>(area_type));
    if (message != safety_areas_.at(area_number).end())
    {
      safety_area_publishers_[static_cast<protocol::EYRAreaType>(area_type)]->publish(*message->second.second);
    }
  }
}

void UamROS::scanWatchdogTimerCallback()
{
  // If the timeout has expired, try to reconnect to the lidar
  std::lock_guard<std::mutex> lock(watchdog_mutex_);
  const rclcpp::Time now = this->get_clock()->now();
  const rclcpp::Time last_activity = std::max(configured_stamp_, scan_stamp_);
  const rclcpp::Duration elapsed_time = now - last_activity;
  if (configured_ && elapsed_time > params_.scan_timeout)
  {
    LOCUS_LOG_WARN_STREAM(logger_,
      "No scan sector messages have been received from Hokuyo lidar ("
      << params_.ip_address << ") in the last " << std::setprecision(3) << elapsed_time.seconds() << " seconds (since "
      << scan_stamp_.nanoseconds() << "). Resetting the lidar.");

    auto lidar_reset_done = lidarHardReset();
    LOCUS_LOG_WARN_STREAM_COND(
      logger_, !lidar_reset_done, "Lidar reset skipped. Check logs! Only reconfigure will be performed.");
    triggerReconfigure();
  }
}

bool UamROS::lidarHardReset()
{
  if (
    !last_restart_time_.has_value() ||
    this->get_clock()->now() - last_restart_time_.value() > params_.lidar_power_cycle_interval)
  {
    try
    {
      last_restart_time_ = this->get_clock()->now();
      auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
      auto future = lidar_hard_reset_client_->async_send_request(request);
      // Blocks this call until the response is ready. The response is delivered by the
      // lidar_hard_reset_callback_group_ (a Reentrant group), which the MultiThreadedExecutor
      // spinning this node can service concurrently with this (blocked) callback.
      if (future.wait_for(std::chrono::seconds(10)) == std::future_status::ready)
      {
        auto response = future.get();
        if (response->success)
        {
          LOCUS_LOG_WARN_STREAM(logger_, "Lidar reset successful.");
          return true;
        }
        else
        {
          LOCUS_LOG_ERROR_STREAM(logger_, "Lidar reset failed: " << response->message);
          return false;
        }
      }
      else
      {
        LOCUS_LOG_ERROR_STREAM(logger_, "Failed to call lidar reset service.");
        return false;
      }
    }
    catch (const std::exception& e)
    {
      LOCUS_LOG_ERROR_STREAM(logger_, "Error while restarting the lidar: " << e.what());
      return false;
    }
  }
  else
  {
    LOCUS_LOG_WARN_STREAM(logger_, "Lidar was restarted too recently. Skipping lidar restart.");
    return false;
  }
}

void UamROS::statusCallback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std_srvs::srv::Trigger::Request::SharedPtr req,
  const std_srvs::srv::Trigger::Response::SharedPtr res)
{
  (void) request_header;
  (void) req;

  // Check if the lidar is connected/configured
  if (!configured_)
  {
    res->success = false;
    res->message = "Status update requested but lidar is not connected!";
    return;
  }

  // Number of scan cycles to wait for status update
  // Because we wait 1/2 of scan cycle for the status
  // Half cycle expected to be 0.03[s] / 2 = 0.015 [s]
  // 2 scan cycles -> 4 half(s) of scan
  static const uint32_t scan_cycles_timeout = 2;
  static const uint32_t half_scan_cycles_timeout = scan_cycles_timeout * 2;
  const auto sleep_time = std::chrono::duration<double>(scan_params_.getScanPeriod() / 2);

  // Let the received thread know that we are expecting status
  publish_status_requested_ = true;

  // Number of half scan cycles waited
  uint32_t cycles_waited = 0;

  // Mimic old behaviour: status is published in the service call
  while (rclcpp::ok() && true == publish_status_requested_.load() && cycles_waited < half_scan_cycles_timeout)
  {
    // Sleep for 1/2 scan period
    std::this_thread::sleep_for(sleep_time);
    cycles_waited++;
  }

  if (!publish_status_requested_.load())
  {
    res->success = true;
    res->message = "Detailed status was successfully published";
  }
  else
  {
    LOCUS_LOG_WARN_STREAM_THROTTLE(logger_,
      5.0,
      "Failed to retrieved detailed status after " << scan_cycles_timeout << " scan cycles.");
    res->success = false;
    res->message = "Failed to retrieved detailed status";
  }
}

void UamROS::configureTimerCallback()
{
  // Stop the timer so that it may be restarted later (ROS2 wall timers are
  // periodic, so we emulate the original one-shot timer by cancelling it here
  // and only resetting it below if configuration is still not successful).
  configure_timer_->cancel();

  // Try to configure the lidar again
  if (!configured_ && !configure())
  {
    // Configuration failed. Restart the timer to try again.
    configure_timer_->reset();
  }
}

void UamROS::triggerReconfigure()
{
  lidar_.disconnect();
  configured_ = false;
  configure_timer_->reset();
}

bool UamROS::configure()
{
  try
  {
    lidar_.connect(params_.ip_address, params_.ip_port);
    LOCUS_LOG_INFO_STREAM(logger_, "Connected to Uam lidar.");

    // TODO(cribeiromendes): make scip commands work seamlessly. Right now we
    // need to ask this before starting continuous async reads

    // At this point the receiver thread is not yet running, so we can safely
    // write into scan_params
    scan_params_ = lidar_.getScanDetails();
    scan_params_.setAngleLimits(params_.angle_min, params_.angle_max);

    auto version_details = toString(lidar_.getVersionDetails());
    LOCUS_LOG_INFO_STREAM(logger_, "Sensor details: " << version_details);

    // Read laser safety areas
    readSafetyAreas();

    // Update laser status
    updateStatus(lidar_.getSensorStatus(), true);

    // Set discard deadline before starting streaming to ensure no startup packets slip through
    {
      std::lock_guard<std::mutex> lock(watchdog_mutex_);
      configured_stamp_ = this->get_clock()->now();
      if (params_.discard_startup_data_s > 0.0)
      {
        discard_data_until_ = configured_stamp_ + rclcpp::Duration::from_seconds(params_.discard_startup_data_s);
        LOCUS_LOG_WARN_STREAM(logger_,
          "Discarding UAM scan data for " << params_.discard_startup_data_s << " seconds after startup/reconnect.");
      }
      else
      {
        discard_data_until_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
      }
    }

    // Start streaming
    lidar_.startStreaming(params_.use_intensity, params_.use_multi_echo);
    configure_attempts_ = 0;
    configured_ = true;
  }
  catch (const std::exception& e)
  {
    LOCUS_LOG_ERROR_STREAM(logger_, "Error while configuring the lidar: " << e.what());
    lidar_.disconnect();
    configure_attempts_++;
    if (configure_attempts_ > static_cast<uint32_t>(params_.configure_attempts))
    {
      LOCUS_LOG_WARN_STREAM(logger_, "Could not configure lidar after " << configure_attempts_ << " attempts. Triggering hard reset");
      if (lidarHardReset())
      {
        configure_attempts_ = 0;
      }
    }
    configured_ = false;
  }
  return configured_;
}

void UamROS::publishPointsInArea(const std::vector<std::pair<size_t, float>>& ranges, const rclcpp::Time& stamp)
  {
    auto getLut = [this](const ScanParameters& scan_params)
    {
      LaserScanCachedLookup lut;
      lut.scan_angle_min = scan_params_.getAngleMinLimit();
      lut.scan_angle_max = scan_params_.getAngleMaxLimit();
      lut.scan_angle_increment = scan_params_.getAngleIncrement();

      auto number_of_steps = scan_params_.getLastStep() - scan_params_.getFirstStep() + 1;
      lut.sin_lookup.reserve(number_of_steps);
      lut.cos_lookup.reserve(number_of_steps);

      for (size_t i = 0; i < number_of_steps; ++i)
      {
        const float angle = lut.scan_angle_min + static_cast<float>(i) * lut.scan_angle_increment;
        lut.sin_lookup.emplace_back(::sinf(angle));
        lut.cos_lookup.emplace_back(::cosf(angle));
      }
      return lut;
    };

    auto buildCloudStaticFields = [](const std::string& frame_id) -> sensor_msgs::msg::PointCloud2
    {
      // Fill metadata
      sensor_msgs::msg::PointCloud2 cloud_out;
      cloud_out.header.frame_id = frame_id;
      cloud_out.height = 1u;
      cloud_out.is_bigendian = false;
      cloud_out.is_dense = true;

      // Setup fields (x, y, z as packed float32, matching the previous manual addPointField calls)
      sensor_msgs::PointCloud2Modifier modifier(cloud_out);
      modifier.setPointCloud2FieldsByString(1, "xyz");
      return cloud_out;
    };

    auto getRayMarker = [](const std::string& frame_id, const std::string& ns)
    {
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = frame_id;
      marker.type = visualization_msgs::msg::Marker::LINE_LIST;
      marker.ns = ns;
      marker.scale.x = 0.001;
      marker.color.a = 1.0;
      marker.color.r = 1.0;
      return marker;
    };

    // Compute the Lookup table of sen and cos
    static LaserScanCachedLookup lut = getLut(scan_params_);
    // Get the cloud, we can re-use it and clear the points
    static sensor_msgs::msg::PointCloud2 cloud_out = buildCloudStaticFields(params_.frame_id);
    cloud_out.header.stamp = stamp;
    cloud_out.data.clear();
    sensor_msgs::PointCloud2Modifier modifier(cloud_out);
    cloud_out.width = ranges.size();
    modifier.resize(ranges.size());

    // Get the marker
    auto marker = getRayMarker(params_.frame_id, params_.frame_id);
    marker.header.stamp = stamp;
    marker.points.reserve(ranges.size() * 2);

    // Create pntcld2 iterator
    sensor_msgs::PointCloud2Iterator<float> out_x(cloud_out, "x");
    // Iterate over the dtz ranges
    for (const auto& [index, range] : ranges)
    {
      // Get x and y for the pair index + range(m)
      geometry_msgs::msg::Point pt;
      pt.x = 0;
      pt.y = 0;
      marker.points.emplace_back(pt);
      pt.x = static_cast<double>(range * lut.cos_lookup[index]);
      pt.y = static_cast<double>(range * lut.sin_lookup[index]);
      marker.points.emplace_back(pt);
      out_x[0] = pt.x;
      out_x[1] = pt.y;
      out_x[2] = 0.;
      // Increment Pntcld counter
      ++out_x;
    }

    if (!marker.points.empty())
    {
      marker.action = visualization_msgs::msg::Marker::ADD;
    }
    else
    {
      marker.action = visualization_msgs::msg::Marker::DELETE;
    }
    points_in_area_markers_pub_->publish(marker);
    points_in_area_pub_->publish(cloud_out);
  };

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

  const bool on_request_status = override_check || publish_status_requested_.load();
  const bool on_update_status = override_check || !equal(last_received_status_, sensing_data);

  if (on_request_status || on_update_status)
  {
    last_received_status_ = sensing_data;
    urg_node_msgs::msg::Status msg;
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
    if (on_request_status)
    {
      status_on_request_publisher_->publish(msg);
      publishSafetyArea(sensing_data.area_number);
      publish_status_requested_ = false;
    }
    if (on_update_status)
    {
      status_on_update_publisher_->publish(msg);
      publishSafetyArea(sensing_data.area_number);
    }
  }
}

void UamROS::readSafetyAreas()
{
  // Areas already read
  if (!safety_areas_.empty())
    return;

  LOCUS_LOG_INFO_STREAM(logger_, "Going to read Laser Safety areas");

  auto toSafetyArea = [this](const protocol::YRCommandReply& yr) -> ScanWithCRC
  {
    sensor_msgs::msg::LaserScan msg;
    msg.header.frame_id = params_.frame_id;
    msg.angle_min = scan_params_.getAngleMinLimit();
    msg.angle_max = scan_params_.getAngleMaxLimit();
    msg.angle_increment = scan_params_.getAngleIncrement();
    msg.scan_time = scan_params_.getScanPeriod();
    msg.time_increment = scan_params_.getTimeIncrement();
    msg.range_min = scan_params_.getRangeMin();
    msg.range_max = scan_params_.getRangeMax();

    msg.ranges.reserve(yr.area_data.size());
    std::transform(
      yr.area_data.begin(),
      yr.area_data.end(),
      std::back_inserter(msg.ranges),
      [](const auto& range)
      { return (range < 0x7FFF) ? static_cast<float>(range) / 1000.0f : std::numeric_limits<float>::quiet_NaN(); });
    return std::make_pair(yr.footer.crc, std::make_shared<const sensor_msgs::msg::LaserScan>(msg));
  };

  decltype(safety_areas_) cached_areas;
  for (size_t area_number = 1; area_number <= protocol::c_max_safety_area_index; area_number++)
  {
    for (uint16_t area_type = static_cast<uint16_t>(protocol::EYRAreaType::protection_1);
         area_type < static_cast<uint16_t>(uam::protocol::EYRAreaType::MAX);
         area_type++)
    {
      auto yr_area = lidar_.getSafetyArea(
        static_cast<uam::protocol::EYRAreaType>(area_type),
        area_number,
        0,
        uam::protocol::c_nr_ranges);
      if (yr_area.has_value())
      {
        cached_areas[area_number][static_cast<uam::protocol::EYRAreaType>(area_type)] = toSafetyArea(*yr_area);
      }
    }
  }
  // We got the complete list of areas, swap
  std::swap(cached_areas, safety_areas_);
  LOCUS_LOG_INFO_STREAM(logger_, "Done reading Laser Safety areas.");
  if (params_.log_safety_areas_crc)
  {
    std::string area_crc_array = "";
    for (const auto& [area_number, area] : safety_areas_)
    {
      for (const auto& [area_type, safety_area] : area)
      {
        area_crc_array += std::to_string(area_number) + "," + std::to_string(static_cast<uint16_t>(area_type)) + "," +
                          std::to_string(safety_area.first);
      }
      area_crc_array += ";";
    }
    LOCUS_LOG_INFO_STREAM(logger_, "Safety area CRC: " << area_crc_array);
  }
}
}  // namespace uam

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(uam::UamROS)
