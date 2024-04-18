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
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <urg_node_msgs/Status.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Trigger.h>

#include <algorithm>
#include <atomic>
#include <limits>
#include <mutex>
#include <string>
#include <vector>
#include <utility>

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
  status_on_request_publisher_ = node_handle_.advertise<urg_node_msgs::Status>(params.status_topic, 1, true);
  status_on_update_publisher_ =
    node_handle_.advertise<urg_node_msgs::Status>(status_on_request_publisher_.getTopic() + "_update", 1, true);
  if (params_.provide_laser_status_service)
    request_status_service_ =
      node_handle_.advertiseService(params.request_status_service, &UamROS::statusCallback, this);

  configure_timer_ =
    node_handle_.createTimer(params_.reconfiguration_timeout, &UamROS::configureTimerCallback, this, true, false);

  points_in_area_pub_ =
    private_node_handle_.advertise<sensor_msgs::PointCloud2>(params_.points_in_safety_area_topic, 10);
  points_in_area_markers_pub_ =
    private_node_handle_.advertise<visualization_msgs::Marker>(params_.points_in_safety_area_topic + "_markers", 10);

  lidar_hard_reset_client_ = node_handle_.serviceClient<std_srvs::Trigger>(params_.lidar_hard_reset_service);
  lidar_hard_reset_client_.waitForExistence();

  last_restart_time_ = ros::Time::now();
  lidar_power_cycle_interval_ = params_.lidar_power_cycle_interval;

  // Advertise safety area publisher
  advertiseSafetyAreaPublishers();

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

void UamROS::advertiseSafetyAreaPublishers()
{
  safety_area_publishers_[protocol::EYRAreaType::protection_1] =
    private_node_handle_.advertise<sensor_msgs::LaserScan>("safety_area/protection_1", 1, true);
  safety_area_publishers_[protocol::EYRAreaType::protection_2] =
    private_node_handle_.advertise<sensor_msgs::LaserScan>("safety_area/protection_2", 1, true);
  safety_area_publishers_[protocol::EYRAreaType::warning_1] =
    private_node_handle_.advertise<sensor_msgs::LaserScan>("safety_area/warning_1", 1, true);
  safety_area_publishers_[protocol::EYRAreaType::warning_2] =
    private_node_handle_.advertise<sensor_msgs::LaserScan>("safety_area/warning_2", 1, true);
  safety_area_publishers_[protocol::EYRAreaType::muting_1] =
    private_node_handle_.advertise<sensor_msgs::LaserScan>("safety_area/muting_1", 1, true);
  safety_area_publishers_[protocol::EYRAreaType::muting_2] =
    private_node_handle_.advertise<sensor_msgs::LaserScan>("safety_area/muting_2", 1, true);
  safety_area_publishers_[protocol::EYRAreaType::reference_center] =
    private_node_handle_.advertise<sensor_msgs::LaserScan>("safety_area/reference_center", 1, true);
  safety_area_publishers_[protocol::EYRAreaType::reference_max] =
    private_node_handle_.advertise<sensor_msgs::LaserScan>("safety_area/reference_max", 1, true);
  safety_area_publishers_[protocol::EYRAreaType::reference_min] =
    private_node_handle_.advertise<sensor_msgs::LaserScan>("safety_area/reference_min", 1, true);
}

void UamROS::publishSafetyArea(const size_t area_number)
{
  if (safety_areas_.empty())
    return;
  else if (!safety_areas_.count(area_number))
  {
    ROS_ERROR_STREAM("Cannot find requested area number in the database.");
    return;
  }

  for (uint16_t area_type = static_cast<uint16_t>(protocol::EYRAreaType::protection_1);
       area_type < static_cast<uint16_t>(uam::protocol::EYRAreaType::MAX);
       area_type++)
  {
    auto message = safety_areas_.at(area_number).find(static_cast<protocol::EYRAreaType>(area_type));
    if (message != safety_areas_.at(area_number).end())
    {
      safety_area_publishers_[static_cast<protocol::EYRAreaType>(area_type)].publish(message->second.second);
    }
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
  }
  if (should_reset_lidar)
  {
    ROS_WARN_STREAM("Trying to restart the lidar.");
    if (lidarHardReset())
    {
      ROS_WARN_STREAM("Lidar reset successful. Reconfiguring the lidar.");
    }
  }
  triggerReconfigure()
}

bool UamROS::lidarHardReset()
{
  if (ros::Time::now() - last_restart_time_ > lidar_power_cycle_interval_)
  {
    try
    {
      last_restart_time_ = ros::Time::now();
      std_srvs::Trigger srv;
      if (lidar_hard_reset_client_.call(srv))
      {
        if (srv.response.success)
        {
          ROS_WARN_STREAM("Lidar reset successful.");
          return true;
        }
        else
        {
          ROS_ERROR_STREAM("Lidar reset failed: " << srv.response.message);
          return false;
        }
      }
      else
      {
        ROS_ERROR_STREAM("Failed to call lidar reset service.");
        return false;
      }
    }
    catch (const std::exception& e)
    {
      ROS_ERROR_STREAM("Error while restarting the lidar: " << e.what());
      return false;
    }
  }
  else
  {
    ROS_WARN_STREAM("Lidar was restarted too recently. Skipping lidar restart.");
    return false;
  }
}

bool UamROS::statusCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
  // Check if the lidar is connected/configured
  if (!configured_)
  {
    res.success = false;
    res.message = "Status update requested but lidar is not connected!";
    return true;
  }

  // Number of scan cycles to wait for status update
  // Because we wait 1/2 of scan cycle for the status
  // Half cycle expected to be 0.03[s] / 2 = 0.015 [s]
  // 2 scan cycles -> 4 half(s) of scan
  static const uint32_t scan_cycles_timeout = 2;
  static const uint32_t half_scan_cycles_timeout = scan_cycles_timeout * 2;
  static const ros::Duration sleep_time(scan_params_.getScanPeriod() / 2);

  // Let the received thread know that we are expecting status
  publish_status_requested_ = true;

  // Number of half scan cycles waited
  uint32_t cycles_waited = 0;

  // Mimic old behaviour: status is published in the service call
  while (ros::ok() && true == publish_status_requested_.load() && cycles_waited < half_scan_cycles_timeout)
  {
    // Sleep for 1/2 scan period
    sleep_time.sleep();
    cycles_waited++;
  }

  if (!publish_status_requested_.load())
  {
    res.success = true;
    res.message = "Detailed status was successfully published";
  }
  else
  {
    ROS_WARN_STREAM_THROTTLE(
      5.0,
      "Failed to retrieved detailed status after " << scan_cycles_timeout << " scan cycles.");
    res.success = false;
    res.message = "Failed to retrieved detailed status";
  }
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

    // At this point the receiver thread is not yet running, so we can safely
    // write into scan_params
    scan_params_ = lidar_.getScanDetails();
    // Since the spinner is running this method and is also responsible to
    // run the reconfigure callback, we can safely access params_ without mutex
    scan_params_.setAngleLimits(params_.angle_min, params_.angle_max);

    // Set the reconfigure limits after fetching scan details
    updateReconfigureLimits();
    auto version_details = toString(lidar_.getVersionDetails());
    ROS_INFO_STREAM("Sensor details: " << version_details);

    // Read laser safety areas
    readSafetyAreas();

    // Update laser status
    updateStatus(lidar_.getSensorStatus(), true);

    // Start streaming
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

void UamROS::publishPointsInArea(const std::vector<std::pair<size_t, float>>& ranges, const ros::Time& stamp)
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

    auto buildCloudStaticFields = [](const std::string& frame_id) -> sensor_msgs::PointCloud2
    {
      // Fill metadata
      sensor_msgs::PointCloud2 cloud_out;
      cloud_out.header.frame_id = frame_id;
      cloud_out.height = 1u;
      cloud_out.is_bigendian = false;
      cloud_out.is_dense = true;

      // Setup fields
      auto offset = 0;
      offset = addPointField(cloud_out, "x", 1, sensor_msgs::PointField::FLOAT32, offset);
      offset = addPointField(cloud_out, "y", 1, sensor_msgs::PointField::FLOAT32, offset);
      offset = addPointField(cloud_out, "z", 1, sensor_msgs::PointField::FLOAT32, offset);
      offset += sizeOfPointField(sensor_msgs::PointField::FLOAT32);
      cloud_out.point_step = offset;
      return cloud_out;
    };

    auto getRayMarker = [](const std::string& frame_id, const std::string& ns)
    {
      visualization_msgs::Marker marker;
      marker.header.frame_id = frame_id;
      marker.type = visualization_msgs::Marker::LINE_LIST;
      marker.ns = ns;
      marker.scale.x = 0.001;
      marker.color.a = 1.0;
      marker.color.r = 1.0;
      return marker;
    };

    // Compute the Lookup table of sen and cos
    static LaserScanCachedLookup lut = getLut(scan_params_);
    // Get the cloud, we can re-use it and clear the points
    static sensor_msgs::PointCloud2 cloud_out = buildCloudStaticFields(params_.frame_id);
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
      geometry_msgs::Point pt;
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
      marker.action = visualization_msgs::Marker::ADD;
    }
    else
    {
      marker.action = visualization_msgs::Marker::DELETE;
    }
    points_in_area_markers_pub_.publish(marker);
    points_in_area_pub_.publish(cloud_out);
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
    urg_node_msgs::Status msg;
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
      status_on_request_publisher_.publish(msg);
      publishSafetyArea(sensing_data.area_number);
      publish_status_requested_ = false;
    }
    if (on_update_status)
    {
      status_on_update_publisher_.publish(msg);
      publishSafetyArea(sensing_data.area_number);
    }
  }
}

void UamROS::readSafetyAreas()
{
  // Areas already read
  if (!safety_areas_.empty())
    return;

  ROS_INFO_STREAM("Going to read Laser Safety areas");

  auto toSafetyArea = [this](const protocol::YRCommandReply& yr) -> ScanWithCRC
  {
    sensor_msgs::LaserScan msg;
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
    return std::make_pair(yr.footer.crc, boost::make_shared<const sensor_msgs::LaserScan>(msg));
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
  ROS_INFO_STREAM("Done reading Laser Safety areas.");
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
    ROS_INFO_STREAM("Safety area CRC: " << area_crc_array);
  }
}
}  // namespace uam
