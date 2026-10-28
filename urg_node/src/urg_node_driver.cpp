/*
 * Copyright (c) 2013, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * Author: Chad Rockey, Michael Carroll, Mike O'Driscoll
 */

#include "urg_node/urg_node_driver.hpp"

#include <memory>
#include <sstream>
#include <string>
#include <vector>

namespace urg_node
{

// Useful typedefs
typedef diagnostic_updater::FrequencyStatusParam FrequencyStatusParam;

UrgNode::UrgNode(const rclcpp::NodeOptions & node_options)
: Node("urg_node", node_options),
  diagnostic_updater_(this),
  error_count_(0),
  freq_min_(0.0),
  close_diagnostics_(true),
  close_scan_(true),
  ip_port_(10940),
  ip_address_(""),
  serial_port_("/dev/ttyACM0"),
  serial_baud_(115200),
  calibrate_time_(false),
  synchronize_time_(false),
  publish_intensity_(true),
  publish_multiecho_(false),
  error_limit_(4),
  diagnostics_tolerance_(0.05),
  diagnostics_window_time_(5.0),
  detailed_status_(false),
  frame_id_("laser"),
  time_offset_(0.0),
  range_offset_(0.0f),
  angle_min_(-3.141592),
  angle_max_(3.141592),
  cluster_(1),
  skip_(0),
  service_yield_(false)
{
  initSetup();
}

void UrgNode::initSetup()
{
  device_status_.detailed_status.error_code = 0;
  device_status_.detailed_status.lockout_state = false;

  // Declare parameters so we can change these later. Dynamic reconfiguration
  // (changing values at runtime through `ros2 param set`) is not implemented
  // yet; values are read once here and applied when connecting to the laser.
  ip_address_ = this->declare_parameter<std::string>("ip_address", ip_address_);
  ip_port_ = this->declare_parameter<int>("ip_port", ip_port_);
  serial_port_ = this->declare_parameter<std::string>("serial_port", serial_port_);
  serial_baud_ = this->declare_parameter<int>("serial_baud", serial_baud_);
  calibrate_time_ = this->declare_parameter<bool>("calibrate_time", calibrate_time_);
  synchronize_time_ = this->declare_parameter<bool>("synchronize_time", synchronize_time_);
  publish_intensity_ = this->declare_parameter<bool>("publish_intensity", publish_intensity_);
  publish_multiecho_ = this->declare_parameter<bool>("publish_multiecho", publish_multiecho_);
  error_limit_ = this->declare_parameter<int>("error_limit", error_limit_);
  diagnostics_tolerance_ = this->declare_parameter<double>(
    "diagnostics_tolerance", diagnostics_tolerance_);
  diagnostics_window_time_ = this->declare_parameter<double>(
    "diagnostics_window_time", diagnostics_window_time_);
  detailed_status_ = this->declare_parameter<bool>("get_detailed_status", detailed_status_);
  frame_id_ = this->declare_parameter<std::string>("frame_id", frame_id_);
  time_offset_ = this->declare_parameter<double>("time_offset", time_offset_);
  range_offset_ = static_cast<float>(
    this->declare_parameter<double>("range_offset", static_cast<double>(range_offset_)));
  angle_min_ = this->declare_parameter<double>("angle_min", angle_min_);
  angle_max_ = this->declare_parameter<double>("angle_max", angle_max_);
  cluster_ = this->declare_parameter<int>("cluster", cluster_);
  skip_ = this->declare_parameter<int>("skip", skip_);

  // Set up publishers and diagnostics updaters, we only need one
  if (publish_multiecho_)
  {
    echoes_pub_ = std::make_unique<laser_proc::LaserPublisher>(this->get_node_topics_interface(), 20);
  }
  else
  {
    laser_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", 20);
  }

  status_service_ = this->create_service<std_srvs::srv::Trigger>(
    "update_laser_status",
    std::bind(
      &UrgNode::statusCallback, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  status_pub_ = this->create_publisher<urg_node_msgs::msg::Status>("laser_status", 1);

  diagnostic_updater_.add("Hardware Status", this, &UrgNode::populateDiagnosticsStatus);

  parameters_callback_handle_ = this->add_on_set_parameters_callback(
    std::bind(&UrgNode::paramChangeCallback, this, std::placeholders::_1));

  // Put this in a separate thread since connecting might take a while, and a
  // Node's constructor must return promptly (e.g. when loaded as a component).
  run_thread_ = std::thread(std::bind(&UrgNode::run, this));
}

UrgNode::~UrgNode()
{
  if (run_thread_.joinable())
  {
    run_thread_.join();
  }
  if (diagnostics_thread_.joinable())
  {
    // Clean up our diagnostics thread.
    close_diagnostics_ = true;
    diagnostics_thread_.join();
  }
  if (scan_thread_.joinable())
  {
    close_scan_ = true;
    scan_thread_.join();
  }
}

bool UrgNode::updateStatus()
{
  bool result = false;
  service_yield_ = true;
  std::unique_lock<std::mutex> lock(lidar_mutex_);

  if (urg_)
  {
    device_status_.status_str = urg_->getSensorStatus();
    if (detailed_status_)
    {
      if (urg_->getAR00Status(device_status_.detailed_status))
      {
        urg_node_msgs::msg::Status msg;
        msg.operating_mode = device_status_.detailed_status.operating_mode;
        msg.error_status = device_status_.detailed_status.error_state;
        msg.error_code = device_status_.detailed_status.error_code;
        msg.lockout_status = device_status_.detailed_status.lockout_state;
        msg.area_number = device_status_.detailed_status.area_number;
        msg.ossd1_state = device_status_.detailed_status.ossd1_state;
        msg.ossd2_state = device_status_.detailed_status.ossd2_state;
        msg.warning1_state = device_status_.detailed_status.warning1_state;
        msg.warning2_state = device_status_.detailed_status.warning2_state;
        msg.optical_window_contaminated = device_status_.detailed_status.optical_window_contaminated;
        // TODO(carlos-m159): Detection log should not be requested together with status.
        // For now keep the old code commented out.
        // UrgDetectionReport report;
        // if (urg_->getDL00Status(report))
        // {
        //   msg.area_number = report.area;
        //   msg.distance = report.distance;
        //   msg.angle = report.angle;
        // }
        // else
        // {
        //   RCLCPP_WARN(this->get_logger(), "Failed to get detection report.");
        // }

        // Publish the status on the latched topic for inspection.
        status_pub_->publish(msg);
        result = true;
      }
      else
      {
        RCLCPP_WARN(this->get_logger(), "Failed to retrieve status");
      }
    }
  }
  return result;
}

void UrgNode::statusCallback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std_srvs::srv::Trigger::Request::SharedPtr req,
  const std_srvs::srv::Trigger::Response::SharedPtr res)
{
  (void) request_header;
  (void) req;

  RCLCPP_INFO(this->get_logger(), "Got update lidar status callback");
  res->success = false;
  res->message = "Laser not ready";

  if (updateStatus())
  {
    res->message = "Status retrieved";
    res->success = true;
  }
  else
  {
    res->message = "Failed to update status";
    res->success = false;
  }
}

rcl_interfaces::msg::SetParametersResult UrgNode::paramChangeCallback(
  const std::vector<rclcpp::Parameter>& parameters)
{
  // Dynamic reconfiguration is not implemented yet (see initSetup() comment);
  // this only validates that a requested change is well-formed. A follow-up
  // change will make these parameters truly dynamic.
  auto result = rcl_interfaces::msg::SetParametersResult();
  result.successful = true;

  std::stringstream reason;

  for (const auto& parameter : parameters)
  {
    const rclcpp::ParameterType type = parameter.get_type();
    const std::string& name = parameter.get_name();

    if (name == "frame_id" || name == "ip_address" || name == "serial_port")
    {
      if (type != rclcpp::ParameterType::PARAMETER_STRING)
      {
        reason << "The parameter " << name << " must be a string.\n";
        result.successful = false;
      }
    }
    else if (name == "ip_port" || name == "serial_baud" || name == "error_limit")
    {
      if (type != rclcpp::ParameterType::PARAMETER_INTEGER)
      {
        reason << "The parameter " << name << " must be an integer.\n";
        result.successful = false;
      }
    }
    else if (name == "calibrate_time" || name == "synchronize_time" ||
      name == "publish_intensity" || name == "publish_multiecho" || name == "get_detailed_status")
    {
      if (type != rclcpp::ParameterType::PARAMETER_BOOL)
      {
        reason << "The parameter " << name << " must be a bool.\n";
        result.successful = false;
      }
    }
    else if (name == "diagnostics_tolerance" || name == "diagnostics_window_time" ||
      name == "time_offset" || name == "range_offset" || name == "angle_min" || name == "angle_max")
    {
      if (type != rclcpp::ParameterType::PARAMETER_DOUBLE && type != rclcpp::ParameterType::PARAMETER_INTEGER)
      {
        reason << "The parameter " << name << " must be a double.\n";
        result.successful = false;
      }
    }
    else if (name == "cluster")
    {
      if (type != rclcpp::ParameterType::PARAMETER_INTEGER ||
        parameter.as_int() < 1 || parameter.as_int() > 99)
      {
        reason << "The parameter " << name << " must be an integer between 1 and 99.\n";
        result.successful = false;
      }
    }
    else if (name == "skip")
    {
      if (type != rclcpp::ParameterType::PARAMETER_INTEGER ||
        parameter.as_int() < 0 || parameter.as_int() > 9)
      {
        reason << "The parameter " << name << " must be an integer between 0 and 9.\n";
        result.successful = false;
      }
    }
  }
  result.reason = reason.str();
  return result;
}

void UrgNode::calibrate_time_offset()
{
  std::unique_lock<std::mutex> lock(lidar_mutex_);
  if (!urg_)
  {
    RCLCPP_DEBUG(this->get_logger(), "Unable to calibrate time offset. Not Ready.");
    return;
  }
  try
  {
    // Don't let outside interruption effect lidar offset.
    RCLCPP_INFO(this->get_logger(), "Starting calibration. This will take a few seconds.");
    RCLCPP_WARN(this->get_logger(), "Time calibration is still experimental.");
    rclcpp::Duration latency = urg_->computeLatency(10);
    RCLCPP_INFO(this->get_logger(), "Calibration finished. Latency is: %.4f.", latency.seconds());
  }
  catch (const std::runtime_error& e)
  {
    RCLCPP_FATAL(this->get_logger(), "Could not calibrate time offset: %s", e.what());
    rclcpp::sleep_for(std::chrono::seconds(1));
    rclcpp::shutdown();
  }
}

// Diagnostics update task to be run in a thread.
void UrgNode::updateDiagnostics()
{
  while (!close_diagnostics_)
  {
    diagnostic_updater_.force_update();
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }
}

// Populate a diagnostics status message.
void UrgNode::populateDiagnosticsStatus(diagnostic_updater::DiagnosticStatusWrapper& stat)
{
  if (!urg_)
  {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Not Connected");
    return;
  }

  if (!urg_->getIPAddress().empty())
  {
    stat.add("IP Address", urg_->getIPAddress());
    stat.add("IP Port", urg_->getIPPort());
  }
  else
  {
    stat.add("Serial Port", urg_->getSerialPort());
    stat.add("Serial Baud", urg_->getSerialBaud());
  }

  if (!urg_->isStarted())
  {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Not Connected: " + device_status_.status_str);
  }
  else if (!sensorStatusOk(device_status_.status_str))
  {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Abnormal status: " + device_status_.status_str);
  }
  else if (device_status_.detailed_status.error_code != 0)
  {
    stat.summaryf(
      diagnostic_msgs::msg::DiagnosticStatus::ERROR,
      "Lidar reporting error code: %X",
      device_status_.detailed_status.error_code);
  }
  else if (device_status_.detailed_status.lockout_state)
  {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Lidar locked out.");
  }
  else
  {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Streaming");
  }

  stat.add("Vendor Name", device_status_.vendor_name);
  stat.add("Product Name", device_status_.product_name);
  stat.add("Firmware Version", device_status_.firmware_version);
  stat.add("Firmware Date", device_status_.firmware_date);
  stat.add("Protocol Version", device_status_.protocol_version);
  stat.add("Device ID", device_status_.device_id);
  stat.add("Computed Latency", urg_->getComputedLatency().seconds());
  stat.add("User Time Offset", urg_->getUserTimeOffset().seconds());

  // Things not explicitly required by REP-0138, but still interesting.
  stat.add("Device Status", device_status_.status_str);
  stat.add("Scan Retrieve Error Count", error_count_);

  stat.add("Lidar Error Code", device_status_.detailed_status.error_code);
  stat.add("Locked out", device_status_.detailed_status.lockout_state);
}

bool UrgNode::connect()
{
  // Don't let external access to retrieve
  // status during the connection process.
  std::unique_lock<std::mutex> lock(lidar_mutex_);

  try
  {
    urg_.reset();  // Clear any previous connections();
    if (!ip_address_.empty())
    {
      EthernetConnection connection{ip_address_, ip_port_};
      urg_ = std::make_unique<urg_node::URGCWrapper>(
        connection, publish_intensity_, publish_multiecho_, synchronize_time_, this->get_logger());
    }
    else
    {
      SerialConnection connection{serial_port_, serial_baud_};
      urg_ = std::make_unique<urg_node::URGCWrapper>(
        connection, publish_intensity_, publish_multiecho_, synchronize_time_, this->get_logger());
    }

    std::stringstream ss;
    ss << "Connected to";
    if (publish_multiecho_)
    {
      ss << " multiecho";
    }
    if (!ip_address_.empty())
    {
      ss << " network";
    }
    else
    {
      ss << " serial";
    }
    ss << " device with";
    if (publish_intensity_)
    {
      ss << " intensity and";
    }
    ss << " ID: " << urg_->getDeviceID();
    RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());

    device_status_.status_str = urg_->getSensorStatus();
    device_status_.vendor_name = urg_->getVendorName();
    device_status_.product_name = urg_->getProductName();
    device_status_.firmware_version = urg_->getFirmwareVersion();
    device_status_.firmware_date = urg_->getFirmwareDate();
    device_status_.protocol_version = urg_->getProtocolVersion();
    device_status_.device_id = urg_->getDeviceID();

    diagnostic_updater_.setHardwareID(urg_->getDeviceID());

    // Configure initial properties. Dynamic reconfiguration (being able to
    // change these at runtime) is not implemented yet; see initSetup().

    // The publish frequency changes based on the number of skipped scans.
    // Update accordingly here.
    freq_min_ = 1.0 / (urg_->getScanPeriod() * (skip_ + 1));

    urg_->setAngleLimitsAndCluster(angle_min_, angle_max_, cluster_);
    urg_->setSkip(skip_);
    urg_->setFrameId(frame_id_);
    urg_->setUserLatency(time_offset_);
    urg_->setRangeOffset(range_offset_);

    return true;
  }
  catch (const std::runtime_error& e)
  {
    RCLCPP_ERROR(this->get_logger(), "Error connecting to Hokuyo: %s", e.what());
    urg_.reset();
    return false;
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(this->get_logger(), "Unknown error connecting to Hokuyo: %s", e.what());
    urg_.reset();
    return false;
  }

  return false;
}

void UrgNode::scanThread()
{
  while (!close_scan_)
  {
    if (!urg_)
    {
      if (!connect())
      {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        continue;  // Connect failed, sleep, try again.
      }
    }

    if (calibrate_time_)
    {
      calibrate_time_offset();
    }

    if (!urg_ || !rclcpp::ok())
    {
      continue;
    }

    // Before starting, update the status
    updateStatus();

    // Start the urgwidget
    try
    {
      // If the connection failed, don't try and connect
      // pointer is invalid.
      if (!urg_)
      {
        continue;  // Return to top of main loop, not connected.
      }
      device_status_.status_str = urg_->getSensorStatus();
      urg_->start();
      RCLCPP_INFO(this->get_logger(), "Streaming data.");
      // Clear the error count.
      error_count_ = 0;
    }
    catch (const std::runtime_error& e)
    {
      RCLCPP_ERROR(this->get_logger(), "Error starting Hokuyo: %s", e.what());
      urg_.reset();
      rclcpp::sleep_for(std::chrono::seconds(1));
      continue;  // Return to top of main loop
    }
    catch (...)
    {
      RCLCPP_ERROR(this->get_logger(), "Unknown error starting Hokuyo");
      urg_.reset();
      rclcpp::sleep_for(std::chrono::seconds(1));
      continue;  // Return to top of main loop
    }

    while (!close_scan_)
    {
      // Don't allow external access during grabbing the scan.
      try
      {
        std::unique_lock<std::mutex> lock(lidar_mutex_);
        if (publish_multiecho_)
        {
          sensor_msgs::msg::MultiEchoLaserScan msg;
          if (urg_->grabScan(msg))
          {
            echoes_pub_->publish(msg);
            echoes_freq_->tick();
          }
          else
          {
            RCLCPP_WARN(this->get_logger(), "Could not grab multi echo scan.");
            device_status_.status_str = urg_->getSensorStatus();
            error_count_++;
          }
        }
        else
        {
          sensor_msgs::msg::LaserScan msg;
          if (urg_->grabScan(msg))
          {
            laser_pub_->publish(msg);
            laser_freq_->tick();
          }
          else
          {
            RCLCPP_WARN(this->get_logger(), "Could not grab single echo scan.");
            device_status_.status_str = urg_->getSensorStatus();
            error_count_++;
          }
        }
      }
      catch (...)
      {
        RCLCPP_ERROR(this->get_logger(), "Unknown error grabbing Hokuyo scan.");
        error_count_++;
      }

      if (service_yield_)
      {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        service_yield_ = false;
      }

      // Reestablish conneciton if things seem to have gone wrong.
      if (error_count_ > error_limit_)
      {
        RCLCPP_ERROR(this->get_logger(), "Error count exceeded limit, reconnecting.");
        urg_.reset();
        rclcpp::sleep_for(std::chrono::seconds(2));

        break;  // Return to top of main loop
      }
    }
  }
}

bool UrgNode::sensorStatusOk(const std::string& status) const
{
  static std::string SENSOR_WORKS_WELL = std::string("Sensor works well.");
  static std::string SENSOR_STABLE_NO_ERROR = std::string("Stable 000 no error.");
  static std::string SENSOR_WORKING_NORMALLY = std::string("sensor is working normally");

  return (status == SENSOR_WORKS_WELL || status == SENSOR_STABLE_NO_ERROR || status == SENSOR_WORKING_NORMALLY);
}

void UrgNode::run()
{
  // Setup initial connection
  connect();

  // Stop diagnostics
  if (!close_diagnostics_)
  {
    close_diagnostics_ = true;
    diagnostics_thread_.join();
  }

  if (publish_multiecho_)
  {
    echoes_freq_ = std::make_unique<diagnostic_updater::HeaderlessTopicDiagnostic>(
      "Laser Echoes",
      diagnostic_updater_,
      FrequencyStatusParam(&freq_min_, &freq_min_, diagnostics_tolerance_, diagnostics_window_time_));
  }
  else
  {
    laser_freq_ = std::make_unique<diagnostic_updater::HeaderlessTopicDiagnostic>(
      "Laser Scan",
      diagnostic_updater_,
      FrequencyStatusParam(&freq_min_, &freq_min_, diagnostics_tolerance_, diagnostics_window_time_));
  }

  // Now that we are setup, kick off diagnostics.
  close_diagnostics_ = false;
  diagnostics_thread_ = std::thread(std::bind(&UrgNode::updateDiagnostics, this));

  // Start scanning now that everything is configured.
  close_scan_ = false;
  scan_thread_ = std::thread(std::bind(&UrgNode::scanThread, this));
}
}  // namespace urg_node

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(urg_node::UrgNode)
