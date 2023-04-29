/**
Software License Agreement (proprietary)
\file      uam_driver.cpp
\authors   Carlos Mendes <cribeiromendes@locusrobotics.com>
\copyright Copyright (c) (2023,), Locus Robotics Corp., All rights reserved.
Unauthorized copying of this file, via any medium, is strictly prohibited.
Proprietary and confidential.
**/
#include <uam/uam_driver.h>

namespace uam
{
UamDriver::UamDriver() :
  io_service_(std::make_shared<boost::asio::io_service>()),
  client_(std::bind(&UamDriver::packetCallback, this, std::placeholders::_1), io_service_),
  uam_packet_worker_(),
  scip_pp_worker_(),
  subscription_mode_(ESubscriptionMode::INVALID)
{
  // Register the subscription handlers (AR02, AR03 ...)
  uam_packet_worker_.registerCallback<AR02Worker>(
    std::bind(&UamDriver::subscribeCallback<AR02Worker>, this, std::placeholders::_1));
  uam_packet_worker_.registerCallback<AR04Worker>(
    std::bind(&UamDriver::subscribeCallback<AR04Worker>, this, std::placeholders::_1));
  uam_packet_worker_.registerCallback<AR07Worker>(
    std::bind(&UamDriver::subscribeCallback<AR07Worker>, this, std::placeholders::_1));
  // Register the unsubscription handlers
  uam_packet_worker_.registerCallback<AR02Worker>(
    std::bind(&UamDriver::subscribeCallback<AR02Worker>, this, std::placeholders::_1));
  uam_packet_worker_.registerCallback<AR04Worker>(
    std::bind(&UamDriver::subscribeCallback<AR04Worker>, this, std::placeholders::_1));
  uam_packet_worker_.registerCallback<AR07Worker>(
    std::bind(&UamDriver::subscribeCallback<AR07Worker>, this, std::placeholders::_1));

  client_.registerFilterCallback(std::bind(&UamDriver::filterCallback, this, std::placeholders::_1));

  // Add a fake task to the io_service to prevent it from exiting until desired
  io_work_ = std::make_unique<boost::asio::io_service::work>(*io_service_);

  // Start the io_service thread that handles the UDP comms
  io_thread_ = std::thread([this]() { this->io_service_->run(); });
}

UamDriver::~UamDriver()
{
  // Cancel the fake job so that the io_service object will terminate cleanly
  io_work_.reset();

  // And disconnect from any connected sockets
  client_.disconnect();

  // Wait for the io thread to terminate cleanly
  if (io_thread_.joinable())
  {
    io_thread_.join();
  }
}

void UamDriver::connect(const std::string& ip, const uint16_t port)
{
  if (!client_.connect(ip, port))
  {
    std::stringstream ss;
    ss << "Could not connect to " << ip << " : " << port << "\n";
    throw std::runtime_error(ss.str());
  }
}

void UamDriver::disconnect()
{
  client_.disconnect();
}

std::string UamDriver::getVersionDetails()
{
  // TODO(cribeirmendes): Should we support async send?
  if (isTheSensorStreaming())
  {
    this->stopStreaming();
  }

  auto reply = syncSendCommandWithReply<uam::VR00Worker>();
  if (!reply.has_value())
  {
    std::stringstream ss;
    ss << "Could not Request Version details for Hokuyo:\n";
    throw std::runtime_error(ss.str());
  }
  return std::string(
    "Firmware_version is: " +
    std::string(reply->version_details.firmware_version.data(), reply->version_details.firmware_version.size()) +
    std::string(
      "\nSensor_model is: " +
      std::string(reply->version_details.sensor_model.data(), reply->version_details.sensor_model.size())) +
    std::string(
      "\nSerial Number is: " +
      std::string(reply->version_details.serial_number.data(), reply->version_details.serial_number.size())));
}

protocol::sensing_data::SensingDataHeader UamDriver::getSensorStatus()
{
  if (isTheSensorStreaming())
  {
    this->stopStreaming();
  }
  auto status_reply = syncSendCommandWithReply<uam::XR00Worker>();
  if (!status_reply)
  {
    std::stringstream ss;
    ss << "Could not retrieve sensor status for Hokuyo:\n";
    throw std::runtime_error(ss.str());
  }
  last_received_status_->area_number = status_reply->data.area_number;
  last_received_status_->error_code = status_reply->data.error_code;
  last_received_status_->error_state = status_reply->data.error_state;
  last_received_status_->lockout_state = status_reply->data.lockout_state;
  last_received_status_->operating_mode = status_reply->data.operating_mode;
  last_received_status_->optical_window_contaminated = status_reply->data.optical_window_contaminated;
  last_received_status_->ossd1_state = status_reply->data.ossd1_state;
  last_received_status_->ossd2_state = status_reply->data.ossd1_state;
  last_received_status_->warning1_state = status_reply->data.warning1_state;
  last_received_status_->warning2_state = status_reply->data.warning2_state;
  return *last_received_status_;
}


ScanParameters UamDriver::getScanDetails()
{
  // TODO(cribeirmendes): Should we support async send?
  if (isTheSensorStreaming())
  {
    this->stopStreaming();
  }

  auto reply = sendScipCommandWithReply<uam::PPWorker>();
  if (!reply)
  {
    std::stringstream ss;
    ss << "Could not retrieve scan detailsn";
    throw std::runtime_error(ss.str());
  }
  return convertScanParameters(*reply);
}



ScanParameters UamDriver::convertScanParameters(const uam::scip_protocol::PPReply& scan_params) const
{
  ScanParameters scan_details;
  scan_details.range_min = scan_params.min_distance / 1000.0;
  scan_details.range_max = scan_params.max_distance / 1000.0;
  auto min_step = scan_params.start_step - scan_params.front_data_index;
  auto max_step = scan_params.end_step - scan_params.front_data_index;
  scan_details.angle_increment = 2.0 * M_PI / scan_params.angular_resolution;
  scan_details.angle_min = (2.0 * M_PI) * min_step / scan_params.angular_resolution;
  scan_details.angle_max = (2.0 * M_PI) * max_step / scan_params.angular_resolution;
  scan_details.frame_id = "scan";
  scan_details.scan_period = 60.0 / static_cast<double>(scan_params.rpm);
  // Urg_c does the following for timeout
  // (1000 * 1000 * 60 / scan_params.rpm) >> (10 - 4)  which gives 468 ("milliseconds" I believe)
  // We should do 2x1/Scanning frequency (ignoring scan_skip mode for now).
  scan_details.timeout = 2 * scan_details.scan_period * 1.e3;
  auto circle_fraction = (scan_details.angle_max - scan_details.angle_min) / (2.0 * M_PI);
  scan_details.time_increment = circle_fraction * scan_details.scan_period / static_cast<double>(max_step - min_step);
  return scan_details;
}

void UamDriver::startStreaming(const bool publish_intensity, const bool publish_multiecho)
{
  if (subscription_mode_ != ESubscriptionMode::INVALID)
  {
    ROS_WARN_STREAM("Already streaming!");
    return;
  }

  if (!client_.startAsyncReadTask())
  {
    std::stringstream ss;
    ss << "Failed to start read task!";
    throw std::runtime_error(ss.str());
  }

  auto subscription_mode = publish_intensity ? ESubscriptionMode::AR04 :
                                               (publish_multiecho ? ESubscriptionMode::AR04 : ESubscriptionMode::AR02);

  switch (subscription_mode)
  {
    case ESubscriptionMode::AR02:
    {
      auto reply = asyncSend<uam::AR02Worker>();
      if (!reply)
      {
        std::stringstream ss;
        ss << "Failed to subscribe to pointcloud with AR02!";
        throw std::runtime_error(ss.str());
      }
      break;
    }
    case ESubscriptionMode::AR04:
    {
      auto reply = asyncSend<uam::AR04Worker>();
      if (!reply)
      {
        std::stringstream ss;
        ss << "Failed to subscribe to pointcloud with AR04!";
        throw std::runtime_error(ss.str());
      }
      break;
    }
    case ESubscriptionMode::AR07:
    {
      auto reply = asyncSend<uam::AR07Worker>();
      if (!reply)
      {
        std::stringstream ss;
        ss << "Failed to subscribe to pointcloud with AR07!";
        throw std::runtime_error(ss.str());
      }
      break;
    }
    default:
    {
    }
  }
  // Start command was sent, we could poll but let it go for now
  auto previous_stamp = ros::Time::now();
  auto poll_interval = ros::Duration(1);
  auto timeout_counter = 0;
  const uint32_t max_timeout_counter = 10;
  while (ros::ok() && subscription_mode_.load() != subscription_mode && timeout_counter < max_timeout_counter)
  {
    poll_interval.sleep();
    timeout_counter++;
  }
  if (timeout_counter >= 10 )
  {
    ROS_ERROR_STREAM("Failed to request pointcloud streaming after " << max_timeout_counter << " seconds");
  }
}

void UamDriver::stopStreaming()
{
  auto subscription_mode = subscription_mode_.load();

  if (subscription_mode == ESubscriptionMode::INVALID)
  {
    ROS_WARN_STREAM("Not streaming, skipping action!");
    return;
  }
  switch (subscription_mode)
  {
    case ESubscriptionMode::AR02:
    {
      auto ignore = asyncSend<uam::AR03Worker>();
      break;
    }
    case ESubscriptionMode::AR04:
    {
      auto reply = asyncSend<uam::AR05Worker>();
      if (!reply)
      {
        std::stringstream ss;
        ss << "Failed to request subscribe to pointcloud with AR04!";
        throw std::runtime_error(ss.str());
      }
      break;
    }
    case ESubscriptionMode::AR07:
    {
      auto reply = asyncSend<uam::AR08Worker>();
      if (!reply)
      {
        std::stringstream ss;
        ss << "Failed to subscribe to pointcloud with AR07!";
        throw std::runtime_error(ss.str());
      }
      break;
    }
    default:
    {
    }
  };

  auto previous_stamp = ros::Time::now();
  auto poll_interval = ros::Duration(1);
  while (ros::ok() && subscription_mode_.load() != ESubscriptionMode::INVALID)
  {
    poll_interval.sleep();
  }

  this->client_.stopAsyncReadTask();
}


std::optional<protocol::YRCommandReply> UamDriver::getSafetyArea(
  const protocol::EYRAreaType area_type,
  const uint16_t area_number,
  const uint32_t start_step,
  const uint32_t end_step)
{
  // TODO(cribeirmendes): Should we support async send?
  if (isTheSensorStreaming())
  {
    this->stopStreaming();
  }
  const uint16_t adjusted_area_number = area_number == 0 ? area_number : area_number - 1;
  const uint32_t adjusted_start_step = start_step == 0 ? start_step : start_step - 1;
  const uint32_t adjusted_end_step = end_step == 0 ? end_step : end_step - 1;
  auto reply = syncSendCommandWithReply<YRWorker>(area_type, adjusted_area_number, adjusted_start_step, adjusted_end_step);
  if (!reply)
  {
    ROS_ERROR_STREAM(
      "Could not decode area configuration data for area type: " << area_type << " and area number: " << area_number);
    return std::nullopt;
  }
  return *reply;
}



} // namespace uam
