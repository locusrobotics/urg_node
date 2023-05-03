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

#include <uam/uam_driver.h>

namespace uam
{
UamDriver::UamDriver() :
  io_service_(std::make_shared<boost::asio::io_service>()),
  client_(std::bind(&UamDriver::packetCallback, this, std::placeholders::_1, std::placeholders::_2), io_service_),
  uam_packet_worker_(),
  scip_pp_worker_(),
  command_timeout_(10.0),
  subscription_mode_(ESubscriptionMode::INVALID)
{
  // Register command callback (VR, XR, YR)
  uam_packet_worker_.registerCallback<VR00Worker>(
    std::bind(&UamDriver::processCommandReply<VR00Worker::Reply>, this, std::placeholders::_1));
  uam_packet_worker_.registerCallback<XR00Worker>(
    std::bind(&UamDriver::processCommandReply<XR00Worker::Reply>, this, std::placeholders::_1));
  uam_packet_worker_.registerCallback<YRWorker>(
    std::bind(&UamDriver::processCommandReply<YRWorker::Reply>, this, std::placeholders::_1));
  // Register command callback (ARs)
  uam_packet_worker_.registerCallback<AR02Worker>(
    std::bind(&UamDriver::processCommandReply<AR02Worker::Reply>, this, std::placeholders::_1));
  uam_packet_worker_.registerCallback<AR04Worker>(
    std::bind(&UamDriver::processCommandReply<AR04Worker::Reply>, this, std::placeholders::_1));
  uam_packet_worker_.registerCallback<AR07Worker>(
    std::bind(&UamDriver::processCommandReply<AR07Worker::Reply>, this, std::placeholders::_1));
  uam_packet_worker_.registerCallback<AR03Worker>(
    std::bind(&UamDriver::processCommandReply<AR03Worker::Reply>, this, std::placeholders::_1));
  uam_packet_worker_.registerCallback<AR05Worker>(
    std::bind(&UamDriver::processCommandReply<AR05Worker::Reply>, this, std::placeholders::_1));
  uam_packet_worker_.registerCallback<AR08Worker>(
    std::bind(&UamDriver::processCommandReply<AR08Worker::Reply>, this, std::placeholders::_1));

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
  if (client_.isStopped())
    client_.startAsyncReadTask();

  // We do not need to stop lidar cloud to get Version details
  auto reply = sendCommandWithReply<uam::VR00Worker>(command_timeout_);

  return std::string(
    "Firmware_version is: " +
    std::string(reply.version_details.firmware_version.data(), reply.version_details.firmware_version.size()) +
    std::string(
      "\nSensor_model is: " +
      std::string(reply.version_details.sensor_model.data(), reply.version_details.sensor_model.size())) +
    std::string(
      "\nSerial Number is: " +
      std::string(reply.version_details.serial_number.data(), reply.version_details.serial_number.size())));
}

protocol::sensing_data::SensingDataHeader UamDriver::getSensorStatus()
{
  if (client_.isStopped())
    client_.startAsyncReadTask();

  if (isTheSensorStreaming())
  {
    this->stopStreaming();
  }

  auto status_reply = sendCommandWithReply<uam::XR00Worker>(command_timeout_);
  protocol::sensing_data::SensingDataHeader status;
  status.area_number = status_reply.data.area_number;
  status.error_code = status_reply.data.error_code;
  status.error_state = status_reply.data.error_state;
  status.lockout_state = status_reply.data.lockout_state;
  status.operating_mode = status_reply.data.operating_mode;
  status.optical_window_contaminated = status_reply.data.optical_window_contaminated;
  status.ossd1_state = status_reply.data.ossd1_state;
  status.ossd2_state = status_reply.data.ossd1_state;
  status.warning1_state = status_reply.data.warning1_state;
  status.warning2_state = status_reply.data.warning2_state;
  return status;
}

ScanParameters UamDriver::getScanDetails()
{
  if (!client_.isStopped())
  {
    std::stringstream ss;
    ss << "Failed to get scan details. Client is running async read tasks!";
    throw std::runtime_error(ss.str());
  }

  auto reply = sendPPCommandWithReply(command_timeout_);

  return  ScanParameters::fromMessage(reply);
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
                                               (publish_multiecho ? ESubscriptionMode::AR07 : ESubscriptionMode::AR02);

  switch (subscription_mode)
  {
    case ESubscriptionMode::AR02:
    {
      subscribe<uam::AR02Worker>();
      break;
    }
    case ESubscriptionMode::AR04:
    {
      subscribe<uam::AR04Worker>();
      break;
    }
    case ESubscriptionMode::AR07:
    {
      subscribe<uam::AR07Worker>();
      break;
    }
    default:
    {
    }
  };
}

void UamDriver::stopStreaming()
{
  if (subscription_mode_ == ESubscriptionMode::INVALID)
  {
    ROS_WARN_STREAM("Not streaming, skipping action!");
    return;
  }

  switch (subscription_mode_)
  {
    case ESubscriptionMode::AR02:
    {
      unsubscribe<uam::AR03Worker>();
      break;
    }
    case ESubscriptionMode::AR04:
    {
      unsubscribe<uam::AR05Worker>();
      break;
    }
    case ESubscriptionMode::AR07:
    {
      unsubscribe<uam::AR06Worker>();
      break;
    }
    default:
    {
    }
  };
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

  return sendCommandWithReply<YRWorker>(
    command_timeout_,
    area_type,
    adjusted_area_number,
    adjusted_start_step,
    adjusted_end_step);
}

}  // namespace uam
