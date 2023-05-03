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

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/make_shared.hpp>
#include <boost/system/error_code.hpp>

#include <ros/console.h>
#include <uam/io/tcp_client.h>

namespace uam
{
TcpClient::TcpClient(OnNewDataCallback callback) :
  connected_(false),
  io_service_owner_(true),
  io_service_(std::make_shared<boost::asio::io_service>()),
  socket_(*io_service_),
  callback_(callback),
  stopped_(true),
  stop_requested_(false)
{
  // Add a fake task to the io_service to prevent it from exiting until desired
  io_work_ = std::make_unique<boost::asio::io_service::work>(*io_service_);
  // Start the io_service thread that handles the tcp comms
  io_thread_ = std::thread([this]() { this->io_service_->run(); });
}

TcpClient::TcpClient(OnNewDataCallback callback, std::shared_ptr<boost::asio::io_service>& external_context) :
  connected_(false),
  io_service_owner_(false),
  io_service_(external_context),
  socket_(*io_service_),
  callback_(callback),
  stopped_(true),
  stop_requested_(false)
{
}

TcpClient::~TcpClient()
{
  this->disconnect();
}

void TcpClient::disconnect()
{
  if (connected_)
  {
    // Mark the sockets as disconnected to avoid some race conditions between when the socket says it is closed and
    // when the callbacks get cancelled.
    connected_ = false;

    // Close the UDP socket. This will cancel any pending async operations.
    auto error_code = boost::system::error_code();
    if (socket_.is_open())
    {
      socket_.close(error_code);
      if (error_code)
      {
        ROS_WARN_STREAM("Failed to close socket. " << error_code.message());
      }
    }
  }
}

bool TcpClient::connect(const std::string& remote_ip, uint16_t remote_port)
{
  if (connected_)
  {
    disconnect();
  }

  if (!remote_ip.empty())
  {
    boost::system::error_code error_code;
    // Parse the input IP address string to make sure it is valid
    auto remote_address = boost::asio::ip::address::from_string(remote_ip, error_code);
    if (error_code)
    {
      ROS_ERROR_STREAM(
        "Failed to convert the provided remote ip string (" << remote_ip << ") into a valid IP address. "
                                                            << error_code.message());
      return false;
    }
    remote_endpoint_ = boost::asio::ip::tcp::endpoint(remote_address, remote_port);
  }

  // Open the socket
  boost::system::error_code error_code;
  remote_endpoint_.address().to_string();
  socket_.connect(remote_endpoint_, error_code);

  if (error_code)
  {
    ROS_ERROR_STREAM("Failed to connect: " << error_code.message());
    return false;
  }
  connected_ = true;
  // Set tcp no delay
  boost::asio::ip::tcp::no_delay option(true);
  socket_.set_option(option, error_code);
  if (error_code)
  {
    ROS_ERROR_STREAM("Failed to set option: " << error_code.message());
    disconnect();
    return false;
  }

  ROS_INFO_STREAM("Connected to: " << remote_ip << ":" << remote_port);
  return true;
}

void TcpClient::asyncReadData(const size_t packet_offset)
{
  if (!connected_ || stop_requested_)
  {
    stopped_ = true;
    return;
  }

  // Try read  with the minimum reply size
  boost::asio::async_read(
    socket_,
    boost::asio::buffer(
      receive_buffer_.getRawPacket().data() + packet_offset,
      sizeof(protocol::CommandReplyHeader) - packet_offset),
    boost::asio::transfer_exactly(sizeof(protocol::CommandReplyHeader)),
    boost::bind(
      &TcpClient::handleReceive,
      this,
      boost::asio::placeholders::error,
      boost::asio::placeholders::bytes_transferred));
}

void TcpClient::handleReceive(const boost::system::error_code& error_code, size_t bytes_transferred)
{
  // Grab a packet received time as soon as possible
  auto received_stamp = ros::Time::now();
  size_t packet_offset = 0;

  if (error_code)
  {
    // "operation_aborted" errors will occur if the socket is closed for any reason. Don't spam the logs with expected
    // error conditions.
    if (connected_ || error_code != boost::asio::error::operation_aborted)
    {
      ROS_ERROR_STREAM("Error receiving lidar message. " << error_code.message());
    }
  }
  else if (sizeof(protocol::CommandReplyHeader) != bytes_transferred)
  {
    ROS_WARN_STREAM(
      "Failed to read: " << error_code.message() << " Received " << bytes_transferred << "bytes\n"
                         << "Expected " << sizeof(protocol::CommandReplyHeader) << " bytes");
  }
  else
  {
    // All the packets are expected to contain a header. We read those bytes
    // and then we call the read to read the rest
    bool should_try_complete_read =
      filter_callback_ ? filter_callback_(receive_buffer_.get<protocol::CommandReplyHeader>().header) : true;

    if (should_try_complete_read)
    {
      auto stamp = ros::Time::now();
      auto expected_total_size = receive_buffer_.get<protocol::CommandReplyHeader>().cmd_size;
      decodeField(expected_total_size);
      if (expected_total_size > sizeof(protocol::AR01CommandReply))
      {
        ROS_WARN_STREAM("Error in the expected size!");
      }
      else
      {
        boost::system::error_code new_error_code;
        auto missing_read = expected_total_size - bytes_transferred;
        auto recv_bytes = boost::asio::read(
          socket_,
          boost::asio::buffer(receive_buffer_.getRawPacket().data() + bytes_transferred, missing_read),
          boost::asio::transfer_exactly(missing_read),
          new_error_code);
        if (new_error_code || recv_bytes != missing_read)
        {
          ROS_WARN_STREAM(
            "Failed to read: " << new_error_code.message() << " Received " << recv_bytes << "bytes\n"
                               << "Expected " << expected_total_size << " bytes");
        }
        else
        {
          callback_(receive_buffer_, stamp);
        }
      }
    }
    else
    {
      ROS_WARN_STREAM("Packet was received but could not make sense of them, skipping it.");
    }
  }
  asyncReadData(packet_offset);
}

}  // namespace uam
