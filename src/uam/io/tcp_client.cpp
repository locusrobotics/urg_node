/**
Software License Agreement (proprietary)
\file      tcp_client.h
\authors   Carlos Mendes <cribeiromendes@locusrobotics.com>
\copyright Copyright (c) (2023,), Locus Robotics Corp., All rights reserved.
Unauthorized copying of this file, via any medium, is strictly prohibited.
Proprietary and confidential.
**/

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
  callback_(callback)
{
  // Add a fake task to the io_service to prevent it from exiting until desired
  io_work_ = std::make_unique<boost::asio::io_service::work>(*io_service_);
  // Start the io_service thread that handles the tcp comms
  io_thread_ = std::thread([this]() { this->io_service_->run(); });
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
        ROS_WARN_STREAM_NAMED(name_, "Failed to close socket. " << error_code.message());
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
  // Set tcp no delay
  boost::asio::ip::tcp::no_delay option(true);
  socket_.set_option(option, error_code);
  if (error_code)
  {
    ROS_ERROR_STREAM("Failed to set option: " << error_code.message());
    return false;
  }
  ROS_INFO_STREAM("Connected to: " << remote_ip << ":" << remote_port);
  connected_ = true;
  return true;
}

bool TcpClient::startAsyncRead(const double timeout)
{
  if (async_read_started_)
    return true;
  if (!connected_)
    return false;
  boost::asio::async_read(
    socket_,
    boost::asio::buffer(receive_buffer_.getRawPacket()),
    boost::asio::transfer_exactly(sizeof(protocol::CommandReplyHeader)),
    boost::bind(
      &TcpClient::handleReceive,
      this,
      boost::asio::placeholders::error,
      boost::asio::placeholders::bytes_transferred));
      async_read_started_= true;
  return true;
}

void TcpClient::handleReceive(const boost::system::error_code& error_code, size_t bytes_transferred)
{
  // Grab a packet received time as soon as possible
  auto received_stamp = ros::Time::now();

  if (error_code)
  {
    // "operation_aborted" errors will occur if the socket is closed for any reason. Don't spam the logs with expected
    // error conditions.
    if (connected_ || error_code != boost::asio::error::operation_aborted)
    {
      ROS_ERROR_STREAM_NAMED(name_, "Error receiving lidar message. " << error_code.message());
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
    auto expected_total_size = receive_buffer_.getPacketHeader().cmd_size;
    decodeField(expected_total_size);
    if (expected_total_size > sizeof(protocol::AR01CommandReply))
    {
      ROS_WARN_STREAM("Error in the expected size!");
      exit(1);
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
        return;
      }
      callback_(receive_buffer_);
    }
  }
  boost::asio::async_read(
    socket_,
    boost::asio::buffer(receive_buffer_.getRawPacket()),
    boost::asio::transfer_exactly(sizeof(protocol::CommandReplyHeader)),
    boost::bind(
      &TcpClient::handleReceive,
      this,
      boost::asio::placeholders::error,
      boost::asio::placeholders::bytes_transferred));
}

}  // namespace uam
