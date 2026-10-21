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

#include <uam/io/tcp_client.h>

#include <ros/console.h>

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/make_shared.hpp>
#include <boost/system/error_code.hpp>

#include <algorithm>
#include <cstring>
#include <memory>
#include <string>
#include <utility>

namespace uam
{
TcpClient::TcpClient(OnNewDataCallback callback) :
  connected_(false),
  io_service_owner_(true),
  io_service_(std::make_shared<boost::asio::io_service>()),
  socket_(*io_service_),
  callback_(callback),
  stopped_(true)
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
  stopped_(true)
{
}

TcpClient::~TcpClient()
{
  this->disconnect();
  if (io_service_owner_)
  {
    io_work_.reset();
    // Wait for the io thread to terminate cleanly
    if (io_thread_.joinable())
    {
      io_thread_.join();
    }
  }
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
    // Clear async read flag
    stopped_ = true;
  }
  stopProcessingThread();
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

  // Open the socket and enlarge the kernel receive buffer before connecting so the
  // kernel can absorb bursts while packets are being processed.
  {
    boost::system::error_code error_code;
    // Discard any stale socket left open by a previously failed connect attempt
    if (socket_.is_open())
    {
      socket_.close(error_code);
      ROS_WARN_STREAM_COND(error_code, "Failed to close stale socket: " << error_code.message());
    }
    socket_.open(remote_endpoint_.protocol(), error_code);
    if (error_code)
    {
      ROS_ERROR_STREAM("Failed to open socket: " << error_code.message());
      return false;
    }
    socket_.set_option(boost::asio::socket_base::receive_buffer_size(c_receive_buffer_size), error_code);
    ROS_WARN_STREAM_COND(error_code, "Failed to set receive buffer size: " << error_code.message());
  }

  // Connect the socket
  {
    auto connect_future = socket_.async_connect(remote_endpoint_, boost::asio::use_future);
    if (std::future_status::ready != connect_future.wait_for(std::chrono::seconds(5)))
    {
      ROS_ERROR_STREAM("Connect Timed out...");
      boost::system::error_code error_code;
      socket_.close(error_code);
      ROS_ERROR_STREAM_COND(error_code, "Error while closing socket: " << error_code.message());
      return false;
    }

    try
    {
      connect_future.get();
    }
    catch (const boost::system::system_error& error)
    {
      ROS_ERROR_STREAM("Failed to connect: " << error.what());
      return false;
    }
  }

  // Set connected flag
  connected_ = true;

  // Set tcp no delay
  boost::asio::ip::tcp::no_delay option(true);
  boost::system::error_code error_code;
  socket_.set_option(option, error_code);
  if (error_code)
  {
    ROS_ERROR_STREAM("Failed to set option: " << error_code.message());
    disconnect();
    return false;
  }

  ROS_INFO_STREAM("Connected to: " << remote_ip << ":" << remote_port);

  int recv_buf_size;
  socklen_t recv_buf_size_len = sizeof(recv_buf_size);
  auto rc = getsockopt(socket_.native_handle(), SOL_SOCKET, SO_RCVBUF, &recv_buf_size, &recv_buf_size_len);
  if (rc == 0)
  {
    ROS_INFO_STREAM("Actual receive buffer size: " << recv_buf_size << " bytes");
  }
  else
  {
    ROS_ERROR_STREAM("Error getting receive buffer size: " << strerror(errno));
  }

  startProcessingThread();

  return true;
}

void TcpClient::startProcessingThread()
{
  if (processing_thread_.joinable())
  {
    return;
  }
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    processing_shutdown_ = false;
    packet_queue_.clear();
  }
  processing_thread_ = std::thread([this]() { this->processingLoop(); });
}

void TcpClient::stopProcessingThread()
{
  if (!processing_thread_.joinable())
  {
    return;
  }
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    processing_shutdown_ = true;
  }
  queue_signal_.notify_all();
  processing_thread_.join();
  std::lock_guard<std::mutex> lock(queue_mutex_);
  packet_queue_.clear();
}

void TcpClient::processingLoop()
{
  while (true)
  {
    std::pair<protocol::ShapeShifterPacket, ros::Time> item;
    {
      std::unique_lock<std::mutex> lock(queue_mutex_);
      queue_signal_.wait(lock, [this] { return processing_shutdown_ || !packet_queue_.empty(); });
      if (processing_shutdown_)
      {
        return;
      }
      item = packet_queue_.front();
      packet_queue_.pop_front();
    }
    if (callback_)
    {
      callback_(item.first, item.second);
    }
  }
}

void TcpClient::enqueuePacket(const ros::Time& stamp)
{
  std::lock_guard<std::mutex> lock(queue_mutex_);
  if (packet_queue_.size() >= c_max_queued_packets)
  {
    ROS_WARN_STREAM_THROTTLE(1.0, "Packet processing queue is full, dropping oldest packet!");
    packet_queue_.pop_front();
  }
  packet_queue_.emplace_back(receive_buffer_, stamp);
  queue_signal_.notify_one();
}

void TcpClient::asyncReadData(const size_t packet_offset)
{
  if (!connected_)
  {
    stopped_ = true;
    return;
  }

  // Read the remainder of the fixed-size reply header, keeping any bytes salvaged by resyncStream()
  boost::asio::async_read(
    socket_,
    boost::asio::buffer(
      receive_buffer_.getRawPacket().data() + packet_offset,
      sizeof(protocol::CommandReplyHeader) - packet_offset),
    boost::asio::transfer_exactly(sizeof(protocol::CommandReplyHeader) - packet_offset),
    boost::bind(
      &TcpClient::handleReceive,
      this,
      boost::asio::placeholders::error,
      boost::asio::placeholders::bytes_transferred,
      packet_offset));
}

void TcpClient::resyncStream()
{
  constexpr size_t header_size = sizeof(protocol::CommandReplyHeader);
  auto& raw = receive_buffer_.getRawPacket();

  // Hunt for a candidate frame start past the current (invalid) one
  const auto begin = raw.data() + 1;
  const auto end = raw.data() + header_size;
  const auto stx_it = std::find(begin, end, static_cast<char>(protocol::STX_ID));

  if (stx_it == end)
  {
    // No candidate in the buffered bytes, discard them and read a fresh header
    asyncReadData();
    return;
  }

  // Shift the candidate frame start to the front and read the missing header bytes
  const size_t salvaged_bytes = static_cast<size_t>(end - stx_it);
  std::memmove(raw.data(), stx_it, salvaged_bytes);
  asyncReadData(salvaged_bytes);
}

void TcpClient::handlePacket(
  const boost::system::error_code& error_code,
  size_t bytes_transferred,
  const size_t missing_read,
  const ros::Time& stamp)
{
  if (error_code)
  {
    // "operation_aborted" errors will occur if the socket is closed for any reason. Don't spam the logs with expected
    // error conditions.
    if (connected_ || error_code != boost::asio::error::operation_aborted)
    {
      ROS_ERROR_STREAM("Error receiving lidar message. " << error_code.message());
    }
  }
  else if (bytes_transferred != missing_read)
  {
    ROS_WARN_STREAM(
      "Failed to read missing bytes. Received " << bytes_transferred << "bytes\n"
                                                << "Expected " << missing_read << " bytes");
  }
  else
  {
    const size_t total_size = sizeof(protocol::CommandReplyHeader) + missing_read;
    const char etx = receive_buffer_.getRawPacket()[total_size - 1];
    if (etx != static_cast<char>(protocol::ETX_ID))
    {
      // Framing is off even though the header looked plausible; the header hunt in
      // handleReceive() will re-align the stream on the next read.
      ROS_WARN_STREAM_THROTTLE(1.0, "Packet is missing the ETX terminator, dropping it.");
    }
    else
    {
      // Hand the packet to the processing thread so the socket keeps being drained
      enqueuePacket(stamp);
    }
  }
  // Resume async read
  asyncReadData();
}

void TcpClient::handleReceive(
  const boost::system::error_code& error_code,
  size_t bytes_transferred,
  size_t packet_offset)
{
  constexpr size_t header_size = sizeof(protocol::CommandReplyHeader);
  const size_t expected_read = header_size - packet_offset;

  if (error_code)
  {
    // "operation_aborted" errors will occur if the socket is closed for any reason. Don't spam the logs with expected
    // error conditions.
    if (connected_ || error_code != boost::asio::error::operation_aborted)
    {
      ROS_ERROR_STREAM("Error receiving lidar message. " << error_code.message());
    }
    asyncReadData();
    return;
  }

  if (expected_read != bytes_transferred)
  {
    ROS_WARN_STREAM(
      "Failed to read header. Received " << bytes_transferred << " bytes\n"
                                         << "Expected " << expected_read << " bytes");
    asyncReadData();
    return;
  }

  const auto& header = receive_buffer_.get<protocol::CommandReplyHeader>();

  // Validate framing before trusting any header field
  bool valid_header = header.stx == protocol::STX_ID;
  valid_header = valid_header && (filter_callback_ ? filter_callback_(header.header) : true);

  auto expected_total_size = header.cmd_size;
  valid_header = valid_header && decodeField(expected_total_size) &&
                 expected_total_size >= sizeof(protocol::EmptyCommandReply) &&
                 expected_total_size <= sizeof(protocol::AR01CommandReply);

  if (!valid_header)
  {
    ROS_WARN_STREAM_THROTTLE(1.0, "Invalid packet header, resynchronizing stream.");
    resyncStream();
    return;
  }

  // Grab a packet received time as soon as possible
  auto stamp = ros::Time::now();
  // Add an async read task to read the rest of the packet
  const size_t missing_read = expected_total_size - header_size;
  boost::asio::async_read(
    socket_,
    boost::asio::buffer(receive_buffer_.getRawPacket().data() + header_size, missing_read),
    boost::asio::transfer_exactly(missing_read),
    boost::bind(
      &TcpClient::handlePacket,
      this,
      boost::asio::placeholders::error,
      boost::asio::placeholders::bytes_transferred,
      missing_read,
      stamp));
}

}  // namespace uam
