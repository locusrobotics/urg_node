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

#ifndef UAM_IO_TCP_CLIENT_H
#define UAM_IO_TCP_CLIENT_H

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/make_shared.hpp>
#include <boost/system/error_code.hpp>

#include <ros/init.h>
#include <uam/shape_shifter_packet.h>
#include <uam/uam_visitors.h>

#include <memory>
#include <string>

namespace uam
{
/**
 * @brief TCP client class using boost::asio
 */
class TcpClient
{
public:
  /**
   * @brief type of the Callback that receives the packet
   */
  using OnNewDataCallback = std::function<void(const protocol::ShapeShifterPacket& packet, const ros::Time&)>;

  /**
   * @brief This
   */
  using FilterCallback = std::function<bool(const decltype(protocol::CommandReplyHeader::header)& packet)>;

  /**
   * @brief Default C'tor
   */
  explicit TcpClient(OnNewDataCallback callback);

  /**
   * @brief
   *
   * @param name
   * @param callback
   * @param external_context
   */
  TcpClient(OnNewDataCallback callback, std::shared_ptr<boost::asio::io_service>& external_context);

  /**
   * @brief Default D'tor
   */
  ~TcpClient();

  /**
   * @brief Connect to tcp server
   *
   * @param[in] remote_ip - Server ip address
   * @param[in] remote_port - Server port
   *
   * @return true if connect successful, false otherwise
   */
  bool connect(const std::string& remote_ip = "", uint16_t remote_port = 0u);

  /**
   * @brief Returns the state of the socket
   *
   * @return true if connected, false otherwise
   */
  inline bool isConnected() const { return connected_; }

  /**
   * @brief Returns if async read is queued or not
   *
   * @return true if async read is in progress, alse otherwise
   */
  inline bool isStopped() const { return stopped_; }

  /**
   * @brief Disconnect client
   */
  void disconnect();

  /**
   * @brief Place and send order and wait for it to be sent
   *
   * @param[in] command - Command to send
   * @param[in] timeout - How many seconds to wait until send command times out.
   *
   * @return true if command successfully sent, false otherwise.
   */
  inline bool asyncSend(
    const std::string& command,
    const std::chrono::seconds timeout = std::chrono::seconds(5))
  {
    if (!connected_)
      return false;
    auto send_length = socket_.async_send(boost::asio::buffer(command, command.size()), boost::asio::use_future);
    auto status = send_length.wait_for(timeout);
    if (status != std::future_status::ready)
    {
      // we are not expecting this to happen too often, no need to throttle this warning.
      ROS_WARN_STREAM("Send command timed out...");
      return false;
    }
    return (send_length.get() == command.size());
  }

  /**
   * @brief Start Async Read Task (to read UAM commands)
   *
   * @return true if async task was created false otherwise
   */
  inline bool startAsyncReadTask()
  {
    if (!connected_)
    {
      ROS_WARN_STREAM("Not connected, cannot start async read task");
      return false;
    }

    if (!stopped_)
    {
      return true;
    }

    // TODO(carlos-m159): add guard to check if async read line task in
    // in progress.
    asyncReadData();
    stopped_ = false;
    return true;
  }

  /**
   * @brief Create an async read line operation
   *
   * @param[in] handler - Async read handler
   */
  template <typename TReply, typename Handler>
  void asyncReadLine(Handler handler)
  {
    if (!stopped_)
    {
      ROS_ERROR_STREAM("Cannot send async readline with async receive in progress.");
      return;
    }

    // Start the asynchronous operation.
    boost::asio::async_read_until(
      socket_,
      readline_buffer_,
      '\n',
      [&](const boost::system::error_code& result_error, std::size_t result_n)
      {
        if (result_error || result_n == 0)
        {
          return;
        }

        TReply reply;
        const auto nr_lines_to_read = reply.size();
        size_t nr_lines_read = 0;
        std::istream is(&readline_buffer_);
        std::string line;
        while (std::getline(is, line) && nr_lines_read < nr_lines_to_read)
        {
          reply.at(nr_lines_read) = line;
          nr_lines_read++;
          line.clear();
        }
        handler(reply);
      });  //NOLINT

    return;
  }

  /**
   * @brief Register Callback to filter if a packet is valid or not
   * @param[in] f_ - Filter callback
   */
  void registerFilterCallback(FilterCallback f_) { filter_callback_ = f_; }

private:
  /**
   * @brief Start Async Read in the worker thread
   */
  void asyncReadData(const size_t packet_offset = 0);

  /**
   * @brief Receive handler for Async read
   *
   * @param[in] error_code - Error Code
   * @param[in] bytes_transferred - Number of bytes written into the buffer
   */
  void handleReceive(const boost::system::error_code& error_code, size_t bytes_transferred);

private:
  /**
   * @brief Shape shifter buffer used for UAM packets
   */
  protocol::ShapeShifterPacket receive_buffer_;

  /**
   * @brief Read line buffer used for SCIP packets
   */
  boost::asio::streambuf readline_buffer_;

  /**
   * @brief The Boost IO Service object that manages the asynchronous operations
   */
  std::shared_ptr<boost::asio::io_service> io_service_;

  /**
   * @brief Flag if io_service is owned (true) or taken externally (false)
   */
  bool io_service_owner_;

  /**
   * @brief A dedicated thread for running the Boost ASIO event loop
   */
  std::thread io_thread_;

  /**
   * @brief A fake task to prevent the io_service from terminating until desired
   */
  std::unique_ptr<boost::asio::io_service::work> io_work_;

  /**
   * @brief Remote endpoint
   */
  boost::asio::ip::tcp::endpoint remote_endpoint_;

  /**
   * @brief Flag indicating the class is currently connected to the lidar
   *
   * By setting the flag false before closing the sockets, we solve some race conditions around exactly when callbacks
   * are cancelled and when the socket says it is closed.
   */
  std::atomic_bool connected_;

  /**
   * @brief The TCP socket used for communicating with the lidar
   */
  boost::asio::ip::tcp::socket socket_;

  /**
   * @brief On new data callback for uam packets
   */
  OnNewDataCallback callback_;

  /**
   * @brief On new data callback for scip packets
   */
  OnNewDataCallback scip_callback_;

  /**
   * @brief Callback to filter packet header
   */
  FilterCallback filter_callback_;

  /**
   * @brief State of the async read task
   */
  std::atomic_bool stopped_;
};

}  // namespace uam

#endif  // UAM_IO_TCP_CLIENT_H
