/**
Software License Agreement (proprietary)
\file      tcp_client.h
\authors   Carlos Mendes <cribeiromendes@locusrobotics.com>
\copyright Copyright (c) (2023,), Locus Robotics Corp., All rights reserved.
Unauthorized copying of this file, via any medium, is strictly prohibited.
Proprietary and confidential.
**/

#ifndef INCLUDE_URG_NODE_CONNECTION_TCP_CLIENT_H_
#define INCLUDE_URG_NODE_CONNECTION_TCP_CLIENT_H_

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/make_shared.hpp>
#include <boost/system/error_code.hpp>

#include <ros/init.h>
#include <uam/shape_shifter_buffer.h>
#include <uam/uam_visitors.h>

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
  using OnNewDataCallback = std::function<void(const protocol::ShapeShifterBuffer& packet)>;

  /**
   * @brief This
   */
  using FilterCallback = std::function<bool(const protocol::CommandReplyHeader& packet)>;

  /**
   * @brief Default C'tor
   */
  TcpClient(OnNewDataCallback callback);

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
   * @brief Disconnect client
   */
  void disconnect();

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
   * @brief Place and send order and wait for it to be sent
   *
   * @param[in] command - Command to send
   * @return true if command successfully sent, false otherwise.
   */
  inline bool asyncSend(const std::string& command)
  {
    if (!connected_)
      return false;
    auto send_length = socket_.async_send(boost::asio::buffer(command, command.size()), boost::asio::use_future);
    return (send_length.get() == command.size());
  }

  /**
   * @brief
   * @return
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
      ROS_WARN_STREAM("Async read in progress");
      return true;
    }

    asyncReadData();
    stopped_ = false;
    stop_requested_ = false;
    return true;
  }

  inline bool stopAsyncReadTask()
  {
    if (!connected_)
    {
      ROS_WARN_STREAM("Not connected");
      return false;
    }
    if (stopped_)
    {
      ROS_WARN_STREAM("Async read already stopped");
      return true;
    }
    // Request stop
    stop_requested_ = true;
    while (ros::ok() && !stopped_)
    {
      ROS_INFO_STREAM_THROTTLE(1, "Waiting for the async job to finish...");
      std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }
    return stopped_.load();
  }

  /**
   * @brief Send and receive messages
   *
   * @param[in] worker - Worker of the requested command
   * @param[out] reply - Reply message
   * @return true if success, false otherwise
   */
  template <typename TReply>
  std::optional<TReply> syncSendAndReceive(const std::string& command)
  {
    if (!connected_)
    {
      ROS_WARN_STREAM("Not connected");
      return std::nullopt;
    }

    if (!stopped_)
    {
      ROS_ERROR_STREAM("Sync operation not possible as async read is in progress.");
      return std::nullopt;
    }

    boost::system::error_code error_code;
    socket_.send(boost::asio::buffer(command), command.size(), error_code);
    if (error_code)
    {
      ROS_ERROR_STREAM("Failed to send command: " << error_code.message());
      return std::nullopt;
    }

    // We should now understand if the reply is going to be successful or not.
    // If the device fails to correctly respond, we could get stuck here forever
    // while expecting for the complete packet

    const decltype(uam::protocol::CommandReplyHeader::cmd_size) expected_size = sizeof(TReply);
    auto recv_bytes = boost::asio::read(
      socket_,
      boost::asio::buffer(receive_buffer_.getRawPacket()),
      boost::asio::transfer_exactly(expected_size),
      error_code);
    if (error_code || expected_size != recv_bytes)
    {
      ROS_WARN_STREAM(
        "Failed to read: " << error_code.message() << " Received " << recv_bytes << "bytes\n"
                           << "Expected " << expected_size << " bytes");
      return std::nullopt;
    }
    TReply reply = receive_buffer_.get<TReply>();
    return reply;
  }

  /**
   * @brief Send and wait for reply which will consist in reading N lines
   *
   * @param[in] command - Command to send
   * @return TReply if successful or std::nullopt if it fails.
   */
  template <typename TReply>
  std::optional<TReply> syncSendAndReadLine(const std::string& command)
  {
    if (!stopped_)
    {
      ROS_ERROR_STREAM("Sync operation not possible as async read is in progress.");
      return std::nullopt;
    }
    boost::system::error_code error_code;
    socket_.send(boost::asio::buffer(command), command.size(), error_code);
    if (error_code)
    {
      ROS_ERROR_STREAM("Failed to send command: " << error_code.message());
      ROS_DEBUG_STREAM("Failed to send command: " << command);
      return std::nullopt;
    }

    TReply reply;
    const auto nr_lines_to_read = reply.size();
    size_t nr_lines_read = 0;
    {
      boost::asio::streambuf b;
      auto n_bytes = boost::asio::read_until(socket_, b, '\n', error_code);
      if (error_code || n_bytes == 0)
      {
        throw std::runtime_error(error_code.message());
      }
      std::istream is(&b);
      std::string line;
      while (std::getline(is, line) && nr_lines_read < nr_lines_to_read)
      {
        reply.at(nr_lines_read) = line;
        nr_lines_read++;
        line.clear();
      }
    }

    if (nr_lines_read != (nr_lines_to_read))
    {
      ROS_ERROR_STREAM("Failed to read requested number of lines!");
      return std::nullopt;
    }
    return reply;
  }

  void registerFilterCallback(FilterCallback f_) { filter_callback_ = f_; }

private:
  /**
   * @brief Check deadline timer.
   *
   * This is the handler for the deadline and to close socket if deadline
   * has expired. This will be run by the thread which takes care of the io_service_
   *
   */
  void checkDeadline();

  /**
   * @brief
   */
  void handleConnect();

  /**
   * @brief Start Async Read in the worker thread
   */
  void asyncReadData();

  /**
   * @brief Receive handler for Async read
   *
   * @param[in] error_code - Error Code
   * @param[in] bytes_transferred - Number of bytes written into the buffer
   */
  void handleReceive(const boost::system::error_code& error_code, size_t bytes_transferred);

private:
  /**
   * @brief Shape shifter buffer
   */
  protocol::ShapeShifterBuffer receive_buffer_;

  /**
   * @brief The Boost IO Service object that manages the asynchronous operations
   */
  std::shared_ptr<boost::asio::io_service> io_service_;

  /**
   * @brief Flag if io_service is owned (true) or taken externally (false)
   */
  bool io_service_owner_;

  /**
   * @brief Logger Name
   */
  std::string name_;

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
   * @brief The UDP socket used for communicating with the lidar
   */
  boost::asio::ip::tcp::socket socket_;

  /**
   * @brief
   */
  OnNewDataCallback callback_;

  /**
   * @brief
   */
  FilterCallback filter_callback_;

  /**
   * @brief Async read is in progress
   */
  std::atomic_bool stopped_;

  /**
   * @brief Async read is in progress
   */
  std::atomic_bool stop_requested_;

  /**
   * @brief Filter
   */
  protocol::CommandReplyHeader command_header_buffer_;
  //boost::asio::deadline_timer deadline_connect_;
  //boost::asio::deadline_timer deadline_receive_;
};

}  // namespace uam

#endif  // INCLUDE_URG_NODE_CONNECTION_TCP_CLIENT_H_
