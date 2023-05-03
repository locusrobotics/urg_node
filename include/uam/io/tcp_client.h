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
  using OnNewDataCallback = std::function<void(const protocol::ShapeShifterBuffer& packet, const ros::Time&)>;

  /**
   * @brief This
   */
  using FilterCallback = std::function<bool(const decltype(protocol::CommandReplyHeader::header)& packet)>;

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

    asyncReadData();
    stopped_ = false;
    stop_requested_ = false;
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
      });

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
   * @brief Shape shifter buffer
   */
  protocol::ShapeShifterBuffer receive_buffer_;
  
  /**
   * @brief Read line buffer
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
  OnNewDataCallback scip_callback_;

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
};

}  // namespace uam

#endif  // INCLUDE_URG_NODE_CONNECTION_TCP_CLIENT_H_
