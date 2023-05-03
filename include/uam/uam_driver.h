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

#ifndef UAM_UAM_DRIVER_H
#define UAM_UAM_DRIVER_H

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/make_shared.hpp>
#include <boost/system/error_code.hpp>
#include <uam/io/tcp_client.h>
#include <urg_c/urg_connection.h>
#include <urg_node/Status.h>

#include <sensor_msgs/LaserScan.h>
#include <uam/workers/uam_command_worker.h>
#include <uam/workers/yr_worker.h>

#include <boost/algorithm/string/finder.hpp>
#include <boost/algorithm/string/iter_find.hpp>

#include <uam/workers/scip_worker.h>

#include <algorithm>
#include <memory>
#include <set>
#include <string>
#include <utility>

namespace uam
{
/**
 * @brief Scan parameters structure
 */
struct ScanParameters
{
  /**
   * @brief
   *
   * @param reply
   * @return
   */
  static ScanParameters fromMessage(const scip_protocol::PPReply& reply)
  {
    ScanParameters scan_details;
    // Copy some of the raw params
    scan_details.first_data_index = reply.start_step;
    scan_details.last_data_index = reply.end_step;
    scan_details.front_data_index = reply.front_data_index;
    scan_details.rpm = reply.rpm;
    scan_details.angular_resolution = reply.angular_resolution;

    // Now calculate laser scan metadata
    scan_details.range_min = reply.min_distance / 1000.0;
    scan_details.range_max = reply.max_distance / 1000.0;
    // Index to Step
    scan_details.min_step = reply.start_step - reply.front_data_index;
    scan_details.max_step = reply.end_step - reply.front_data_index;
    //
    scan_details.angle_increment = 2.0 * M_PI / reply.angular_resolution;
    scan_details.angle_min_limit = 2.0 * M_PI * scan_details.min_step / reply.angular_resolution;
    scan_details.angle_max_limit = 2.0 * M_PI * scan_details.max_step / reply.angular_resolution;

    scan_details.scan_period = 60.0 / static_cast<double>(reply.rpm);
    auto circle_fraction = (scan_details.angle_max_limit - scan_details.angle_min_limit) / (2.0 * M_PI);
    scan_details.time_increment =
      circle_fraction * scan_details.scan_period / static_cast<double>(scan_details.max_step - scan_details.min_step);
    scan_details.setAngleLimits(scan_details.angle_min_limit, scan_details.angle_max_limit);
    return scan_details;
  }

  /**
   * @brief
   * @param min_angle
   * @param max_angle
   */
  void setAngleLimits(const double new_angle_min, const double new_angle_max)
  {
    // Set step limits
    first_step = angle2step(new_angle_min);
    last_step = angle2step(new_angle_max);

    // Make sure step limits are not the same
    if (first_step == last_step)
    {
      // Make sure we're not at a limit
      if (first_step == max_step)  // At beginning of range
      {
        last_step = first_step + 1;
      }
      else  // At end of range (or all other cases)
      {
        first_step = last_step - 1;
      }
    }
    // Make sure angle_max is greater than angle_min (should check this after end limits)
    if (last_step < first_step)
    {
      double temp = first_step;
      first_step = last_step;
      last_step = temp;
    }

    // Update new angle min and angle max
    angle_min = step2angle(first_step);
    angle_max = step2angle(last_step);

    // Update angular time offset
    angular_time_offset = calculateTimeOffset();
  }

  inline auto getAngleMin() const { return angle_min; }
  inline auto getAngleMax() const { return angle_max; }
  inline auto getAngleIncrement() const { return angle_increment; }
  inline auto getScanPeriod() const { return scan_period; }
  inline auto getTimeIncrement() const { return time_increment; }
  inline auto getRangeMin() const { return range_min; }
  inline auto getRangeMax() const { return range_max; }
  inline auto getAngularTimeOffset() const { return angular_time_offset; }

private:
  double calculateTimeOffset() const
  {
    // Adjust value for Hokuyo's timestamps
    // Hokuyo's timestamps start from the rear center of the device (at Pi according to ROS standards)
    double circle_fraction = 0.0;
    if (first_step == 0 && last_step == 0)
    {
      circle_fraction = (angle_min_limit + M_PI) / (2.0 * M_PI);
    }
    else
    {
      circle_fraction = (getAngleMin() + M_PI) / (2.0 * M_PI);
    }
    return circle_fraction * getScanPeriod();
  }

  /**
   *
   * @param step
   * @param area_resolution
   * @return
   */
  double step2angle(const int step) const { return (2.0 * M_PI) * step / angular_resolution; }

  int angle2step(const double angle) const { return index2step(angle2index(angle)); }

  int angle2index(const double angle) const
  {
    int index = static_cast<int>(std::floor((angular_resolution * angle / (2.0 * M_PI) + 0.5))) + front_data_index;

    return std::min(std::max(0, index), min_step);
  }

  int index2step(const int step) const { return step - front_data_index; }

  double angle_min { 0. };
  double angle_max { 0. };
  int first_step { 0 };
  int last_step { 0 };
  double angular_time_offset { 0. };  // s

  double angle_min_limit { 0. };  // start angle of the scan [rad]
  double angle_max_limit { 0. };  // end angle of the scan [rad]
  double angle_increment { 0. };  // angular distance between measurements [rad]

  double time_increment { 0. };  // time between measurements [seconds]
  double scan_time { 0. };  // time between scans [seconds]

  double range_min { 0. };  // minimum range value [m]
  double range_max { 0. };  // maximum range value [m]

  double scan_period { 0. };

  int first_data_index { 0 };
  int last_data_index { 0 };

  int min_step { 0 };
  int max_step { 0 };

  int front_data_index { 0 };
  int angular_resolution { 0 };
  int rpm;
};

/**
 * @brief Subscription Mode Enum
 */
enum ESubscriptionMode : uint32_t
{
  AR02 = 0, /**< AR02 */
  AR04, /**< AR04 */
  AR07, /**< AR07 */
  INVALID /**< MAX */
};

template <typename TWorker>
constexpr ESubscriptionMode TypeToEnum()
{
  return (
    std::is_same<AR02Worker, TWorker>::value ?
      ESubscriptionMode::AR02 :
      (std::is_same<AR04Worker, TWorker>::value ?
         ESubscriptionMode::AR04 :
         (std::is_same<AR07Worker, TWorker>::value ? ESubscriptionMode::AR07 : ESubscriptionMode::INVALID)));
}
/**
 * @brief Hokuyo UAM-05LP Network Driver
 */
class UamDriver
{
public:
  /**
   * @brief
   */
  UamDriver();

  /**
   * @brief Default D'tor
   */
  ~UamDriver();

  /**
   * @brief Connect to a uam lidar at the specified IP address and port.
   *
   * @param[in] ip - The IP Address of the lidar
   * @param[in] lidar_port - The UDP port number of the lidar
   */
  void connect(const std::string& ip, const uint16_t port);

  /**
   * @brief Disconnect client
   */
  void disconnect();

  /**
   * @brief
   *
   * @return
   */
  inline bool isTheSensorStreaming() const { return this->subscription_mode_ != ESubscriptionMode::INVALID; }

  /**
   * @brief Send a command to the lidar and wait for the reply
   *
   * The command identifier and message will be packaged in the lidar-expected format and sent to the lidar. This
   * command will block until the send is completed and the lidar has replied. If the send operation fails or this
   * function times out waiting on the reply, an exception will be thrown.
   *
   * @param[in] identifier - The two-byte code indicating the type of command message
   * @param[in] message - The message string to send to the lidar
   * @param[in] timeout - How long to wait for the reply message
   * @return The decoded lidar reply message
   */
  template <typename TWorker, typename... TArgs>
  typename TWorker::Reply sendCommandWithReply(
    const std::chrono::duration<double>& timeout = std::chrono::seconds(2),
    const TArgs... args)
  {
    if (!client_.isConnected())
    {
      throw std::runtime_error("Cannot send command to lidar while disconnected.");
    }

    // Prepare some state variables to be populated processCommandReply()
    {
      std::lock_guard<std::mutex> temp_lock(pending_command_mutex_);
      pending_command_reply_ready_ = false;
    }

    if (!client_.asyncSend(uam_packet_worker_.getCommand<TWorker>(args...)))
    {
      throw std::runtime_error("Cannot send command to lidar.");
    }

    std::unique_lock<std::mutex> lock(pending_command_mutex_);
    if (pending_command_signal_.wait_for(lock, timeout, [this] { return pending_command_reply_ready_; }))
    {
      return pending_command_reply_.get<typename TWorker::Reply>();
    }
    else
    {
      throw std::runtime_error("Timed out waiting for response from the lidar.");
    }
  }

  /**
   * @brief Send SCIP command and wait for Reply
   *
   * @return Reply if successful received, std::nullopt otherwise
   */
  scip_protocol::PPReply sendPPCommandWithReply(const std::chrono::duration<double>& timeout = std::chrono::seconds(2))
  {
    if (!client_.isConnected())
    {
      throw std::runtime_error("Cannot send command to lidar while disconnected.");
    }

    if (!client_.asyncSend(scip_pp_worker_.getCommand()))
    {
      throw std::runtime_error("Cannot send command to lidar.");
    }

    // Prepare some state variables to be populated processCommandReply()
    {
      std::lock_guard<std::mutex> temp_lock(pending_command_mutex_);
      pending_command_reply_ready_ = false;
    }

    // async read instruction with handler
    client_.asyncReadLine<PPWorker::RawReply>(
      [this](const PPWorker::RawReply& raw_reply)
      {
        auto reply = scip_pp_worker_.process(raw_reply);
        if (reply)
        {
          std::lock_guard<std::mutex> lock(pending_command_mutex_);
          // Save the reply message in a state variable and signal that it is ready
          pending_scip_reply_ = *reply;
          pending_command_reply_ready_ = true;
          pending_command_signal_.notify_one();
        }
      }); //NOLINT

    std::unique_lock<std::mutex> lock(pending_command_mutex_);
    if (pending_command_signal_.wait_for(lock, timeout, [this] { return pending_command_reply_ready_; }))
    {
      return pending_scip_reply_;
    }
    else
    {
      throw std::runtime_error("Timed out waiting for scip response from the lidar.");
    }
  }

  /**
   * @brief Process Command Reply and notify sender
   *
   * @tparam T
   * @param packet
   */
  template <typename T>
  void processCommandReply(const T& packet)
  {
    std::lock_guard<std::mutex> lock(pending_command_mutex_);
    // Save the reply message in a state variable and signal that it is ready
    pending_command_reply_.set<T>(packet);
    pending_command_reply_ready_ = true;
    pending_command_signal_.notify_one();
  }

  /**
   * @brief Get Version Details
   *
   * @return The serial number + sensor model + protocol version as single string
   * @throws std::exception - If the send operation fails, or a valid reply is not received
   */
  std::string getVersionDetails();

  /**
   * @brief Get Sensor status (using XR command)
   *
   * @return The sensor status
   * @throws std::exception - If the send operation fails, or a valid reply is not received
   */
  protocol::sensing_data::SensingDataHeader getSensorStatus();

  /**
   * @brief Get Scan details
   *
   * @return The scan metadata required to assemble laser scan
   * @throws std::exception - If the send operation fails, or a valid reply is not received
   */
  ScanParameters getScanDetails();

  /**
   * @brief Register a callback method to be executed when a the specific packet
   * is received
   *
   * @param[in] callback - The callback to be executed in the io_service thread
   */
  template <typename TWorker>
  inline void registerCallback(typename TWorker::PacketEventCallback callback)
  {
    uam_packet_worker_.registerCallback<TWorker>(callback);
  }

  /**
   * @brief Start streaming laser scan.
   *
   * @throws std::exception - If the send operation fails, or a valid reply is not received
   */
  void startStreaming(const bool publish_intensity, const bool publish_multiecho = false);

  /**
   * @brief Subscribe to pointcloud
   *
   * @tparam[in] TWorker - Which worker to use for subscription
   */
  template <typename TWorker>
  void subscribe()
  {
    static_assert(ESubscriptionMode::INVALID != TypeToEnum<TWorker>(), "Invalid worker type");
    auto reply = sendCommandWithReply<TWorker>(command_timeout_);
    subscription_mode_ = TypeToEnum<TWorker>();
  }

  /**
   * @brief Unsubscribe to pointcloud
   *
   * @tparam[in] TWorker - Which worker to use for subscription
   */
  template <typename TWorker>
  void unsubscribe()
  {
    auto reply = sendCommandWithReply<TWorker>(command_timeout_);
    subscription_mode_ = ESubscriptionMode::INVALID;
  }

  /**
   * @brief
   */
  void stopStreaming();

  template <typename TAny>
  void unsubscribeCallback(const TAny message)
  {
    if (subscription_mode_ == ESubscriptionMode::INVALID)
    {
      ROS_WARN_STREAM("Bug, it should not be here!");
    }
    else
    {
      ROS_INFO_STREAM(
        "Stopping subscription, with: " << message.header[0] << message.header[1] << message.sub_header[0]
                                        << message.sub_header[1]);
      subscription_mode_ = ESubscriptionMode::INVALID;
    }
  }

  /**
   * @brief Handle Unsubscribe
   */
  void handleUnsubscribe();

  /**
   * @brief Packet Callback from the io handlers
   *
   * @param Stamped packet
   */
  inline void packetCallback(const uam::protocol::ShapeShifterPacket& packet, const ros::Time& wall_time)
  {
    auto success = uam_packet_worker_.processByHandler(packet, wall_time);
    ROS_ERROR_STREAM_COND(!success, "Failed processing packet");
  }

  /**
   * @brief
   * @param header
   * @return
   */
  inline bool filterCallback(const std::array<char, 2>& header)
  {
    return uam_packet_worker_.validateCommandHeader(header);
  }

  /**
   * @brief Get Safety Areas
   *
   * @param[in] area_type - Type of the safety area (check protocol::EYRAreaType)
   * @param[in] area_number - Safety area [1, to 32] (this depends on fw version)
   * @param[in] start step - 1 to 1081
   * @param[in] end_step - 1 to 1081
   * @return YRCommandReply if safety area was successfully decoded
   */
  std::optional<protocol::YRCommandReply> getSafetyArea(
    const protocol::EYRAreaType area_type,
    const uint16_t area_number,
    const uint32_t start_step,
    const uint32_t end_step);

  template <typename TWorker, typename... TArgs>
  inline bool asyncSend(TArgs&&... args)
  {
    return client_.asyncSend(uam_packet_worker_.getCommand<TWorker>(std::forward<TArgs>(args)...));
  }

private:
  /**
   * \defgroup io ASIO event loop variables for sending and receiving TCP packets
   * @{
   */

  /**
   * @brief The Boost IO Service object that manages the asynchronous operations
   */
  std::shared_ptr<boost::asio::io_service> io_service_;

  /**
   * @brief A dedicated thread for running the Boost ASIO event loop
   */
  std::thread io_thread_;

  /**
   * @brief A fake task to prevent the io_service from terminating until desired
   */
  std::unique_ptr<boost::asio::io_service::work> io_work_;

  /**
   * @brief connection client
   */
  uam::TcpClient client_;

  /**@}*/

  /**
   * \defgroup Packet Worker member
   * @{
   */

  /**
   * @brief UAM custom packet Worker
   */
  uam::UamPacketWorker uam_packet_worker_;

  /**
   * @brief Scip PP packet worker
   */
  uam::PPWorker scip_pp_worker_;

  /**@}*/

  /**
   * @brief Timeout used when waiting for a response from the lidar
   */
  std::chrono::duration<double> command_timeout_;

  /**
   * @brief Flag to store message subscription mode
   */
  ESubscriptionMode subscription_mode_;

  /**
   * @brief Pending command mutex
   */
  std::mutex pending_command_mutex_;

  /**
   * @brief Flag indicating the reply is ready for the pending command
   */
  bool pending_command_reply_ready_;

  /**
   * @brief Condition variable used to signal the main thread once the pending command reply has been populated
   */
  std::condition_variable pending_command_signal_;

  /**
   * @brief The reply message associated with the pending command
   *
   * This is populated by the receive event sequence after the pending command has been sent.
   */
  protocol::ShapeShifterPacket pending_command_reply_;

  /**
   * @brief The reply message associated with the pending scip command
   *
   * This is populated by the receive event sequence after the pending command has been sent.
   */
  scip_protocol::PPReply pending_scip_reply_;
};
}  // namespace uam

#endif  // UAM_UAM_DRIVER_H
