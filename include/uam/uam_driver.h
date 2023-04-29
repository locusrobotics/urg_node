/*
 * scan_worker.cpp
 *
 *  Created on: 03/04/2023
 *      Author: cribeiromendes
 */

#ifndef INCLUDE_URG_NODE_UAM_DRIVER
#define INCLUDE_URG_NODE_UAM_DRIVER

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/make_shared.hpp>
#include <boost/system/error_code.hpp>
#include <uam/io/tcp_client.h>
#include <urg_c/urg_connection.h>
#include <urg_node/Status.h>

#include <sensor_msgs/LaserScan.h>
#include <string>
#include <uam/workers/uam_command_worker.h>
#include <uam/workers/yr_worker.h>

#include <boost/algorithm/string/finder.hpp>
#include <boost/algorithm/string/iter_find.hpp>

#include <uam/workers/scip_worker.h>

namespace uam
{
/**
 * @brief Scan parameters structure
 */
struct ScanParameters
{
  std::string frame_id { "" };
  int timeout;
  int extended_timeout;
  double angle_min { 0. };
  double angle_max { 0. };
  double angle_increment { 0. };
  double time_increment { 0. };
  double scan_period { 0. };
  double range_min { 0. };
  double range_max { 0. };
};

//:
  //    worker_(),
  //    client_(std::bind(&UamDriver::packetCallback, this, std::placeholders::_1))
  //  {
  //    // Try to connect to the lidar
  //    bool success = client_.connect(ip_address, ip_port);
  //    if (!success)
  //    {
  //      std::stringstream ss;
  //      ss << "Could not open network Hokuyo:\n";
  //      ss << ip_address << ":" << ip_port << "\n";
  //      throw std::runtime_error(ss.str());
  //    }
  //
  //    pub_test_ = ros::NodeHandle().advertise<sensor_msgs::LaserScan>("test_scan", 10);
  //    worker_.registerCallback<uam::AR01Worker>(std::bind(&UamDriver::publishScan, this, std::placeholders::_1));
  //  }


/**
 * @brief Hokuyo UAM-05LP Network Driver
 */
class UamDriver
{
  /**
   * @brief Subscription Mode Enum
   */
  enum ESubscriptionMode : uint32_t
  {
    AR02 = 0,/**< AR02 */
    AR04,/**< AR04 */
    AR07,/**< AR07 */
    INVALID  /**< MAX */
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
  inline bool isTheSensorStreaming() const { return this->subscription_mode_.load() != ESubscriptionMode::INVALID; }

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
   * @brief Convert scan parameters
   *
   * @param[in] scan_params - Scan Parameters
   * @return Scan parameters
   */
  ScanParameters convertScanParameters(const uam::scip_protocol::PPReply& scan_params) const;

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

  template <typename TAny>
  void subscribeCallback(const typename TAny::Reply message)
  {
    subscription_mode_.store(TypeToEnum<TAny>());
  }

  /**
   * @brief
   */
  void stopStreaming();

  template <typename TAny>
  void unsubscribeCallback(const TAny message)
  {
    auto last = subscription_mode_.load();
    if (last == ESubscriptionMode::INVALID)
    {
      ROS_WARN_STREAM("Bug, it should not be here!");
    }
    else
    {
      ROS_INFO_STREAM(
        "Stopping subscription, with: " << message.header[0] << message.header[1] << message.sub_header[0]
                                        << message.sub_header[1]);
      subscription_mode_.store(ESubscriptionMode::INVALID);
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
  inline void packetCallback(const uam::protocol::ShapeShifterBuffer& packet)
  {
    auto success = uam_packet_worker_.processByHandler(packet);
    ROS_ERROR_STREAM_COND(!success, "Failed processing packet");
  }

  inline bool filterCallback(const uam::protocol::CommandReplyHeader& header)
  {
    return uam_packet_worker_.validateReplyHeader(header);
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

  /**
   * @brief Send Uam custom command and wait for the reply
   *
   * @tparam TWorker
   * @return
   */
  template <typename TWorker, typename... TArgs>
  std::optional<typename TWorker::Reply> syncSendCommandWithReply(const TArgs... args)
  {
    auto raw_reply = client_.syncSendAndReceive<typename TWorker::Reply>(uam_packet_worker_.getCommand<TWorker>(args...));
    if (raw_reply)
    {
      return uam_packet_worker_.process<TWorker>(*raw_reply);
    }
    return std::nullopt;
  }

  template <typename TWorker, typename... TArgs>
  inline bool asyncSend(TArgs&&... args)
  {
    return client_.asyncSend(uam_packet_worker_.getCommand<TWorker>(std::forward<TArgs>(args)...));
  }

  /**
   * @brief Send SCIP command and wait for Reply
   *
   * @return Reply if successful received, std::nullopt otherwise
   */
  template <typename TWorker>
  std::optional<typename TWorker::Reply> sendScipCommandWithReply()
  {
    auto raw_reply = client_.syncSendAndReadLine<typename TWorker::RawReply>(scip_pp_worker_.getCommand());
    if (raw_reply)
    {
      return scip_pp_worker_.process(*raw_reply);
    }
    return std::nullopt;
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
   * @brief Last Received Scan parameters
   */
  std::optional<uam::scip_protocol::PPReply> last_received_scan_params_;

  /**
   * @brief Last received sensing data status
   */
  std::optional<protocol::sensing_data::SensingDataHeader> last_received_status_;

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
   * @brief Flag to store message subscription mode
   */
  std::atomic<ESubscriptionMode> subscription_mode_;

  /**
   * @brief Scan details which are used to assemble laser scan
   */
  // ScanDetails scan_details_;

  /**
   * @brief Safety areas
   */
  std::vector<sensor_msgs::LaserScan> safety_areas_;

  ros::Publisher pub_test_;
};
}  // namespace uam

#endif  // INCLUDE_URG_NODE_UAM_DRIVER
