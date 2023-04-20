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

namespace urg_node
{
struct ScanDetails
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

struct AreaDataUint32_t
{
  union
  {
    struct
    {
      uint32_t reserved_bits : 17;
      uint32_t value : 15;
    } s;
    uint32_t v;
  };
};

static_assert(sizeof(AreaDataUint32_t) == 4);

/**
 * @brief
 */
class UamDriver
{
public:
	/**
	 *
	 * @param ip_address
	 * @param ip_port
	 */
  UamDriver(const std::string& ip_address, const int ip_port) :
    worker_(),
    client_(std::bind(&UamDriver::packetCallback, this, std::placeholders::_1))
  {
    // Try to connect to the lidar
    bool success = client_.connect(ip_address, ip_port);
    if (!success)
    {
      std::stringstream ss;
      ss << "Could not open network Hokuyo:\n";
      ss << ip_address << ":" << ip_port << "\n";
      throw std::runtime_error(ss.str());
    }

    pub_test_ = ros::NodeHandle().advertise<sensor_msgs::LaserScan>("test_scan", 10);
    worker_.registerCallback<uam::AR01Worker>(std::bind(&UamDriver::publishScan, this, std::placeholders::_1));
  }

  /**
   * @brief Packet Callback from the io handlers
   * @param Stamped packet
   */
  inline void packetCallback(const uam::protocol::ShapeShifterBuffer& packet)
  {
    auto success = worker_.processByHandler(packet);
    ROS_ERROR_STREAM_COND(!success, "Failed processing packet");
  }


  inline void processYR(const uam::protocol::YRCommandReply& message)
  {
    sensor_msgs::LaserScan msg;
    msg.header.frame_id = scan_details_.frame_id;
    msg.angle_min = scan_details_.angle_min;
    msg.angle_max = scan_details_.angle_max;
    msg.angle_increment = scan_details_.angle_increment;
    msg.scan_time = scan_details_.scan_period;
    msg.time_increment = scan_details_.time_increment;
    msg.range_min = scan_details_.range_min;
    msg.range_max = scan_details_.range_max;

    // Grab scan
    long time_stamp = 0;
    unsigned long long system_time_stamp = 0;

    // Fill scan
    if (synchronize_time_)
    {
      msg.header.stamp = ros::Time::now();
    }
    else
    {
      msg.header.stamp.fromNSec((uint64_t)system_time_stamp);
    }
    // msg.header.stamp = msg.header.stamp + system_latency_ + user_latency_ + getAngularTimeOffset();
    msg.ranges.resize(message.area_data.size());

    for (size_t i = 0; i < message.area_data.size(); i++)
    {
      AreaDataUint32_t range;
      range.v = message.area_data[i];
      auto adjusted_range = range.s.value;
      if (adjusted_range != 0)
      {
        msg.ranges[i] = /*range_offset_ +*/ static_cast<float>(adjusted_range) / 1000.0f;
      }
      else
      {
        msg.ranges[i] = std::numeric_limits<float>::quiet_NaN();
        continue;
      }
    }
    pub_test_.publish(msg);
  }


  ~UamDriver()
  {
    if (started_continuous_mode_)
    {
      stop();
    }
    client_.disconnect();
  }

  void initialize()
  {
    if (started_continuous_mode_)
    {
      ROS_WARN_STREAM("Cannot initialize while continuous mode is on!");
      return;
    }
    // todo: missing high resolution
    // Register callback for 01
    worker_.registerCallback<uam::AR01Worker>(std::bind(&UamDriver::publishScan, this, std::placeholders::_1));
    // Check the sensor fw version
    validateVersionDetails();
    // Now that we are connected let us retrieve some information to understand what is the status
    // of the sensor
    manualUpdateSensorStatus();
    // Get sensor laser scan details (min angle, max angle, angle increment)
    getScanDetails();
    // Get and publish safety areas
    getSafetyAreas();
    // We can start
    can_start_ = true;
  }

  void getSafetyAreas()
  {
    auto reply = client_.syncSendAndReceive<decltype(yr_worker)::Reply>(
      yr_worker.getCommand(0, uam::EAreaType::protectetion_1, 0, 1080));
    if (!reply)
    {
      std::stringstream ss;
      ss << "Could not retrieve scan detailsn";
      throw std::runtime_error(ss.str());
    }
    processYR(*reply);
    ROS_WARN_STREAM("Done for protection zone 1");
    ROS_WARN_STREAM("Done for protection zone 2");
    ROS_WARN_STREAM("Done for warning zone 1");
    ROS_WARN_STREAM("Done for warning zone 2");
  }

  void fillScanDetails(const uam::scip_protocol::PPReply& scan_params)
  {
    scan_details_.range_min = scan_params.min_distance / 1000.0;
    scan_details_.range_max = scan_params.max_distance / 1000.0;
    auto min_step = scan_params.start_step - scan_params.front_data_index;
    auto max_step = scan_params.end_step - scan_params.front_data_index;
    scan_details_.angle_increment = 2.0 * M_PI / scan_params.angular_resolution;
    scan_details_.angle_min = (2.0 * M_PI) * min_step / scan_params.angular_resolution;
    scan_details_.angle_max = (2.0 * M_PI) * max_step / scan_params.angular_resolution;
    scan_details_.frame_id = "scan";
    scan_details_.scan_period = 60.0 / static_cast<double>(scan_params.rpm);
    // Urg_c does the following for timeout
    // (1000 * 1000 * 60 / scan_params.rpm) >> (10 - 4)  which gives 468 ("milliseconds" I believe)
    // We should do 2x1/Scanning frequency (ignoring scan_skip mode for now).
    scan_details_.timeout = 2 * scan_details_.scan_period * 1.e3;
    auto circle_fraction = (scan_details_.angle_max - scan_details_.angle_min) / (2.0 * M_PI);
    scan_details_.time_increment =
      circle_fraction * scan_details_.scan_period / static_cast<double>(max_step - min_step);
  }

  void getScanDetails()
  {
    // This needs to be with scip
    if (started_continuous_mode_)
      return;

    auto reply = sendScipCommandWithReply<uam::PPWorker>();

    if (!reply)
    {
      std::stringstream ss;
      ss << "Could not retrieve scan detailsn";
      throw std::runtime_error(ss.str());
    }
    fillScanDetails(*reply);
  }

  void start()
  {
    if (started_continuous_mode_ || !can_start_)
      return;
    auto reply = sendCommandWithReply<uam::AR04Worker>();
    if (!reply)
    {
      std::stringstream ss;
      ss << "Failed to subscribe to pointcloud!";
      throw std::runtime_error(ss.str());
    }

    client_.startAsyncRead();
    ROS_INFO_STREAM("Subscription successful!");
    started_continuous_mode_ = true;
  }

  void stop()
  {
    if (!started_continuous_mode_ || !can_start_)
      return;

    auto raw_reply = client_.syncSendAndReceive<uam::AR05Worker::Reply>(worker_.getCommand<uam::AR05Worker>());
    if (!raw_reply || !worker_.process<uam::AR05Worker>(*raw_reply))
    {
      ROS_WARN_STREAM("Failed to unsubscribe to pointcloud!");
      return;
    }
    started_continuous_mode_ = false;
  }

  void publishScan(const uam::protocol::AR01CommandReply& reply)
  {
    sensor_msgs::LaserScan msg;
    msg.header.frame_id = scan_details_.frame_id;
    msg.angle_min = scan_details_.angle_min;
    msg.angle_max = scan_details_.angle_max;
    msg.angle_increment = scan_details_.angle_increment;
    msg.scan_time = scan_details_.scan_period;
    msg.time_increment = scan_details_.time_increment;
    msg.range_min = scan_details_.range_min;
    msg.range_max = scan_details_.range_max;

    // Grab scan
    long time_stamp = 0;
    unsigned long long system_time_stamp = 0;

    // Fill scan
    if (synchronize_time_)
    {
      msg.header.stamp = ros::Time::now();
    }
    else
    {
      msg.header.stamp.fromNSec((uint64_t)system_time_stamp);
    }
    // msg.header.stamp = msg.header.stamp + system_latency_ + user_latency_ + getAngularTimeOffset();
    msg.ranges.resize(reply.ranges.size());
    msg.intensities.resize(reply.intensities.size());

    for (size_t i = 0; i < reply.ranges.size(); i++)
    {
      auto& range = reply.ranges[i];
      auto& intensity = reply.intensities[i];

      // According to the doc:
      // 1. Values more than 40000 are error code (0xFFFF).
      // 2. If object is not detected value will be 65534 (0xFFFE)
      // 3. If object is at a very close range the value will be 65533 (0xFFFD).
      // 4. When the device is in laser off state the value will be 65532 (0xFFFC)
      if (range != 0 && range < 0xFFFC)
      {
        msg.ranges[i] = /*range_offset_ +*/ static_cast<float>(reply.ranges[i]) / 1000.0f;
        msg.intensities[i] = reply.intensities[i];
      }
      else
      {
        msg.ranges[i] = std::numeric_limits<float>::quiet_NaN();
        continue;
      }
    }
    pub_test_.publish(msg);
  }

  void manualUpdateSensorStatus()
  {
    if (started_continuous_mode_)
      return;

    auto status_reply = sendCommandWithReply<uam::XR00Worker>();
    if (!status_reply)
    {
      std::stringstream ss;
      ss << "Could not retrieve sensor status for Hokuyo:\n";
      throw std::runtime_error(ss.str());
    }
    last_received_status_->area_number = status_reply->data.area_number;
    last_received_status_->error_code = status_reply->data.error_code;
    last_received_status_->error_status = status_reply->data.error_state;
    last_received_status_->lockout_status = status_reply->data.lockout_state;
    last_received_status_->operating_mode = status_reply->data.operating_mode;
    last_received_status_->optical_window_contaminated = status_reply->data.optical_window_contaminated;
    last_received_status_->ossd1_state = status_reply->data.ossd1_state;
    last_received_status_->ossd2_state = status_reply->data.ossd1_state;
    last_received_status_->warning1_state = status_reply->data.warning1_state;
    last_received_status_->warning2_state = status_reply->data.warning2_state;
    ROS_WARN_STREAM_COND(
      status_reply->data.lockout_state,
      "Sensor in lockout state! Lidar pw reset might be required!");
  }

  template <typename TWorker>
  std::optional<typename TWorker::Reply> sendCommandWithReply()
  {
    auto raw_reply = client_.syncSendAndReceive<typename TWorker::Reply>(worker_.getCommand<TWorker>());
    if (raw_reply)
    {
      return worker_.process<TWorker>(*raw_reply);
    }
    return std::nullopt;
  }

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

  /**
   *
   * @return
   */
  void validateVersionDetails()
  {
    auto reply = sendCommandWithReply<uam::VR00Worker>();
    if (!reply.has_value())
    {
      std::stringstream ss;
      ss << "Could not Request Version details for Hokuyo:\n";
      throw std::runtime_error(ss.str());
    }
    // Should we check against a min version?
    ROS_INFO_STREAM(
      "Firmware_version is: " << std::string(
        reply->version_details.firmware_version.data(),
        reply->version_details.firmware_version.size()));
    ROS_INFO_STREAM(
      "sensor_model is: " << std::string(
        reply->version_details.sensor_model.data(),
        reply->version_details.sensor_model.size()));
    ROS_INFO_STREAM(
      "serial_number is: " << std::string(
        reply->version_details.serial_number.data(),
        reply->version_details.serial_number.size()));
  }

private:
  std::optional<urg_node::Status> last_received_status_;
  bool started_continuous_mode_ { false };
  bool can_start_ { false };
  ScanDetails scan_details_;
  bool use_intensity_ { true };
  bool use_high_resolution_ { false };
  bool synchronize_time_ { true };

  /**
   * @brief connection client
   */
  uam::TcpClient client_;

  uam::UamPacketWorker worker_;
  uam::YRWorker yr_worker;
  uam::PPWorker scip_pp_worker_;
  ros::Publisher pub_test_;
};
}  // namespace urg_node

#endif  // INCLUDE_URG_NODE_UAM_DRIVER
