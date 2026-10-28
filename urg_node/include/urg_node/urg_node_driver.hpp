/*
 * Copyright (c) 2013, Willow Garage, Inc.
 * Copyright (c) 2017, Clearpath Robotics, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * Author: Mike O'Driscoll
 */

#ifndef URG_NODE_URG_NODE_DRIVER_H
#define URG_NODE_URG_NODE_DRIVER_H

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <diagnostic_updater/publisher.hpp>
#include <laser_proc/laser_publisher.hpp>

#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>

#include <std_srvs/srv/trigger.hpp>

#include <urg_node_msgs/msg/status.hpp>

#include "urg_node/urg_c_wrapper.hpp"

namespace urg_node
{
class UrgNode : public rclcpp::Node
{
public:
  explicit UrgNode(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions());

  ~UrgNode();

  /**
   * @brief Start's the nodes threads to run the lidar.
   */
  void run();

  /**
   * @brief Trigger an update of the lidar's status
   * publish the latest known information about the lidar on latched topic.
   * @return True on update successful, false otherwise.
   */
  bool updateStatus();

private:
  void initSetup();
  bool connect();
  rcl_interfaces::msg::SetParametersResult paramChangeCallback(
    const std::vector<rclcpp::Parameter>& parameters);
  void calibrate_time_offset();
  void updateDiagnostics();
  void populateDiagnosticsStatus(diagnostic_updater::DiagnosticStatusWrapper &stat);
  void scanThread();
  bool sensorStatusOk(const std::string& status) const;
  void statusCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std_srvs::srv::Trigger::Request::SharedPtr req,
    const std_srvs::srv::Trigger::Response::SharedPtr res);

  std::thread run_thread_;
  std::thread diagnostics_thread_;
  std::thread scan_thread_;

  std::unique_ptr<urg_node::URGCWrapper> urg_;
  diagnostic_updater::Updater diagnostic_updater_;
  std::unique_ptr<diagnostic_updater::HeaderlessTopicDiagnostic> laser_freq_;
  std::unique_ptr<diagnostic_updater::HeaderlessTopicDiagnostic> echoes_freq_;

  std::mutex lidar_mutex_;

  /**
   * @brief Detailed device status,
   */
  struct DeviceStatus
  {
    std::string status_str;
    std::string vendor_name;
    std::string product_name;
    std::string firmware_version;
    std::string firmware_date;
    std::string protocol_version;
    std::string device_id;
    /**
     * @brief Detailed status, when this is available (ex: UAM-05LP)
     */
    uam::protocol::sensing_data::SensingDataHeader detailed_status;
  } device_status_; // NOLINT

  int error_count_;
  double freq_min_;
  bool close_diagnostics_;
  std::atomic_bool close_scan_;

  int ip_port_;
  std::string ip_address_;
  std::string serial_port_;
  int serial_baud_;
  bool calibrate_time_;
  bool synchronize_time_;
  bool publish_intensity_;
  bool publish_multiecho_;
  int error_limit_;
  double diagnostics_tolerance_;
  double diagnostics_window_time_;
  bool detailed_status_;

  /** The laser output frame_id. */
  std::string frame_id_;
  /** A manually calibrated offset (in seconds) added to the timestamp before publication. */
  double time_offset_;
  /** Adjustment, in meters, for ranges returned at all angles. */
  float range_offset_;
  /** Controls the angle of the first range measurement, in radians. */
  double angle_min_;
  /** Controls the angle of the last range measurement, in radians. */
  double angle_max_;
  /** Number of adjacent range measurements to cluster into a single reading. */
  int cluster_;
  /** Number of input messages to skip between each output message. */
  int skip_;

  std::atomic_bool service_yield_;

  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_pub_;
  std::unique_ptr<laser_proc::LaserPublisher> echoes_pub_;
  rclcpp::Publisher<urg_node_msgs::msg::Status>::SharedPtr status_pub_;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr status_service_;

  // Need to hold reference to callback, or it gets deregistered
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameters_callback_handle_;
};

}  // namespace urg_node

#endif  // URG_NODE_URG_NODE_DRIVER_H
