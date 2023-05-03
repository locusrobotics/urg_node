/*
 * Copyright (c) 2013, Willow Garage, Inc.
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
 * Author: Chad Rockey
 */

#ifndef URG_NODE_URG_C_WRAPPER_H
#define URG_NODE_URG_C_WRAPPER_H
#include <stdexcept>
#include <sstream>
#include <vector>
#include <string>

#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/MultiEchoLaserScan.h>
#include <ros/ros.h>

#include <urg_c/urg_sensor.h>
#include <urg_c/urg_connection.h>
#include <urg_c/urg_utils.h>
#include <uam/protocol_types/uam_protocol_types.h>
#include <uam/uam_visitors.h>



namespace urg_node
{

class UrgDetectionReport
{
public:
  UrgDetectionReport()
  {
    status = 0;
    area = 0;
    distance = 0;
    angle = 0;
  }

  uint16_t status;
  uint16_t area;
  uint16_t distance;
  float angle;
};

class URGCWrapper
{
public:
  using UAMStatus = uam::protocol::AR00CommandReply;
  URGCWrapper(const std::string& ip_address, const int ip_port,
      bool& using_intensity, bool& using_multiecho, bool synchronize_time);

  URGCWrapper(const int serial_baud, const std::string& serial_port,
      bool& using_intensity, bool& using_multiecho, bool synchronize_time);

  ~URGCWrapper();

  void start();

  void stop();

  bool isStarted() const;

  double getRangeMin() const;

  double getRangeMax() const;

  double getAngleMin() const;

  double getAngleMax() const;

  double getAngleMinLimit() const;

  double getAngleMaxLimit() const;

  double getAngleIncrement() const;

  double getScanPeriod() const;

  double getTimeIncrement() const;

  std::string getIPAddress() const;

  int getIPPort() const;

  std::string getSerialPort() const;

  int getSerialBaud() const;

  std::string getVendorName();

  std::string getProductName();

  std::string getFirmwareVersion();

  std::string getFirmwareDate();

  std::string getProtocolVersion();

  std::string getDeviceID();

  ros::Duration getComputedLatency() const;

  ros::Duration getUserTimeOffset() const;

  std::string getSensorStatus();

  std::string getSensorState();

  void setFrameId(const std::string& frame_id);

  void setUserLatency(const double latency);

  bool setAngleLimitsAndCluster(double& angle_min, double& angle_max, int cluster);

  bool setRangeOffset(const float range_offset);

  void setSkip(int skip);

  ros::Duration computeLatency(size_t num_measurements);

  bool grabScan(const sensor_msgs::LaserScanPtr& msg);

  bool grabScan(const sensor_msgs::MultiEchoLaserScanPtr& msg);

  bool getAR00Status(uam::protocol::AR00CommandReply& status);

  bool getOtherStatus(const uint16_t req);

  bool getDL00Status(UrgDetectionReport& report);

private:
  void initialize(bool& using_intensity, bool& using_multiecho, bool synchronize_time);

  bool isIntensitySupported();

  bool isMultiEchoSupported();

  ros::Duration getAngularTimeOffset() const;

  ros::Duration getNativeClockOffset(size_t num_measurements);

  ros::Duration getTimeStampOffset(size_t num_measurements);

  /**
   * @brief Get synchronized time stamp using hardware clock
   * @param time_stamp The current hardware time stamp.
   * @param system_time_stamp The current system time stamp.
   * @returns ros::Time stamp representing synchronized time
   */
  ros::Time getSynchronizedTime(long time_stamp, long long system_time_stamp);

  /**
   * @brief Set the Hokuyo URG-04LX from SCIP 1.1 mode to SCIP 2.0 mode.
   * @returns True if successful and false if not.
   */
  bool setToSCIP2();

  /**
   * @brief calculate the crc of a given set of bytes.
   * @param bytes The bytes array to be processed.
   * @param size The size of the bytes array.
   * @return the calculated CRC of the bytes.
   */
  uint16_t checkCRC(const char* bytes, const uint32_t size);

  /**
   * @brief Send an arbitrary serial command to the lidar. These commands
   * can also be sent via the ethernet socket.
   * @param cmd The arbitrary command fully formatted to be sent as provided
   * @returns The textual response of the Lidar, empty if, but may return lidar's own error string.
   */
  std::string sendCommand(const std::string& cmd);

  template <typename T, typename TReply = typename T::Reply>
  bool sendAndReceive(T& worker, TReply& reply, const int timeout = 60)
  {
    bool restart = false;
    const decltype(uam::protocol::CommandReplyHeader::cmd_size) expected_size = sizeof(TReply);
    const auto& cmd = worker.getCommand();
    if (connection_write(&urg_.connection, cmd.c_str(), cmd.size()) < 0)
    {
      ROS_ERROR_STREAM("Failed to send message. Skipping!");
      return false;
    }
    std::string recv_buffer;
    recv_buffer.resize(expected_size);

    int recv_bytes = -1;
    ssize_t nr_bytes_read = 0;
    do
    {
      recv_bytes =
        connection_read(&urg_.connection, &recv_buffer.at(nr_bytes_read), expected_size - nr_bytes_read, urg_.timeout);
      if (recv_bytes <= 0)
      {
        ROS_ERROR("Read socket failed: %s", strerror(errno));
        recv_buffer.clear();
        return false;
      }
      nr_bytes_read += recv_bytes;
    } while (nr_bytes_read != expected_size && ros::ok());

//    auto opt_reply = worker.process(&recv_buffer);
//    if (!opt_reply.has_data())
//    {
//      ROS_ERROR_STREAM("Failed while parsing reply!");
//      return false;
//    }
//    reply = *opt_reply;
    return true;
  }

  /**
   * @brief Deserialize UAMStatus from received data (which has ASCII encoding)
   * @param f_buffer Received buffer
   * @param sensing_data Sensing data
   * @param start_position Start index in the buffer, case an offset is wanted
   * @return true if success, false otherwise
   */
  bool deserializeSensingData(const std::string& f_buffer, UAMStatus& sensing_data, const size_t& start_position = 0) const;

  std::string frame_id_;  ///< Output frame_id for each laserscan.

  urg_t urg_;
  std::atomic_bool started_;

  std::vector<long> data_;
  std::vector<unsigned short> intensity_;

  bool use_intensity_;
  bool use_multiecho_;
  urg_measurement_type_t measurement_type_;
  int first_step_;
  int last_step_;
  int cluster_;
  int skip_;
  float range_offset_;

  ros::Duration system_latency_;
  ros::Duration user_latency_;

  // used for clock synchronziation
  bool synchronize_time_;
  double hardware_clock_;
  long last_hardware_time_stamp_;
  double hardware_clock_adj_;
  const double adj_alpha_ = .01;
  uint64_t adj_count_;

  std::string ip_address_;
  int ip_port_;
  std::string serial_port_;
  int serial_baud_;
};
}  // namespace urg_node

#endif  // URG_NODE_URG_C_WRAPPER_H
