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

#ifndef INCLUDE_URG_NODE_UAM_UAM_WORKER_BASE_H_
#define INCLUDE_URG_NODE_UAM_UAM_WORKER_BASE_H_

#include <ros/ros.h>
#include <urg_node/uam/uam_protocol_types.h>
#include <urg_node/uam/uam_visitors.h>

#include <boost/crc.hpp>
#include <iomanip>
#include <string>

namespace uam
{
/**
 * @brief
 * @tparam T
 */
template <char HeaderMSB, char HeaderLSB, char SubHeaderMSB, char SubHaderLSB>
class WorkerBase
{
  static constexpr uint8_t STX_ID = 0x02;
  static constexpr uint8_t ETX_ID = 0x03;

public:
  /**
   * @brief provide access to the request type
   */
  using Request = protocol::CommandRequest;

protected:
  /**
   * @brief Default C'tor
   *
   * This initialize
   *
   */
  WorkerBase(const uint32_t header_offset = 0, const uint32_t footer_offset = sizeof(protocol::CommandReplyHeader)) :
    request_(Request { protocol::CommandRequestHeader { STX_ID,
                                                        sizeof(Request),
                                                        { HeaderMSB, HeaderLSB },
                                                        { SubHeaderMSB, SubHaderLSB }},
                       protocol::CommandFooter { 0, ETX_ID } }),
    header_visitor_(header_offset),
	footer_visitor_(footer_offset)
  {
    request_.footer.crc = calculateCrc(request_);
    encoded_request_ = encodeCommand(request_);
  }

  template <typename T>
  static std::string to_hex_string(const T& i, const size_t& size = sizeof(T))
  {
    std::stringstream stream;
    stream << std::uppercase << std::setfill('0') << std::setw(size) << std::hex << i;
    return stream.str();
  }

  /**
   * @brief Method to encode the command type Request
   *
   * @param[in] request - Request to encode.
   * @return
   */
  static std::string encodeCommand(const Request& request)
  {
    std::string encoded_request;
    encoded_request.reserve(sizeof(Request));
    encoded_request += request.header.stx;
    encoded_request += to_hex_string(request.header.cmd_size);
    encoded_request += request.header.header[0];
    encoded_request += request.header.header[1];
    encoded_request += request.header.sub_header[0];
    encoded_request += request.header.sub_header[1];
    encoded_request += to_hex_string(request.footer.crc);
    encoded_request += request.footer.etx;
    return encoded_request;
  }

  uint16_t calculateReplyCrc(const std::string* buffer)
  {
    // validate crc with expected crc:
    const auto buffer_size = buffer->size();
    const auto crc_buffer_size =
      buffer_size - (sizeof(protocol::CommandReplyHeader::stx) + sizeof(protocol::CommandFooter));
    return calculateCrc(
      buffer->substr(sizeof(protocol::CommandReplyHeader::stx), buffer_size - sizeof(protocol::CommandFooter)).data(),
      crc_buffer_size);
  }

  template <typename T>
  bool validate(const std::string* buffer, T& reply)
  {
    // Process it is faster to calculate the CRC with the raw byte array:
    const auto buffer_size = buffer->size();
    if (buffer_size != sizeof(T))
    {
      ROS_WARN_STREAM(
        "Invalid buffer size. Current buffer size is: " << buffer_size << " and expected is: " << sizeof(T));
      return false;
    }
    // Calculate reply crc
    auto current_crc = calculateReplyCrc(buffer);
    footer_visitor_.crc.get(buffer, reply.footer.crc);
    if (current_crc != reply.footer.crc)
    {
      ROS_WARN_STREAM("Invalid CRC. Calculated CRC: " << current_crc << " expected is: " << reply.footer.crc);
      return false;
    }
    // Check if status is ok
    header_visitor_.status.get(buffer, reply.header.status);
    if (reply.header.status != 0)
    {
      ROS_ERROR_STREAM("Received bad status!");
      return false;
    }
    return true;
  }
  /**
   * @brief Calculate crc for the request type
   *
   * @param msg
   */
  uint16_t calculateCrc(const Request& msg)
  {
    // CRC Calculation
    // create a tmp string of cmd and header
    std::string size_cmd_str = to_hex_string(msg.header.cmd_size) + msg.header.header[0] + msg.header.header[1] + msg.header.sub_header[0] + msg.header.sub_header[1];
    return calculateCrc(size_cmd_str.data(), size_cmd_str.size());
  }

  /**
   * @brief Calculate crc
   *
   * CRC Standard: Kermit
   * Polynomial: 0x1021
   * Shift Direction: Right
   * Initial Value: 0x0000
   * Byte Swap: Yes
   * Reverse CRC Result: Yes
   *
   * @param[in] buffer - Buffer
   * @param[in] byte_count - Number of bytes in the buffer to use
   * @return checksum
   */
  uint16_t calculateCrc(const char* buffer, const std::size_t& byte_count)
  {
    boost::crc_optimal<16, 0x1021, 0, 0, true, true> crc_kermit_type;
    crc_kermit_type.process_bytes(buffer, byte_count);
    return crc_kermit_type.checksum();
  }
  /**
   * @brief Encoded request. This is
   */
  std::string encoded_request_;
  /**
   * @brief Request command
   */
  Request request_;
  /**
   * @brief Header Visitor to validate CRC and status
   */
  CommandHeaderVisitor header_visitor_;
  /**
   * @brief Footer Visitor
   */
  CommandFooterVisitor footer_visitor_;

public:
  /**
   * @brief Return encoded request
   * @return request message
   */
  inline const std::string& getCommand() const { return encoded_request_; }
};

}  // namespace uam

#endif  // INCLUDE_URG_NODE_UAM_UAM_WORKER_BASE_H_
