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


#ifndef URG_NODE_UAM_WORKERS_YR_WORKER_H
#define URG_NODE_UAM_WORKERS_YR_WORKER_H

#include <uam/protocol_types/uam_protocol_types.h>
#include <uam/uam_error_codes.h>
#include <uam/workers/uam_worker_base.h>

namespace uam
{
enum EAreaNumber : uint16_t
{
  MAX = 32 /**< MAX */
};

/**
 * @brief XR00 command worker
 */
class YRWorker
{
public:
  /**
   * @brief provide access to the request type
   */
  using Request = protocol::YRCommandRequest;

  /**
   * @brief Make Reply type public
   */
  using Reply = protocol::YRCommandReply;
  /**
   * @brief Packet callback type
   */
  using PacketEventCallback = std::function<void(const Reply)>;

  /**
   * @brief Method to register the callback to be called by the derived class
   * when processing the packet.
   *
   * @param[in] callback - Callback to execute when finalising processImpl call
   */
  inline void registerEventCallback(PacketEventCallback callback) { callback_ = callback; }

  /**
   * @brief
   * @param reply
   * @return
   */
  inline bool validateReplyType(const protocol::YRCommandReplyHeader& packet) const
  {
    return packet.header.header[0] == 'Y' && packet.header.header[1] == 'R';
  }

  /**
   * @brief
   * @param reply
   * @return
   */
  inline bool validateCommandHeader(const std::array<char,2>& header) const
  {
    return header[0] == 'Y' && header[1] == 'R';
  }
  /**
   * @brief
   *
   * @param[in] area_type -
   * @param[in] area_number - from 1 to TBD
   * @param[in] start_step
   * @param[in] end_step
   * @return
   */
  std::string getCommand(
    const protocol::EYRAreaType area_type,
    const uint16_t area_number,
    const uint32_t start_step,
    const uint32_t end_step)
  {
    Request request;
    request.header.stx = protocol::STX_ID;
    request.header.header[0] = 'Y';
    request.header.header[1] = 'R';
    request.header.cmd_size = sizeof(Request);
    request.header.area_number = area_number;
    request.header.area_type = area_type;
    request.header.start_step = start_step;
    request.header.end_step = end_step;
    request.header.resolution = 1;
    request.footer.etx = protocol::ETX_ID;
    request.footer.crc = calculateCrc(request);
    return encodeCommand(request);
  }


  std::optional<Reply> process(const Reply& raw_reply) const
  {
    Reply reply = raw_reply;
    decodeHeaderAndfooter(reply);
    decodeField(reply.area_data);
    return reply;
  }

  bool processByHandler(const Reply& raw_reply)
  {
    auto reply = process(raw_reply);
    if (!reply)
      return false;

    if (callback_)
    {
      callback_(*reply);
    }
    return true;
  }

protected:
  /**
   * @bief Convert value to Hexadecimal in string form
   *
   * @param[in] value - Value to be converted
   * @param[in] size - The number of bytes of the value
   * @return The hexadecimal value in string form
   */
  template <typename T>
  static std::string toHexString(const T& value, const size_t& size = sizeof(T))
  {
    std::stringstream stream;
    stream << std::uppercase << std::setfill('0') << std::setw(size) << std::hex << value;
    return stream.str();
  }

  static std::string encodeCommand(const Request& request)
  {
    std::string encoded_request;
    encoded_request.reserve(sizeof(Request));
    encoded_request += request.header.stx;
    encoded_request += toHexString(request.header.cmd_size);
    encoded_request += request.header.header[0];
    encoded_request += request.header.header[1];
    encoded_request += toHexString(request.header.area_type);
    encoded_request += toHexString(request.header.area_number);
    encoded_request += toHexString(request.header.start_step);
    encoded_request += toHexString(request.header.end_step);
    encoded_request += toHexString(request.header.resolution);
    encoded_request += toHexString(request.footer.crc);
    encoded_request += request.footer.etx;
    return encoded_request;
  }

  uint16_t calculateCrc(const Request& msg) const
  {
    // CRC Calculation
    // create a tmp string of cmd and header
    std::string size_cmd_str = toHexString(msg.header.cmd_size) + msg.header.header[0] + msg.header.header[1] +
                               toHexString(msg.header.area_type) + toHexString(msg.header.area_number) +
                               toHexString(msg.header.start_step) + toHexString(msg.header.end_step) +
                               toHexString(msg.header.resolution);
    return calculateCrc(size_cmd_str.data(), size_cmd_str.size());
  }

  uint16_t calculateReplyCrc(const Reply& message) const
  {
    // validate crc with expected crc:
    const auto crc_buffer_size =
      sizeof(Reply) - (sizeof(protocol::CommandReplyHeader::stx) + sizeof(protocol::CommandFooter));
    return calculateCrc(
      reinterpret_cast<const char*>(&message) + sizeof(protocol::CommandReplyHeader::stx),
      crc_buffer_size);
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
  uint16_t calculateCrc(const char* buffer, const std::size_t& byte_count) const
  {
    boost::crc_optimal<16, 0x1021, 0, 0, true, true> crc_kermit_type;
    crc_kermit_type.process_bytes(buffer, byte_count);
    return crc_kermit_type.checksum();
  }

  /**
   * @brief Validate reply message, namely buffer size, expected crc vs calculated crc
   * using the reply structure. CRC and status are decoded to the reply message
   *
   * @param[in/out] reply - Raw reply (not decoded)
   * @return true if crc and size are valid false otherwise
   */
  bool validateCrc(Reply& reply) const
  {
    // Calculate reply crc
    auto current_crc = calculateReplyCrc(reply);
    if (current_crc != reply.footer.crc)
    {
      ROS_WARN_STREAM("Invalid CRC. Calculated CRC: " << current_crc << " expected is: " << reply.footer.crc);
      return false;
    }
    // Parse status
    return true;
  }

  inline void decodeHeaderAndfooter(Reply& reply) const
  {
    decodeField(reply.header.header.cmd_size);
    decodeField(reply.header.header.area_type);
    decodeField(reply.header.header.area_number);
    decodeField(reply.header.header.start_step);
    decodeField(reply.header.header.end_step);
    decodeField(reply.header.header.resolution);
    decodeField(reply.header.status);
    decodeField(reply.footer.crc);
  }

  inline bool validateStatus(const uint16_t& status) const
  {
    // Check if status is ok
    if (status != 0)
    {
      std::string error_msg;
      if (error_codes::YRStatusErrorCodeToString.count(status))
      {
        error_msg = std::string("Received bad status: ") + error_codes::YRStatusErrorCodeToString.at(status);
      }
      else if (error_codes::StatusErrorCodeToString.count(status))
      {
        // If the error is not in the YR table, then it might be due to a malformed request, using the std status table
        error_msg = std::string("Received bad status: ") + error_codes::StatusErrorCodeToString.at(status);
      }
      else
      {
        error_msg = std::string("Received bad status: unknown error " + status);
      }
      ROS_ERROR_STREAM(error_msg);
      return false;
    }
    return true;
  }

  /**
   * @brief Callback to execute when processing the packet
   */
  PacketEventCallback callback_ { nullptr };
};

}  // namespace uam

#endif  // URG_NODE_UAM_WORKERS_YR_WORKER_H
