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

#ifndef UAM_WORKERS_UAM_WORKER_BASE_H
#define UAM_WORKERS_UAM_WORKER_BASE_H

#include <ros/ros.h>
#include <uam/protocol_types/uam_protocol_types.h>
#include <uam/uam_error_codes.h>
#include <uam/uam_visitors.h>

#include <boost/crc.hpp>
#include <iomanip>
#include <string>

#include <optional>

namespace uam
{
/**
 * @brief This template class for UAM workers. To decode each individual reply, each
 * child will have to implement custom decode methods. To decode each packet there
 * are two options, or sending a byte array in the for of a std::string or sending
 * already the raw_reply. This last approach does not require visitor members.
 *
 * @tparam TDerived - CRTP Pattern to allow for static dispatch whenever possible.
 * @tparam HeaderMSB - This is the MSByte in the Command header
 * @tparam HeaderLSB - This is the LSByte in the Command header
 * @tparam SubHeaderMSB - This is the MSByte in the Command sub header
 * @tparam SubHeaderLSB - This is the LSByte in the Command sub header
 * @tparam TReply - The reply type for the shild worker
 */
template <typename TDerived, char HeaderMSB, char HeaderLSB, char SubHeaderMSB, char SubHeaderLSB, typename TReply>
class WorkerBase
{
public:
  /**
   * @brief provide access to the request type
   */
  using Request = protocol::CommandRequest;

  /**
   * @brief Make Reply type public
   */
  using Reply = TReply;

  /**
   * @brief Make Reply type public
   */
  using ReplyHeader = decltype(TReply::header);

  /**
   * @brief Make Reply type public
   */
  using ReplyFooter = decltype(TReply::footer);

  /**
   * @brief Packet callback type
   */
  using PacketEventCallback = std::function<void(const Reply, const ros::Time&)>;

  /**
   * @brief Return encoded request
   *
   * @return request message
   */
  inline const std::string& getCommand() const { return encoded_request_; }

  /**
   * @brief Validate if the reply is of the type TDerived
   *
   * @param[in] header - Command reply header
   * @return true if the reply is of the type TDerived::Reply, false otherwise
   */
  inline bool validateCommandHeader(const std::array<char, 2>& header) const
  {
    return header[0] == HeaderMSB && header[1] == HeaderLSB;
  }

  /**
   * @brief Validate if the reply is of the type TDerived
   *
   * @param[in] header - Command reply header
   * @return true if the reply is of the type TDerived::Reply, false otherwise
   */
  inline bool validateReplyType(const protocol::CommandReplyHeader& header) const
  {
    return header.header[0] == HeaderMSB && header.header[1] == HeaderLSB && header.sub_header[0] == SubHeaderMSB &&
           header.sub_header[1] == SubHeaderLSB;
  }
  /**
   * @brief Method to register the callback to be called by the derived class
   * when processing the packet.
   *
   * @param[in] callback - Callback to execute when finalising processImpl call
   */
  inline void registerEventCallback(PacketEventCallback callback) { callback_ = callback; }

  /**
   * @brief Process raw buffer and return decoded message
   *
   * @param[in] buffer - Raw buffer
   * @return Reply decoded message if successful, std::nullopt otherwise
   */
  std::optional<Reply> process(const std::string* buffer) const
  {
    // Before decoding the message we need to double check if we got the expected
    // message size
    auto recv_bytes = buffer->size();

    if (!static_cast<const TDerived*>(this)->validateSize(recv_bytes))
    {
        ROS_ERROR_STREAM("Failed to validate size");
    	return std::nullopt;
    }

    // Validate Status
    uint16_t status;
    header_visitor_.status.get(buffer, status);

    // Status check failed?
    if (!validateStatus(status))
    {
      return std::nullopt;
    }

    //  Validate CRC. We might get a different reply other than the official supported
    // one (different protocol version)
    bool is_crc_valid = false;

    if (!validateCrc(buffer))
    {
      return std::nullopt;
    }

    // decode everything else
    auto reply = static_cast<const TDerived*>(this)->decode(buffer);
    if (!reply.has_value())
    {
      ROS_ERROR_STREAM("Failed to decode message");
      return std::nullopt;
    }
    return reply;
  }

  /**
   * @brief Process raw received structure and return decoded message
   *
   * @param[in] raw_reply - Received raw reply
   * @return Reply decoded message if successful, std::nullopt otherwise
   */
  std::optional<Reply> process(const Reply& raw_reply) const
  {
    auto recv_bytes = raw_reply.header.cmd_size;
    // Decode number of received bytes
    decodeField(recv_bytes);
    // Decode status
    auto status = raw_reply.header.status;
    decodeField(status);

    // validate size
    bool is_size_valid = static_cast<const TDerived*>(this)->validateSize(recv_bytes);
    bool is_status_ok = validateStatus(status);
    // Did size check or status check failed?
    if (!is_size_valid || !is_status_ok)
    {
      return std::nullopt;
    }
    //  Validate CRC. We might get a different reply other than the official supported
    // one (different protocol version)
    bool is_crc_valid = false;

    if (sizeof(Reply) != recv_bytes)
    {
      auto tmp_buffer = std::string(reinterpret_cast<const char*>(&raw_reply), recv_bytes);
      is_crc_valid = validateCrc(&tmp_buffer);
    }
    else
    {
      is_crc_valid = validateCrc(raw_reply);
    }

    if (!is_crc_valid)
    {
      return std::nullopt;
    }

    // decode everything else
    auto reply = static_cast<const TDerived*>(this)->decode(raw_reply);
    if (!reply.has_value())
    {
      ROS_ERROR_STREAM("Failed to decode message");
      return std::nullopt;
    }
    return reply;
  }

  /**
   * @brief Process the incomming message using raw buffer
   *
   * @return true if message was successfully process, false otherwise
   */
  bool processByHandler(const std::string* buffer, const ros::Time& wall_time) const
  {
    auto reply = process(buffer);
    if (!reply.has_value())
    {
      ROS_ERROR_STREAM("Failed to decode message");
      return false;
    }
    if (callback_)
      callback_(*reply, wall_time);
    return true;
  }

  /**
   * @brief Process raw buffer and return decoded message
   *
   * @param[in] buffer - Raw buffer
   * @return Reply decoded message if successful, std::nullopt otherwise
   */
  bool processByHandler(const Reply& raw_reply, const ros::Time& wall_time) const
  {
    auto reply = process(raw_reply);
    if (!reply.has_value())
    {
      ROS_ERROR_STREAM("Failed to decode message");
      return false;
    }
    if (callback_)
      callback_(*reply, wall_time);
    else
      ROS_WARN_STREAM(
        "No handler for " << HeaderMSB << HeaderLSB << SubHeaderMSB << SubHeaderLSB << " and it is "
                          << (int)(callback_ == nullptr));

    return true;
  }

protected:
  /**
   * @brief Default C'tor
   *
   * This initialize
   *
   */
  explicit WorkerBase(
    const uint32_t header_offset = 0,
    const uint32_t footer_offset = sizeof(protocol::CommandReplyHeader)) :
    request_(Request { protocol::CommandRequestHeader { protocol::STX_ID,  // NOLINT
                                                        sizeof(Request),
                                                        { HeaderMSB, HeaderLSB },
                                                        { SubHeaderMSB, SubHeaderLSB } },
                       protocol::CommandFooter { 0, protocol::ETX_ID } }),
    header_visitor_(header_offset),
    footer_visitor_(footer_offset)
  {
    request_.footer.crc = calculateCrc(request_);
    encoded_request_ = encodeCommand(request_);
  }

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

  /**
   * @brief Method to encode the command type Request
   *
   * @param[in] request - Request to encode.
   * @return encoded command
   */
  static std::string encodeCommand(const Request& request)
  {
    std::string encoded_request;
    encoded_request.reserve(sizeof(Request));
    encoded_request += request.header.stx;
    encoded_request += toHexString(request.header.cmd_size);
    encoded_request += request.header.header[0];
    encoded_request += request.header.header[1];
    encoded_request += request.header.sub_header[0];
    encoded_request += request.header.sub_header[1];
    encoded_request += toHexString(request.footer.crc);
    encoded_request += request.footer.etx;
    return encoded_request;
  }

  /**
   * @brief Calculate Reply CRC using raw buffer
   *
   * @param[in] buffer - Input buffer
   * @return[out] crc value
   */
  uint16_t calculateReplyCrc(const std::string* buffer) const
  {
    // validate crc with expected crc:
    const auto buffer_size = buffer->size();
    const auto crc_buffer_size =
      buffer_size - (sizeof(protocol::CommandReplyHeader::stx) + sizeof(protocol::CommandFooter));
    return calculateCrc(
      buffer->substr(sizeof(protocol::CommandReplyHeader::stx), buffer_size - sizeof(protocol::CommandFooter)).data(),
      crc_buffer_size);
  }

  /**
   * @brief Calculate crc for the request type
   *
   * @param[in] msg - Caculate CRC
   * @return crc value
   */
  uint16_t calculateCrc(const Request& msg) const
  {
    // CRC Calculation
    // create a tmp string of cmd and header
    std::string size_cmd_str = toHexString(msg.header.cmd_size) + msg.header.header[0] + msg.header.header[1] +
                               msg.header.sub_header[0] + msg.header.sub_header[1];
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
  uint16_t calculateCrc(const char* buffer, const std::size_t& byte_count) const
  {
    boost::crc_optimal<16, 0x1021, 0, 0, true, true> crc_kermit_type;
    crc_kermit_type.process_bytes(buffer, byte_count);
    return crc_kermit_type.checksum();
  }

  /**
   * @brief Calulate Reply CRC using structure
   *
   * @param[in] message - Message
   * @return crc value
   */
  template <typename T>
  uint16_t calculateReplyCrc(const T message) const
  {
    // validate crc with expected crc:
    const auto crc_buffer_size =
      sizeof(T) - (sizeof(protocol::CommandReplyHeader::stx) + sizeof(protocol::CommandFooter));
    return calculateCrc(
      reinterpret_cast<const char*>(&message) + sizeof(protocol::CommandReplyHeader::stx),
      crc_buffer_size);
  }

  /**
   * @brief Validate reply message, namely buffer size, expected crc vs calculated crc
   * using raw buffer. CRC and status are decoded to the reply message
   *
   * @param[in] buffer - Input buffer
   * @param[out] reply - Output message with header and footer decoded
   * @return true if crc and size are valid
   */
  bool validateCrc(const std::string* buffer) const
  {
    // Calculate reply crc
    auto current_crc = calculateReplyCrc(buffer);
    uint32_t decoded_crc;
    footer_visitor_.crc.get(buffer, decoded_crc);

    if (current_crc != decoded_crc)
    {
      ROS_WARN_STREAM("Invalid CRC. Calculated CRC: " << current_crc << " expected is: " << decoded_crc);
      return false;
    }
    // Parse status
    return true;
  }

  /**
   * @brief Validate reply message, namely buffer size, expected crc vs calculated crc
   * using the reply structure. CRC and status are decoded to the reply message
   *
   * @param[in/out] reply - Raw reply (not decoded)
   * @return true if crc and size are valid false otherwise
   */
  bool validateCrc(const Reply& reply) const
  {
    // Calculate reply crc;
    auto current_crc = calculateReplyCrc(reply);
    auto decoded_crc = reply.footer.crc;
    decodeField(decoded_crc);
    if (current_crc != decoded_crc)
    {
      ROS_WARN_STREAM("Invalid CRC. Calculated CRC: " << current_crc << " expected is: " << decoded_crc);
      return false;
    }
    return true;
  }

  /**
   * @brief Decode header
   * @param reply
   * @return
   */
  void decodeHeaderAndFooter(Reply& reply) const
  {
    decodeField(reply.header.cmd_size);
    decodeField(reply.header.status);
    decodeField(reply.footer.crc);
  }

  /**
   * @brief Decode header
   * @param reply
   * @return
   */
  void decodeHeaderAndFooter(const std::string* buffer, Reply& reply) const
  {
    header_visitor_.cmd_size.get(buffer, reply.header.cmd_size);
    header_visitor_.status.get(buffer, reply.header.status);
    footer_visitor_.crc.get(buffer, reply.footer.crc);
  }

  /**
   * @brief Validate status field.
   * Here, in case the status field is not 0, an Error code is looked up in the
   * error_code::StatusErrorCodeToString table. YR command should not use this
   * as the error codes are different
   *
   * @param[in] reply - Decoded reply
   * @return[out] true if message is valid, false otherwise
   */
  inline bool validateStatus(const uint16_t status) const
  {
    // Check if status is ok
    if (status != 0)
    {
      std::string error_msg;
      if (error_codes::StatusErrorCodeToString.count(status))
      {
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
   * @brief Decode sensing data
   * Method placed here as most of the ARXX commands have this.
   *
   * @param[out] sensing_data - Decoded sensing data
   */
  void decodeSensingData(protocol::sensing_data::SensingDataHeader& sensing_data) const
  {
    decodeField(sensing_data.operating_mode);
    decodeField(sensing_data.area_number);
    sensing_data.area_number += 1;

    // Grab the Error Status
    decodeField(sensing_data.error_state);
    // Grab the error code and offset by 0x40 if non-zero as per documentation
    decodeField(sensing_data.error_code);
    if (sensing_data.error_code != 0)
    {
      sensing_data.error_code += 0x40;
    }
    // Grab the lockout_state
    decodeField(sensing_data.lockout_state);
    decodeField(sensing_data.ossd1_state);
    decodeField(sensing_data.ossd2_state);
    decodeField(sensing_data.warning1_state);
    decodeField(sensing_data.warning2_state);
    decodeField(sensing_data.ossd3_state);
    decodeField(sensing_data.ossd4_state);

    decodeField(sensing_data.muting_state1);
    decodeField(sensing_data.muting_state2);
    decodeField(sensing_data.reset_request1);
    decodeField(sensing_data.reset_request2);
    decodeField(sensing_data.encoder_speed);
    decodeField(sensing_data.timestamp);
    decodeField(sensing_data.laser_state_off);
    decodeField(sensing_data.optical_window_contaminated);
  }

  /**
   * @brief Encoded request.
   */
  std::string encoded_request_;

  /**
   * @brief Request command which can be computed at construction time
   */
  Request request_;

  /**
   * @brief Header Visitor to validate CRC and status (used for buffer decoded option)
   */
  CommandHeaderVisitor header_visitor_;

  /**
   * @brief Footer Visitor (used for buffer decoded option)
   */
  CommandFooterVisitor footer_visitor_;

  /**
   * @brief Callback to execute when processing the packet
   */
  PacketEventCallback callback_ { nullptr };
};

}  // namespace uam

#endif  // UAM_WORKERS_UAM_WORKER_BASE_H
