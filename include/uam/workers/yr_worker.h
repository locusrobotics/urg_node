/**
Software License Agreement (proprietary)
\file      xr_00_worker.h
\authors   Carlos Mendes <cribeiromendes@locusrobotics.com>
\copyright Copyright (c) (2023,), Locus Robotics Corp., All rights reserved.
Unauthorized copying of this file, via any medium, is strictly prohibited.
Proprietary and confidential.
**/

#ifndef INCLUDE_URG_NODE_UAM_SENSING_DATA_WORKERS_YR_WORKER_H_
#define INCLUDE_URG_NODE_UAM_SENSING_DATA_WORKERS_YR_WORKER_H_

#include <uam/protocol_types/uam_protocol_types.h>
#include <uam/uam_error_codes.h>
#include <uam/workers/uam_worker_base.h>

namespace uam
{
enum EAreaType : uint16_t
{
  protectetion_1 = 0,
  protectetion_2,
  warning_1,
  warning_2,
  muting_1,
  muting_2,
  reference_center,
  reference_max,
  reference_min,
  MAX
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


  std::string getCommand(
    const uint16_t area_type,
    const uint16_t area_number,
    const uint32_t start_step,
    const uint32_t end_step)
  {
    Request request;
    request.stx = 0x02;
    request.header[0] = 'Y';
    request.header[1] = 'R';
    request.cmd_size = sizeof(Request);
    request.area_number = area_number;
    request.area_type = area_type;
    request.start_step = start_step;
    request.end_step = end_step;
    request.resolution = 1;
    request.footer.etx = 0x03;
    request.footer.crc = calculateCrc(request);
    return encodeCommand(request);
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
    encoded_request += request.stx;
    encoded_request += toHexString(request.cmd_size);
    encoded_request += request.header[0];
    encoded_request += request.header[1];
    encoded_request += toHexString(request.area_type);
    encoded_request += toHexString(request.area_number);
    encoded_request += toHexString(request.start_step);
    encoded_request += toHexString(request.end_step);
    encoded_request += toHexString(request.resolution);
    encoded_request += toHexString(request.footer.crc);
    encoded_request += request.footer.etx;
    return encoded_request;
  }

  uint16_t calculateCrc(const Request& msg) const
  {
    // CRC Calculation
    // create a tmp string of cmd and header
    std::string size_cmd_str = toHexString(msg.cmd_size) + msg.header[0] + msg.header[1] + toHexString(msg.area_type) +
                               toHexString(msg.area_number) + toHexString(msg.start_step) + toHexString(msg.end_step) +
                               toHexString(msg.resolution);
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
  bool validate(Reply& reply) const
  {
    // Calculate reply crc
    auto current_crc = calculateReplyCrc(reply);
    decodeField(reply.footer.crc);
    if (current_crc != reply.footer.crc)
    {
      ROS_WARN_STREAM("Invalid CRC. Calculated CRC: " << current_crc << " expected is: " << reply.footer.crc);
      return false;
    }
    // Parse status
    decodeField(reply.status);
    return validateStatus(reply);
  }

  inline bool validateStatus(Reply& reply) const
  {
    // Check if status is ok
    if (reply.status != 0)
    {
      std::string error_msg;
      if (error_codes::YRStatusErrorCodeToString.count(reply.status))
      {
        error_msg = std::string("Received bad status: ") + error_codes::YRStatusErrorCodeToString.at(reply.status);
      }
      else
      {
        error_msg = std::string("Received bad status: unknown error " + reply.status);
      }
      ROS_ERROR_STREAM(error_msg);
      return false;
    }
    return true;
  }

  std::optional<Reply> process(const Reply& raw_reply) const
  {
    Reply reply = raw_reply;
    if (!validate(reply))
    {
      return std::nullopt;
    }
    decodeField(reply.cmd_size);
    decodeField(reply.area_type);
    decodeField(reply.area_number);
    decodeField(reply.start_step);
    decodeField(reply.end_step);
    decodeField(reply.resolution);
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

  /**
   * @brief Callback to execute when processing the packet
   */
  PacketEventCallback callback_ { nullptr };
};

}  // namespace uam

#endif  // INCLUDE_URG_NODE_UAM_SENSING_DATA_WORKERS_YR_WORKER_H_
