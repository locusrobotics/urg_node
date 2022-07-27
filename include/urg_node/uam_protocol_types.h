/*
 * uam_protocol_types.h
 *
 *  Created on: 25/07/2022
 *      Author: cribeiromendes
 */

#ifndef INCLUDE_URG_NODE_UAM_PROTOCOL_TYPES_H_
#define INCLUDE_URG_NODE_UAM_PROTOCOL_TYPES_H_

#include <cstddef>
#include <cstdint>

#include <urg_node/accessor.h>

namespace protocol
{
/**
 * @brief Host to UAM command struct
 */
#pragma pack(1)
struct CommandRequest
{
  /**
   * @brief start of frame
   */
  uint8_t stx;
  /**
   * @brief It is the total length of ASCII characters in a command.
   * Command size is encoded to hexadecimal strings.
   */
  uint32_t cmd_size;

  /**
   * @brief It is a unique code to differentiate the type of command.
   */
  uint16_t header;

  /**
   * @brief It is a unique code to differentiate the type of command.
   */
  uint16_t sub_header;
  /**
   * @brief It is a 16-Bit code for checking the data integrity.
   * Command size, header, sub-header and data are included in CRC calculation.
   */
  uint32_t crc;
  /**
   * @brief end of frame
   */
  uint8_t etx;
};
#pragma pack()

namespace version_details
{
/**
 * @brief
 */
struct VersionDetailsReply
{
  /**
   * @brief start of frame
   */
  uint8_t stx;
  /**
   * @brief It is the total length of ASCII characters in a command.
   * Command size is encoded to hexadecimal strings.
   */
  uint32_t cmd_size;
  /**
   * @brief It is a unique code to differentiate the type of command.
   */
  uint16_t header;
  /**
   * @brief It is a unique code to differentiate the type of command.
   */
  uint16_t sub_header;
  /**
   * @brief Sensor model
   */
  char sensor_model[29];
  /**
   * @brief Reserved byte
   */
  char reserved0;
  /**
   * @brief Sensor model
   */
  char firmware_version[29];
  /**
   * @brief Reserved byte
   */
  char reserved1;
  /**
   * @brief reserved
   */
  char reserved[37];
  /**
   * @brief Reserved byte
   */
  char reserved2;
  /**
   * @brief Reserved byte
   */
  char serial_number[8];
  /**
   * @brief Reserved byte
   */
  char reserved3;
  /**
   * @brief
   */
  uint32_t crc;
};

}  // namespace version_details

/**
 * @brief Sensing Data commands (AR Commands)
 */
namespace sensing_data
{
/**
 * @brief Sensing data reply without distance and intensity
 */
#pragma pack(1)
struct SensingDataReply
{
  /**
   * @brief It is a code to inform the success or failure of the command execution.
   * Status other than “00” is error code
   */
  uint16_t status { 0 };
  /**
   * @brief Operating mode can be:
   * 0 - normal
   * 1 - setting
   */
  bool operating_mode { 0 };
  /**
   * @brief Current Safety Area - This is the value that appears in the laser display
   */
  uint16_t area_number { 0 };
  /**
   * @brief Inform error status
   * 0 - no error
   * 1 - error is detected
   */
  bool error_state { false };
  /**
   * @brief Error code, used together with error status to show the error number.
   * Check datasheet to see error codes
   */
  uint16_t error_code { 0 };
  /**
   * @brief Use this information with Error Code to show the error status.
   * Also check the Error State.
   */
  bool lockout_state { false };
  /**
   * @brief
   * 0 - no detection
   * 1 - detection
   */
  bool ossd1_state { false };
  /**
   * @brief
   * 0 - no detection
   * 1 - detection
   */
  bool ossd2_state { false };
  /**
   * @brief
   * 0 - no detection
   * 1 - detection
   */
  bool warning1_state { false };
  /**
   * @brief
   * 0 - no detection
   * 1 - detection
   */
  bool warning2_state { false };
  /**
   * @brief
   * 0 - no detection
   * 1 - detection
   */
  bool ossd3_state { false };
  /**
   * @brief
   * 0 - no detection
   * 1 - detection
   */
  bool ossd4_state { false };
  /**
   * @brief reserved
   */
  char reserved[2];
  /**
   * @brief
   * 0 - not active
   * 1 - active
   */
  bool muting_state1 { false };
  /**
   * @brief
   * 0 - not active
   * 1 - active
   */
  bool muting_state2 { false };
  /**
   * @brief
   * 0 - Off
   * 1 - On
   */
  bool reset_request1 { false };
  /**
   * @brief
   * 0 - Off
   * 1 - On
   */
  bool reset_request2 { false };
  /**
   * @brief Encoder Speed
   */
  uint32_t encoder_speed { 0 };
  /**
   * @brief Unit is millisecond
   */
  uint64_t timestamp { 0 };
  /**
   * @brief
   * 0 - Laser is emitting
   * 1 - Laser is stopped
   */
  bool laser_state_off { false };
  /**
   * @brief Data will be 1 when optical window starts to accumulate dust.
   * Use this information to clean the optical window before it
   * becomes severe leading the device to error state (error 85, b1 ~ c1).
   */
  bool optical_window_contaminated { false };
};
#pragma pack()
/**
 * @brief Command type
 */
enum SubHeader : uint8_t
{
  E_00 = 0,
  E_01,
  E_02,
  E_03,
  E_04,
  E_05,
  E_06,
  E_07,
  E_08,
  E_MAX
};

/**
 * @brief Helper function to deserialize/decode fields in the buffer
 */
class SensingDataReplyHelper
{
#define ACCESSOR_MEMBER_IMPL(field) ACCESSOR_MEMBER(SensingDataReply,field)
public:
  SensingDataReplyHelper(const std::string* buffer, const size_t initial_offset) :
    status(buffer, initial_offset),
    operating_mode(buffer, initial_offset),
    area_number(buffer, initial_offset, 1),
    error_state(buffer, initial_offset),
    error_code(buffer, initial_offset),
    lockout_state(buffer, initial_offset),
    ossd1_state(buffer, initial_offset),
    ossd2_state(buffer, initial_offset),
    warning1_state(buffer, initial_offset),
    warning2_state(buffer, initial_offset),
    ossd3_state(buffer, initial_offset),
    ossd4_state(buffer, initial_offset),
    muting_state1(buffer, initial_offset),
    muting_state2(buffer, initial_offset),
    reset_request1(buffer, initial_offset),
    reset_request2(buffer, initial_offset),
    encoder_speed(buffer, initial_offset),
    timestamp(buffer, initial_offset),
    laser_state_off(buffer, initial_offset),
    optical_window_contaminated(buffer, initial_offset)
  {
  }

  ACCESSOR_MEMBER_IMPL(status);
  ACCESSOR_MEMBER_IMPL(operating_mode);
  ACCESSOR_MEMBER_IMPL(area_number);
  ACCESSOR_MEMBER_IMPL(error_state);
  ACCESSOR_MEMBER_IMPL(error_code);
  ACCESSOR_MEMBER_IMPL(lockout_state);
  ACCESSOR_MEMBER_IMPL(ossd1_state);
  ACCESSOR_MEMBER_IMPL(ossd2_state);
  ACCESSOR_MEMBER_IMPL(warning1_state);
  ACCESSOR_MEMBER_IMPL(warning2_state);
  ACCESSOR_MEMBER_IMPL(ossd3_state);
  ACCESSOR_MEMBER_IMPL(ossd4_state);
  ACCESSOR_MEMBER_IMPL(muting_state1);
  ACCESSOR_MEMBER_IMPL(muting_state2);
  ACCESSOR_MEMBER_IMPL(reset_request1);
  ACCESSOR_MEMBER_IMPL(reset_request2);
  ACCESSOR_MEMBER_IMPL(encoder_speed);
  ACCESSOR_MEMBER_IMPL(timestamp);
  ACCESSOR_MEMBER_IMPL(laser_state_off);
  ACCESSOR_MEMBER_IMPL(optical_window_contaminated);
};

// For now, keep the same decoding mechanism. Therefore, verify against old index
// are correct
// stx and etx are not taken into account as they are removed from the buffer
constexpr size_t c_sensing_data_start_idx { offsetof(CommandRequest, sub_header) - sizeof(CommandRequest::stx) +
                                            sizeof(CommandRequest::sub_header) };
constexpr size_t c_sensing_data_packet_size { sizeof(SensingDataReply) };
// The following asserts are just here to confirm that nothing was broken from this update
static_assert(c_sensing_data_start_idx == 8);
static_assert(c_sensing_data_packet_size == 35);
static_assert(c_sensing_data_start_idx + offsetof(SensingDataReply, operating_mode) == 10);
static_assert(c_sensing_data_start_idx + offsetof(SensingDataReply, area_number) == 11);
static_assert(c_sensing_data_start_idx + offsetof(SensingDataReply, error_state) == 13);
static_assert(c_sensing_data_start_idx + offsetof(SensingDataReply, error_code) == 14);
static_assert(c_sensing_data_start_idx + offsetof(SensingDataReply, lockout_state) == 16);
static_assert(c_sensing_data_start_idx + offsetof(SensingDataReply, optical_window_contaminated) == 42);
}  // namespace sensing_data
}  // namespace protocol

#endif /* INCLUDE_URG_NODE_UAM_PROTOCOL_TYPES_H_ */
