/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2022, Locus Robotics
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

#ifndef INCLUDE_URG_NODE_UAM_PROTOCOL_TYPES_H_
#define INCLUDE_URG_NODE_UAM_PROTOCOL_TYPES_H_

#include <cstddef>
#include <cstdint>

#include <urg_node/visitor.h>

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
 * @brief Helper function to deserialize/decode fields in the buffer
 */
class SensingDataReplyHelper
{
#define VISITOR_MEMBER_IMPL(field) VISITOR_MEMBER(SensingDataReply, field)
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

  VISITOR_MEMBER_IMPL(status);
  VISITOR_MEMBER_IMPL(operating_mode);
  VISITOR_MEMBER_IMPL(area_number);
  VISITOR_MEMBER_IMPL(error_state);
  VISITOR_MEMBER_IMPL(error_code);
  VISITOR_MEMBER_IMPL(lockout_state);
  VISITOR_MEMBER_IMPL(ossd1_state);
  VISITOR_MEMBER_IMPL(ossd2_state);
  VISITOR_MEMBER_IMPL(warning1_state);
  VISITOR_MEMBER_IMPL(warning2_state);
  VISITOR_MEMBER_IMPL(ossd3_state);
  VISITOR_MEMBER_IMPL(ossd4_state);
  VISITOR_MEMBER_IMPL(muting_state1);
  VISITOR_MEMBER_IMPL(muting_state2);
  VISITOR_MEMBER_IMPL(reset_request1);
  VISITOR_MEMBER_IMPL(reset_request2);
  VISITOR_MEMBER_IMPL(encoder_speed);
  VISITOR_MEMBER_IMPL(timestamp);
  VISITOR_MEMBER_IMPL(laser_state_off);
  VISITOR_MEMBER_IMPL(optical_window_contaminated);
};

// For now, keep the same decoding procedure.
// The following asserts are just to verify that the expected indexes remain correct
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
