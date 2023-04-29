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

#include <array>
#include <urg_node/visitor.h>

namespace uam
{
namespace protocol
{
#pragma pack(1)
struct CommandRequestHeader
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
  char header[2];

  /**
   * @brief It is a unique code to differentiate the type of command.
   */
  char sub_header[2];
};
#pragma pack()

#pragma pack(1)
struct CommandReplyHeader
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
  char header[2];

  /**
   * @brief It is a unique code to differentiate the type of command.
   */
  char sub_header[2];

  /**
   * @brief code to inform the success or failure of the command execution.
   * Status other than “00” is error code.
   */
  uint16_t status;
};
#pragma pack()

#pragma pack(1)
struct CommandFooter
{
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
#pragma pack(1)
struct VersionDetails
{
  /**
   * @brief Sensor model
   */
  std::array<char, 29> sensor_model;
  /**
   * @brief Reserved byte
   */
  char comma0;
  /**
   * @brief Sensor model
   */
  std::array<char, 29> firmware_version;
  /**
   * @brief Reserved byte
   */
  char comma1;
  /**
   * @brief reserved
   */
  char reserved0[37];
  /**
   * @brief Reserved byte
   */
  char comma2;
  /**
   * @brief Reserved byte
   */
  std::array<char, 8> serial_number;
  /**
   * @brief Reserved byte
   */
  char comma3;
};
#pragma pack()

}  // namespace version_details

/**
 * @brief Sensing Data commands (AR Commands)
 */
namespace sensing_data
{
/**
 * @brief Distance measurement
 * Note:
 * 1. Values more than 40000 are error code (0xFFFF).
 * 2. If object is not detected value will be 65534 (0xFFFE)
 * 3. If object is at a very close range the value will be 65533 (0xFFFD).
 * 4. When the device is in laser off state the value will be 65532 (0xFFFC)
 */
using DistanceData = uint32_t;
template <size_t TSize>
using DistanceDataArray = std::array<DistanceData, TSize>;
/**
 * @brief
 * Note:
 * 1. If object is not detected the value will be 0.
 * 2. Do not use the value if distance of the corresponding step has error.
 * 3. When the device is in laser off state the value will be 65532 (0xFFFC)
 */
using IntensityData = uint32_t;
template <size_t TSize>
using IntensityDataArray = std::array<IntensityData, TSize>;

/**
 * @brief Sensing data reply without distance and intensity
 */
#pragma pack(1)
struct SensingDataHeader
{
  /**
   * @brief Operating mode can be:
   * 0 - normal
   * 1 - setting
   */
  bool operating_mode;
  /**
   * @brief Current Safety Area - This is the value that appears in the laser display
   */
  uint16_t area_number;
  /**
   * @brief Inform error status
   * 0 - no error
   * 1 - error is detected
   */
  bool error_state;
  /**
   * @brief Error code, used together with error status to show the error number.
   * Check datasheet to see error codes
   */
  uint16_t error_code;
  /**
   * @brief Use this information with Error Code to show the error status.
   * Also check the Error State.
   */
  bool lockout_state;
  /**
   * @brief
   * 0 - no detection
   * 1 - detection
   */
  bool ossd1_state;
  /**
   * @brief
   * 0 - no detection
   * 1 - detection
   */
  bool ossd2_state;
  /**
   * @brief
   * 0 - no detection
   * 1 - detection
   */
  bool warning1_state;
  /**
   * @brief
   * 0 - no detection
   * 1 - detection
   */
  bool warning2_state;
  /**
   * @brief
   * 0 - no detection
   * 1 - detection
   */
  bool ossd3_state;
  /**
   * @brief
   * 0 - no detection
   * 1 - detection
   */
  bool ossd4_state;
  /**
   * @brief reserved
   */
  char reserved0[2];
  /**
   * @brief
   * 0 - not active
   * 1 - active
   */
  bool muting_state1;
  /**
   * @brief
   * 0 - not active
   * 1 - active
   */
  bool muting_state2;
  /**
   * @brief
   * 0 - Off
   * 1 - On
   */
  bool reset_request1;
  /**
   * @brief
   * 0 - Off
   * 1 - On
   */
  bool reset_request2;
  /**
   * @brief Encoder Speed
   */
  uint32_t encoder_speed;
  /**
   * @brief Unit is millisecond
   */
  uint64_t timestamp;
  /**
   * @brief
   * 0 - Laser is emitting
   * 1 - Laser is stopped
   */
  bool laser_state_off;
  /**
   * @brief Data will be 1 when optical window starts to accumulate dust.
   * Use this information to clean the optical window before it
   * becomes severe leading the device to error state (error 85, b1 ~ c1).
   */
  bool optical_window_contaminated;
  /**
   * @brief reserved
   */
  char reserved1[6];
};
#pragma pack()

/**
 * @brief Status data reply (XR Command), similar to sensing Data but without
 * distance
 */
#pragma pack(1)
struct StatusData
{
  /**
   * @brief Operating mode can be:
   * 0 - normal
   * 1 - setting
   */
  bool operating_mode;
  /**
   * @brief Current Safety Area - This is the value that appears in the laser display
   */
  uint16_t area_number;
  /**
   * @brief Inform error status
   * 0 - no error
   * 1 - error is detected
   */
  bool error_state;
  /**
   * @brief Error code, used together with error status to show the error number.
   * Check datasheet to see error codes
   */
  uint16_t error_code;
  /**
   * @brief Use this information with Error Code to show the error status.
   * Also check the Error State.
   */
  bool lockout_state;
  /**
   * @brief
   * 0 - no detection
   * 1 - detection
   */
  bool ossd1_state;
  /**
   * @brief
   * 0 - no detection
   * 1 - detection
   */
  bool ossd2_state;
  /**
   * @brief
   * 0 - no detection
   * 1 - detection
   */
  bool warning1_state;
  /**
   * @brief
   * 0 - no detection
   * 1 - detection
   */
  bool warning2_state;
  /**
   * @brief
   * 0 - no detection
   * 1 - detection
   */
  bool ossd3_state;
  /**
   * @brief
   * 0 - no detection
   * 1 - detection
   */
  bool ossd4_state;
  /**
   * @brief reserved
   */
  char reserved0[2];
  /**
   * @brief
   * 0 - not active
   * 1 - active
   */
  bool muting_state1;
  /**
   * @brief
   * 0 - not active
   * 1 - active
   */
  bool muting_state2;
  /**
   * @brief
   * 0 - Off
   * 1 - On
   */
  bool reset_request1;
  /**
   * @brief
   * 0 - Off
   * 1 - On
   */
  bool reset_request2;
  /**
   * @brief Encoder Speed
   */
  uint32_t encoder_speed;
  /**
   * @brief
   * 0 - Laser is emitting
   * 1 - Laser is stopped
   */
  bool laser_state_off;
  /**
   * @brief
   */
  bool slave1_ossd1_2_state;
  /**
   * @brief
   */
  bool slave2_ossd1_2_state;
  /**
   * @brief
   */
  bool slave3_ossd1_2_state;
  /**
   * @brief
   */
  bool slave1_ossd3_4_state;
  /**
   * @brief
   */
  bool slave2_ossd3_4_state;
  /**
   * @brief
   */
  bool slave3_ossd3_4_state;
  /**
   * @brief
   */
  bool slave1_warning_1_state;
  /**
   * @brief
   */
  bool slave2_warning_1_state;
  /**
   * @brief
   */
  bool slave3_warning_1_state;
  /**
   * @brief
   */
  bool slave1_warning_2_state;
  /**
   * @brief
   */
  bool slave2_warning_2_state;
  /**
   * @brief
   */
  bool slave3_warning_2_state;
  /**
   * @brief
   */
  bool slave1_error_state;
  /**
   * @brief
   */
  bool slave2_error_state;
  /**
   * @brief
   */
  bool slave3_error_state;

  /**
   * @brief
   */
  bool slave1_laser_off_state;
  /**
   * @brief
   */
  bool slave2_laser_off_state;
  /**
   * @brief
   */
  bool slave3_laser_off_state;

  /**
   * @brief Unit is millisecond
   */
  uint64_t timestamp;
  /**
   * @brief Data will be 1 when optical window starts to accumulate dust.
   * Use this information to clean the optical window before it
   * becomes severe leading the device to error state (error 85, b1 ~ c1).
   */
  bool optical_window_contaminated;
  /**
   * @brief Reserved bytes
   */
  char reserved1[39];
};
#pragma pack()

}  // namespace sensing_data

/**
 * @brief Host to UAM command struct
 */
#pragma pack(1)
struct CommandRequest
{
  CommandRequestHeader header;
  CommandFooter footer;
};
#pragma pack()
/**
 * @brief Host to UAM command struct
 */
#pragma pack(1)
struct EmptyCommandReply
{
  CommandReplyHeader header;
  CommandFooter footer;
};
#pragma pack()

#pragma pack(1)
struct VR00CommandReply
{
  CommandReplyHeader header;
  version_details::VersionDetails version_details;
  CommandFooter footer;
};
#pragma pack()

#pragma pack(1)
struct AR00CommandReply
{
  CommandReplyHeader header;
  sensing_data::SensingDataHeader sensing_data;
  sensing_data::DistanceDataArray<1081> ranges;
  CommandFooter footer;
};
#pragma pack()

#pragma pack(1)
struct AR01CommandReply
{
  CommandReplyHeader header;
  sensing_data::SensingDataHeader sensing_data;
  sensing_data::DistanceDataArray<1081> ranges;
  sensing_data::IntensityDataArray<1081> intensities;
  CommandFooter footer;
};
#pragma pack()
/**
 * @brief AR02 first reply, later sends only AR00CommandReply replies types
 */
using AR02CommandReply = EmptyCommandReply;
/**
 * @brief AR03 reply to the the stop continuous data initiated by AR02 request
 */
using AR03CommandReply = EmptyCommandReply;
/**
 * @brief AR04 first reply, later sends only AR01CommandReply replies types
 */
using AR04CommandReply = EmptyCommandReply;
/**
 * @brief AR05 reply to the the stop continuous data initiated by AR04 request
 */
using AR05CommandReply = EmptyCommandReply;

/**
 * @brief Command reply
 */
#pragma pack(1)
struct AR06CommandReply
{
  CommandReplyHeader header;
  sensing_data::SensingDataHeader sensing_data;
  sensing_data::DistanceDataArray<2161> ranges;
  CommandFooter footer;
};
#pragma pack()
/**
 * @brief AR07 first reply, later sends only AR06CommandReply replies types
 */
using AR07CommandReply = EmptyCommandReply;
/**
 * @brief AR05 reply to the the stop continuous data initiated by AR07 request
 */
using AR08CommandReply = EmptyCommandReply;

#pragma pack(1)
struct XR00CommandReply
{
  CommandReplyHeader header;
  sensing_data::StatusData data;
  CommandFooter footer;
};
#pragma pack()

/**
 * @brief Host to UAM command struct
 */
#pragma pack(1)
struct YRCommandHeader
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
  char header[2];
  /**
   * @brief
   * 00: Protection Zone 1
   * 01: Protection Zone 2
   * 02: Warning Zone 1
   * 03: Warning Zone 2
   * 04: Muting Area 1
   * 05: Muting Area 2
   * 06: Reference Area (Centre)
   * 07: Reference Area (Max value)
   * 08: Reference Area (Min Value)
   */
  uint16_t area_type;

  /**
   * @brief Provide the area numbers in hexadecimal equivalent
   * characters ( 0 to 1F).
   * Area number should not exceed the configured active area count
   *
   * 00: Area 1
   * 01: Area 2
   * …
   * …
   * 1F: Area 32
   */
  uint16_t area_number;

  /**
   * @brief
   * Provide the step values in hexadecimal equivalent
   * characters
   * - Step values should not exceed the maximum range 0438 (1081 in decimal).
   * - Start step should not be greater than the end step.
   */
  uint32_t start_step;
  uint32_t end_step;
  /**
   * @brief
   *
   * this might to what the documentation refers as grouping?
   * 00/01: No grouping
   * 02: Grouping two data
   * 03: Group three data
   * …
   * …
   * 09: Group nine data
   */
  uint16_t resolution;
};
#pragma pack()

/**
 * @brief Host to UAM command struct
 */
#pragma pack(1)
struct YRCommandReplyHeader
{
  YRCommandHeader header;
  uint16_t status;
};
#pragma pack()

#pragma pack(1)
struct YRCommandRequest
{
  YRCommandHeader header;
  CommandFooter footer;
};
#pragma pack()

#pragma pack(1)
struct YRCommandReply
{
  YRCommandReplyHeader header;
  std::array<uint32_t,1080> area_data;
  CommandFooter footer;
};
#pragma pack()


/**
 * @brief Area Types
 */
enum EYRAreaType : uint16_t
{
  protection_1 = 0,/**< protection_1 */
  protection_2,    /**< protection_2 */
  warning_1,       /**< warning_1 */
  warning_2,       /**< warning_2 */
  muting_1,        /**< muting_1 */
  muting_2,        /**< muting_2 */
  reference_center,/**< reference_center */
  reference_max,   /**< reference_max */
  reference_min,   /**< reference_min */
  MAX              /**< MAX */
};

/**
 * @brief Max safety area index for this the supported fw version
 */
constexpr uint16_t c_max_safety_area_index = 32;

constexpr size_t c_sensing_data_start_idx { sizeof(protocol::CommandReplyHeader) };
constexpr size_t c_distance_start_idx { sizeof(protocol::CommandReplyHeader) +
                                        sizeof(protocol::sensing_data::SensingDataHeader) };
constexpr size_t c_intensity_start_idx { c_distance_start_idx + sizeof(protocol::AR01CommandReply::ranges) };

}  // namespace protocol
}  // namespace uam

#endif /* INCLUDE_URG_NODE_UAM_PROTOCOL_TYPES_H_ */
