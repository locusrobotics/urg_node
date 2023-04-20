/**
Software License Agreement (proprietary)
\file      uam_error_codes.h
\authors   Carlos Mendes <cribeiromendes@locusrobotics.com>
\copyright Copyright (c) (2023,), Locus Robotics Corp., All rights reserved.
Unauthorized copying of this file, via any medium, is strictly prohibited.
Proprietary and confidential.
**/

#ifndef INCLUDE_URG_NODE_UAM_ERROR_CODES_H_
#define INCLUDE_URG_NODE_UAM_ERROR_CODES_H_

#include <array>
#include <boost/assign/list_of.hpp>
#include <map>

#include <type_traits>

namespace uam
{
template <typename T>
constexpr auto getUnderlyingType(T f_enum) -> typename std::underlying_type<T>::type
{
  return static_cast<typename std::underlying_type<T>::type>(f_enum);
}

namespace error_codes
{
/**
 * @brief
 */
enum class EStatusErrorCodes : uint16_t
{
  NO_ERROR = 0x00, /**< NO_ERROR */
  COMMAND_INVALID_FIELDS = 0x12, /**< Received command does not contain
                                    the minimum required fields or received data
                                    size exceeds the maximum size of internal buffer.*/
  COMMAND_WITHOUT_STX = 0x31, /**< Command is received without STX */
  COMMAND_HEADER_WITH_UNSPECIFIED_CHARS = 0x34, /**< Command header contains unspecified characters */
  DATA_WITH_UNSPECIFIED_CHARS = 0x35, /**< Data contains unspecified characters */
  DATA_SIZE_MISMATCH = 0x36, /**< Data size is not equal to the size mentioned in the command */
  CRC_MISMATCH = 0x37, /**< CRC of received data is not equal to CRC in the command */
  UNSPECIFIED_COMMAND_0 = 0x41, /**< Unspecified command is received_0 */
  UNSPECIFIED_COMMAND_1 = 0x42, /**< Unspecified command is received_1 */
  SUB_HEADER_OUT_OF_RANGE = 0x44, /**< Sub header is out-of-range */
  SUB_HEADER_NAN = 0x45, /**< Sub header is not a number */
  INCOMPLETE_CONFIGURATION = 0x66, /**<  Configuration of UAM is incomplete */
  CANNOT_PROCESS_COMMANDS = 0x73 /**< Unable to process commands (AR02 and AR04) as the device is in setting
                                      mode (Continuous data output mode can not be started when the device is
                                      in setting mode).*/
};

const auto StatusErrorCodeToString = std::map<uint16_t, std::string>(
  { { getUnderlyingType(EStatusErrorCodes::NO_ERROR), "No error" },
    { getUnderlyingType(EStatusErrorCodes::COMMAND_INVALID_FIELDS),
      "Received command does not contain the minimum "
      "required fields or received data size exceeds the maximum "
      "size of internal buffer" },
    { getUnderlyingType(EStatusErrorCodes::COMMAND_WITHOUT_STX), "Command is received without STX " },
    { getUnderlyingType(EStatusErrorCodes::COMMAND_HEADER_WITH_UNSPECIFIED_CHARS), "Command header contains unspecified characters" },
    { getUnderlyingType(EStatusErrorCodes::DATA_WITH_UNSPECIFIED_CHARS), "Data contains unspecified characters" },
    { getUnderlyingType(EStatusErrorCodes::DATA_SIZE_MISMATCH), "Data size is not equal to the size mentioned in the command" },
    { getUnderlyingType(EStatusErrorCodes::CRC_MISMATCH), "CRC of received data is not equal to CRC in the command" },
    { getUnderlyingType(EStatusErrorCodes::UNSPECIFIED_COMMAND_0), "Unspecified command is received" },
    { getUnderlyingType(EStatusErrorCodes::UNSPECIFIED_COMMAND_1), "Unspecified command is received" },
    { getUnderlyingType(EStatusErrorCodes::SUB_HEADER_OUT_OF_RANGE), "Sub header is out-of-range" },
    { getUnderlyingType(EStatusErrorCodes::SUB_HEADER_NAN), "Sub header is not a number " },
    { getUnderlyingType(EStatusErrorCodes::INCOMPLETE_CONFIGURATION), "Configuration of UAM is incomplete" },
    { getUnderlyingType(EStatusErrorCodes::CANNOT_PROCESS_COMMANDS),
      "Unable to process commands (AR02 and AR04) as the device is in setting"
      "mode (Continuous data output mode can not be started when the device is in setting mode)" } });

enum class EYRStatusErrorCodes : uint16_t
{
  NO_ERROR = 0x00, /**< NO_ERROR */
  AREA_TYPE_GROUP_COUNT_EXCEEDED =
    0x44, /**<- Grouping count exceed the maximum value or Area type exceeds the maximum value>**/
  START_END_STEP_MISCONFIG = 0x52, /**<Start and/or end step exceeds the maximum value or Start step is greater than end
                                      step 0x54 Area number exceeds the maximum val>**/
  AREA_NR_EXCEEDED_MAX = 0x54,
  AREA_NR_EXCEEDS_ACTIVE_AREA_COUND = 0x55,
  PROTECTION_AREA2_NOT_ACTIVE = 0x81,
  WARNING1_NOT_ACTIVE = 0x82,
  WARNING2_NOT_ACTIVE = 0x83,
  MUTING1_NOT_ACTIVE = 0x84,
  MUTING2_NOT_ACTIVE = 0x85,
  REFERENCE_DATA_NOT_ACTIVE = 0x86,
  REFERENCE_MAX_DATA_NOT_ACTIVE = 0x87,
  REFERENCE_MIN_DATA_NOT_ACTIVE = 0x88
};

const auto YRStatusErrorCodeToString = std::map<uint16_t, std::string>(
  { { getUnderlyingType(EYRStatusErrorCodes::NO_ERROR), "No error" },
    { getUnderlyingType(EYRStatusErrorCodes::AREA_TYPE_GROUP_COUNT_EXCEEDED),
      "Grouping count exceed the maximum value or area type exceeds the maximum value" },
    { getUnderlyingType(EYRStatusErrorCodes::START_END_STEP_MISCONFIG),
      "Start and/or end step exceeds the maximum value or Start step is greater than end step 0x54 Area number exceeds "
      "the maximum val" },
    { getUnderlyingType(EYRStatusErrorCodes::AREA_NR_EXCEEDED_MAX), "Area number exceeds the maximum value" },
    { getUnderlyingType(EYRStatusErrorCodes::AREA_NR_EXCEEDS_ACTIVE_AREA_COUND),
      "Area number exceeds the active area count in the sensor" },
    { getUnderlyingType(EYRStatusErrorCodes::PROTECTION_AREA2_NOT_ACTIVE),
      "Protection2 data is requested (YR01) without activating the Protection2 area" },
    { getUnderlyingType(EYRStatusErrorCodes::WARNING1_NOT_ACTIVE),
      "Warning1 data is requested (YR02) without activating the Warning1 area" },
    { getUnderlyingType(EYRStatusErrorCodes::WARNING2_NOT_ACTIVE),
      "Warning2 data is requested (YR03) without activating the Warning1 area" },
    { getUnderlyingType(EYRStatusErrorCodes::MUTING1_NOT_ACTIVE),
      "Muting1 data is requested (YR04) without activating the muting1 area" },
    { getUnderlyingType(EYRStatusErrorCodes::MUTING2_NOT_ACTIVE),
      "Muting2 data is requested (YR05) without activating the muting1 area" },
    { getUnderlyingType(EYRStatusErrorCodes::REFERENCE_DATA_NOT_ACTIVE),
      "Reference data is requested (YR06) without activating the reference area" },
    { getUnderlyingType(EYRStatusErrorCodes::REFERENCE_MAX_DATA_NOT_ACTIVE),
      "Reference max data is requested (YR07) without activating the reference area" },
    { getUnderlyingType(EYRStatusErrorCodes::REFERENCE_MIN_DATA_NOT_ACTIVE),
      "Reference min data is requested (YR08) without activating the reference data" } });
}  // namespace error_codes

}  // namespace uam

#endif  // INCLUDE_URG_NODE_UAM_ERROR_CODES_H_
