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

namespace uam
{
namespace error_codes
{
/**
 * @brief
 */
enum class EStatusErrorCodes : uint8_t
{
  // No Error
  NO_ERROR = 0x00, /**< NO_ERROR */
  //
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

const auto StatusErrorCodeToString = std::map<EStatusErrorCodes, std::string>(
  { { EStatusErrorCodes::NO_ERROR, "No error" },
    { EStatusErrorCodes::COMMAND_INVALID_FIELDS,
      "Received command does not contain the minimum "
      "required fields or received data size exceeds the maximum "
      "size of internal buffer" },
    { EStatusErrorCodes::COMMAND_WITHOUT_STX, "Command is received without STX " },
    { EStatusErrorCodes::COMMAND_HEADER_WITH_UNSPECIFIED_CHARS, "Command header contains unspecified characters" },
    { EStatusErrorCodes::DATA_WITH_UNSPECIFIED_CHARS, "Data contains unspecified characters" },
    { EStatusErrorCodes::DATA_SIZE_MISMATCH, "Data size is not equal to the size mentioned in the command" },
    { EStatusErrorCodes::CRC_MISMATCH, "CRC of received data is not equal to CRC in the command" },
    { EStatusErrorCodes::UNSPECIFIED_COMMAND_0, "Unspecified command is received" },
    { EStatusErrorCodes::UNSPECIFIED_COMMAND_1, "Unspecified command is received" },
    { EStatusErrorCodes::SUB_HEADER_OUT_OF_RANGE, "Sub header is out-of-range" },
    { EStatusErrorCodes::SUB_HEADER_NAN, "Sub header is not a number " },
    { EStatusErrorCodes::INCOMPLETE_CONFIGURATION, "Configuration of UAM is incomplete" },
    { EStatusErrorCodes::CANNOT_PROCESS_COMMANDS,
      "Unable to process commands (AR02 and AR04) as the device is in setting"
      "mode (Continuous data output mode can not be started when the device is in setting mode)" } });
}  // namespace error_codes

}  // namespace uam

#endif  // INCLUDE_URG_NODE_UAM_ERROR_CODES_H_
