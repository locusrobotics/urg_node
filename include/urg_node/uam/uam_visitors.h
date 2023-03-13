/**
Software License Agreement (proprietary)
\file      fileds.h
\authors   Carlos Mendes <cribeiromendes@locusrobotics.com>
\copyright Copyright (c) (2023,), Locus Robotics Corp., All rights reserved.
Unauthorized copying of this file, via any medium, is strictly prohibited.
Proprietary and confidential.
**/

#ifndef INCLUDE_URG_NODE_UAM_UAM_VISITORS_H_
#define INCLUDE_URG_NODE_UAM_UAM_VISITORS_H_

#include <urg_node/uam/uam_protocol_types.h>
#include <urg_node/visitor.h>

namespace uam
{
/**
 * @brief
 */
class CommandHeaderVisitor
{
public:
  CommandHeaderVisitor(const size_t initial_offset) :
    stx(initial_offset),
    cmd_size(initial_offset),
    header(initial_offset),
    sub_header(initial_offset),
    status(initial_offset)
  {
  }

#define VISITOR_MEMBER_IMPL_CMD_HEADER(field) VISITOR_MEMBER(protocol::CommandReplyHeader, field)
  VISITOR_MEMBER_IMPL_CMD_HEADER(stx);
  VISITOR_MEMBER_IMPL_CMD_HEADER(cmd_size);
  VISITOR_MEMBER_IMPL_CMD_HEADER(header);
  VISITOR_MEMBER_IMPL_CMD_HEADER(sub_header);
  VISITOR_MEMBER_IMPL_CMD_HEADER(status);
};

class CommandFooterVisitor
{
public:
  CommandFooterVisitor(const size_t initial_offset) : crc(initial_offset), etx(initial_offset) {}

#define VISITOR_MEMBER_IMPL_CMD_FOOTER(field) VISITOR_MEMBER(protocol::CommandFooter, field)
  VISITOR_MEMBER_IMPL_CMD_FOOTER(crc);
  VISITOR_MEMBER_IMPL_CMD_FOOTER(etx);
};

/**
 * @brief Helper function to deserialize/decode fields in the buffer
 */
class SensingDataVisitor
{
public:
  SensingDataVisitor(const size_t initial_offset) :
    operating_mode(initial_offset),
    area_number(initial_offset, 1),
    error_state(initial_offset),
    error_code(initial_offset),
    lockout_state(initial_offset),
    ossd1_state(initial_offset),
    ossd2_state(initial_offset),
    warning1_state(initial_offset),
    warning2_state(initial_offset),
    ossd3_state(initial_offset),
    ossd4_state(initial_offset),
    muting_state1(initial_offset),
    muting_state2(initial_offset),
    reset_request1(initial_offset),
    reset_request2(initial_offset),
    encoder_speed(initial_offset),
    timestamp(initial_offset),
    laser_state_off(initial_offset),
    optical_window_contaminated(initial_offset)
  {
  }
#define VISITOR_MEMBER_IMPL_SENSE_DATA(field) VISITOR_MEMBER(protocol::sensing_data::SensingDataHeader, field)
  VISITOR_MEMBER_IMPL_SENSE_DATA(operating_mode);
  VISITOR_MEMBER_IMPL_SENSE_DATA(area_number);
  VISITOR_MEMBER_IMPL_SENSE_DATA(error_state);
  VISITOR_MEMBER_IMPL_SENSE_DATA(error_code);
  VISITOR_MEMBER_IMPL_SENSE_DATA(lockout_state);
  VISITOR_MEMBER_IMPL_SENSE_DATA(ossd1_state);
  VISITOR_MEMBER_IMPL_SENSE_DATA(ossd2_state);
  VISITOR_MEMBER_IMPL_SENSE_DATA(warning1_state);
  VISITOR_MEMBER_IMPL_SENSE_DATA(warning2_state);
  VISITOR_MEMBER_IMPL_SENSE_DATA(ossd3_state);
  VISITOR_MEMBER_IMPL_SENSE_DATA(ossd4_state);
  VISITOR_MEMBER_IMPL_SENSE_DATA(muting_state1);
  VISITOR_MEMBER_IMPL_SENSE_DATA(muting_state2);
  VISITOR_MEMBER_IMPL_SENSE_DATA(reset_request1);
  VISITOR_MEMBER_IMPL_SENSE_DATA(reset_request2);
  VISITOR_MEMBER_IMPL_SENSE_DATA(encoder_speed);
  VISITOR_MEMBER_IMPL_SENSE_DATA(timestamp);
  VISITOR_MEMBER_IMPL_SENSE_DATA(laser_state_off);
  VISITOR_MEMBER_IMPL_SENSE_DATA(optical_window_contaminated);
};

/**
 * @brief Helper function to deserialize/decode status in the buffer
 */
class StatusDataVisitor
{
public:
  StatusDataVisitor(const size_t initial_offset) :
    operating_mode(initial_offset),
    area_number(initial_offset),
    error_state(initial_offset),
    error_code(initial_offset),
    lockout_state(initial_offset),
    ossd1_state(initial_offset),
    ossd2_state(initial_offset),
    warning1_state(initial_offset),
    warning2_state(initial_offset),
    ossd3_state(initial_offset),
    ossd4_state(initial_offset),
    muting_state1(initial_offset),
    muting_state2(initial_offset),
    reset_request1(initial_offset),
    reset_request2(initial_offset),
    encoder_speed(initial_offset),
    laser_state_off(initial_offset),
    slave1_ossd1_2_state(initial_offset),
    slave2_ossd1_2_state(initial_offset),
    slave3_ossd1_2_state(initial_offset),
    slave1_ossd3_4_state(initial_offset),
    slave2_ossd3_4_state(initial_offset),
    slave3_ossd3_4_state(initial_offset),
    slave1_warning_1_state(initial_offset),
    slave2_warning_1_state(initial_offset),
    slave3_warning_1_state(initial_offset),
    slave1_warning_2_state(initial_offset),
    slave2_warning_2_state(initial_offset),
    slave3_warning_2_state(initial_offset),
    slave1_error_state(initial_offset),
    slave2_error_state(initial_offset),
    slave3_error_state(initial_offset),
    slave1_laser_off_state(initial_offset),
    slave2_laser_off_state(initial_offset),
    slave3_laser_off_state(initial_offset),
    timestamp(initial_offset),
    optical_window_contaminated(initial_offset)
  {
  }

#define VISITOR_MEMBER_IMPL_STATUS_DATA(field) VISITOR_MEMBER(protocol::sensing_data::StatusData, field)

  VISITOR_MEMBER_IMPL_STATUS_DATA(operating_mode);
  VISITOR_MEMBER_IMPL_STATUS_DATA(area_number);
  VISITOR_MEMBER_IMPL_STATUS_DATA(error_state);
  VISITOR_MEMBER_IMPL_STATUS_DATA(error_code);
  VISITOR_MEMBER_IMPL_STATUS_DATA(lockout_state);
  VISITOR_MEMBER_IMPL_STATUS_DATA(ossd1_state);
  VISITOR_MEMBER_IMPL_STATUS_DATA(ossd2_state);
  VISITOR_MEMBER_IMPL_STATUS_DATA(warning1_state);
  VISITOR_MEMBER_IMPL_STATUS_DATA(warning2_state);
  VISITOR_MEMBER_IMPL_STATUS_DATA(ossd3_state);
  VISITOR_MEMBER_IMPL_STATUS_DATA(ossd4_state);
  VISITOR_MEMBER_IMPL_STATUS_DATA(muting_state1);
  VISITOR_MEMBER_IMPL_STATUS_DATA(muting_state2);
  VISITOR_MEMBER_IMPL_STATUS_DATA(reset_request1);
  VISITOR_MEMBER_IMPL_STATUS_DATA(reset_request2);
  VISITOR_MEMBER_IMPL_STATUS_DATA(encoder_speed);
  VISITOR_MEMBER_IMPL_STATUS_DATA(laser_state_off);
  VISITOR_MEMBER_IMPL_STATUS_DATA(slave1_ossd1_2_state);
  VISITOR_MEMBER_IMPL_STATUS_DATA(slave2_ossd1_2_state);
  VISITOR_MEMBER_IMPL_STATUS_DATA(slave3_ossd1_2_state);
  VISITOR_MEMBER_IMPL_STATUS_DATA(slave1_ossd3_4_state);
  VISITOR_MEMBER_IMPL_STATUS_DATA(slave2_ossd3_4_state);
  VISITOR_MEMBER_IMPL_STATUS_DATA(slave3_ossd3_4_state);
  VISITOR_MEMBER_IMPL_STATUS_DATA(slave1_warning_1_state);
  VISITOR_MEMBER_IMPL_STATUS_DATA(slave2_warning_1_state);
  VISITOR_MEMBER_IMPL_STATUS_DATA(slave3_warning_1_state);
  VISITOR_MEMBER_IMPL_STATUS_DATA(slave1_warning_2_state);
  VISITOR_MEMBER_IMPL_STATUS_DATA(slave2_warning_2_state);
  VISITOR_MEMBER_IMPL_STATUS_DATA(slave3_warning_2_state);
  VISITOR_MEMBER_IMPL_STATUS_DATA(slave1_error_state);
  VISITOR_MEMBER_IMPL_STATUS_DATA(slave2_error_state);
  VISITOR_MEMBER_IMPL_STATUS_DATA(slave3_error_state);
  VISITOR_MEMBER_IMPL_STATUS_DATA(slave1_laser_off_state);
  VISITOR_MEMBER_IMPL_STATUS_DATA(slave2_laser_off_state);
  VISITOR_MEMBER_IMPL_STATUS_DATA(slave3_laser_off_state);
  VISITOR_MEMBER_IMPL_STATUS_DATA(timestamp);
  VISITOR_MEMBER_IMPL_STATUS_DATA(optical_window_contaminated);
};

/**
 * @brief Helper function to deserialize/decode status in the buffer
 */
template <size_t Steps>
class DistanceDataVisitor
{
public:
  DistanceDataVisitor(const size_t initial_offset) : distances(initial_offset) {}

  Visitor<protocol::sensing_data::DistanceDataArray<Steps>, 0> distances;
};
template <size_t Steps>
class IntensityDataArrayVisitor
{
public:
  IntensityDataArrayVisitor(const size_t initial_offset) : intensities(initial_offset) {}
  Visitor<protocol::sensing_data::IntensityDataArray<Steps>, 0> intensities;
};

class VersionDetailsVisitor
{
public:
  VersionDetailsVisitor(const size_t initial_offset) :
    sensor_model(initial_offset),
    firmware_version(initial_offset),
    serial_number(initial_offset)
  {
  }

#define VISITOR_MEMBER_IMPL_VERSION_DETAILS(field) VISITOR_MEMBER(protocol::version_details::VersionDetails, field)

  VISITOR_MEMBER_IMPL_VERSION_DETAILS(sensor_model);
  VISITOR_MEMBER_IMPL_VERSION_DETAILS(firmware_version);
  VISITOR_MEMBER_IMPL_VERSION_DETAILS(serial_number);
};

// The following asserts are just here to confirm that nothing was broken from this update
static_assert(protocol::c_sensing_data_start_idx == 11);
static_assert(sizeof(protocol::AR00CommandReply) == 4379);
static_assert(offsetof(protocol::AR00CommandReply, sensing_data) == 11);
static_assert(offsetof(protocol::AR00CommandReply, sensing_data) == protocol::c_sensing_data_start_idx);
static_assert(
  protocol::c_sensing_data_start_idx + offsetof(protocol::sensing_data::SensingDataHeader, area_number) == 12);
static_assert(
  protocol::c_sensing_data_start_idx + offsetof(protocol::sensing_data::SensingDataHeader, error_state) == 14);
static_assert(
  protocol::c_sensing_data_start_idx + offsetof(protocol::sensing_data::SensingDataHeader, error_code) == 15);
static_assert(
  protocol::c_sensing_data_start_idx + offsetof(protocol::sensing_data::SensingDataHeader, lockout_state) == 17);
static_assert(
  protocol::c_sensing_data_start_idx +
    offsetof(protocol::sensing_data::SensingDataHeader, optical_window_contaminated) ==
  43);

static_assert(offsetof(protocol::AR00CommandReply, ranges) == protocol::c_distance_start_idx);
static_assert(offsetof(protocol::AR00CommandReply, ranges) == 50);
static_assert(offsetof(protocol::AR01CommandReply, intensities) == 50 + 4324);
static_assert(offsetof(protocol::AR06CommandReply, ranges) == 50);
static_assert(sizeof(protocol::XR00CommandReply) == 106);

}  // namespace uam

#endif  // INCLUDE_URG_NODE_UAM_UAM_VISITORS_H_
