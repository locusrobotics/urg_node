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

#ifndef UAM_UAM_VISITORS_H
#define UAM_UAM_VISITORS_H

#include <uam/protocol_types/uam_protocol_types.h>
#include <uam/visitor.h>

namespace uam
{
/**
 * @brief
 */
class CommandHeaderVisitor
{
public:
  explicit CommandHeaderVisitor(const size_t initial_offset) :
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
  explicit CommandFooterVisitor(const size_t initial_offset) : crc(initial_offset), etx(initial_offset) {}

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
  explicit SensingDataVisitor(const size_t initial_offset) :
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
  explicit StatusDataVisitor(const size_t initial_offset) :
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
  explicit DistanceDataVisitor(const size_t initial_offset) : distances(initial_offset) {}

  TVisitor<protocol::sensing_data::DistanceDataArray<Steps>, 0> distances;
};
template <size_t Steps>
class IntensityDataArrayVisitor
{
public:
  explicit IntensityDataArrayVisitor(const size_t initial_offset) : intensities(initial_offset) {}
  TVisitor<protocol::sensing_data::IntensityDataArray<Steps>, 0> intensities;
};

class VersionDetailsVisitor
{
public:
  /**
   * @brief Constructor if visitors receive raw buffer
   * @param initial_offset
   */
  explicit VersionDetailsVisitor(const size_t initial_offset) :
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

class ConfigurationIdVisitor
{
public:
  /**
   * @brief Constructor if visitors receive raw buffer
   * @param initial_offset
   */
  explicit ConfigurationIdVisitor(const size_t initial_offset) : id_1(initial_offset), id_2(initial_offset) {}

#define VISITOR_MEMBER_IMPL_CONFIGURATION_DETAILS(field) \
  VISITOR_MEMBER(protocol::configuration_details::ConfigurationID, field)

  VISITOR_MEMBER_IMPL_CONFIGURATION_DETAILS(id_1);
  VISITOR_MEMBER_IMPL_CONFIGURATION_DETAILS(id_2);
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

static_assert(offsetof(protocol::AR00CommandReply, ranges) == protocol::c_distance_start_idx);
static_assert(offsetof(protocol::AR00CommandReply, ranges) == 50);
static_assert(offsetof(protocol::AR01CommandReply, intensities) == 50 + 4324);
static_assert(offsetof(protocol::AR06CommandReply, ranges) == 50);
static_assert(sizeof(protocol::XR00CommandReply) == 106);

}  // namespace uam

#endif  // UAM_UAM_VISITORS_H
