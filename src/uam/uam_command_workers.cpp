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

#include <urg_node/uam/uam_command_workers.h>

#include <ros/ros.h>

namespace uam
{
AR00Worker::AR00Worker(const uint32_t idx_offset) :
  WorkerBase<'A', 'R', '0', '0'>(idx_offset, idx_offset + offsetof(protocol::AR00CommandReply, footer)),
  sensing_data_visitor_(idx_offset + protocol::c_sensing_data_start_idx),
  distances_visitor_(idx_offset + protocol::c_distance_start_idx)
{
}

bool AR00Worker::process(const std::string* buffer, protocol::AR00CommandReply& reply)
{
  if (!validate(buffer, reply))
  {
    return false;
  }
  decodeSensingData(buffer, reply.sensing_data);
  decodeDistances(buffer, reply.ranges);
  return true;
}

void AR00Worker::decodeSensingData(const std::string* buffer, protocol::sensing_data::SensingDataHeader& sensing_data)
{
  sensing_data_visitor_.operating_mode.get(buffer, sensing_data.operating_mode);
  sensing_data_visitor_.area_number.get(buffer, sensing_data.area_number);

  // Grab the Error Status
  sensing_data_visitor_.error_state.get(buffer, sensing_data.error_state);
  // Grab the error code and offset by 0x40 if non-zero as per documentation
  sensing_data_visitor_.error_code.get(buffer, sensing_data.error_code);
  if (sensing_data.error_code != 0)
  {
    sensing_data.error_code += 0x40;
  }
  // Grab the lockout_state
  sensing_data_visitor_.lockout_state.get(buffer, sensing_data.lockout_state);
  sensing_data_visitor_.ossd1_state.get(buffer, sensing_data.ossd1_state);
  sensing_data_visitor_.ossd2_state.get(buffer, sensing_data.ossd2_state);
  sensing_data_visitor_.warning1_state.get(buffer, sensing_data.warning1_state);
  sensing_data_visitor_.warning2_state.get(buffer, sensing_data.warning2_state);
  sensing_data_visitor_.optical_window_contaminated.get(buffer, sensing_data.optical_window_contaminated);
}

void AR00Worker::decodeDistances(
  const std::string* buffer,
  protocol::sensing_data::DistanceDataArray<1081>& distance_data)
{
  distances_visitor_.distances.get(buffer, distance_data);
}

// ----------------------------------------------------------------------------

AR01Worker::AR01Worker(const uint32_t idx_offset) :
  WorkerBase<'A', 'R', '0', '1'>(idx_offset, idx_offset + offsetof(protocol::AR01CommandReply, footer)),
  sensing_data_visitor_(idx_offset + protocol::c_sensing_data_start_idx),
  distances_visitor_(idx_offset + protocol::c_distance_start_idx),
  intensities_visitor_(idx_offset + offsetof(protocol::AR01CommandReply, intensities))
{
}
bool AR01Worker::process(const std::string* buffer, protocol::AR01CommandReply& reply)
{
  if (!validate(buffer, reply))
  {
    return false;
  }
  decodeSensingData(buffer, reply.sensing_data);
  decodeDistances(buffer, reply.ranges);
  decodeIntensities(buffer, reply.intensities);
  return true;
}

void AR01Worker::decodeSensingData(const std::string* buffer, protocol::sensing_data::SensingDataHeader& sensing_data)
{
  sensing_data_visitor_.operating_mode.get(buffer, sensing_data.operating_mode);
  sensing_data_visitor_.area_number.get(buffer, sensing_data.area_number);

  // Grab the Error Status
  sensing_data_visitor_.error_state.get(buffer, sensing_data.error_state);
  // Grab the error code and offset by 0x40 if non-zero as per documentation
  sensing_data_visitor_.error_code.get(buffer, sensing_data.error_code);
  if (sensing_data.error_code != 0)
  {
    sensing_data.error_code += 0x40;
  }
  // Grab the lockout_state
  sensing_data_visitor_.lockout_state.get(buffer, sensing_data.lockout_state);
  sensing_data_visitor_.ossd1_state.get(buffer, sensing_data.ossd1_state);
  sensing_data_visitor_.ossd2_state.get(buffer, sensing_data.ossd2_state);
  sensing_data_visitor_.warning1_state.get(buffer, sensing_data.warning1_state);
  sensing_data_visitor_.warning2_state.get(buffer, sensing_data.warning2_state);
  sensing_data_visitor_.optical_window_contaminated.get(buffer, sensing_data.optical_window_contaminated);
}

void AR01Worker::decodeDistances(
  const std::string* buffer,
  protocol::sensing_data::DistanceDataArray<1081>& distance_data)
{
  distances_visitor_.distances.get(buffer, distance_data);
}
void AR01Worker::decodeIntensities(
  const std::string* buffer,
  protocol::sensing_data::IntensityDataArray<1081>& intensity_data)
{
  intensities_visitor_.intensities.get(buffer, intensity_data);
}

///-----------------------------------------------------------------------------
///
XR00Worker::XR00Worker(const uint32_t idx_offset) :
  WorkerBase<'X', 'R', '0', '0'>(idx_offset, idx_offset + offsetof(protocol::XR00CommandReply, footer)),
  status_data_visitor_(idx_offset + offsetof(protocol::XR00CommandReply, data))
{
}
bool XR00Worker::process(const std::string* buffer, protocol::XR00CommandReply& reply)
{
  if (!validate(buffer, reply))
  {
    return false;
  }
  decodeStatusData(buffer, reply.data);
  return true;
}

void XR00Worker::decodeStatusData(const std::string* buffer, protocol::sensing_data::StatusData& status_data)
{
  status_data_visitor_.area_number.get(buffer, status_data.area_number);
  status_data_visitor_.encoder_speed.get(buffer, status_data.encoder_speed);
  status_data_visitor_.error_code.get(buffer, status_data.error_code);
  // Grab the error code and offset by 0x40 if non-zero as per documentation
  if (status_data.error_code != 0)
  {
    status_data.error_code += 0x40;
  }
  status_data_visitor_.error_state.get(buffer, status_data.error_state);
  status_data_visitor_.laser_state_off.get(buffer, status_data.laser_state_off);
  status_data_visitor_.lockout_state.get(buffer, status_data.lockout_state);
  status_data_visitor_.muting_state1.get(buffer, status_data.muting_state1);
  status_data_visitor_.muting_state2.get(buffer, status_data.muting_state2);
  status_data_visitor_.operating_mode.get(buffer, status_data.operating_mode);
  status_data_visitor_.optical_window_contaminated.get(buffer, status_data.optical_window_contaminated);
  status_data_visitor_.ossd1_state.get(buffer, status_data.ossd1_state);
  status_data_visitor_.ossd2_state.get(buffer, status_data.ossd2_state);
  status_data_visitor_.ossd3_state.get(buffer, status_data.ossd3_state);
  status_data_visitor_.ossd4_state.get(buffer, status_data.ossd4_state);

  status_data_visitor_.warning1_state.get(buffer, status_data.warning1_state);
  status_data_visitor_.warning2_state.get(buffer, status_data.warning2_state);
  status_data_visitor_.reset_request1.get(buffer, status_data.reset_request1);
  status_data_visitor_.reset_request2.get(buffer, status_data.reset_request2);

  status_data_visitor_.slave1_error_state.get(buffer, status_data.slave1_error_state);
  status_data_visitor_.slave1_laser_off_state.get(buffer, status_data.slave1_laser_off_state);
  status_data_visitor_.slave1_ossd1_2_state.get(buffer, status_data.slave1_ossd1_2_state);
  status_data_visitor_.slave1_ossd3_4_state.get(buffer, status_data.slave1_ossd3_4_state);
  status_data_visitor_.slave1_warning_1_state.get(buffer, status_data.slave1_warning_1_state);
  status_data_visitor_.slave1_warning_2_state.get(buffer, status_data.slave1_warning_2_state);

  status_data_visitor_.slave2_error_state.get(buffer, status_data.slave2_error_state);
  status_data_visitor_.slave2_laser_off_state.get(buffer, status_data.slave2_laser_off_state);
  status_data_visitor_.slave2_ossd1_2_state.get(buffer, status_data.slave2_ossd1_2_state);
  status_data_visitor_.slave2_ossd3_4_state.get(buffer, status_data.slave2_ossd3_4_state);
  status_data_visitor_.slave2_warning_1_state.get(buffer, status_data.slave2_warning_1_state);
  status_data_visitor_.slave2_warning_2_state.get(buffer, status_data.slave2_warning_2_state);

  status_data_visitor_.slave3_error_state.get(buffer, status_data.slave3_error_state);
  status_data_visitor_.slave3_laser_off_state.get(buffer, status_data.slave3_laser_off_state);
  status_data_visitor_.slave3_ossd1_2_state.get(buffer, status_data.slave3_ossd1_2_state);
  status_data_visitor_.slave3_ossd3_4_state.get(buffer, status_data.slave3_ossd3_4_state);
  status_data_visitor_.slave3_warning_1_state.get(buffer, status_data.slave3_warning_1_state);
  status_data_visitor_.slave3_warning_2_state.get(buffer, status_data.slave3_warning_2_state);
  status_data_visitor_.timestamp.get(buffer, status_data.timestamp);
}

///-----------------------------------------------------------------------------
///
VR00Worker::VR00Worker(const uint32_t idx_offset) :
  WorkerBase<'V', 'R', '0', '0'>(idx_offset, idx_offset + offsetof(protocol::VR00CommandReply, footer)),
  version_detail_visitor_(idx_offset + offsetof(protocol::VR00CommandReply, version_details))
{
}
bool VR00Worker::process(const std::string* buffer, protocol::VR00CommandReply& reply)
{
  if (!validate(buffer, reply))
  {
    return false;
  }
  decodeVersionDetails(buffer, reply.version_details);
  return true;
}

void VR00Worker::decodeVersionDetails(
  const std::string* buffer,
  protocol::version_details::VersionDetails& version_details)
{
  version_detail_visitor_.sensor_model.get(buffer, version_details.sensor_model);
  version_detail_visitor_.firmware_version.get(buffer, version_details.firmware_version);
  version_detail_visitor_.serial_number.get(buffer, version_details.serial_number);
}

}  // namespace uam
