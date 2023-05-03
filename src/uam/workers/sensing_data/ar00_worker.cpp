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

#include <uam/protocol_types/uam_protocol_types.h>
#include <uam/workers/sensing_data/ar00_worker.h>

#include <optional>
#include <string>

namespace uam
{
AR00Worker::AR00Worker(const uint32_t idx_offset) :
  WorkerBase<AR00Worker, 'A', 'R', '0', '0', protocol::AR00CommandReply>(
    idx_offset,
    idx_offset + offsetof(Reply, footer)),
  sensing_data_visitor_(idx_offset + protocol::c_sensing_data_start_idx),
  distances_visitor_(idx_offset + protocol::c_distance_start_idx)
{
}

std::optional<AR00Worker::Reply> AR00Worker::decode(const std::string* buffer) const
{
  Reply reply;
  decodeHeaderAndFooter(buffer, reply);
  decodeSensingData(buffer, reply.sensing_data);
  decodeDistances(buffer, reply.ranges);
  return reply;
}

std::optional<AR00Worker::Reply> AR00Worker::decode(const Reply& raw_reply) const
{
  Reply reply = raw_reply;
  decodeHeaderAndFooter(reply);
  WorkerBase<AR00Worker, 'A', 'R', '0', '0', protocol::AR00CommandReply>::decodeSensingData(reply.sensing_data);
  // Decode ranges
  decodeField(reply.ranges);
  return reply;
}

void AR00Worker::decodeSensingData(const std::string* buffer, protocol::sensing_data::SensingDataHeader& sensing_data)
  const
{
  sensing_data_visitor_.operating_mode.get(buffer, sensing_data.operating_mode);
  sensing_data_visitor_.area_number.get(buffer, sensing_data.area_number, true);

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
  sensing_data_visitor_.ossd3_state.get(buffer, sensing_data.ossd3_state);
  sensing_data_visitor_.ossd4_state.get(buffer, sensing_data.ossd4_state);

  sensing_data_visitor_.muting_state1.get(buffer, sensing_data.muting_state1);
  sensing_data_visitor_.muting_state2.get(buffer, sensing_data.muting_state2);
  sensing_data_visitor_.reset_request1.get(buffer, sensing_data.reset_request1);
  sensing_data_visitor_.reset_request2.get(buffer, sensing_data.reset_request2);
  sensing_data_visitor_.encoder_speed.get(buffer, sensing_data.encoder_speed);
  sensing_data_visitor_.timestamp.get(buffer, sensing_data.timestamp);
  sensing_data_visitor_.laser_state_off.get(buffer, sensing_data.laser_state_off);
  sensing_data_visitor_.optical_window_contaminated.get(buffer, sensing_data.optical_window_contaminated);
}

void AR00Worker::decodeDistances(
  const std::string* buffer,
  protocol::sensing_data::DistanceDataArray<1081>& distance_data) const
{
  distances_visitor_.distances.get(buffer, distance_data);
}

}  // namespace uam
