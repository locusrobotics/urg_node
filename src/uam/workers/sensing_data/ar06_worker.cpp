/**
Software License Agreement (proprietary)
\file      ar_06_worker.cpp
\authors   Carlos Mendes <cribeiromendes@locusrobotics.com>
\copyright Copyright (c) (2023,), Locus Robotics Corp., All rights reserved.
Unauthorized copying of this file, via any medium, is strictly prohibited.
Proprietary and confidential.
**/

#include <uam/protocol_types/uam_protocol_types.h>
#include <uam/workers/sensing_data/ar06_worker.h>

namespace uam
{
AR06Worker::AR06Worker(const uint32_t idx_offset) :
  WorkerBase<AR06Worker, 'A', 'R', '0', '6', protocol::AR06CommandReply>(
    idx_offset,
    idx_offset + offsetof(Reply, footer)),
  sensing_data_visitor_(idx_offset + protocol::c_sensing_data_start_idx),
  distances_visitor_(idx_offset + protocol::c_distance_start_idx)
{
}

std::optional<AR06Worker::Reply> AR06Worker::decode(const std::string* buffer) const
{
  Reply reply;
  decodeHeaderAndFooter(buffer,reply);
  decodeSensingData(buffer, reply.sensing_data);
  decodeDistances(buffer, reply.ranges);
  return reply;
}

std::optional<AR06Worker::Reply> AR06Worker::decode(const Reply& raw_reply) const
{
  Reply reply = raw_reply;
  decodeHeaderAndFooter(reply);
  WorkerBase<AR06Worker, 'A', 'R', '0', '6', protocol::AR06CommandReply>::decodeSensingData(reply.sensing_data);
  decodeField(reply.ranges);
  return reply;
}

void AR06Worker::decodeSensingData(const std::string* buffer, protocol::sensing_data::SensingDataHeader& sensing_data)
  const
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

void AR06Worker::decodeDistances(
  const std::string* buffer,
  protocol::sensing_data::DistanceDataArray<2161>& distance_data) const
{
  distances_visitor_.distances.get(buffer, distance_data);
}
}  // namespace uam
