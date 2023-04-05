/**
Software License Agreement (proprietary)
\file      xr_00_worker.cpp
\authors   Carlos Mendes <cribeiromendes@locusrobotics.com>
\copyright Copyright (c) (2023,), Locus Robotics Corp., All rights reserved.
Unauthorized copying of this file, via any medium, is strictly prohibited.
Proprietary and confidential.
**/

#include <uam/workers/xr00_worker.h>

namespace uam
{
XR00Worker::XR00Worker(const uint32_t idx_offset) :
  WorkerBase<XR00Worker, 'X', 'R', '0', '0', protocol::XR00CommandReply>(
    idx_offset,
    idx_offset + offsetof(Reply, footer)),
  status_data_visitor_(idx_offset + offsetof(protocol::XR00CommandReply, data))
{
}
std::optional<XR00Worker::Reply> XR00Worker::decode(const std::string* buffer) const
{
  Reply reply;
  if (!validate(buffer, reply))
  {
    return std::nullopt;
  }
  decodeStatusData(buffer, reply.data);
  return reply;
}

std::optional<XR00Worker::Reply> XR00Worker::decode(const Reply& raw_reply) const
{
  Reply reply = raw_reply;
  if (!validate(reply))
  {
    return std::nullopt;
  }
  decodeStatusData(reply.data);
  return reply;
}

void XR00Worker::decodeStatusData(const std::string* buffer, protocol::sensing_data::StatusData& status_data) const
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

void XR00Worker::decodeStatusData(protocol::sensing_data::StatusData& status_data) const
{
  decodeField(status_data.area_number);
  status_data.area_number += 1;
  decodeField(status_data.encoder_speed);
  decodeField(status_data.error_code);
  // Grab the error code and offset by 0x40 if non-zero as per documentation
  if (status_data.error_code != 0)
  {
    status_data.error_code += 0x40;
  }
  decodeField(status_data.error_state);
  decodeField(status_data.laser_state_off);
  decodeField(status_data.lockout_state);
  decodeField(status_data.muting_state1);
  decodeField(status_data.muting_state2);
  decodeField(status_data.operating_mode);
  decodeField(status_data.optical_window_contaminated);
  decodeField(status_data.ossd1_state);
  decodeField(status_data.ossd2_state);
  decodeField(status_data.ossd3_state);
  decodeField(status_data.ossd4_state);

  decodeField(status_data.warning1_state);
  decodeField(status_data.warning2_state);
  decodeField(status_data.reset_request1);
  decodeField(status_data.reset_request2);

  decodeField(status_data.slave1_error_state);
  decodeField(status_data.slave1_laser_off_state);
  decodeField(status_data.slave1_ossd1_2_state);
  decodeField(status_data.slave1_ossd3_4_state);
  decodeField(status_data.slave1_warning_1_state);
  decodeField(status_data.slave1_warning_2_state);

  decodeField(status_data.slave2_error_state);
  decodeField(status_data.slave2_laser_off_state);
  decodeField(status_data.slave2_ossd1_2_state);
  decodeField(status_data.slave2_ossd3_4_state);
  decodeField(status_data.slave2_warning_1_state);
  decodeField(status_data.slave2_warning_2_state);

  decodeField(status_data.slave3_error_state);
  decodeField(status_data.slave3_laser_off_state);
  decodeField(status_data.slave3_ossd1_2_state);
  decodeField(status_data.slave3_ossd3_4_state);
  decodeField(status_data.slave3_warning_1_state);
  decodeField(status_data.slave3_warning_2_state);
  decodeField(status_data.timestamp);
}

}  // namespace uam
