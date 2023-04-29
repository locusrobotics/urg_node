/**
Software License Agreement (proprietary)
\file      vr_00_worker.cpp
\authors   Carlos Mendes <cribeiromendes@locusrobotics.com>
\copyright Copyright (c) (2023,), Locus Robotics Corp., All rights reserved.
Unauthorized copying of this file, via any medium, is strictly prohibited.
Proprietary and confidential.
**/

#include <uam/workers/vr00_worker.h>

namespace uam
{
VR00Worker::VR00Worker(const uint32_t idx_offset) :
  WorkerBase<VR00Worker,'V', 'R', '0', '0', protocol::VR00CommandReply>(
    idx_offset,
    idx_offset + offsetof(Reply, footer)),
  version_detail_visitor_(idx_offset + offsetof(Reply, version_details))
{
}
std::optional<VR00Worker::Reply> VR00Worker::decode(const std::string* buffer) const
{
  Reply reply;
  decodeHeaderAndFooter(buffer, reply);
  decodeVersionDetails(buffer, reply.version_details);
  return reply;
}

std::optional<VR00Worker::Reply> VR00Worker::decode(const protocol::VR00CommandReply& raw_reply) const
{
  Reply reply = raw_reply;
  decodeHeaderAndFooter(reply);
  decodeVersionDetails(reply.version_details);
  return reply;
}

void VR00Worker::decodeVersionDetails(
  const std::string* buffer,
  protocol::version_details::VersionDetails& version_details) const
{
  version_detail_visitor_.sensor_model.get(buffer, version_details.sensor_model);
  version_detail_visitor_.firmware_version.get(buffer, version_details.firmware_version);
  version_detail_visitor_.serial_number.get(buffer, version_details.serial_number);
}

void VR00Worker::decodeVersionDetails(protocol::version_details::VersionDetails& version_details) const
{
  decodeField(version_details.sensor_model);
  decodeField(version_details.comma0);
  decodeField(version_details.firmware_version);
  decodeField(version_details.serial_number);
}

}  // namespace uam
