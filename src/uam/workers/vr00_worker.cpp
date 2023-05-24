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

#include <uam/workers/vr00_worker.h>

#include <string_view>
#include <optional>

namespace uam
{
VR00Worker::VR00Worker(const uint32_t idx_offset) :
  WorkerBase<VR00Worker, 'V', 'R', '0', '0', protocol::VR00CommandReply>(
    idx_offset,
    idx_offset + offsetof(Reply, footer)),
  version_detail_visitor_(idx_offset + offsetof(Reply, version_details))
{
}
std::optional<VR00Worker::Reply> VR00Worker::decode(const std::string_view& buffer) const
{
  Reply reply;
  if (!decodeHeaderAndFooter(buffer, reply))
    return std::nullopt;
  if (!decodeVersionDetails(buffer, reply.version_details))
    return std::nullopt;
  return reply;
}

std::optional<VR00Worker::Reply> VR00Worker::decode(const protocol::VR00CommandReply& raw_reply) const
{
  Reply reply = raw_reply;
  if (!decodeHeaderAndFooter(reply))
    return std::nullopt;
  if (!decodeVersionDetails(reply.version_details))
    return std::nullopt;
  return reply;
}

bool VR00Worker::decodeVersionDetails(
  const std::string_view& buffer,
  protocol::version_details::VersionDetails& version_details) const
{
  if (!version_detail_visitor_.sensor_model.getRaw(buffer, version_details.sensor_model))
    return false;
  if (!version_detail_visitor_.firmware_version.getRaw(buffer, version_details.firmware_version))
    return false;
  if (!version_detail_visitor_.serial_number.getRaw(buffer, version_details.serial_number))
    return false;
  return true;
}

bool VR00Worker::decodeVersionDetails(protocol::version_details::VersionDetails& version_details) const
{
  // VR command is not encoded
  return true;
}

}  // namespace uam
