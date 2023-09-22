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

#include <uam/workers/id00_worker.h>

#include <optional>
#include <string_view>

namespace uam
{
ID00Worker::ID00Worker(const uint32_t idx_offset) :
  WorkerBase<ID00Worker, 'I', 'D', '0', '0', protocol::ID00CommandReply>(
    idx_offset,
    idx_offset + offsetof(Reply, footer)),
  configuration_id_visitor_(idx_offset + offsetof(protocol::ID00CommandReply, config_id))
{
}
std::optional<ID00Worker::Reply> ID00Worker::decode(const std::string_view& buffer) const
{
  Reply reply;
  if (!decodeHeaderAndFooter(buffer, reply))
    return std::nullopt;
  if (!decodeConfigurationID(buffer, reply.config_id))
    return std::nullopt;
  return reply;
}

std::optional<ID00Worker::Reply> ID00Worker::decode(const Reply& raw_reply) const
{
  Reply reply = raw_reply;
  if (!decodeHeaderAndFooter(reply))
    return std::nullopt;
  return reply;
}

bool ID00Worker::decodeConfigurationID(const std::string_view& buffer,
                                       protocol::configuration_details::ConfigurationID& configuration_id) const
{
  if (configuration_id_visitor_.id_1.getRaw(buffer, configuration_id.id_1))
    return false;
  if (configuration_id_visitor_.id_2.getRaw(buffer, configuration_id.id_2))
      return false;
  return true;
}

}  // namespace uam
