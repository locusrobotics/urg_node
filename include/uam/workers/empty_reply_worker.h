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

#ifndef URG_NODE_UAM_WORKERS_EMPTY_REPLY_WORKER_H
#define URG_NODE_UAM_WORKERS_EMPTY_REPLY_WORKER_H

#include <uam/workers/uam_worker_base.h>

#include <optional>

namespace uam
{
/**
 * @brief Empty Reply command worker
 *
 * @tparam HeaderMSB - This is the MSByte in the Command header
 * @tparam HeaderLSB - This is the LSByte in the Command header
 * @tparam SubHeaderMSB - This is the MSByte in the Command sub header
 * @tparam SubHeaderLSB - This is the LSByte in the Command sub header
 */
template <char HeaderMSB, char HeaderLSB, char SubHeaderMSB, char SubHeaderLSB>
class EmptyReplyWorker :
  public WorkerBase<
    EmptyReplyWorker<HeaderMSB, HeaderLSB, SubHeaderMSB, SubHeaderLSB>,
    HeaderMSB,
    HeaderLSB,
    SubHeaderMSB,
    SubHeaderLSB,
    protocol::EmptyCommandReply>
{
public:
  /**
   * @brief Default C'tor
   *
   * @param[in] idx_offset - Offset to be used in the buffer deserialization
   */
  explicit EmptyReplyWorker(const uint32_t idx_offset = 0) :
    WorkerBase<
      EmptyReplyWorker<HeaderMSB, HeaderLSB, SubHeaderMSB, SubHeaderLSB>,
      HeaderMSB,
      HeaderLSB,
      SubHeaderMSB,
      SubHeaderLSB,
      protocol::EmptyCommandReply>(idx_offset, idx_offset + offsetof(protocol::EmptyCommandReply, footer))
  {
  }

  /**
   * @brief Decode using raw buffer
   *
   * @param[in] buffer - Raw byte array
   * @return Decoded Reply or std::nullopt if decode failed
   */
  std::optional<protocol::EmptyCommandReply> decode(const std::string* buffer) const
  {
    protocol::EmptyCommandReply reply;
    this->decodeHeaderAndFooter(buffer, reply);
    if (!this->validateCrc(buffer, reply))
    {
      return std::nullopt;
    }
    return reply;
  }

  /**
   * @brief Decode the incoming raw_reply
   *
   * @param[in] raw_reply - Raw reply
   * @return Decoded reply, or std::nullopt if decode fails
   */
  std::optional<protocol::EmptyCommandReply> decode(const protocol::EmptyCommandReply& raw_reply) const
  {
    protocol::EmptyCommandReply reply = raw_reply;
    this->decodeHeaderAndFooter(reply);
    return reply;
  }

  /**
   * @brief Validate the expected size of the reply
   *
   * @param[in] recv_bytes - Number of received bytes
   * @return true if size check passes, false otherwise
   */
  inline const bool validateSize(const size_t recv_bytes) const
  {
    return recv_bytes == sizeof(protocol::EmptyCommandReply);
  }
};
}  // namespace uam

#endif  // URG_NODE_UAM_WORKERS_EMPTY_REPLY_WORKER_H
