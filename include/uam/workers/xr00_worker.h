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

#ifndef URG_NODE_UAM_WORKERS_XR_00_WORKER_H
#define URG_NODE_UAM_WORKERS_XR_00_WORKER_H

#include <uam/protocol_types/uam_protocol_types.h>
#include <uam/workers/uam_worker_base.h>

namespace uam
{
/**
 * @brief XR00 command worker
 */
class XR00Worker : public WorkerBase<XR00Worker, 'X', 'R', '0', '0', protocol::XR00CommandReply>
{
public:
  /**
   * @brief Default C'tor
   *
   * @param[in] idx_offset - Offset to be used in the buffer deserialization
   */
  explicit XR00Worker(const uint32_t idx_offset = 0);

  /**
   * @brief Decode using raw buffer
   *
   * @param[in] buffer - Raw byte array
   * @return Decoded Reply or std::nullopt if decode failed
   */
  std::optional<Reply> decode(const std::string* buffer) const;

  /**
   * @brief Decode the incoming raw_reply
   *
   * @param[in] raw_reply - Raw reply
   * @return Decoded reply, or std::nullopt if decode fails
   */
  std::optional<Reply> decode(const Reply& raw_reply) const;

  /**
   * @brief Validate the expected size of the reply
   *
   * @param[in] recv_bytes - Number of received bytes
   * @return true if size check passes, false otherwise
   */
  inline const bool validateSize(const size_t recv_bytes) const { return recv_bytes == sizeof(Reply); }

private:
  /**
   * @brief Decode status data using raw buffer
   *
   * @param[in] buffer - Raw byte array
   * @param[out] sensing_data - Decoded sensing data
   */
  void decodeStatusData(const std::string* buffer, protocol::sensing_data::StatusData& status_data) const;

  /**
   * @brief Decode status data using raw reply
   *
   * @param[in/out] status_data - Raw reply struct
   */
  void decodeStatusData(protocol::sensing_data::StatusData& status_data) const;

  /**
   * @brief Status data visitor (if buffer)
   */
  StatusDataVisitor status_data_visitor_;
};

}  // namespace uam

#endif  // URG_NODE_UAM_WORKERS_XR_00_WORKER_H
