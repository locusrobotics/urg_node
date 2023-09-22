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

#ifndef UAM_WORKERS_ID00_WORKER_H
#define UAM_WORKERS_ID00_WORKER_H

#include <uam/protocol_types/uam_protocol_types.h>
#include <uam/workers/uam_worker_base.h>
#include <uam/uam_visitors.h>

#include <string_view>
#include <optional>

namespace uam
{
/**
 * @brief ID00 command worker
 */
class ID00Worker : public WorkerBase<ID00Worker, 'I', 'D', '0', '0', protocol::ID00CommandReply>
{
public:
  /**
   * @brief Default C'tor
   *
   * @param[in] idx_offset - Offset to be used in the buffer deserialization
   */
  explicit ID00Worker(const uint32_t idx_offset = 0);

  /**
   * @brief Decode using raw buffer
   *
   * @param[in] buffer - Raw byte array
   * @return Decoded Reply or std::nullopt if decode failed
   */
  std::optional<Reply> decode(const std::string_view& buffer) const;

  /**
   * @brief Decode the incoming raw_reply
   *
   * @param[in] raw_reply - Raw reply
   * @return Decoded reply, or std::nullopt if decode fails
   */
  std::optional<Reply> decode(const Reply& raw_reply) const;

private:
  /**
   * @brief Decode status data using raw buffer
   *
   * @param[in] buffer - Raw byte array
   * @param[out] configuration_id - Decoded configuratio id
   */
  bool decodeConfigurationID(const std::string_view& buffer,
                             protocol::configuration_details::ConfigurationID& configuration_id) const;

  /**
   * @brief Configuration ID buffer visitor
   */
  ConfigurationIdVisitor configuration_id_visitor_;
};

}  // namespace uam

#endif  // UAM_WORKERS_ID00_WORKER_H
