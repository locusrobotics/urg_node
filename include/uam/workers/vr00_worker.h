/**
Software License Agreement (proprietary)
\file      vr_00_worker.h
\authors   Carlos Mendes <cribeiromendes@locusrobotics.com>
\copyright Copyright (c) (2023,), Locus Robotics Corp., All rights reserved.
Unauthorized copying of this file, via any medium, is strictly prohibited.
Proprietary and confidential.
**/

#ifndef INCLUDE_URG_NODE_UAM_SENSING_DATA_WORKERS_VR_00_WORKER_H_
#define INCLUDE_URG_NODE_UAM_SENSING_DATA_WORKERS_VR_00_WORKER_H_

#include <uam/protocol_types/uam_protocol_types.h>
#include <uam/workers/uam_worker_base.h>

namespace uam
{
/**
 * @brief VR00 command worker
 */
class VR00Worker : public WorkerBase<VR00Worker, 'V', 'R', '0', '0', protocol::VR00CommandReply>
{
public:
  /**
   * @brief Default C'tor
   *
   * @param[in] idx_offset - Offset to be used in the buffer deserialization
   */
  explicit VR00Worker(const uint32_t idx_offset = 0);

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
   * @brief Decode version details using raw buffer
   *
   * @param[in] buffer - Raw byte array
   * @param[out] sensing_data - Decoded sensing data
   */
  void decodeVersionDetails(const std::string* buffer, protocol::version_details::VersionDetails& version_details)
    const;

  /**
   * @brief Decode version data using raw_reply
   *
   * @param[in] version_details - Raw reply
   * @param[out] sensing_data - Decoded sensing data
   */
  void decodeVersionDetails(protocol::version_details::VersionDetails& version_details) const;

  /**
   * @brief Version details visitor (if buffer)
   */
  VersionDetailsVisitor version_detail_visitor_;
};

}  // namespace uam

#endif  // INCLUDE_URG_NODE_UAM_SENSING_DATA_WORKERS_VR_00_WORKER_H_
