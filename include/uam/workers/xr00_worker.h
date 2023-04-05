/**
Software License Agreement (proprietary)
\file      xr_00_worker.h
\authors   Carlos Mendes <cribeiromendes@locusrobotics.com>
\copyright Copyright (c) (2023,), Locus Robotics Corp., All rights reserved.
Unauthorized copying of this file, via any medium, is strictly prohibited.
Proprietary and confidential.
**/

#ifndef INCLUDE_URG_NODE_UAM_SENSING_DATA_WORKERS_XR_00_WORKER_H_
#define INCLUDE_URG_NODE_UAM_SENSING_DATA_WORKERS_XR_00_WORKER_H_

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

#endif  // INCLUDE_URG_NODE_UAM_SENSING_DATA_WORKERS_XR_00_WORKER_H_
