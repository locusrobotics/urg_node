/**
Software License Agreement (proprietary)
\file      ar_01_worker.h
\authors   Carlos Mendes <cribeiromendes@locusrobotics.com>
\copyright Copyright (c) (2023,), Locus Robotics Corp., All rights reserved.
Unauthorized copying of this file, via any medium, is strictly prohibited.
Proprietary and confidential.
**/

#ifndef INCLUDE_URG_NODE_UAM_SENSING_DATA_AR_01_WORKER_H_
#define INCLUDE_URG_NODE_UAM_SENSING_DATA_AR_01_WORKER_H_

#include <uam/protocol_types/uam_protocol_types.h>
#include <uam/workers/uam_worker_base.h>

namespace uam
{
/**
 * @brief AR01 command worker
 */
class AR01Worker : public WorkerBase<AR01Worker, 'A', 'R', '0', '1', protocol::AR01CommandReply>
{
public:
  /**
   * @brief Default C'tor
   */
  explicit AR01Worker(const uint32_t idx_offset = 0);

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
   * @brief Decode sensing data using raw buffer
   *
   * @param[in] buffer - Raw byte array
   * @param[out] sensing_data - Decoded sensing data
   */
  void decodeSensingData(const std::string* buffer, protocol::sensing_data::SensingDataHeader& sensing_data) const;

  /**
   * @brief Decode distances using raw buffer
   *
   * @param[in] buffer - Raw byte array
   * @param[out] distance_data - Decoded distance data
   */
  void decodeDistances(const std::string* buffer, protocol::sensing_data::DistanceDataArray<1081>& distance_data) const;

  /**
   * @brief Decode intensity using raw buffer
   *
   * @param[in] buffer - Raw byte array
   * @param[out] distance_data - Decoded intensity data
   */
  void decodeIntensities(const std::string* buffer, protocol::sensing_data::IntensityDataArray<1081>& intensity_data)
    const;

  /**
   * @brief Sensing data visitor (if buffer)
   */
  SensingDataVisitor sensing_data_visitor_;

  /**
   * @brief Distance data visitor (if buffer)
   */
  DistanceDataVisitor<1081> distances_visitor_;

  /**
   * @brief Intensity data visitor (if buffer)
   */
  IntensityDataArrayVisitor<1081> intensities_visitor_;
};
}  // namespace uam

#endif  // INCLUDE_URG_NODE_UAM_SENSING_DATA_AR_01_WORKER_H_
