/**
Software License Agreement (proprietary)
\file      uam_command_workers.h
\authors   Carlos Mendes <cribeiromendes@locusrobotics.com>
\copyright Copyright (c) (2023,), Locus Robotics Corp., All rights reserved.
Unauthorized copying of this file, via any medium, is strictly prohibited.
Proprietary and confidential.
**/

#ifndef INCLUDE_URG_NODE_UAM_UAM_COMMAND_WORKERS_H_
#define INCLUDE_URG_NODE_UAM_UAM_COMMAND_WORKERS_H_

#include <urg_node/uam/uam_protocol_types.h>
#include <urg_node/uam/uam_worker_base.h>
#include <urg_node/uam/uam_visitors.h>

namespace uam
{
/**
 * @brief AR00 command worker
 */
class AR00Worker : public WorkerBase<'A', 'R', '0', '0'>
{
public:
  /**
   * @brief Default C'tor
   */
  explicit AR00Worker(const uint32_t idx_offset = 0);
  /**
   * @brief
   *
   * @param[in] buffer
   * @param[out] reply
   * @return
   */
  bool process(const std::string* buffer, protocol::AR00CommandReply& reply);

  /**
   * @brief
   *
   * @param[in] buffer
   * @param[out] sensing_data
   */
  void decodeSensingData(const std::string* buffer, protocol::sensing_data::SensingDataHeader& sensing_data);

  /**
   * @brief
   *
   * @param[in] buffer
   * @param[out] distance_data
   */
  void decodeDistances(const std::string* buffer, protocol::sensing_data::DistanceDataArray<1081>& distance_data);

private:
  /**
   * @brief
   *
   */
  SensingDataVisitor sensing_data_visitor_;
  /**
   * @brief
   *
   */
  DistanceDataVisitor<1081> distances_visitor_;
};

/**
 * @brief AR01 command worker
 */
class AR01Worker : public WorkerBase<'A', 'R', '0', '1'>
{
public:
  /**
   * @brief Default C'tor
   */
  explicit AR01Worker(const uint32_t idx_offset = 0);
  bool process(const std::string* buffer, protocol::AR01CommandReply& reply);

private:
  void decodeSensingData(const std::string* buffer, protocol::sensing_data::SensingDataHeader& sensing_data);
  void decodeDistances(const std::string* buffer, protocol::sensing_data::DistanceDataArray<1081>& distance_data);
  void decodeIntensities(const std::string* buffer, protocol::sensing_data::IntensityDataArray<1081>& intensity_data);

private:
  SensingDataVisitor sensing_data_visitor_;
  DistanceDataVisitor<1081> distances_visitor_;
  IntensityDataArrayVisitor<1081> intensities_visitor_;
};

/**
 * @brief XR00 command worker
 */
class XR00Worker : public WorkerBase<'X', 'R', '0', '0'>
{
public:
  /**
   * @brief Default C'tor
   */
  explicit XR00Worker(const uint32_t idx_offset = 0);
  bool process(const std::string* buffer, protocol::XR00CommandReply& reply);
private:
  void decodeStatusData(const std::string* buffer, protocol::sensing_data::StatusData& status_data);
  StatusDataVisitor status_data_visitor_;
};

/**
 * @brief VR00 command worker
 */
class VR00Worker : public WorkerBase<'V', 'R', '0', '0'>
{
public:
  /**
   * @brief Default C'tor
   */
  explicit VR00Worker(const uint32_t idx_offset = 0);
  bool process(const std::string* buffer, protocol::VR00CommandReply& reply);
private:
  void decodeVersionDetails(const std::string* buffer, protocol::version_details::VersionDetails& version_details);
  VersionDetailsVisitor version_detail_visitor_;
};
}  // namespace uam

#endif  // INCLUDE_URG_NODE_UAM_UAM_COMMAND_WORKERS_H_
