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

// sensing data workers
#include <uam/shape_shifter_buffer.h>
#include <uam/workers/empty_reply_worker.h>
#include <uam/workers/sensing_data/ar00_worker.h>
#include <uam/workers/sensing_data/ar01_worker.h>
#include <uam/workers/sensing_data/ar06_worker.h>
#include <uam/workers/vr00_worker.h>
#include <uam/workers/xr00_worker.h>
#include <uam/workers/yr_worker.h>

#include <tuple>
#include <type_traits>

namespace uam
{
/**
 * @brief Empty Reply commands.
 */
using AR02Worker = EmptyReplyWorker<'A', 'R', '0', '2'>;
using AR03Worker = EmptyReplyWorker<'A', 'R', '0', '3'>;
using AR04Worker = EmptyReplyWorker<'A', 'R', '0', '4'>;
using AR05Worker = EmptyReplyWorker<'A', 'R', '0', '5'>;
using AR07Worker = EmptyReplyWorker<'A', 'R', '0', '7'>;
using AR08Worker = EmptyReplyWorker<'A', 'R', '0', '8'>;

/**
 * @brief UAM packet worker to assemble all the possible UAM workers
 */
class UamPacketWorker
{
public:
  /**
   * @brief Call specific worker and get processed message
   *
   * @param[in] packet - Packet to process
   * return Packet decoded or std::nullopt if
   */
  template <typename TWorkerType, typename... TArgs>
  std::string getCommand(const TArgs... args)
  {
    return std::get<TWorkerType>(workers_).getCommand(args...);
  }

  /**
   * @brief
   * @param header
   * @return
   */
  inline bool validateCommandHeader(const std::array<char,2>& header) const
  {
    return validateCommandHeaderImplementation(header, workers_);
  }

  /**
   * @brief Call specific worker and get processed message
   *
   * @param[in] packet - Packet to process
   * return Packet decoded or std::nullopt if
   */
  template <typename TWorkerType, typename TReply = typename TWorkerType::Reply>
  std::optional<TReply> process(const TReply& packet)
  {
    return std::get<TWorkerType>(workers_).process(packet);
  }

  /**
   * @brief This method is used to process a message which can assume different
   * types
   *
   * @param[in] packet - Packet
   * @return
   */
  bool processByHandler(const protocol::ShapeShifterBuffer& packet, const ros::Time& wall_time);

  /**
   * @brief Call specific worker and result will be processed by the registered handler
   *
   * @param[in] f_ - Callback to execute
   * @return true if successfully processed, false otherwise
   */
  template <typename T>
  bool processByHandler(const typename T::Reply f_)
  {
    return std::get<T>(workers_).processByHandler(f_);
  }

  /**
   * @brief Register callback for the specific worker callback
   *
   * @param[in] f_ - Callback to execute
   */
  template <typename T>
  inline void registerCallback(typename T::PacketEventCallback f_)
  {
    std::get<T>(workers_).registerEventCallback(f_);
  }

private:
  /**
   * @brief Filter function to validate if we have a valid packet based on header
   * @param header
   * @param f_tuple
   * @return
   */
  template <typename... TWorkers>
  static inline bool validateCommandHeaderImplementation(const std::array<char,2>& header, std::tuple<TWorkers...> const& workers)
  {
    // Verify packet header
    return std::apply(
      [&header](TWorkers const&... worker) -> bool { return ((worker.validateCommandHeader(header)) || ...); },
      workers);
  }

  /**
   * @brief UAM tupple of packet workers
   */
  std::tuple<
    uam::AR00Worker,
    uam::AR01Worker,
    uam::AR02Worker,
    uam::AR03Worker,
    uam::AR04Worker,
    uam::AR05Worker,
    uam::AR06Worker,
    uam::AR07Worker,
    uam::AR08Worker,
    uam::VR00Worker,
    uam::XR00Worker,
	uam::YRWorker>
    workers_;
};
}  // namespace uam

#endif  // INCLUDE_URG_NODE_UAM_UAM_COMMAND_WORKERS_H_
