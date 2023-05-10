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

#ifndef UAM_WORKERS_UAM_COMMAND_WORKER_H
#define UAM_WORKERS_UAM_COMMAND_WORKER_H

// sensing data workers
#include <uam/shape_shifter_packet.h>
#include <uam/workers/empty_reply_worker.h>
#include <uam/workers/sensing_data/ar00_worker.h>
#include <uam/workers/sensing_data/ar01_worker.h>
#include <uam/workers/sensing_data/ar06_worker.h>
#include <uam/workers/vr00_worker.h>
#include <uam/workers/xr00_worker.h>
#include <uam/workers/yr_worker.h>

#include <string>
#include <tuple>
#include <type_traits>
#include <optional>

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
   * return Packet decoded or std::nullopt if decoding failed
   */
  template <typename TWorkerType, typename... TArgs>
  std::string getCommand(const TArgs... args)
  {
    return std::get<TWorkerType>(workers_).getCommand(args...);
  }

  /**
   * @brief Validate if the command header against known packets
   *
   * @param[in] header - Incoming header
   * @return true if the command is recognised by a packet worker, false otherwise
   */
  inline bool validateCommandHeader(const std::array<char, 2>& header) const
  {
    return validateCommandHeaderImplementation(header, workers_);
  }

  /**
   * @brief Call specific worker and get processed message
   *
   * @param[in] packet - Packet to process
   * @return Packet decoded or std::nullopt if decoding failed
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
   * @param[in] packet - The shape shifter packet
   * @param[in] wall_time - Wall time (the moment when the packet header was captured)
   * @return true if packet was successfully processed, false otherwise
   */
  bool processByHandler(const protocol::ShapeShifterPacket& packet, const ros::Time& wall_time);

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
   * @param[in] header - Incoming packet header
   * @param[in] f_tuple - Tuple of workers
   * @return true if a worker validated the header, false otherwise
   */
  template <typename... TWorkers>
  static inline bool validateCommandHeaderImplementation(
    const std::array<char, 2>& header,
    std::tuple<TWorkers...> const& workers)
  {
    // Verify packet header
    return std::apply(
      [&header](TWorkers const&... worker) -> bool { return ((worker.validateCommandHeader(header)) || ...); },
      workers);
  }

  /**
   * @brief UAM tuple of packet workers
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

#endif  // UAM_WORKERS_UAM_COMMAND_WORKER_H
