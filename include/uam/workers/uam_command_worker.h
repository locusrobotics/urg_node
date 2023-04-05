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
   */
  template <typename T>
  auto process(const typename T::Reply packet)
  {
    return std::get<T>(workers_).process(packet);
  }

  /**
   * @brief Call specific worker and result will be processed by the registered handler
   *
   * @param[in] f_ - Callback to execute
   */
  template <typename T>
  auto processByHandler(const typename T::Reply f_)
  {
    return std::get<T>(workers_).processByHandler(f_);
  }

  /**
   * @brief Register callback for the specific worker callback
   *
   * @param[in] f_ - Callback to execute
   */
  template <typename T>
  inline void registerCallback(typename T::PacketEventCallback&& f_)
  {    
    std::get<T>(workers_).registerEventCallback(std::forward<typename T::PacketEventCallback>(f_));
  }

  /**
   *
   * @tparam T
   * @return
   */
  template <typename T>
  std::string getCommand() const
  {
    return std::get<T>(workers_).getCommand();
  }

  /**
   * @brief This method is used to process
   * messages that were subscribed in continuous mode (AR02, AR04, AR07)
   *
   * @param[in] packet - Packet
   * @return
   */
  bool subscribeCallback(const protocol::ShapeShifterBuffer& packet)
  {
    const auto packet_type = packet.getPacketHeader();
    bool valid_packet = false;
    bool processed_successfully = false;

    if (std::get<AR02Worker>(workers_).validateReplyType(packet.getPacketHeader()))
    {
      valid_packet = true;
      processed_successfully = std::get<AR00Worker>(workers_).processByHandler(packet.get<AR00Worker::Reply>());
    }
    else if (std::get<AR04Worker>(workers_).validateReplyType(packet.getPacketHeader()))
    {
      valid_packet = true;
      processed_successfully = std::get<AR01Worker>(workers_).processByHandler(packet.get<AR01Worker::Reply>());
    }
    else if (std::get<AR07Worker>(workers_).validateReplyType(packet.getPacketHeader()))
    {
      valid_packet = true;
      processed_successfully = std::get<AR06Worker>(workers_).processByHandler(packet.get<AR06Worker::Reply>());
    }

    ROS_ERROR_STREAM_COND(
      !valid_packet,
      "Invalid packet type with: " << packet_type.header[0] << packet_type.header[1] << packet_type.sub_header[0]
                                   << packet_type.sub_header[1]);
    ROS_ERROR_STREAM_COND(!processed_successfully, "Processed fail for packet");
    return processed_successfully;
  }

private:
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
    uam::XR00Worker>
    workers_;
};
}  // namespace uam

#endif  // INCLUDE_URG_NODE_UAM_UAM_COMMAND_WORKERS_H_
