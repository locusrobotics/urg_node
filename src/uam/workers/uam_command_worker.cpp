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

#include <uam/workers/uam_command_worker.h>

#include <tuple>

namespace uam
{

bool UamPacketWorker::processByHandler(const protocol::ShapeShifterPacket& packet, const ros::Time& wall_time)
{
  // Verify packet header
  const auto packet_type = packet.get<protocol::CommandReplyHeader>();
  auto received_packet_size = packet_type.cmd_size;
  decodeField(received_packet_size);
  bool valid_packet = false;
  bool processed_successfully = false;

  // todo(cribeiromendes): Replace this with something else,
  bool is_ar00_packet = std::get<AR00Worker>(workers_).validateReplyType(packet_type);
  bool is_ar01_packet = (!is_ar00_packet && std::get<AR01Worker>(workers_).validateReplyType(packet_type));
  bool is_ar02_packet = (!is_ar01_packet && std::get<AR02Worker>(workers_).validateReplyType(packet_type));
  bool is_ar03_packet = (!is_ar02_packet && std::get<AR03Worker>(workers_).validateReplyType(packet_type));
  bool is_ar04_packet = (!is_ar03_packet && std::get<AR04Worker>(workers_).validateReplyType(packet_type));
  bool is_ar05_packet = (!is_ar04_packet && std::get<AR05Worker>(workers_).validateReplyType(packet_type));
  bool is_ar06_packet = (!is_ar05_packet && std::get<AR06Worker>(workers_).validateReplyType(packet_type));
  bool is_ar07_packet = (!is_ar06_packet && std::get<AR07Worker>(workers_).validateReplyType(packet_type));
  bool is_ar08_packet = (!is_ar07_packet && std::get<AR08Worker>(workers_).validateReplyType(packet_type));
  bool is_vr00_packet = (!is_ar08_packet && std::get<VR00Worker>(workers_).validateReplyType(packet_type));
  bool is_xr00_packet = (!is_vr00_packet && std::get<XR00Worker>(workers_).validateReplyType(packet_type));
  bool is_yr_packet =
    (!is_xr00_packet &&
     std::get<YRWorker>(workers_).validateReplyType(packet.get<protocol::YRCommandReplyHeader>()));

  // Subscription commands can have 2 different reply types:
  // - Empty reply flagging if subscription was successful
  // - AR00 for AR02
  // - AR06 for AR07
  if (is_ar00_packet || (is_ar02_packet && received_packet_size == sizeof(AR00Worker::Reply)))
  {
    valid_packet = true;
    processed_successfully = std::get<AR00Worker>(workers_).processByHandler(packet.get<AR00Worker::Reply>(), wall_time);
  }
  else if (is_ar01_packet || (is_ar04_packet && received_packet_size == sizeof(AR01Worker::Reply)))
  {
    processed_successfully = std::get<AR01Worker>(workers_).processByHandler(packet.get<AR01Worker::Reply>(), wall_time);
  }
  else if (is_ar06_packet || (is_ar07_packet && received_packet_size == sizeof(AR06Worker::Reply)))
  {
    processed_successfully = std::get<AR06Worker>(workers_).processByHandler(packet.get<AR06Worker::Reply>(), wall_time);
  }
  else if (is_ar02_packet)
  {
    processed_successfully = std::get<AR02Worker>(workers_).processByHandler(packet.get<AR02Worker::Reply>(), wall_time);
  }
  else if (is_ar03_packet)
  {
    processed_successfully = std::get<AR03Worker>(workers_).processByHandler(packet.get<AR03Worker::Reply>(), wall_time);
  }
  else if (is_ar04_packet)
  {
    processed_successfully = std::get<AR04Worker>(workers_).processByHandler(packet.get<AR04Worker::Reply>(), wall_time);
  }
  else if (is_ar05_packet)
  {
    processed_successfully = std::get<AR05Worker>(workers_).processByHandler(packet.get<AR05Worker::Reply>(), wall_time);
  }
  else if (is_ar07_packet)
  {
    processed_successfully = std::get<AR07Worker>(workers_).processByHandler(packet.get<AR07Worker::Reply>(), wall_time);
  }
  else if (is_ar08_packet)
  {
    processed_successfully = std::get<AR08Worker>(workers_).processByHandler(packet.get<AR08Worker::Reply>(), wall_time);
  }
  else if (is_vr00_packet)
  {
    processed_successfully = std::get<VR00Worker>(workers_).processByHandler(packet.get<VR00Worker::Reply>(), wall_time);
  }
  else if (is_xr00_packet)
  {
    processed_successfully = std::get<XR00Worker>(workers_).processByHandler(packet.get<XR00Worker::Reply>(), wall_time);
  }
  else if (is_yr_packet)
  {
    processed_successfully =
      std::get<XR00Worker>(workers_).processByHandler(packet.get<XR00Worker::Reply>(), wall_time);
  }
  else
  {
    ROS_ERROR_STREAM(
      "Unknown packet type with: " << packet_type.header[0] << packet_type.header[1]);
  }
  return processed_successfully;
}
}  // namespace uam
