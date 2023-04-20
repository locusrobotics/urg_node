/**
Software License Agreement (proprietary)
\file      uam_command_worker.cpp
\authors   Carlos Mendes <cribeiromendes@locusrobotics.com>
\copyright Copyright (c) (2023,), Locus Robotics Corp., All rights reserved.
Unauthorized copying of this file, via any medium, is strictly prohibited.
Proprietary and confidential.
**/

#include <uam/workers/uam_command_worker.h>

namespace uam
{
bool UamPacketWorker::processByHandler(const protocol::ShapeShifterBuffer& packet)
{
  const auto packet_type = packet.getPacketHeader();
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

  // Subscription commands can have 2 different reply types:
  // - Empty reply flagging if subscription was successful
  // - AR00 for AR02
  // - AR01 for AR04
  // - AR06 for AR07
  if (is_ar00_packet || (is_ar02_packet && received_packet_size == sizeof(AR00Worker::Reply)))
  {
    valid_packet = true;
    processed_successfully = std::get<AR00Worker>(workers_).processByHandler(packet.get<AR00Worker::Reply>());
  }
  else if (is_ar01_packet || (is_ar04_packet && received_packet_size == sizeof(AR01Worker::Reply)))
  {
    processed_successfully = std::get<AR01Worker>(workers_).processByHandler(packet.get<AR01Worker::Reply>());
  }
  else if (is_ar06_packet || (is_ar07_packet && received_packet_size == sizeof(AR06Worker::Reply)))
  {
    processed_successfully = std::get<AR06Worker>(workers_).processByHandler(packet.get<AR06Worker::Reply>());
  }
  else if (is_ar02_packet)
  {
    processed_successfully = std::get<AR02Worker>(workers_).processByHandler(packet.get<AR02Worker::Reply>());
  }
  else if (is_ar03_packet)
  {
    processed_successfully = std::get<AR03Worker>(workers_).processByHandler(packet.get<AR03Worker::Reply>());
  }
  else if (is_ar04_packet)
  {
    processed_successfully = std::get<AR04Worker>(workers_).processByHandler(packet.get<AR04Worker::Reply>());
  }
  else if (is_ar05_packet)
  {
    processed_successfully = std::get<AR05Worker>(workers_).processByHandler(packet.get<AR05Worker::Reply>());
  }
  else if (is_ar07_packet)
  {
    processed_successfully = std::get<AR07Worker>(workers_).processByHandler(packet.get<AR07Worker::Reply>());
  }
  else if (is_ar08_packet)
  {
    processed_successfully = std::get<AR08Worker>(workers_).processByHandler(packet.get<AR08Worker::Reply>());
  }
  else if (is_vr00_packet)
  {
    processed_successfully = std::get<VR00Worker>(workers_).processByHandler(packet.get<VR00Worker::Reply>());
  }
  else if (is_xr00_packet)
  {
    processed_successfully = std::get<XR00Worker>(workers_).processByHandler(packet.get<XR00Worker::Reply>());
  }
  else
  {
    ROS_ERROR_STREAM(
      "Invalid packet type with: " << packet_type.header[0] << packet_type.header[1] << packet_type.sub_header[0]
                                   << packet_type.sub_header[1]);
  }
  return processed_successfully;
}
}  // namespace uam
