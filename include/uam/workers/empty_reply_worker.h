/**
Software License Agreement (proprietary)
\file      empty_reply_worker.h
\authors   Carlos Mendes <cribeiromendes@locusrobotics.com>
\copyright Copyright (c) (2023,), Locus Robotics Corp., All rights reserved.
Unauthorized copying of this file, via any medium, is strictly prohibited.
Proprietary and confidential.
**/

#ifndef INCLUDE_URG_NODE_UAM_WORKERS_EMPTY_REPLY_WORKER_H_
#define INCLUDE_URG_NODE_UAM_WORKERS_EMPTY_REPLY_WORKER_H_

#include <uam/workers/uam_worker_base.h>

#include <optional>

namespace uam
{
template <char HeaderMSB, char HeaderLSB, char SubHeaderMSB, char SubHeaderLSB>
class EmptyReplyWorker :
  public WorkerBase<
    EmptyReplyWorker<HeaderMSB, HeaderLSB, SubHeaderMSB, SubHeaderLSB>,
    HeaderMSB,
    HeaderLSB,
    SubHeaderMSB,
    SubHeaderLSB,
    protocol::EmptyCommandReply>
{
public:
  /**
   * @brief Default C'tor
   */
  explicit EmptyReplyWorker(const uint32_t idx_offset = 0) :
    WorkerBase<
      EmptyReplyWorker<HeaderMSB, HeaderLSB, SubHeaderMSB, SubHeaderLSB>,
      HeaderMSB,
      HeaderLSB,
      SubHeaderMSB,
      SubHeaderLSB,
      protocol::EmptyCommandReply>(idx_offset, idx_offset + offsetof(protocol::EmptyCommandReply, footer))
  {
  }

  std::optional<protocol::EmptyCommandReply> decode(const std::string* buffer) const
  {
    protocol::EmptyCommandReply reply;
    this->decodeHeaderAndFooter(buffer, reply);
    if (!this->validateCrc(buffer, reply))
    {
      return std::nullopt;
    }
    return reply;
  }

  std::optional<protocol::EmptyCommandReply> decode(const protocol::EmptyCommandReply& raw_reply) const
  {
    protocol::EmptyCommandReply reply = raw_reply;
    this->decodeHeaderAndFooter(reply);
    if (!this->validateCrc(reply))
    {
      return std::nullopt;
    }
    return reply;
  }

  inline const bool validateSize(const size_t recv_bytes) const { return recv_bytes == sizeof(protocol::EmptyCommandReply); }
};
}  // namespace uam

#endif  // INCLUDE_URG_NODE_UAM_WORKERS_EMPTY_REPLY_WORKER_H_
