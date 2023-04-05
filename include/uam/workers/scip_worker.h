/**
Software License Agreement (proprietary)
\file      vr_00_decoder.h
\authors   Carlos Mendes <cribeiromendes@locusrobotics.com>
\copyright Copyright (c) (2023,), Locus Robotics Corp., All rights reserved.
Unauthorized copying of this file, via any medium, is strictly prohibited.
Proprietary and confidential.
**/

#ifndef INCLUDE_URG_NODE_UAM_DECODER_PP_WORKER_H_
#define INCLUDE_URG_NODE_UAM_DECODER_PP_WORKER_H_

#include <uam/protocol_types/scip_protocol_types.h>
#include <uam/workers/uam_worker_base.h>

#include <optional>

namespace uam
{
template <typename TDerived, size_t NR_LINES, typename TReply>
class SCIPWorker
{
public:
  /**
   * @brief provide access to the request type
   */
  using Request = std::string;

  /**
   * @brief Make Reply type public
   */
  using Reply = TReply;

  /**
   * @brief Make Reply type public
   */
  using RawReply = std::array<std::string, NR_LINES>;

  /**
   * @brief Packet callback type
   */
  using PacketEventCallback = std::function<void(Reply)>;

  /**
   * @brief Return encoded request
   *
   * @return request message
   */
  inline const std::string& getCommand() const { return static_cast<const TDerived*>(this)->getCommand(); }

  /**
   * @brief Method to register the callback to be called by the derived class
   * when processing the packet.
   *
   * @param[in] callback - Callback to execute when finalising processImpl call
   */
  inline void registerEventCallback(PacketEventCallback callback) { callback_ = callback; }

  /**
   * @brief Process raw buffer and return decoded message
   *
   * @param[in] buffer - Raw buffer
   * @return Reply decoded message if successful, std::nullopt otherwise
   */
  std::optional<Reply> process(const RawReply& raw_reply) const
  {
    std::optional<Reply> reply = static_cast<const TDerived*>(this)->decode(raw_reply);
    if (!reply.has_value())
    {
      ROS_ERROR_STREAM("Failed to decode message");
      return std::nullopt;
    }
    return reply;
  }

protected:
  static std::string findSubstring(
    const std::string& input,
    const std::string& first = ":",
    const std::string& last = ";")
  {
    std::string result = "";
    size_t first_pos = input.find(first);

    first_pos = first_pos == std::string::npos ? first_pos : first_pos + 1;
    if (first_pos != std::string::npos)
    {
      auto last_pos = input.find(last, first_pos);
      if (last_pos < first_pos)
      {
        return result;
      }
      result = input.substr(first_pos, last_pos - first_pos);
    }
    return result;
  }

private:
  PacketEventCallback callback_;
};

/**
 * @brief VR00 command worker
 */
class PPWorker : public SCIPWorker<PPWorker, scip_protocol::PPReplyLineIndex::NR_LINES, scip_protocol::PPReply>
{
public:
  PPWorker() : SCIPWorker<PPWorker, scip_protocol::PPReplyLineIndex::NR_LINES, scip_protocol::PPReply>() {}

  std::optional<Reply> decode(const RawReply& raw_reply) const
  {
    if (raw_reply.size() != scip_protocol::PPReplyLineIndex::NR_LINES)
    {
      ROS_ERROR_STREAM(
        "Invalid PP response with only: " << raw_reply.size()
                                          << " line. Expected: " << scip_protocol::PPReplyLineIndex::NR_LINES);
      return std::nullopt;
    }
    Reply reply;

    for (auto& i : raw_reply)
    {
      std::cout << i << std::endl;
    }

    bool failed = !decodeField(raw_reply.at(scip_protocol::PPReplyLineIndex::MIN_DISTANCE), reply.min_distance);
    failed = failed || !decodeField(raw_reply.at(scip_protocol::PPReplyLineIndex::MAX_DISTANCE), reply.max_distance);
    failed = failed ||
             !decodeField(raw_reply.at(scip_protocol::PPReplyLineIndex::ANGULAR_RESOLUTION), reply.angular_resolution);
    failed = failed || !decodeField(raw_reply.at(scip_protocol::PPReplyLineIndex::STARTING_STEP), reply.start_step);
    failed = failed || !decodeField(raw_reply.at(scip_protocol::PPReplyLineIndex::END_STEP), reply.end_step);
    failed = failed ||
             !decodeField(raw_reply.at(scip_protocol::PPReplyLineIndex::STEP_FRONT_DIRECTION), reply.front_data_index);
    failed = failed || !decodeField(raw_reply.at(scip_protocol::PPReplyLineIndex::RPM), reply.rpm);
    if (failed)
    {
      ROS_ERROR_STREAM("Failed To decode PP reply params");
      return std::nullopt;
    }
    return reply;
  }

  inline const std::string getCommand() const { return "PP\n"; }

private:
  bool decodeField(const std::string& field, int& value) const
  {
    auto sub_str = findSubstring(field);
    if (!sub_str.empty())
    {
      value = std::stoi(sub_str, nullptr, 10);
      return true;
    }
    return false;
  }
  bool decodeField(const std::string& field, long& value)
  {
    auto sub_str = findSubstring(field);
    if (!sub_str.empty())
    {
      value = std::stol(sub_str, nullptr, 10);
      return true;
    }
    return false;
  }
};

}  // namespace uam

#endif  // INCLUDE_URG_NODE_UAM_DECODER_PP_WORKER_H_
