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

#ifndef UAM_WORKERS_SCIP_WORKER_H
#define UAM_WORKERS_SCIP_WORKER_H

#include <uam/protocol_types/scip_protocol_types.h>
#include <uam/workers/uam_worker_base.h>

#include <optional>
#include <string_view>

namespace uam
{
/**
 * @brief This template class for SCIP workers. To decode each individual reply, each
 * child will have to implement custom decode methods.
 *
 * @tparam TDerived - CRTP Pattern to allow for static dispatch whenever possible.
 * @tparam NR_LINES - Number of lines of the reply
 * @tparam TReply - The reply type for the child worker
 */
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
    if (raw_reply.size() != NR_LINES)
    {
      ROS_ERROR_STREAM(
        "Invalid response with only: " << raw_reply.size()
                                          << " line. Expected: " << NR_LINES);
      return std::nullopt;
    }

    std::optional<Reply> reply = static_cast<const TDerived*>(this)->decode(raw_reply);
    if (!reply.has_value())
    {
      ROS_ERROR_STREAM("Failed to decode message");
      return std::nullopt;
    }
    return reply;
  }

protected:
  /**
   * @brief Find a substring in between two delimiters
   *
   * @param[in] input - Complete string
   * @param[in] first - First delimiter
   * @param[in] last - Last delimiter
   * @return
   */
  static std::string findSubstring(
    const std::string_view& input,
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
  /**
   * @brief The packet callback
   */
  PacketEventCallback callback_;
};

/**
 * @brief PP command worker
 */
class PPWorker : public SCIPWorker<PPWorker, scip_protocol::PPReplyLineIndex::NR_LINES, scip_protocol::PPReply>
{
public:
  /**
   * @brief Default C'tor
   */
  PPWorker() : SCIPWorker<PPWorker, scip_protocol::PPReplyLineIndex::NR_LINES, scip_protocol::PPReply>() {}

  /**
   * @brief Decode raw reply into Reply
   * @param[in] raw_reply - Raw reply (array of strings)
   * @return[out] Decoded reply if success, std::nullopt otherwise
   */
  std::optional<Reply> decode(const RawReply& raw_reply) const
  {
    Reply reply;
    bool failed = !decodeField(raw_reply.at(scip_protocol::PPReplyLineIndex::MIN_DISTANCE), reply.min_distance);
    failed = failed || !decodeField(raw_reply.at(scip_protocol::PPReplyLineIndex::MAX_DISTANCE), reply.max_distance);
    failed = failed ||
             !decodeField(raw_reply.at(scip_protocol::PPReplyLineIndex::ANGULAR_RESOLUTION), reply.angular_resolution);
    failed = failed || !decodeField(raw_reply.at(scip_protocol::PPReplyLineIndex::STARTING_STEP), reply.first_step);
    failed = failed || !decodeField(raw_reply.at(scip_protocol::PPReplyLineIndex::END_STEP), reply.last_step);
    failed = failed ||
             !decodeField(raw_reply.at(scip_protocol::PPReplyLineIndex::STEP_FRONT_DIRECTION), reply.front_data_step);
    failed = failed || !decodeField(raw_reply.at(scip_protocol::PPReplyLineIndex::RPM), reply.rpm);
    if (failed)
    {
      ROS_ERROR_STREAM("Failed To decode PP reply params");
      return std::nullopt;
    }
    return reply;
  }

  /**
   * @brief Retrieve the command
   *
   * @return The scip PP command
   */
  inline const std::string getCommand() const { return "PP\n"; }

private:
  /**
   * @brief Transform string into an integer value
   *
   * @param[in] field - Input string
   * @param[out] value - Output value
   * @return true if transformation was successful, false otherwise
   */
  bool decodeField(const std::string_view& field, int& value) const
  {
    auto sub_str = findSubstring(field);
    if (!sub_str.empty())
    {
      value = std::stoi(sub_str, nullptr, 10);
      return true;
    }
    return false;
  }
};

}  // namespace uam

#endif  // UAM_WORKERS_SCIP_WORKER_H
