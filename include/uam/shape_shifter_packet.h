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

#ifndef UAM_SHAPE_SHIFTER_PACKET_H
#define UAM_SHAPE_SHIFTER_PACKET_H

#include <ros/console.h>
#include <uam/protocol_types/uam_protocol_types.h>
#include <uam/type_traits.h>

#include <array>
#include <tuple>
#include <utility>
#include <variant>

namespace uam
{
namespace protocol
{
/**
 * @brief Smart Buffer type
 *
 * TODO(crbeiromendes): how about using a templated class that can use a variadic union inside and receive all the
 * types that the buffer contain
 */
class ShapeShifterPacket
{
  /**
   * @brief Supported Type Alias
   * TODO(cribeirmendes): once we have a variadic union, this can be removed
   */
  using SupportedTypes = std::tuple<
    CommandReplyHeader,
    AR00CommandReply,
    AR01CommandReply,
    EmptyCommandReply,
    AR06CommandReply,
    XR00CommandReply,
    VR00CommandReply,
    YRCommandReply>;

  /**
   * @brief Buffer type
   *
   * TODO(cribeiromendes): replace this with a variadic union or std::variant
   */
  union UBufferType
  {
    CommandReplyHeader header;
    AR00CommandReply ar00_reply;
    AR01CommandReply ar01_reply;
    EmptyCommandReply empty_reply;
    AR06CommandReply ar06_reply;
    XR00CommandReply xr00_reply;
    VR00CommandReply vr00_reply;
    YRCommandReply yr_reply;
    std::array<char, getMaxSizeTuple<SupportedTypes>()> raw_buffer;
  } buffer;  // NOLINT

  /**
   * @brief Validate union size against expected size
   */
  static_assert(sizeof(AR01CommandReply) == sizeof(UBufferType), "Invalid Union Size!");

public:
  /**
   * @brief Retrieve raw message buffer but stating which message type will be written into the buffer
   *
   * Nothing is preventing to call this with a different type just
   * to change the tag. But this is just an extra guard.
   * @return raw byte array
   */
  auto& getRawPacket() { return buffer.raw_buffer; }

  /**
   * @brief Retrieve
   *
   * @return
   */
  template <typename T>
  inline const T& get() const
  {
    // Easier to understand why compilation failed with this
    static_assert(tupleContains<T, SupportedTypes>(), "Invalid expected message type!");
    return getImplementation<T>();
  }

  /**
   * @brief Write directly a packet
   * @param[in] message - Message to write
   */
  template <typename T>
  inline void set(T message)
  {
    static_assert(tupleContains<T, SupportedTypes>(), "Invalid expected message type!");
    getImplementationRef<T>() = message;
  }

private:
  /**
   * @brief Get Packet ref in the buffer
   * @return reference for the message in the buffer
   */
  template <typename T>
  inline T& getImplementationRef();
  /**
   * @brief Retrieve const reference for the message entry in the buffer
   *
   * @return const reference for the message in the buffer
   */
  template <typename T>
  inline const T& getImplementation() const;
};

template <>
inline const AR00CommandReply& ShapeShifterPacket::getImplementation() const
{
  return buffer.ar00_reply;
}

template <>
inline const AR01CommandReply& ShapeShifterPacket::getImplementation() const
{
  return buffer.ar01_reply;
}

template <>
inline const EmptyCommandReply& ShapeShifterPacket::getImplementation() const
{
  return buffer.empty_reply;
}

template <>
inline const AR06CommandReply& ShapeShifterPacket::getImplementation() const
{
  return buffer.ar06_reply;
}

template <>
inline const VR00CommandReply& ShapeShifterPacket::getImplementation() const
{
  return buffer.vr00_reply;
}

template <>
inline const XR00CommandReply& ShapeShifterPacket::getImplementation() const
{
  return buffer.xr00_reply;
}

template <>
inline const YRCommandReply& ShapeShifterPacket::getImplementation() const
{
  return buffer.yr_reply;
}

template <>
inline const CommandReplyHeader& ShapeShifterPacket::getImplementation() const
{
  return buffer.header;
}

template <>
inline AR00CommandReply& ShapeShifterPacket::getImplementationRef()
{
  return buffer.ar00_reply;
}

template <>
inline AR01CommandReply& ShapeShifterPacket::getImplementationRef()
{
  return buffer.ar01_reply;
}

template <>
inline EmptyCommandReply& ShapeShifterPacket::getImplementationRef()
{
  return buffer.empty_reply;
}

template <>
inline AR06CommandReply& ShapeShifterPacket::getImplementationRef()
{
  return buffer.ar06_reply;
}

template <>
inline VR00CommandReply& ShapeShifterPacket::getImplementationRef()
{
  return buffer.vr00_reply;
}

template <>
inline XR00CommandReply& ShapeShifterPacket::getImplementationRef()
{
  return buffer.xr00_reply;
}

template <>
inline YRCommandReply& ShapeShifterPacket::getImplementationRef()
{
  return buffer.yr_reply;
}

template <>
inline CommandReplyHeader& ShapeShifterPacket::getImplementationRef()
{
  return buffer.header;
}

}  // namespace protocol
}  // namespace uam

#endif  // UAM_SHAPE_SHIFTER_PACKET_H
