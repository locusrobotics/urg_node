/**
Software License Agreement (proprietary)
\file      smart_buffer.h
\authors   Carlos Mendes <cribeiromendes@locusrobotics.com>
\copyright Copyright (c) (2023,), Locus Robotics Corp., All rights reserved.
Unauthorized copying of this file, via any medium, is strictly prohibited.
Proprietary and confidential.
**/

#ifndef INCLUDE_URG_NODE_UAM_SHAPE_SHIFTER_BUFFER_H_
#define INCLUDE_URG_NODE_UAM_SHAPE_SHIFTER_BUFFER_H_

#include <ros/console.h>
#include <uam/protocol_types/uam_protocol_types.h>

#include <array>
#include <optional>
#include <type_traits>
#include <typeinfo>
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
class ShapeShifterBuffer
{
  template <typename T>
  static constexpr bool isSupportedType()
  {
    return (
      std::is_same<AR00CommandReply, T>::value || std::is_same<AR01CommandReply, T>::value ||
      std::is_same<EmptyCommandReply, T>::value || std::is_same<AR06CommandReply, T>::value ||
      std::is_same<XR00CommandReply, T>::value || std::is_same<VR00CommandReply, T>::value ||
      std::is_same<YRCommandReply, T>::value || std::is_same<YRCommandReplyHeader, T>::value ||
      std::is_same<CommandReplyHeader, T>::value);
  }
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
    YRCommandReplyHeader yr_reply_header;
    std::array<char, sizeof(AR01CommandReply)> raw_buffer;
  } buffer;

  /**
   * @brief Validate union size agains expected size
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
    static_assert(isSupportedType<T>(), "Invalid expected message type!");
    return getImplementation<T>();
  }

  template <typename T>
  inline void set(T message)
  {
    static_assert(isSupportedType<T>(), "Invalid expected message type!");
    getImplementationRef<T>() = message;
  }

private:
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
inline const AR00CommandReply& ShapeShifterBuffer::getImplementation() const
{
  return buffer.ar00_reply;
}

template <>
inline const AR01CommandReply& ShapeShifterBuffer::getImplementation() const
{
  return buffer.ar01_reply;
}

template <>
inline const EmptyCommandReply& ShapeShifterBuffer::getImplementation() const
{
  return buffer.empty_reply;
}

template <>
inline const AR06CommandReply& ShapeShifterBuffer::getImplementation() const
{
  return buffer.ar06_reply;
}

template <>
inline const VR00CommandReply& ShapeShifterBuffer::getImplementation() const
{
  return buffer.vr00_reply;
}

template <>
inline const XR00CommandReply& ShapeShifterBuffer::getImplementation() const
{
  return buffer.xr00_reply;
}

template <>
inline const YRCommandReply& ShapeShifterBuffer::getImplementation() const
{
  return buffer.yr_reply;
}

template <>
inline const YRCommandReplyHeader& ShapeShifterBuffer::getImplementation() const
{
  return buffer.yr_reply_header;
}
template <>
inline const CommandReplyHeader& ShapeShifterBuffer::getImplementation() const
{
  return buffer.header;
}

template <>
inline AR00CommandReply& ShapeShifterBuffer::getImplementationRef()
{
  return buffer.ar00_reply;
}

template <>
inline AR01CommandReply& ShapeShifterBuffer::getImplementationRef()
{
  return buffer.ar01_reply;
}

template <>
inline EmptyCommandReply& ShapeShifterBuffer::getImplementationRef()
{
  return buffer.empty_reply;
}

template <>
inline AR06CommandReply& ShapeShifterBuffer::getImplementationRef()
{
  return buffer.ar06_reply;
}

template <>
inline VR00CommandReply& ShapeShifterBuffer::getImplementationRef()
{
  return buffer.vr00_reply;
}

template <>
inline XR00CommandReply& ShapeShifterBuffer::getImplementationRef()
{
  return buffer.xr00_reply;
}

template <>
inline YRCommandReply& ShapeShifterBuffer::getImplementationRef()
{
  return buffer.yr_reply;
}

template <>
inline YRCommandReplyHeader& ShapeShifterBuffer::getImplementationRef()
{
  return buffer.yr_reply_header;
}
template <>
inline CommandReplyHeader& ShapeShifterBuffer::getImplementationRef()
{
  return buffer.header;
}

}  // namespace protocol
}  // namespace uam

#endif  // INCLUDE_URG_NODE_UAM_SHAPE_SHIFTER_BUFFER_H_
