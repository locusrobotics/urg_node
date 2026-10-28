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

#ifndef UAM_VISITOR_H
#define UAM_VISITOR_H

#include <array>
#include <charconv>
#include <iostream>
#include <sstream>
#include <string_view>
namespace uam
{
template <class T>
struct is_array : std::is_array<T>
{
};
template <class T, std::size_t N>
struct is_array<std::array<T, N>> : std::true_type
{
};

template <typename Tp>
inline bool getFromChars(const char* first, const char* last, Tp& value, int base = 16)
{
  if constexpr (std::is_same<bool, Tp>())
  {
    // From bytes does not accept bool. We need a cast here
    char tmp_value;
    if (std::from_chars(first, last, tmp_value, 16).ec != std::errc {})
    {
      return false;
    }
    value = static_cast<bool>(tmp_value);
    return true;
  }
  else
  {
    return std::from_chars(first, last, value, 16).ec == std::errc {};
  }
}

template <typename TField>
inline bool getFromChars(TField& field)
{
  auto char_ptr = reinterpret_cast<const char*>(&field);
  return getFromChars(char_ptr, char_ptr + sizeof(TField), field, 16);
}

template <typename TField>
bool decodeField(
  const std::string_view& buffer,
  const size_t index,
  const uint32_t param_offset,
  const uint32_t width,
  TField& field)
{
  if constexpr (is_array<TField>::value)
  {
    const auto step = sizeof(typename TField::value_type);
    const auto buffer_size = buffer.size();
    for (size_t idx { 0 }; idx < field.size(); idx++)
    {
      if (buffer_size <= (index + (step * idx) + width))
      {
        return false;
      }
      const char* char_ptr = &buffer.at(index + (step * idx));
      if (!getFromChars(char_ptr, char_ptr + width, field[idx], 16))
        return false;
      field[idx] += param_offset;
    }
  }
  else
  {
    if (buffer.size() <= (index + width))
    {
      return false;
    }

    const char* char_ptr = &buffer.at(index);
    if (!getFromChars(char_ptr, char_ptr + width, field, 16))
    {
      return false;
    }
    field += param_offset;
  }
  return true;
}

template <typename TField>
bool decodeField(TField& field)
{
  if constexpr (is_array<TField>::value)
  {
    for (auto& element : field)
    {
      if (!getFromChars(element))
        return false;
    }
  }
  else
  {
    return getFromChars(field);
  }
  return true;
}

/**
 * @brief A visitor class for the field of type TField
 * @tparam TField
 */
template <typename TField, size_t TIndex>
class TVisitor
{
public:
  /**
   * @brief Default C'tor
   * @param buffer buffer where fields are accessed
   * @param field_index Index of the field in the buffer
   * @param offset Custom offset that needs to be applied to the field
   */
  explicit TVisitor(const size_t idx_offset, const uint32_t param_offset = 0) :
    index_(TIndex + idx_offset),
    offset_(param_offset),
    width_(sizeof(TField))
  {
  }
  /**
   * @brief Default c'tor,
   */
  TVisitor() = default;

  /**
   * @brief Get the field in the buffer
   *
   * @param[in] buffer - Buffer
   * @param[in] field - Field to decode
   */
  bool get(const std::string_view& buffer, TField& field) const
  {
    return decodeField(buffer, this->index_, this->offset_, this->width_, field);
  }

  /**
   * @brief Get raw value in the buffer into the struct
   *
   * @param[in] buffer - Buffer
   * @param[in] field - Field
   */
  bool getRaw(const std::string_view& buffer, TField& field) const
  {
    if constexpr (is_array<TField>::value)
    {
      const auto step = sizeof(typename TField::value_type);
      for (size_t idx { 0 }; idx < field.size(); idx++)
      {
        std::stringstream ss;
        ss << buffer.substr(this->index_ + (step * idx), step);
        ss >> field[idx];
      }
    }
    else
    {
      std::stringstream ss;
      ss << buffer.substr(this->index_, this->width_);
      ss >> field;
    }
    return true;
  }

private:
  /**
   * @brief Index of the field in the buffer
   */
  const uint32_t index_;
  /**
   * @brief Custom offset that need to be added into the field
   */
  const uint32_t offset_;
  /**
   * @brief Size of the field in the buffer in bytes
   */
  const uint32_t width_;
};

/**
 * @brief Useful macro to declare an accessor.
 */
#ifndef VISITOR_MEMBER
#define VISITOR_MEMBER(struct_name, field) TVisitor<decltype(struct_name::field), offsetof(struct_name, field)> field
#endif

}  // namespace uam
#endif  // UAM_VISITOR_H
