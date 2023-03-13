/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2022, Locus Robotics
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

#ifndef INCLUDE_URG_NODE_VISITOR_H_
#define INCLUDE_URG_NODE_VISITOR_H_

#include <array>
#include <sstream>
#include <string>

template <class T>
struct is_array : std::is_array<T>
{
};
template <class T, std::size_t N>
struct is_array<std::array<T, N>> : std::true_type
{
};
template <typename TField>
typename std::enable_if<is_array<TField>::value>::type toFieldFromBuffer(
  const std::string* buffer,
  const size_t index,
  const uint32_t param_offset,
  const uint32_t width,
  TField& array)
{
  const auto step = sizeof(typename TField::value_type);
  for (size_t idx { 0 }; idx < array.size(); idx++)
  {
    std::stringstream ss;
    ss << buffer->substr(index + (step * idx), step);
    ss >> std::hex >> array[idx];
    array[idx] += param_offset;
  }
}

template <typename TField>
typename std::enable_if<!is_array<TField>::value>::type toFieldFromBuffer(
  const std::string* buffer,
  const size_t index,
  const uint32_t param_offset,
  const uint32_t width,
  TField& field)
{
  std::stringstream ss;
  ss << buffer->substr(index, width);
  ss >> std::hex >> field;
  field += param_offset;
}

/**
 * @brief A visitor class for the field of type TField
 * @tparam TField
 */
template <typename TField, size_t TIndex>
class Visitor
{
public:
  /**
   * @brief Default C'tor
   * @param buffer buffer where fields are accessed
   * @param field_index Index of the field in the buffer
   * @param offset Custom offset that needs to be applied to the field
   */
  Visitor(const uint32_t idx_offset = 0, const uint32_t param_offset = 0) :
    index_(TIndex + idx_offset),
    offset_(param_offset),
    width_(sizeof(TField))
  {
  }
  /**
   * @brief
   * @param field
   */
  void get(const std::string* buffer, TField& field) {
	  toFieldFromBuffer(buffer, this->index_, this->offset_, this->width_, field); }


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
 * @brief Usefull macro to declare an accessor.
 */
#define VISITOR_MEMBER(struct_name, field) Visitor<decltype(struct_name::field), offsetof(struct_name, field)> field

#endif /* INCLUDE_URG_NODE_VISITOR_H_ */
