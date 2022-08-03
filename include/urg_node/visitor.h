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

#include <sstream>
#include <string>

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
  Visitor(const std::string* buffer, const uint8_t idx_offset = 0, const uint8_t param_offset = 0) :
    buffer_(buffer),
    index(TIndex + idx_offset),
    offset_(param_offset),
    width(sizeof(TField))
  {
  }

  /**
   * @brief Method to retrieve field from buffer
   * @param field
   */
  void get(TField& field) const
  {
    std::stringstream ss;
    ss << buffer_->substr(this->index, this->width);
    ss >> std::hex >> field;
    field += offset_;
  }

private:
  /**
   * @brief Pointer to buffer
   */
  const std::string* buffer_;
  /**
   * @brief Index of the field in the buffer
   */
  const uint8_t index;
  /**
   * @brief Custom offset that need to be added into the field
   */
  const uint8_t offset_;
  /**
   * @brief Size of the field in the buffer in bytes
   */
  const uint8_t width;
};
/**
 * @brief Usefull macro to declare an accessor.
 */
#define VISITOR_MEMBER(struct_name, field) Visitor<decltype(struct_name::field), offsetof(struct_name, field)> field

#endif /* INCLUDE_URG_NODE_VISITOR_H_ */
