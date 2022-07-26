/*
 * accessor.h
 *
 *  Created on: 26/07/2022
 *      Author: cribeiromendes
 */

#ifndef INCLUDE_URG_NODE_ACCESSOR_H_
#define INCLUDE_URG_NODE_ACCESSOR_H_

#include <sstream>
#include <string>

/**
 * @brief Pattern taken from
 * https://github.com/ros-drivers/um7/blob/466e6e05a5bba0cd0e2bd20c8fbb7101fc71e872/include/um7/registers.h
 */

/**
 * @brief This class provides an accessor of fields.
 */
class Accessor_
{
public:
  /**
   * @brief Default C'tor
   * @param buffer buffer where fields are accessed
   * @param field_index Index of the field in the buffer
   * @param field_width Size of the field in the buffer
   */
  Accessor_(const std::string* buffer, uint8_t field_index, uint8_t field_width) :
    index(field_index),
    width(field_width),
    buffer_(buffer)
  {
  }

  /**
   * @brief Index of the field in the buffer
   */
  const uint8_t index;

  /**
   * @brief Size of the field in the buffer in bytes
   */
  const uint8_t width;

protected:
  /**
   * @brief Pointer to buffer
   */
  const std::string* buffer_;
};

/**
 * @brief Accessor for the field of type TField
 * @tparam TField
 */
template <typename TField, size_t TIndex>
class Accessor : public Accessor_
{
public:
  /**
   * @brief Default C'tor
   * @param buffer buffer where fields are accessed
   * @param field_index Index of the field in the buffer
   * @param offset Custom offset that needs to be applied to the field
   */
  Accessor(const std::string* buffer, const uint8_t idx_offset = 0, const uint8_t param_offset = 0) :
    Accessor_(buffer, TIndex + idx_offset, sizeof(TField)),
    offset_(param_offset)
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
   * @brief Custom offset that need to be added into the field
   */
  uint8_t offset_ { 0 };
};

#define ACCESSOR_MEMBER(struct_name, field) Accessor<decltype(struct_name::field), offsetof(struct_name, field)> field
#endif /* INCLUDE_URG_NODE_ACCESSOR_H_ */
