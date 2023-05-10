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

#ifndef UAM_TYPE_TRAITS_H
#define UAM_TYPE_TRAITS_H

#include <type_traits>

namespace uam
{
template <typename Tuple, size_t Index0>
constexpr size_t getMaxSize()
{
  return sizeof(std::tuple_element<Index0, Tuple>);
}

template <typename Tuple, size_t Index0, size_t Index1, size_t... TypeN>
constexpr size_t getMaxSize()
{
  return (
    sizeof(std::tuple_element<Index0, Tuple>) >= sizeof(std::tuple_element<Index1, Tuple>) ?
      getMaxSize<Tuple, Index0, TypeN...>() :
      getMaxSize<Tuple, Index1, TypeN...>());
}

template <typename Tuple, size_t... TypeN>
constexpr size_t getMaxSizeSequence(std::index_sequence<TypeN...>)
{
  return getMaxSize<Tuple, TypeN...>();
}

template <typename Tuple>
constexpr size_t getMaxSizeTuple()
{
  return getMaxSizeSequence<Tuple>(std::make_index_sequence<std::tuple_size<Tuple>::value>());
}

template <typename NewType, typename Tuple, size_t... IndexN>
constexpr bool containsSequence(std::index_sequence<IndexN...>)
{
  return (std::is_same<NewType, typename std::tuple_element<IndexN, Tuple>::type>::value || ...);
}

template <typename NewType, typename Tuple>
constexpr bool tupleContains()
{
  return containsSequence<NewType, Tuple>(std::make_index_sequence<std::tuple_size<Tuple>::value>());
}

}  // namespace uam

#endif  // UAM_TYPE_TRAITS_H
