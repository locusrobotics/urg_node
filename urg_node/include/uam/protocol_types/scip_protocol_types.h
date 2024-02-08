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

#ifndef UAM_PROTOCOL_TYPES_SCIP_PROTOCOL_TYPES_H
#define UAM_PROTOCOL_TYPES_SCIP_PROTOCOL_TYPES_H

#include <string>

namespace uam
{
namespace scip_protocol
{
/**
 * @brief PP Reply index enum
 */
enum PPReplyLineIndex : size_t
{
  ACK = 0, /**< ACK */
  STATUS, /**< STATUS */
  SENSOR_MODEL, /**< SENSOR_MODEL */
  MIN_DISTANCE, /**< MIN_DISTANCE */
  MAX_DISTANCE, /**< MAX_DISTANCE */
  ANGULAR_RESOLUTION, /**< ANGULAR_RESOLUTION */
  STARTING_STEP, /**< STARTING_STEP */
  END_STEP, /**< END_STEP */
  STEP_FRONT_DIRECTION, /**< STEP_FRONT_DIRECTION */
  RPM, /**< RPM */
  NR_LINES /**< NR_LINES */
};

/**
 * @brief Structure representing the PP
 */
struct PPReply
{
  int min_distance;
  int max_distance;
  int angular_resolution;
  int first_step;
  int last_step;
  int front_data_step;
  int rpm;
};

}  // namespace scip_protocol
}  // namespace uam

#endif  // UAM_PROTOCOL_TYPES_SCIP_PROTOCOL_TYPES_H
