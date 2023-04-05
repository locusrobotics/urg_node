/**
Software License Agreement (proprietary)
\file      scip_protocol_types.h
\authors   Carlos Mendes <cribeiromendes@locusrobotics.com>
\copyright Copyright (c) (2023,), Locus Robotics Corp., All rights reserved.
Unauthorized copying of this file, via any medium, is strictly prohibited.
Proprietary and confidential.
**/

#ifndef INCLUDE_URG_NODE_UAM_SCIP_PROTOCOL_TYPES_H_
#define INCLUDE_URG_NODE_UAM_SCIP_PROTOCOL_TYPES_H_

#include <array>
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
  ACK = 0,             /**< ACK */
  STATUS,              /**< STATUS */
  SENSOR_MODEL,        /**< SENSOR_MODEL */
  MIN_DISTANCE,        /**< MIN_DISTANCE */
  MAX_DISTANCE,        /**< MAX_DISTANCE */
  ANGULAR_RESOLUTION,  /**< ANGULAR_RESOLUTION */
  STARTING_STEP,       /**< STARTING_STEP */
  END_STEP,            /**< END_STEP */
  STEP_FRONT_DIRECTION,/**< STEP_FRONT_DIRECTION */
  RPM,                 /**< RPM */
  NR_LINES             /**< PP_LINES */
};

/**
 * @brief Structure representing the PP
 */
struct PPReply
{
  std::string ack;
  std::string status;
  int min_distance;
  int max_distance;
  int angular_resolution;
  int start_step;
  int end_step;
  int front_data_index;
  int rpm;
};

}
}


#endif  // INCLUDE_URG_NODE_UAM_SCIP_PROTOCOL_TYPES_H_
