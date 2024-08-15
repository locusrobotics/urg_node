/***************************************************************************
 * Copyright (C) 2024 Locus Robotics. All rights reserved.
 * Unauthorized copying of this file, via any medium, is strictly prohibited
 * Proprietary and confidential
 ***************************************************************************/
#ifndef URG_NODE_HOKUYO_METRICS_H_
#define URG_NODE_HOKUYO_METRICS_H_

#include <string>

#include <cpp-statsd-client/StatsdClient.hpp>
#include <locus_cpp/ros_util.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>

namespace urg_node
{
class HokuyoMetrics
{
public:
/**
 * @brief Gather metrics related to topological routing.
 * Currently only captrues information about robot following the current topo route
 * @param nhs The public and private node handles
 */
HokuyoMetrics(locus_cpp::NodeHandles & nhs);
protected:

void publishMetrics(const std_msgs::Empty::ConstPtr & msg);

Statsd::StatsdClient stats_;
ros::Subscriber restart_event_sub_;
};
}  // namespace urg_node
#endif  // URG_NODE_HOKUYO_METRICS_H_
