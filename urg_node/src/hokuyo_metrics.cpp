/***************************************************************************
 * Copyright (C) 2024 Locus Robotics. All rights reserved.
 * Unauthorized copying of this file, via any medium, is strictly prohibited
 * Proprietary and confidential
 ***************************************************************************/

#include "urg_node/hokuyo_metrics.h"

namespace urg_node
{
  HokuyoMetrics::HokuyoMetrics(locus_cpp::NodeHandles &nhs) : stats_(
                              locus_cpp::findAndLoadParam(nhs.nh_private, "stats_hostname", std::string("localhost")),
                              locus_cpp::findAndLoadParam(nhs.nh_private, "stats_port", 8125),
                              locus_cpp::findAndLoadParam(nhs.nh_private, "stats_name", std::string("hokuyo")))
  {
    auto restart_topic =
        locus_cpp::findAndLoadParam(nhs.nh_private, "lidar_restart_topic", std::string("restart_counter"));
    restart_event_sub_ = nhs.nh.subscribe(restart_topic, 1, &HokuyoMetrics::publishMetrics, this);
    ROS_INFO("HokuyoMetrics initialized.");
  }

  void HokuyoMetrics::publishMetrics(const std_msgs::Empty::ConstPtr &msg)
  {
    // ros log info indicating that the lidar has been restarted
    ROS_INFO("Publishing lidar power cycle metrics. Incrementing counter.");
    stats_.increment("lidar_power_cycles");
  }

} // namespace urg_node

int main(int argc, char **argv)
{
  auto node_handles = locus_cpp::init(argc, argv, "hokuyo_metrics");
  urg_node::HokuyoMetrics hokuyo_metrics(node_handles);
  ros::spin();
  return 0;
}
