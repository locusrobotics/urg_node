/*
 * uam_driver_node.cpp
 *
 *  Created on: 05/04/2023
 *      Author: cribeiromendes
 */


#include "uam/uam_driver.h"
#include <ros/callback_queue.h>
#include <uam/uam_driver_ros_params.h>
#include <uam/uam_driver_ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "uam_driver_node");
  auto lidar = uam::UamROS(ros::NodeHandle(), uam::UamROSParams::loadFromROS(ros::NodeHandle("~")));
  ros::spin();
}
