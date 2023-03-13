/*
 * uam_test_node.cpp
 *
 *  Created on: 13/03/2023
 *      Author: cribeiromendes
 */


#include <ros/ros.h>

#include <urg_node/uam/uam_command_workers.h>

int
main(int argc, char** argv)
{
  ros::Time::init();

  // Initialize node and nodehandles
  ros::init(argc, argv, "urg_node");

  uam::AR01Worker ar01;
  uam::AR00Worker ar00;
  uam::XR00Worker xr;
  uam::VR00Worker vr;
  ROS_WARN_STREAM("AR01 cmd: " << ar01.getCommand());
  ROS_WARN_STREAM("AR00 cmd: " << ar00.getCommand());

  ROS_WARN_STREAM("XR00Worker cmd: " << xr.getCommand());
  ROS_WARN_STREAM("VR00Worker cmd: " << vr.getCommand());


}
