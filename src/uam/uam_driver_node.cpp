/*
 * uam_driver_node.cpp
 *
 *  Created on: 05/04/2023
 *      Author: cribeiromendes
 */


#include "uam/uam_driver.h"
#include <ros/callback_queue.h>

int main(int argc, char **argv)
{
  // Initialize node and nodehandles
  ros::init(argc, argv, "urg_node");
  std::cout << " Starting " << std::endl;
  auto nh_prv =  ros::NodeHandle("~");
  auto nh =  ros::NodeHandle();
  std::string ip_address;
  int ip_port;
  nh_prv.param<std::string>("ip_address", ip_address, "127.0.0.1");
  ros::NodeHandle("~").param<int>("ip_port", ip_port, 10940);
  std::shared_ptr<urg_node::UamDriver> driver;
  sensor_msgs::LaserScan msg;
  auto run = [&]() -> void
  {
    try
    {
      driver = std::make_shared<urg_node::UamDriver>(ip_address, ip_port);
      driver->initialize();
      driver->start();
      ros::WallDuration timeout(0.1f);
      auto global_cb_queue = ros::getGlobalCallbackQueue();
      while (ros::ok())
      {
        global_cb_queue->callAvailable(timeout);
      }
    }
    catch (std::runtime_error& e)
    {
      ROS_ERROR_THROTTLE(10.0, "Error connecting to Hokuyo: %s", e.what());
      driver.reset();
      return;
    }
    catch (std::exception& e)
    {
      ROS_ERROR_THROTTLE(10.0, "Unknown error connecting to Hokuyo: %s", e.what());
      driver.reset();
      return;
    }
    return;
  };


  run();

  return 0;
}
