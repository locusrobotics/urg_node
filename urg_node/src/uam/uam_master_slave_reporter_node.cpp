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

#include <ros/ros.h>
#include <urg_node_msgs/MasterSlaveStatus.h>

#include <string>
#include <vector>

namespace uam
{
/**
 * @brief Master Slave Status reporter params
 */
struct MasterSlaveReporterParams
{
  static MasterSlaveReporterParams loadFromROS(const ros::NodeHandle& nh)
  {
    MasterSlaveReporterParams params;
    nh.param("slave_lidar_status_topics", params.slave_lidar_topics, params.slave_lidar_topics);
    nh.param("master_lidar_status_topic", params.master_topic, params.master_topic);
    nh.param("publish_on_change", params.publish_on_change, params.publish_on_change);
    return params;
  }
  std::vector<std::string> slave_lidar_topics {};
  std::string master_topic { "" };
  bool publish_on_change { false };
};

/**
 * @brief Master Slave Status reporter
 */
class MasterSlaveRerporter
{
public:
  /**
   * @brief Constructor
   * @param[in] params - Configuration parameters
   */
  explicit MasterSlaveRerporter(const MasterSlaveReporterParams& params) : params_(params)
  {
    auto isTopicNameInvalid = [](const std::string& topic_name) -> bool
    {
      std::string error;
      auto name_valid = ros::names::validate(topic_name, error);
      if (name_valid && topic_name.empty())
      {
        error = "Cannot have empty topic names";
        name_valid = false;
      }
      ROS_WARN_STREAM_COND(
        !name_valid,
        "Invalid topic name detected: " << error);
      return !name_valid;
    };

    // Check for invalid topics
    params_.slave_lidar_topics.erase(
      std::remove_if(params_.slave_lidar_topics.begin(), params_.slave_lidar_topics.end(), isTopicNameInvalid),
      params_.slave_lidar_topics.end());

    if (isTopicNameInvalid(params_.master_topic) || params_.slave_lidar_topics.empty())
    {
      ROS_ERROR_STREAM("Master topic and/or slave topic(s) not found.");
      return;
    }

    ros::NodeHandle nh;
    output_status_publisher_ = nh.advertise<urg_node_msgs::MasterSlaveStatus>("lidar_sensors_status", 10, true);

    // Resize output message lidar slave vector
    output_status_.slaves.resize(params_.slave_lidar_topics.size());

    // Slave subscribers
    for (size_t slave_idx = 0; slave_idx < params_.slave_lidar_topics.size(); slave_idx++)
    {
      const auto& topic = params_.slave_lidar_topics.at(slave_idx);
      status_subscribers_.push_back(nh.subscribe<urg_node_msgs::Status>(
        topic,
        1,
        boost::bind(
          &MasterSlaveRerporter::statusCallback,
          this,
          boost::placeholders::_1,
          std::ref(output_status_.slaves[slave_idx]))));
    }

    // Master subscriber
    status_subscribers_.push_back(nh.subscribe<urg_node_msgs::Status>(
      params_.master_topic,
      1,
      boost::bind(
        &MasterSlaveRerporter::statusCallback,
        this,
        boost::placeholders::_1,
        std::ref(output_status_.master))));
    configured_ = true;
  }

  /**
   * @brief Spin callback queue if node is properly configured
   */
  void run()
  {
    if (configured_)
      ros::spin();
  }

private:
  /**
   * @brief The status callback to update last received status and publish the output
   * @param[in] status - Incoming status
   * @param[in] last_status - Last staus
   */
  void statusCallback(const urg_node_msgs::StatusConstPtr& status, urg_node_msgs::Status& last_status)
  {
    bool should_publish = params_.publish_on_change ? *status != last_status : true;
    last_status = *status;
    if (should_publish)
      output_status_publisher_.publish(output_status_);
  }

  /**
   * @brief Configured flag
   */
  bool configured_ { false };

  /**
   * @brief Node parameters
   */
  MasterSlaveReporterParams params_;

  /**
   * @brief Ouptut combined status
   */
  urg_node_msgs::MasterSlaveStatus output_status_;

  /**
   * @brief Output combined status publisher
   */
  ros::Publisher output_status_publisher_;

  /**
   * @brief Status Subscriber vector
   */
  std::vector<ros::Subscriber> status_subscribers_;
};

}  // namespace uam

int main(int argc, char** argv)
{
  ros::init(argc, argv, "uam_master_slave_status_reporter");
  uam::MasterSlaveRerporter node(uam::MasterSlaveReporterParams::loadFromROS(ros::NodeHandle("~")));
  node.run();
}
