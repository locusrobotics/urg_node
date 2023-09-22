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
#include <urg_node_msgs/StatusArray.h>

#include <string>
#include <vector>

namespace uam
{
/**
 * @brief Status aggregator params
 */
struct StatusAggregatorParams
{
  static StatusAggregatorParams loadFromROS(const ros::NodeHandle& nh)
  {
    StatusAggregatorParams params;
    nh.param("status_topics", params.status_topics, params.status_topics);
    nh.param("publish_on_change", params.publish_on_change, params.publish_on_change);
    return params;
  }
  std::vector<std::string> status_topics {};
  bool publish_on_change { false };
};

/**
 * @brief Status aggregator
 */
class StatusAggregator
{
public:
  /**
   * @brief Constructor
   * @param[in] params - Configuration parameters
   */
  explicit StatusAggregator(const StatusAggregatorParams& params) : params_(params)
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
      ROS_WARN_STREAM_COND(!name_valid, "Invalid topic name detected: " << error);
      return !name_valid;
    };

    // Check for invalid topics
    params_.status_topics.erase(
      std::remove_if(params_.status_topics.begin(), params_.status_topics.end(), isTopicNameInvalid),
      params_.status_topics.end());

    if (params_.status_topics.empty())
    {
      ROS_ERROR_STREAM("No valid status topic(s) found.");
      return;
    }

    ros::NodeHandle nh;
    output_status_publisher_ = nh.advertise<urg_node_msgs::StatusArray>("lidar_sensors_status", 10, true);

    output_status_.status.resize(params_.status_topics.size());

    // Setup subscriber
    for (size_t topic_idx = 0; topic_idx < params_.status_topics.size(); topic_idx++)
    {
      const auto& topic = params_.status_topics.at(topic_idx);
      status_subscribers_.push_back(nh.subscribe<urg_node_msgs::Status>(
        topic,
        1,
        boost::bind(
          &StatusAggregator::statusCallback,
          this,
          boost::placeholders::_1,
          std::ref(output_status_.status[topic_idx]))));
    }
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
  StatusAggregatorParams params_;

  /**
   * @brief Ouptut combined status
   */
  urg_node_msgs::StatusArray output_status_;

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
  ros::init(argc, argv, "uam_status_aggregator_node");
  uam::StatusAggregator node(uam::StatusAggregatorParams::loadFromROS(ros::NodeHandle("~")));
  node.run();
}
