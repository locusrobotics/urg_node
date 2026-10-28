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

#include <rclcpp/rclcpp.hpp>
#include <urg_node_msgs/msg/status_array.hpp>

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

namespace uam
{
/**
 * @brief Status aggregator params
 */
struct StatusAggregatorParams
{
  static StatusAggregatorParams loadFromROS(rclcpp::Node* node)
  {
    StatusAggregatorParams params;
    params.status_topics =
      node->declare_parameter<std::vector<std::string>>("status_topics", params.status_topics);
    params.publish_on_change = node->declare_parameter<bool>("publish_on_change", params.publish_on_change);
    return params;
  }
  std::vector<std::string> status_topics {};
  bool publish_on_change { false };
};

/**
 * @brief Equality comparison for Status messages, used to detect changes when
 * publish_on_change is enabled. urg_node_msgs::msg::Status does not define
 * operator== in ROS2 the way the ROS1 message did, so it is implemented here.
 */
bool statusEqual(const urg_node_msgs::msg::Status& lhs, const urg_node_msgs::msg::Status& rhs)
{
  return lhs.operating_mode == rhs.operating_mode && lhs.area_number == rhs.area_number &&
         lhs.error_status == rhs.error_status && lhs.error_code == rhs.error_code &&
         lhs.lockout_status == rhs.lockout_status && lhs.ossd1_state == rhs.ossd1_state &&
         lhs.ossd2_state == rhs.ossd2_state && lhs.warning1_state == rhs.warning1_state &&
         lhs.warning2_state == rhs.warning2_state &&
         lhs.optical_window_contaminated == rhs.optical_window_contaminated;
}

/**
 * @brief Status aggregator
 */
class StatusAggregator : public rclcpp::Node
{
public:
  /**
   * @brief Constructor
   */
  explicit StatusAggregator(const rclcpp::NodeOptions& node_options = rclcpp::NodeOptions()) :
    Node("uam_status_aggregator_node", node_options),
    params_(StatusAggregatorParams::loadFromROS(this))
  {
    auto isTopicNameInvalid = [this](const std::string& topic_name) -> bool
    {
      if (topic_name.empty())
      {
        RCLCPP_WARN_STREAM(this->get_logger(), "Invalid topic name detected: Cannot have empty topic names");
        return true;
      }
      try
      {
        rclcpp::expand_topic_or_service_name(topic_name, this->get_name(), this->get_namespace());
        return false;
      }
      catch (const std::exception& e)
      {
        RCLCPP_WARN_STREAM(this->get_logger(), "Invalid topic name detected: " << e.what());
        return true;
      }
    };

    // Check for invalid topics
    params_.status_topics.erase(
      std::remove_if(params_.status_topics.begin(), params_.status_topics.end(), isTopicNameInvalid),
      params_.status_topics.end());

    if (params_.status_topics.empty())
    {
      RCLCPP_ERROR_STREAM(this->get_logger(), "No valid status topic(s) found.");
      return;
    }

    output_status_publisher_ =
      this->create_publisher<urg_node_msgs::msg::StatusArray>("lidar_sensors_status", rclcpp::QoS(10).transient_local());

    output_status_.status.resize(params_.status_topics.size());

    // Setup subscriber
    for (size_t topic_idx = 0; topic_idx < params_.status_topics.size(); topic_idx++)
    {
      const auto& topic = params_.status_topics.at(topic_idx);
      status_subscribers_.push_back(this->create_subscription<urg_node_msgs::msg::Status>(
        topic,
        1,
        [this, topic_idx](const urg_node_msgs::msg::Status::SharedPtr status)
        {
          this->statusCallback(status, output_status_.status[topic_idx]);
        }));
    }
    configured_ = true;
  }

  /**
   * @brief Whether the node was successfully configured with at least one valid status topic
   */
  inline bool isConfigured() const { return configured_; }

private:
  /**
   * @brief The status callback to update last received status and publish the output
   * @param[in] status - Incoming status
   * @param[in] last_status - Last staus
   */
  void statusCallback(const urg_node_msgs::msg::Status::SharedPtr status, urg_node_msgs::msg::Status& last_status)
  {
    bool should_publish = params_.publish_on_change ? !statusEqual(*status, last_status) : true;
    last_status = *status;
    if (should_publish)
      output_status_publisher_->publish(output_status_);
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
  urg_node_msgs::msg::StatusArray output_status_;

  /**
   * @brief Output combined status publisher
   */
  rclcpp::Publisher<urg_node_msgs::msg::StatusArray>::SharedPtr output_status_publisher_;

  /**
   * @brief Status Subscriber vector
   */
  std::vector<rclcpp::Subscription<urg_node_msgs::msg::Status>::SharedPtr> status_subscribers_;
};

}  // namespace uam

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<uam::StatusAggregator>();
  if (node->isConfigured())
  {
    rclcpp::spin(node);
  }
  rclcpp::shutdown();
  return 0;
}
