/**
 * @file beginner_subscriber.cpp
 * @author vinay06@umd.edu
 * @brief This program is used to generate a subscriber which prints the message
 * printed by a publisher to topic
 * @version 0.1
 * @date 2023-11-21
 *
 * @copyright Copyright (c) 2023
 *
 */
#include <memory>
#include <functional>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
using std::placeholders::_1;

/**
 * @brief The class minimal subscriber captures the message whenever published
 * to topic
 *
 */
class MinimalSubscriber : public rclcpp::Node {
 public:
  /**
   * @brief Construct a new Minimal Subscriber object
   * 
   */
  MinimalSubscriber() : Node("minimal_subscriber") {
    subscription_ = this->create_subscription<std_msgs::msg::String>(
        "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
  }

 private:
  /**
   * @brief A callback method to capture the message whenever published
   * 
   * @param msg 
   */
  void topic_callback(const std_msgs::msg::String& msg) const {
    RCLCPP_INFO_STREAM(this->get_logger(), "ROS2 Humble Heard: '%s'"<<msg.data.c_str());
  }
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
