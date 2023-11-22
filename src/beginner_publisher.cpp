/**
 * @file beginner_publisher.cpp
 * @author vinay06@umd.edu
 * @brief This program is used to create a service call to get request and
 * response. Also publishing to a static transform broadcaster
 * @version 0.1
 * @date 2023-11-21
 *
 * @copyright Copyright (c) 2023 Vinay Bukka
 * This code is licensed under the Apache 2.0 License. Please see the
 * accompanying LICENSE file for the full text of the license.
 */
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <beginner_tutorials/srv/custom_service.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <chrono>
#include <functional>
#include <memory>
#include <rclcpp/executors.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>
using namespace std::chrono_literals;

/**
 * @brief The class minimal publisher creates a publisher object
 * with bind method to publish frequently. It also creates a service call object
 * to get the service request and response
 */
class MinimalPublisher : public rclcpp::Node {
 public:
  MinimalPublisher() : Node("minimal_publisher"), count_(0) {
    // set the logger level to DEBUG
    // this->get_logger().set_level(rclcpp::Logger::Level::Debug);
    // RCLCPP_DEBUG_STREAM(this->get_logger(),"Getting frequency parameter
    // value");

    // Parameter for initializig publisher frequency
    auto pub_frequency_info = rcl_interfaces::msg::ParameterDescriptor();
    pub_frequency_info.description = "Custom frequency value for the publisher";

    // default frequency is 1.0
    this->declare_parameter("frequency", 1.0, pub_frequency_info);
    auto pub_frequency = this->get_parameter("frequency").get_parameter_value().get<std::float_t>();
    if (pub_frequency < 0.0) {
      RCLCPP_FATAL_STREAM_ONCE(rclcpp::get_logger("minimal_publisher"),
                               "Frequency Cannot be negative");
      exit(1);
    } else if (pub_frequency == 0.0) {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("minimal_publisher"),
                          "Frequency set to zero");
    } else if (pub_frequency > 100.0) {
      RCLCPP_WARN_STREAM_ONCE(rclcpp::get_logger("minimal_publisher"),
                              "Frequency greater than hundread");
    } else {
      RCLCPP_DEBUG_STREAM(rclcpp::get_logger("minimal_publisher"),
                          "Frequency parameter is " << pub_frequency << " Hz");

      RCLCPP_INFO_STREAM(rclcpp::get_logger("minimal_publisher"),
                         "Publishing at " << pub_frequency << " Hz");
    }

    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    auto time =
        std::chrono::milliseconds(static_cast<int>(1000 / pub_frequency));
    timer_ = this->create_wall_timer(
        time, std::bind(&MinimalPublisher::timer_callback, this));
    // Creating a service object to get request and response
    auto serviceCallbackPtr =
        std::bind(&MinimalPublisher::change_message, this,
                  std::placeholders::_1, std::placeholders::_2);

    service_ = create_service<beginner_tutorials::srv::CustomService>(
        "custom_service", serviceCallbackPtr);
  }

 private:
  /**
   * @brief This is used to publish the messages
   *
   */
  void timer_callback() {
    auto message = std_msgs::msg::String();
    message.data = "Hello ROS2 Humble! " + std::to_string(count_++);
    RCLCPP_INFO_STREAM(rclcpp::get_logger("minimal_publisher"),
                       "Publishing: " << message.data);
    publisher_->publish(message);
    broadcast_transform();
  }
  /**
   * @brief This is used to change the message of service call object
   *
   * @param request , This parameter sets the service call request
   * @param response , This sets the service call response
   */
  void change_message(
      const std::shared_ptr<beginner_tutorials::srv::CustomService::Request>
          request,
      std::shared_ptr<beginner_tutorials::srv::CustomService::Response>
          response) {
    response->response_message =
        request->request_message + "Hi, This is modified service!!";
    RCLCPP_INFO_STREAM(rclcpp::get_logger("minimal_publisher"),
                       "Request message: " << request->request_message);
    RCLCPP_INFO_STREAM(rclcpp::get_logger("minimal_publisher"),
                       "Response message: " << response->response_message);
  }
  void broadcast_transform() {
    tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "world";
    t.child_frame_id = "talk";
    // Translation component in meters
    t.transform.translation.x = 0.2;
    t.transform.translation.y = 0.0;
    t.transform.translation.z = 0.6;
    t.transform.rotation.x = 0.0;
    t.transform.rotation.y = 0.0473595;
    t.transform.rotation.z = 0.0;
    t.transform.rotation.w = 0.0;
    tf_static_broadcaster_->sendTransform(t);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Service<beginner_tutorials::srv::CustomService>::SharedPtr service_;
  size_t count_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
