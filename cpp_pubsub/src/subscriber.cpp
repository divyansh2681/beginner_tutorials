/**
 * @file publisher_member_function.cpp
 * @author Divyansh Agrawal (dagrawa1@umd.edu)
 * @brief ROS2 publisher
 * @version 0.1
 * @date 2022-11-15
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include <memory>

#include "rclcpp/rclcpp.hpp"
// #include "std_msgs/msg/string.hpp"
#include "tutorial_interfaces/msg/num.hpp"
using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node {
 public:
  MinimalSubscriber()
  : Node("minimal_subscriber") {
    // subscription_ = this->create_subscription<std_msgs::msg::String>(
    //   "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
    subscription_ = this->create_subscription<tutorial_interfaces::msg::Num>(          // CHANGE
      "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
  }

 private:
  // void topic_callback(const std_msgs::msg::String::SharedPtr msg) const {
  //   RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
  // }
  // rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  void topic_callback(const tutorial_interfaces::msg::Num::SharedPtr msg) const       // CHANGE
  {
    RCLCPP_INFO(this->get_logger(), "I heard: '%d'", msg->num);              // CHANGE
  }
  rclcpp::Subscription<tutorial_interfaces::msg::Num>::SharedPtr subscription_;       // CHANGE
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}