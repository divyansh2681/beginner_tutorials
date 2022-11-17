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

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
// #include "std_msgs/msg/string.hpp"
#include "tutorial_interfaces/msg/num.hpp"  

class MinimalPublisher : public rclcpp::Node {
 public:
  MinimalPublisher()
  : Node("minimal_publisher"), count_(0) {
    publisher_ = this->create_publisher<tutorial_interfaces::msg::Num>("topic", 10);
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500), std::bind(&MinimalPublisher::timer_callback, this));
  }

 private:
  void timer_callback() {
    // auto message = std_msgs::msg::String();
    // message.data = "Hello, this is the publisher for ENPM808X std::to_string(count_++);
    // RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    // publisher_->publish(message);
    auto message = tutorial_interfaces::msg::Num();                               // CHANGE
    message.num = this->count_++;                                        // CHANGE
    RCLCPP_INFO(this->get_logger(), "Publishing: '%d'", message.num);    // CHANGE
    publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  // rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Publisher<tutorial_interfaces::msg::Num>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}