// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "cpp_pubsub/srv/stringss.hpp"

using namespace std::chrono_literals;
using Stringss = cpp_pubsub::srv::Stringss;
using namespace std::placeholders;
/**
 * @brief class to represent publisher and subscriber
 * 
 */

class MinimalPublisher : public rclcpp::Node {
 public:
 /**
  * @brief Construct a new Minimal Publisher object
  * 
  */
  MinimalPublisher()
  : Node("minimal_publisher"), count_(0) {
    auto descript = rcl_interfaces::msg::ParameterDescriptor{};
    descript.description =
        "\nThis parameter modifies the queue size";
    this->declare_parameter("queue", queues_, descript);
    queues_ = this->get_parameter("queue").get_parameter_value().get<int>();
    RCLCPP_INFO_STREAM(this->get_logger(), \
    "Setting queue size to: " << queues_);

    publisher_ = \
    this->create_publisher<std_msgs::msg::String>("topic", queues_);
    timer_ = this->create_wall_timer(
      5000ms, std::bind(&MinimalPublisher::timer_callback, this));

    service_ = this->create_service<Stringss>("service_to_minimal_publisher", \
    std::bind(&MinimalPublisher::RespondToClient, this, _1, _2));
  }

 private:
  /**
   * @brief callback function for the publisher
   * 
   */
  void timer_callback() {
    auto message = std_msgs::msg::String();
    message.data = "Hello, this is the publisher for ENPM808X by Divyansh " \
    + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
  }
  /**
   * @brief function to respond back to the client request and 
      output the logger levels depending on the use input
   * 
   * @param request 
   * @param response 
   */
  void RespondToClient(const std::shared_ptr<Stringss::Request> request,
          std::shared_ptr<Stringss::Response> response) {
    response->output = request->input;

    RCLCPP_INFO(this->get_logger(), "Incoming request\na: [%s]",
                  request->input.c_str());

    std::string varr = std::string(response->output.c_str());
    if (varr == "DEBUG") {
      RCLCPP_DEBUG(this->get_logger(), response->output.c_str());
    } else if (varr == "WARN") {
      RCLCPP_WARN(this->get_logger(), response->output.c_str());
    } else if (varr == "FATAL") {
      RCLCPP_FATAL(this->get_logger(), response->output.c_str());
    } else if (varr == "ERROR") {
      RCLCPP_ERROR(this->get_logger(), response->output.c_str());
    } else {
      RCLCPP_INFO(this->get_logger(), response->output.c_str());
    }

    RCLCPP_INFO(this->get_logger(), "sending back response: [%s]", \
    response->output.c_str());
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Service<Stringss>::SharedPtr service_;
  int queues_;
  size_t count_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
