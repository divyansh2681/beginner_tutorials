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
#include <cstdlib>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "cpp_pubsub/srv/stringss.hpp"

using namespace std::chrono_literals;

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  if (argc != 2) {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "usage: input a string");
      return 1;
  }
  std::shared_ptr<rclcpp::Node> node = \
  rclcpp::Node::make_shared("client_for_updating_string");
  rclcpp::Client<cpp_pubsub::srv::Stringss>::SharedPtr client =
    node->create_client<cpp_pubsub::srv::Stringss>\
    ("service_to_minimal_publisher");

  auto request = std::make_shared<cpp_pubsub::srv::Stringss::Request>();
  request->input = std::string(argv[1]);

  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), \
      "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), \
    "service not available, waiting again...");
  }

  auto result = client->async_send_request(request);
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node, result) ==
    rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Updated String is: [%s]", \
    result.get()->output.c_str());
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service");
  }
  rclcpp::shutdown();
  return 0;
}
