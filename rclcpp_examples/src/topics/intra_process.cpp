// Copyright 2014 Open Source Robotics Foundation, Inc.
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

#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "zero_value_publisher.hpp"
#include "zero_value_subscriber.hpp"

int main(int argc, char * argv[])
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);

  using ZVP = zero_value::ZeroValuePublisher<std_msgs::msg::String>;
  auto pub_node = std::make_shared<ZVP>("zero_pub", "zero_topic", 10);
  pub_node->run();

  using ZVS = zero_value::ZeroValueSubscriber<std_msgs::msg::String>;
  auto sub_node = std::make_shared<ZVS>("zero_sub", "zero_topic");
  sub_node->run();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(pub_node);
  executor.add_node(sub_node);
  executor.spin();

  return 0;
}
