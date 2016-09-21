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
#pragma once

#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace zero_value
{

template<class T>
class ZeroValuePublisher : public rclcpp::Node
{
public:

  ZeroValuePublisher( std::string name, std::string topic, int frequency):
    Node(std::move(name), true),
    topic_(std::move(topic)),
    frequency_(std::move(frequency))
  {
    msg_ptr_.reset(new T());
    pub_ptr_ = this->create_publisher<T>(topic_, rmw_qos_profile_default);
  }

  void run( )
  {
    auto pub_func = std::bind(&ZeroValuePublisher<T>::publish, this);
    timer_ptr_ = this->create_wall_timer(std::chrono::milliseconds((1000/frequency_)), pub_func);
  }

  void stop( )
  {
    timer_ptr_->cancel();
  }

  void publish( )
  {
    msg_ptr_.reset(new T());
    //printf("Wonderful publisher %p with value %s\n", msg_ptr_.get(), msg_ptr_->data.c_str());
    pub_ptr_->publish(msg_ptr_);
  }

private:
  std::string topic_;
  int frequency_;

  typename rclcpp::Publisher<T>::SharedPtr pub_ptr_;
  //std::unique_ptr<T> msg_ptr_;  // publish only the pointer to the value stored inside this node
  std::unique_ptr<T> msg_ptr_;  // publish only the pointer to the value stored inside this node
  rclcpp::TimerBase::SharedPtr timer_ptr_;
};

} //ns
