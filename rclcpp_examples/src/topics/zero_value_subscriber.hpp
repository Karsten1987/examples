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

template<typename T>
class ZeroValueSubscriber : public rclcpp::Node
{
public:
  ZeroValueSubscriber( std::string name, std::string topic ):
    Node(name, true),
    topic_(std::move(topic))
  {  }

  void run( )
  {
    auto callback_functor = std::bind(&ZeroValueSubscriber<T>::callback, this, std::placeholders::_1);
    sub_ptr_ = this->create_subscription<T>(topic_, callback_functor);
  }

private:

  void callback(typename T::UniquePtr msg)
  {
    static auto i = 0;

    if (old_msg_ptr_ != msg.get())
    {
      printf("Pointer address changed from %p to %p\n", old_msg_ptr_, msg.get());
      printf("Received %d messages with the same pointer before\n", i);
      old_msg_ptr_ = msg.get();
    }

    i++;
    //printf("Received #%d message with address: %p\n", (i++), msg.get());
  }

  T* old_msg_ptr_ = nullptr;

  std::string topic_;
  typename rclcpp::Subscription<T>::SharedPtr sub_ptr_;
};

} // ns
