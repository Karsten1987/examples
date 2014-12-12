#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <string>
#include <sys/time.h>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include <simple_msgs/AllBuiltinTypes.h>
#include <simple_msgs/AllDynamicArrayTypes.h>
#include <simple_msgs/AllPrimitiveTypes.h>
#include <simple_msgs/AllStaticArrayTypes.h>
#include <simple_msgs/Nested.h>
#include <simple_msgs/String.h>
#include <simple_msgs/Uint32.h>

#include <userland_msgs/AddTwoInts.h>


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("add_two_ints_client");

  auto client = node->create_client<userland_msgs::AddTwoInts>("add_two_ints");
  auto req = std::make_shared<userland_msgs::AddTwoInts::Request>();
  req->a = 2;
  req->b = 3;

  std::shared_ptr<userland_msgs::AddTwoInts::Response> response = client->send_request(req);
  std::cout << "Sum: " << response->sum << std::endl;

  response = client->send_request(req);
  std::cout << "Sum: " << response->sum << std::endl;

  response = client->send_request(req);
  std::cout << "Sum: " << response->sum << std::endl;

  response = client->send_request(req);
  std::cout << "Sum: " << response->sum << std::endl;

  return 0;
}
