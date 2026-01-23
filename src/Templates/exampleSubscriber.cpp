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

// PHS: Original file obtained from: wget -O subscriber_member_function.cpp https://raw.githubusercontent.com/ros2/examples/humble/rclcpp/topics/minimal_subscriber/member_function.cpp 18 Sept, 14:45
// PHS: File renamed from subscriber_member_function.cpp
// PHS: Renamed class from MinimalSubscriber
// PHS: Additional comments added to document the code
// PHS: Modified topic name used

//https://en.cppreference.com/w/cpp/header/functional
#include <functional> // provides the std::bind() and placeholders

//https://en.cppreference.com/w/cpp/header/memory
#include <memory> // provides std::make_shared<Class>()

//ROS 2 Specific includes
//https://docs.ros2.org/latest/api/rclcpp/rclcpp_8hpp.html
#include "rclcpp/rclcpp.hpp" // includes most common libraries for ROS 2
//https://index.ros.org/p/std_msgs/
//https://docs.ros.org/en/humble/Concepts/Basic/About-Interfaces.html
#include "std_msgs/msg/u_int32.hpp" // provides basic Unsigned int32 message type for publishing data

//Namespace specified for simplfication when using placeholders library
using std::placeholders::_1;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the subscriber. */

//Declaring a new class as a subclass of the ROS 2 Node class
class ExampleSubscriber : public rclcpp::Node
{
public:
  //Constructor specifying node name for superclass
  ExampleSubscriber()
  : Node("minimal_subscriber")
  {
    // Create the instance of the subscriber that will receive messages
    // of type std_msgs/mgs/UInt32 published to the topic "/hello/world"
    // a queue length of 10 is specified here and a reference is given
    // to the topic_callback method that will process messages that are received.
    subscription_ = this->create_subscription<std_msgs::msg::UInt32>(
      "/hello/world", 10, std::bind(&ExampleSubscriber::topic_callback, this, _1));
  }

private:
  // Private function that will be triggered automatically when messages are received
  // from the topic /hello/world
  // The parameter specifies the expected message type, which must match
  // the type published to the topic
  void topic_callback(const std_msgs::msg::UInt32 & msg)
  {
    //Logger used to print details of the message received (printed to console).
    RCLCPP_INFO(this->get_logger(), "I heard: msg.data=%i", msg.data);
  }

  //Declaration of private fields used for subscriber
  rclcpp::Subscription<std_msgs::msg::UInt32>::SharedPtr subscription_;
};

//Main method defining entry point for program
int main(int argc, char * argv[])
{
  //Initialise ROS 2 for this node
  rclcpp::init(argc, argv);

  //Create the instance of the Node subclass and 
  // start the spinner with a pointer to the instance
  rclcpp::spin(std::make_shared<ExampleSubscriber>());

  //When the node is terminated, shut down ROS 2 for this node
  rclcpp::shutdown();
  return 0;
}
