// PHS: Template combining together publisher and subscriber examples

//General C++ includes as needed
#include <chrono>
#include <functional>
#include <memory>
#include <string>

//ROS 2 Specific includes
//Important: these includes should be reflected in the package.xml and CMakeLists.txt
#include "rclcpp/rclcpp.hpp"

// TODO: Add or replace with message type needed
#include "std_msgs/msg/u_int32.hpp" 

using namespace std::chrono_literals;
using std::placeholders::_1;

/* Template giving outline for creating a program
 * that contains both a publisher and subscriber.
 */

class PubSubTemplate : public rclcpp::Node
{
public:
  //TODO: Update node name to reflect purpose of node
  PubSubTemplate() : Node("pub_sub_template")
  {
    // TODO: Update message type and topic name as appropriate for the publisher. Remember to update private field too.
    publisher_ = this->create_publisher<std_msgs::msg::UInt32>("/hello/world", 10);

    // TODO: Update message type and topic name as appropriate for the subscriber. Remember to update private field too.
     subscription_ = this->create_subscription<std_msgs::msg::UInt32>(
       "/hello/world", 10, std::bind(&PubSubTemplate::topic_callback, this, _1));
    
    // Create a timer that will trigger calls to the method timer_callback
    // Multiple timers with different durations can be used as necessary.
    // TODO: Update timer duration as required
    timer_ = this->create_wall_timer(
      500ms, std::bind(&PubSubTemplate::timer_callback, this));
  }

private:

  //TODO rename method, especially if having multiple timers.
  void timer_callback()
  {
    //TODO add behaviour here to be repeated at regular intervals

    //Create an instance of the appropriate message type for publishing
    auto message = std_msgs::msg::UInt32();

    //Set the values for necessary fields in message type
    // On the commandline use
    //   $ ros2 interface show std_msgs/msg/UInt32
    // To check fields of message type
    message.data = count_;

    //Log output for message to be published
    RCLCPP_INFO(this->get_logger(), "Publishing: msg.data=%i", message.data);

    //publish the message to topic
    publisher_->publish(message);
  }

  //TODO rename method to better reflect topic it is subscribed to
  //TODO update message type as appropriate
  void topic_callback(const std_msgs::msg::UInt32 & msg)
  {
    //Log output of message received
    // On the commandline use
    //   $ ros2 interface show std_msgs/msg/UInt32
    // To check fields of message type
    //TODO update fields used from pre-defined message type
    RCLCPP_INFO(this->get_logger(), "I heard: msg.data=%i", msg.data);
    // Update class variables (based on message data)
    count_++;
  }

  //Declaration of private fields used for timer, publisher and subscriber
  //TODO: Update message types as appropriate for publishers and subscribers
  //Add other class variables here.
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::UInt32>::SharedPtr subscription_;
  size_t count_;
};

//Main method defining entry point for program
int main(int argc, char * argv[])
{
  //Initialise ROS 2 for this node
  rclcpp::init(argc, argv);

  //Create the instance of the Node subclass and 
  // start the spinner with a pointer to the instance
  rclcpp::spin(std::make_shared<PubSubTemplate>());

  //When the node is terminated, shut down ROS 2 for this node
  rclcpp::shutdown();
  return 0;
}
