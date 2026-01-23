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

// PHS: Original file obtained from: https://raw.githubusercontent.com/ros2/examples/humble/rclcpp/topics/minimal_publisher/member_function.cpp 18th Sept 2023, 11:36
// PHS: File renamed from publisher_member_function.cpp
// PHS: Renamed class from MinimalPublisher
// PHS: Additional comments added to document the code
// PHS: Modified topic name used
// Hab75: Modified topic name and use to legged_controller for Unitree Go1.

/*
About: A Ros publisher file that takes recived UDP data from a unitree Go1 to publush
*/

// https://en.cppreference.com/w/cpp/chrono
#include <chrono> // provides system_clock, times and durations

// https://en.cppreference.com/w/cpp/header/functional
#include <functional> // provides the std::bind()

// https://en.cppreference.com/w/cpp/header/memory
#include <memory> // provides std::make_shared<Class>()

// https://en.cppreference.com/w/cpp/header/string
#include <string> // provides std::to_string()

// General includes
#include <math.h>
#include <iostream>
#include <unistd.h>

// ROS 2 Specific includes
// https://docs.ros2.org/latest/api/rclcpp/rclcpp_8hpp.html
#include "rclcpp/rclcpp.hpp" // includes most common libraries for ROS 2
// https://index.ros.org/p/std_msgs/
// https://docs.ros.org/en/humble/Concepts/Basic/About-Interfaces.html
#include "geometry_msgs/msg/twist.hpp" // twist msg https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Twist.html

#include "sensor_msgs/msg/imu.hpp" // IMU msg https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Imu.html
#include "sensor_msgs/msg/temperature.hpp" // Temprature msg

#include "nav_msgs/msg/odometry.hpp" // Odom msg https://docs.ros.org/en/jade/api/nav_msgs/html/msg/Odometry.html

// Unitree ros to real msg's
// Edited to comply with syntax constraints.
#include "unitree_ros2_cpp/msg/bms_state.hpp"
#include "unitree_ros2_cpp/msg/high_state.hpp"
#include "unitree_ros2_cpp/msg/high_cmd.hpp"
#include "unitree_ros2_cpp/msg/motor_state.hpp"
#include "unitree_ros2_cpp/msg/motor_cmd.hpp"

// Important: these includes should be reflected in the package.xml and CMakeLists.txt

// UnitreeSDK includes
/*IMPORTANT SOURCE: https://unitree-docs.readthedocs.io/en/latest/get_started/Go1_Edu.html*/
#include "unitree_legged_sdk/unitree_legged_sdk.h"

// IP Address to Pi   (Default: WiFi - 192.168.12.1,   Ethernet - 192.168.123.161)
const char* piAddress = "192.168.12.1";

// Namespace specified for simplfication when using chrono library
using namespace UNITREE_LEGGED_SDK;
using namespace std::chrono_literals;

/* Legged SDK UDP initializer class.
This class sets up a UDP connection to the robot.
Code sourced & modified from exampleWalk in the unitree_legged_sdk/examples
*/
class UDPLegged
{
public: // <- Change to your robot model- Aliengo, A1, Go1 (Tested), B1
  UDPLegged(uint8_t level) : safe(LeggedType::Go1),
                             udp(level, 8090, piAddress, 8082)
  {
    udp.InitCmdData(cmd);
  }

  void UDPRecv();
  void UDPSend();

  Safety safe;
  UDP udp;
  HighCmd cmd = {}; // Left
  HighState state = {};
};

void UDPLegged::UDPRecv()
{
  udp.Recv();
  udp.GetRecv(state);
}

void UDPLegged::UDPSend()
{
  udp.Send();
}
UDPLegged udpLegged(HIGHLEVEL); // object creation for callback


/*
 * UDP loop service node (would be a ros service but running out of time towards the deadline. This is a hotfix)
 * runs the UDPRecv & UDPSend commands repeatedly. (Will automatically reconnect once the robot is seen on network again)
 * This is to ensure the latest data is recived.
*/
class UDPLoopService : public rclcpp::Node
{
public:
  UDPLoopService() 
    : Node("UDPServer")
    {
      RCLCPP_INFO(this->get_logger(), "UDP Server started.");

      // Start timer loop to update the UDP data
        timer_udp = this->create_wall_timer(
            3ms, std::bind(&UDPLoopService::udp_callback, this));
        
    }
  private:
  void udp_callback()
  {
    udpLegged.UDPSend();
    udpLegged.UDPRecv();
  }
  rclcpp::TimerBase::SharedPtr timer_udp;
};
// End of UDP setup






// Start of Ros Nodes

// Declaring a new class as a subclass of the ROS 2 Node class
class LeggedDataRX : public rclcpp::Node
{

public:
  // Constructor, specifying node name and initialising a counter
  LeggedDataRX()
      : Node("LeggedDataRX"), count_(0)
  {
    // Create the instance of the publisher that will publish messages
    // of type unitree_ros2_cpp/msg/bms_state to the topic "/legged_data/sensors/bms"
    // a queue length of 10 is specified here for the topic
    bms_publisher = this->create_publisher<unitree_ros2_cpp::msg::BmsState>("legged_data/sensors/bms", 10);
    // Create a timer that will trigger calls to the method bms_callback
    // every 0.65s
    timer_bms = this->create_wall_timer(
        650ms, std::bind(&LeggedDataRX::bms_callback, this));


    // Create the instance of the publisher that will publish messages
    // of type unitree_ros2_cpp/msg/high_state to the topic "/legged_data/sensors/foot_force"
    // a queue length of 10 is specified here for the topic
    foot_force_publisher = this->create_publisher<unitree_ros2_cpp::msg::HighState>("legged_data/sensors/foot_force", 10);
    // Create a timer that will trigger calls to the method footForce_callback
    // every 0.15s
    timer_foot_force = this->create_wall_timer(
        150ms, std::bind(&LeggedDataRX::footForce_callback, this));


    // Create the instance of the publisher that will publish messages
    // of type sensor_msgs/msg/IMU to the topic "/legged_data/sensors/imu"
    // a queue length of 10 is specified here for the topic
    imu_publisher = this->create_publisher<sensor_msgs::msg::Imu>("legged_data/sensors/imu", 10);
    // Create a timer that will trigger calls to the method imu_callback
    // every 0.05s
    timer_imu = this->create_wall_timer(
        50ms, std::bind(&LeggedDataRX::imu_callback, this));


    // Create the instance of the publisher that will publish messages
    // of type unitree_ros2_cpp/msg/HighState to the topic "/legged_data/status/mode"
    // a queue length of 10 is specified here for the topic
    mode_publisher = this->create_publisher<unitree_ros2_cpp::msg::HighState>("legged_data/status/mode", 10);
    // Create a timer that will trigger calls to the method imu_callback
    // every 0.7s
    timer_mode = this->create_wall_timer(
        700ms, std::bind(&LeggedDataRX::mode_callback, this));


    // Create the instance of the publisher that will publish messages
    // of type sensor_msgs/Temperature to the topic "/legged_data/sensors/system_temperature"
    // a queue length of 10 is specified here for the topic
    temp_publisher = this->create_publisher<sensor_msgs::msg::Temperature>("legged_data/sensors/system_temperature", 10);
    // Create a timer that will trigger calls to the method imu_callback
    // every 1.0s
    timer_temp = this->create_wall_timer(
        1000ms, std::bind(&LeggedDataRX::temp_callback, this));

        
    // Create the instance of the publisher that will publish messages
    // of type nav_msgs/Odometry to the topic "/odom"
    // a queue length of 10 is specified here for the topic
    odom_publisher = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    // Create a timer that will trigger calls to the method imu_callback
    // every 0.15s
    timer_odom = this->create_wall_timer(
        150ms, std::bind(&LeggedDataRX::odom_callback, this));


    // Create the instance of the publisher that will publish messages
    // of type unitree_ros2_cpp/msg/HighState to the topic "/legged_data/status/gait_type"
    // a queue length of 10 is specified here for the topic
    gait_publisher = this->create_publisher<unitree_ros2_cpp::msg::HighState>("legged_data/status/gait_type", 10);
    // Create a timer that will trigger calls to the method imu_callback
    // every 0.7s
    timer_gait = this->create_wall_timer(
        700ms, std::bind(&LeggedDataRX::gait_callback, this));


    // Create the instance of the publisher that will publish messages
    // of type unitree_ros2_cpp/msg/HighState to the topic "/legged_data/status/about_go1"
    // a queue length of 10 is specified here for the topic
    about_publisher = this->create_publisher<unitree_ros2_cpp::msg::HighState>("legged_data/status/about_go1", 10);
    // Create a timer that will trigger calls to the method imu_callback
    // every 60s
    timer_about = this->create_wall_timer(
        60s, std::bind(&LeggedDataRX::about_callback, this));


    // Create the instance of the publisher that will publish messages
    // of type unitree_ros2_cpp/msg/HighState to the topic "/legged_data/status/foot_raise_height"
    // a queue length of 10 is specified here for the topic
    foot_raise_height_publisher = this->create_publisher<unitree_ros2_cpp::msg::HighState>("legged_data/status/foot_raise_height", 10);
    // Create a timer that will trigger calls to the method imu_callback
    // every 0.7s
    timer_foot_raise_height = this->create_wall_timer(
        700ms, std::bind(&LeggedDataRX::foot_raise_height_callback, this));


    // Create the instance of the publisher that will publish messages
    // of type unitree_ros2_cpp/msg/MotorState to the topic "legged_data/actuators/motor_0"
    // a queue length of 10 is specified here for the topic
    motor_0_publisher = this->create_publisher<unitree_ros2_cpp::msg::MotorState>("legged_data/actuators/motor_0", 10);
    // Create a timer that will trigger calls to the method motor_0_callback
    // every 200ms
    timer_motor_0 = this->create_wall_timer(
        200ms, std::bind(&LeggedDataRX::motor_0_callback, this));

    // Create the instance of the publisher that will publish messages
    // of type unitree_ros2_cpp/msg/MotorState to the topic "legged_data/actuators/motor_1"
    // a queue length of 10 is specified here for the topic
    motor_1_publisher = this->create_publisher<unitree_ros2_cpp::msg::MotorState>("legged_data/actuators/motor_1", 10);
    // Create a timer that will trigger calls to the method motor_1_callback
    // every 200ms
    timer_motor_1 = this->create_wall_timer(
        200ms, std::bind(&LeggedDataRX::motor_1_callback, this));

    // Create the instance of the publisher that will publish messages
    // of type unitree_ros2_cpp/msg/MotorState to the topic "legged_data/actuators/motor_2"
    // a queue length of 10 is specified here for the topic
    motor_2_publisher = this->create_publisher<unitree_ros2_cpp::msg::MotorState>("legged_data/actuators/motor_2", 10);
    // Create a timer that will trigger calls to the method motor_2_callback
    // every 200ms
    timer_motor_2 = this->create_wall_timer(
        200ms, std::bind(&LeggedDataRX::motor_2_callback, this));

    // Create the instance of the publisher that will publish messages
    // of type unitree_ros2_cpp/msg/MotorState to the topic "legged_data/actuators/motor_3"
    // a queue length of 10 is specified here for the topic
    motor_3_publisher = this->create_publisher<unitree_ros2_cpp::msg::MotorState>("legged_data/actuators/motor_3", 10);
    // Create a timer that will trigger calls to the method motor_3_callback
    // every 200ms
    timer_motor_3 = this->create_wall_timer(
        200ms, std::bind(&LeggedDataRX::motor_3_callback, this));

    // Create the instance of the publisher that will publish messages
    // of type unitree_ros2_cpp/msg/MotorState to the topic "legged_data/actuators/motor_4"
    // a queue length of 10 is specified here for the topic
    motor_4_publisher = this->create_publisher<unitree_ros2_cpp::msg::MotorState>("legged_data/actuators/motor_4", 10);
    // Create a timer that will trigger calls to the method motor_4_callback
    // every 200ms
    timer_motor_4 = this->create_wall_timer(
        200ms, std::bind(&LeggedDataRX::motor_4_callback, this));

    // Create the instance of the publisher that will publish messages
    // of type unitree_ros2_cpp/msg/MotorState to the topic "legged_data/actuators/motor_5"
    // a queue length of 10 is specified here for the topic
    motor_5_publisher = this->create_publisher<unitree_ros2_cpp::msg::MotorState>("legged_data/actuators/motor_5", 10);
    // Create a timer that will trigger calls to the method motor_5_callback
    // every 200ms
    timer_motor_5 = this->create_wall_timer(
        200ms, std::bind(&LeggedDataRX::motor_5_callback, this));

    // Create the instance of the publisher that will publish messages
    // of type unitree_ros2_cpp/msg/MotorState to the topic "legged_data/actuators/motor_6"
    // a queue length of 10 is specified here for the topic
    motor_6_publisher = this->create_publisher<unitree_ros2_cpp::msg::MotorState>("legged_data/actuators/motor_6", 10);
    // Create a timer that will trigger calls to the method motor_6_callback
    // every 200ms
    timer_motor_6 = this->create_wall_timer(
        200ms, std::bind(&LeggedDataRX::motor_6_callback, this));

    // Create the instance of the publisher that will publish messages
    // of type unitree_ros2_cpp/msg/MotorState to the topic "legged_data/actuators/motor_7"
    // a queue length of 10 is specified here for the topic
    motor_7_publisher = this->create_publisher<unitree_ros2_cpp::msg::MotorState>("legged_data/actuators/motor_7", 10);
    // Create a timer that will trigger calls to the method motor_7_callback
    // every 200ms
    timer_motor_7 = this->create_wall_timer(
        200ms, std::bind(&LeggedDataRX::motor_7_callback, this));

    // Create the instance of the publisher that will publish messages
    // of type unitree_ros2_cpp/msg/MotorState to the topic "legged_data/actuators/motor_8"
    // a queue length of 10 is specified here for the topic
    motor_8_publisher = this->create_publisher<unitree_ros2_cpp::msg::MotorState>("legged_data/actuators/motor_8", 10);
    // Create a timer that will trigger calls to the method motor_8_callback
    // every 200ms
    timer_motor_8 = this->create_wall_timer(
        200ms, std::bind(&LeggedDataRX::motor_8_callback, this));

    // Create the instance of the publisher that will publish messages
    // of type unitree_ros2_cpp/msg/MotorState to the topic "legged_data/actuators/motor_9"
    // a queue length of 10 is specified here for the topic
    motor_9_publisher = this->create_publisher<unitree_ros2_cpp::msg::MotorState>("legged_data/actuators/motor_9", 10);
    // Create a timer that will trigger calls to the method motor_9_callback
    // every 200ms
    timer_motor_9 = this->create_wall_timer(
        200ms, std::bind(&LeggedDataRX::motor_9_callback, this));

    // Create the instance of the publisher that will publish messages
    // of type unitree_ros2_cpp/msg/MotorState to the topic "legged_data/actuators/motor_10"
    // a queue length of 10 is specified here for the topic
    motor_10_publisher = this->create_publisher<unitree_ros2_cpp::msg::MotorState>("legged_data/actuators/motor_10", 10);
    // Create a timer that will trigger calls to the method motor_10_callback
    // every 200ms
    timer_motor_10 = this->create_wall_timer(
        200ms, std::bind(&LeggedDataRX::motor_10_callback, this));

    // Create the instance of the publisher that will publish messages
    // of type unitree_ros2_cpp/msg/MotorState to the topic "legged_data/actuators/motor_11"
    // a queue length of 10 is specified here for the topic
    motor_11_publisher = this->create_publisher<unitree_ros2_cpp::msg::MotorState>("legged_data/actuators/motor_11", 10);
    // Create a timer that will trigger calls to the method motor_11_callback
    // every 200ms
    timer_motor_11 = this->create_wall_timer(
        200ms, std::bind(&LeggedDataRX::motor_11_callback, this));
      }


// topic Callbacks
private:
  /*
   * BMS Data publisher
   * Takes bms data and updates the msg with latest battery sensor data
   * WARNING: Batterys will require to be on compatable firmware versions to the robot for data.
   */
  void bms_callback()
  {
    // Create an instance of the BmsState message type
    auto message = unitree_ros2_cpp::msg::BmsState();

    udpLegged.UDPSend();
    udpLegged.UDPRecv(); // Fetch the latest state

    // Convert UDP_Raw to BmsState
    message.soc = udpLegged.state.bms.SOC;               // Battery %
    message.current = udpLegged.state.bms.current;       // current in milliamp
    message.cell_vol = udpLegged.state.bms.cell_vol;     // cell voltage in array[10]
    message.version_h = udpLegged.state.bms.version_h;   // Battery firmware version
    message.bms_status = udpLegged.state.bms.bms_status; // Battery status
    message.cycle = udpLegged.state.bms.cycle;           // The current number of cycles of the battery
    message.bq_ntc = udpLegged.state.bms.BQ_NTC;         // Temp output in degrees C
    message.mcu_ntc = udpLegged.state.bms.MCU_NTC;       // Temp output in degrees C

    // Check if the battery is reporting data
    // if the battery SOC was ever actually at 0, the robot would be off & not boot in the first place
    if (message.soc == 0)
    {
      // If no battery data is detected, display error warning
      RCLCPP_WARN(this->get_logger(), "WARNING: Battery Management System \n Data: OutOfExpectedBounds: Please ensure a healthy battery is installed OR of a compatable firmware");
    }

    // publish the message created above to the topic /legged_data/sensors/bms
    bms_publisher->publish(message);
  }
  // Declaration of private fields used for timer, publisher and counter
  rclcpp::TimerBase::SharedPtr timer_bms;
  rclcpp::Publisher<unitree_ros2_cpp::msg::BmsState>::SharedPtr bms_publisher;



  /*
   * FootForce Data publisher
   * Takes HighState data and updates the msg with latest foot force sensor data
   */
  void footForce_callback()
  {
    // Create an instance of the HighState message type
    auto message = unitree_ros2_cpp::msg::HighState();

    message.foot_force = udpLegged.state.footForce;
    // message.foot_force_est = udpLegged.state.footForceEst; // Disabled as was showing 0 

    // publish the message created above to the topic /legged_data/sensors/foot_force
    foot_force_publisher->publish(message);
  }

  // Declaration of private fields used for timer, publisher and counter
  rclcpp::TimerBase::SharedPtr timer_foot_force;
  rclcpp::Publisher<unitree_ros2_cpp::msg::HighState>::SharedPtr foot_force_publisher;



  /*
   * IMU Data publisher
   * Takes IMU data and updates the msg with latest imu data
   */
  void imu_callback() // when under accelerated motion, the attitude of the robot calculated by IMU will drift.
  {
    // Create an instance of the IMU message type
    auto message = sensor_msgs::msg::Imu();

    // geometry_msgs/Quaternion orientation
    message.orientation.w = udpLegged.state.imu.quaternion[0];
    message.orientation.x = udpLegged.state.imu.quaternion[1];
    message.orientation.y = udpLegged.state.imu.quaternion[2];
    message.orientation.z = udpLegged.state.imu.quaternion[3];

    // geometry_msgs/Vector3 angular_velocity
    message.angular_velocity.x = udpLegged.state.imu.gyroscope[0];
    message.angular_velocity.y = udpLegged.state.imu.gyroscope[1];
    message.angular_velocity.z = udpLegged.state.imu.gyroscope[2];

    // geometry_msgs/Vector3 linear_acceleration
    message.linear_acceleration.x = udpLegged.state.imu.accelerometer[0];
    message.linear_acceleration.y = udpLegged.state.imu.accelerometer[1];
    message.linear_acceleration.z = udpLegged.state.imu.accelerometer[2];

    // Not in use by sensor_msgs/Imu
    // message.rpy = udpLegged.state.imu.rpy;
    // message.temperature = udpLegged.state.imu.temperature;


    // publish the message created above to the topic /legged_data/sensors/imu
    imu_publisher->publish(message);
  }

  // Declaration of private fields used for timer, publisher and counter
  rclcpp::TimerBase::SharedPtr timer_imu;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher;



  /*
   * Mode Data publisher
   * Takes HighState data and updates the msg with latest mode data
   */
  void mode_callback()
  {
    // Create an instance of the HighState message type
    auto message = unitree_ros2_cpp::msg::HighState();


    // Set latest known mode to msg
    message.mode = udpLegged.state.mode;


    // publish the message created above to the topic /legged_data/status/mode
    mode_publisher->publish(message);
  }

  // Declaration of private fields used for timer, publisher and counter
  rclcpp::TimerBase::SharedPtr timer_mode;
  rclcpp::Publisher<unitree_ros2_cpp::msg::HighState>::SharedPtr mode_publisher;



  /*
   * Temprature Data publisher
   * Takes IMU temp data and updates the msg with latest temprature data
   */
  void temp_callback()
  {
    // Create an instance of the Temperature message type
    auto message = sensor_msgs::msg::Temperature();


    // Set latest to msg
    message.temperature = udpLegged.state.imu.temperature;

    // publish the message created above to the topic /legged_data/sensors/system_temperature
    temp_publisher->publish(message);
  }

  // Declaration of private fields used for timer, publisher and counter
  rclcpp::TimerBase::SharedPtr timer_temp;
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temp_publisher;

  

  /*
   * Odom Data publisher
   * Takes HighState velocity data and updates the msg with latest odom data
   */
  void odom_callback()
  {
    // Create an instance of the Odometry message type
    auto message = nav_msgs::msg::Odometry();


    // Set latest to msg

    //header
    message.header.frame_id = "odom";
    message.child_frame_id = "base_link";
    message.header.stamp = get_clock()->now(); // set time of data capture
    


    //geometry_msgs/TwistWithCovariance twist
    message.twist.twist.linear.x = udpLegged.state.velocity[0];
    message.twist.twist.linear.y = udpLegged.state.velocity[1];
    message.twist.twist.linear.z = udpLegged.state.velocity[2];

    message.twist.twist.angular.z = udpLegged.state.yawSpeed;

    //geometry_msgs/PoseWithCovariance pose
    message.pose.pose.orientation.w = udpLegged.state.imu.quaternion[0];
    message.pose.pose.orientation.x = udpLegged.state.imu.quaternion[1];
    message.pose.pose.orientation.y = udpLegged.state.imu.quaternion[2];
    message.pose.pose.orientation.z = udpLegged.state.imu.quaternion[3];

    message.pose.pose.position.x = udpLegged.state.position[0];
    message.pose.pose.position.y = udpLegged.state.position[1];
    message.pose.pose.position.z = udpLegged.state.position[2];

    // publish the message created above to the topic /odom
    odom_publisher->publish(message);
  }

  // Declaration of private fields used for timer, publisher and counter
  rclcpp::TimerBase::SharedPtr timer_odom;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher;



  /*
   * Gait_type Data publisher
   * Takes HighState data and updates the msg with latest gait_type data
   */
  void gait_callback()
  {
    // Create an instance of the HighState message type
    auto message = unitree_ros2_cpp::msg::HighState();


    // Set latest known mode to msg
    message.gait_type = udpLegged.state.gaitType;


    // publish the message created above to the topic /legged_data/status/gait_type
    mode_publisher->publish(message);
  }

  // Declaration of private fields used for timer, publisher and counter
  rclcpp::TimerBase::SharedPtr timer_gait;
  rclcpp::Publisher<unitree_ros2_cpp::msg::HighState>::SharedPtr gait_publisher;



  /*
   * About_Go1 Data publisher
   * Takes HighState data and updates the msg with latest about data
   */
  void about_callback()
  {
    // Create an instance of the HighState message type
    auto message = unitree_ros2_cpp::msg::HighState();


    // Set latest known mode to msg
    message.sn = udpLegged.state.SN;
    message.version = udpLegged.state.version;
    message.bandwidth = udpLegged.state.bandWidth;


    // publish the message created above to the topic /legged_data/status/gait_type
    mode_publisher->publish(message);
  }

  // Declaration of private fields used for timer, publisher and counter
  rclcpp::TimerBase::SharedPtr timer_about;
  rclcpp::Publisher<unitree_ros2_cpp::msg::HighState>::SharedPtr about_publisher;



   /*
   * foot_raise_height Data publisher
   * Takes HighState data and updates the msg with latest about data
   */
  void foot_raise_height_callback()
  {
    // Create an instance of the HighState message type
    auto message = unitree_ros2_cpp::msg::HighState();


    // Set latest known mode to msg
    message.foot_raise_height = udpLegged.state.footRaiseHeight;

    // publish the message created above to the topic /legged_data/status/gait_type
    foot_raise_height_publisher->publish(message);
  }

  // Declaration of private fields used for timer, publisher and counter
  rclcpp::TimerBase::SharedPtr timer_foot_raise_height;
  rclcpp::Publisher<unitree_ros2_cpp::msg::HighState>::SharedPtr foot_raise_height_publisher;


  /*
  * Motor 0 status Data publisher
  * Takes MotorStatus data and updates the msg with latest Motor 0 data
  */
  void motor_0_callback()
  {
    // Create an instance of the HighState message type
    auto message = unitree_ros2_cpp::msg::MotorState();

    // Set latest known motor status to msg
    message.mode = udpLegged.state.motorState[0].mode;       
    message.q = udpLegged.state.motorState[0].q;
    message.dq = udpLegged.state.motorState[0].dq;              
    message.ddq = udpLegged.state.motorState[0].ddq;
    message.tau_est = udpLegged.state.motorState[0].tauEst;
    message.q_raw = udpLegged.state.motorState[0].q_raw;
    message.dq_raw = udpLegged.state.motorState[0].dq_raw;
    message.ddq_raw = udpLegged.state.motorState[0].ddq_raw;
    message.temperature = udpLegged.state.motorState[0].temperature;

    // publish the message created above to the topic /legged_data/actuators/motor_0
    motor_0_publisher->publish(message);
  }
  // Declaration of private fields used for timer, publisher and counter
  rclcpp::TimerBase::SharedPtr timer_motor_0;
  rclcpp::Publisher<unitree_ros2_cpp::msg::MotorState>::SharedPtr motor_0_publisher;

  /*
  * Motor 1 status Data publisher
  * Takes MotorStatus data and updates the msg with latest Motor 1 data
  */
  void motor_1_callback()
  {
    auto message = unitree_ros2_cpp::msg::MotorState();
    message.mode = udpLegged.state.motorState[1].mode;       
    message.q = udpLegged.state.motorState[1].q;
    message.dq = udpLegged.state.motorState[1].dq;              
    message.ddq = udpLegged.state.motorState[1].ddq;
    message.tau_est = udpLegged.state.motorState[1].tauEst;
    message.q_raw = udpLegged.state.motorState[1].q_raw;
    message.dq_raw = udpLegged.state.motorState[1].dq_raw;
    message.ddq_raw = udpLegged.state.motorState[1].ddq_raw;
    message.temperature = udpLegged.state.motorState[1].temperature;

    // publish the message created above to the topic /legged_data/actuators/motor_1
    motor_1_publisher->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_motor_1;
  rclcpp::Publisher<unitree_ros2_cpp::msg::MotorState>::SharedPtr motor_1_publisher;

  /*
  * Motor 2 status Data publisher
  * Takes MotorStatus data and updates the msg with latest Motor 2 data
  */
  void motor_2_callback()
  {
    auto message = unitree_ros2_cpp::msg::MotorState();
    message.mode = udpLegged.state.motorState[2].mode;       
    message.q = udpLegged.state.motorState[2].q;
    message.dq = udpLegged.state.motorState[2].dq;              
    message.ddq = udpLegged.state.motorState[2].ddq;
    message.tau_est = udpLegged.state.motorState[2].tauEst;
    message.q_raw = udpLegged.state.motorState[2].q_raw;
    message.dq_raw = udpLegged.state.motorState[2].dq_raw;
    message.ddq_raw = udpLegged.state.motorState[2].ddq_raw;
    message.temperature = udpLegged.state.motorState[2].temperature;

    // publish the message created above to the topic /legged_data/actuators/motor_2
    motor_2_publisher->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_motor_2;
  rclcpp::Publisher<unitree_ros2_cpp::msg::MotorState>::SharedPtr motor_2_publisher;

  /*
  * Motor 3 status Data publisher
  * Takes MotorStatus data and updates the msg with latest Motor 3 data
  */
  void motor_3_callback()
  {
    auto message = unitree_ros2_cpp::msg::MotorState();
    message.mode = udpLegged.state.motorState[3].mode;       
    message.q = udpLegged.state.motorState[3].q;
    message.dq = udpLegged.state.motorState[3].dq;              
    message.ddq = udpLegged.state.motorState[3].ddq;
    message.tau_est = udpLegged.state.motorState[3].tauEst;
    message.q_raw = udpLegged.state.motorState[3].q_raw;
    message.dq_raw = udpLegged.state.motorState[3].dq_raw;
    message.ddq_raw = udpLegged.state.motorState[3].ddq_raw;
    message.temperature = udpLegged.state.motorState[3].temperature;

    // publish the message created above to the topic /legged_data/actuators/motor_3
    motor_3_publisher->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_motor_3;
  rclcpp::Publisher<unitree_ros2_cpp::msg::MotorState>::SharedPtr motor_3_publisher;

  /*
  * Motor 4 status Data publisher
  * Takes MotorStatus data and updates the msg with latest Motor 4 data
  */
  void motor_4_callback()
  {
    auto message = unitree_ros2_cpp::msg::MotorState();
    message.mode = udpLegged.state.motorState[4].mode;       
    message.q = udpLegged.state.motorState[4].q;
    message.dq = udpLegged.state.motorState[4].dq;              
    message.ddq = udpLegged.state.motorState[4].ddq;
    message.tau_est = udpLegged.state.motorState[4].tauEst;
    message.q_raw = udpLegged.state.motorState[4].q_raw;
    message.dq_raw = udpLegged.state.motorState[4].dq_raw;
    message.ddq_raw = udpLegged.state.motorState[4].ddq_raw;
    message.temperature = udpLegged.state.motorState[4].temperature;

    // publish the message created above to the topic /legged_data/actuators/motor_4
    motor_4_publisher->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_motor_4;
  rclcpp::Publisher<unitree_ros2_cpp::msg::MotorState>::SharedPtr motor_4_publisher;

  /*
  * Motor 5 status Data publisher
  * Takes MotorStatus data and updates the msg with latest Motor 5 data
  */
  void motor_5_callback()
  {
    auto message = unitree_ros2_cpp::msg::MotorState();
    message.mode = udpLegged.state.motorState[5].mode;       
    message.q = udpLegged.state.motorState[5].q;
    message.dq = udpLegged.state.motorState[5].dq;              
    message.ddq = udpLegged.state.motorState[5].ddq;
    message.tau_est = udpLegged.state.motorState[5].tauEst;
    message.q_raw = udpLegged.state.motorState[5].q_raw;
    message.dq_raw = udpLegged.state.motorState[5].dq_raw;
    message.ddq_raw = udpLegged.state.motorState[5].ddq_raw;
    message.temperature = udpLegged.state.motorState[5].temperature;

    // publish the message created above to the topic /legged_data/actuators/motor_5
    motor_5_publisher->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_motor_5;
  rclcpp::Publisher<unitree_ros2_cpp::msg::MotorState>::SharedPtr motor_5_publisher;

  /*
  * Motor 6 status Data publisher
  * Takes MotorStatus data and updates the msg with latest Motor 6 data
  */
  void motor_6_callback()
  {
    auto message = unitree_ros2_cpp::msg::MotorState();
    message.mode = udpLegged.state.motorState[6].mode;       
    message.q = udpLegged.state.motorState[6].q;
    message.dq = udpLegged.state.motorState[6].dq;              
    message.ddq = udpLegged.state.motorState[6].ddq;
    message.tau_est = udpLegged.state.motorState[6].tauEst;
    message.q_raw = udpLegged.state.motorState[6].q_raw;
    message.dq_raw = udpLegged.state.motorState[6].dq_raw;
    message.ddq_raw = udpLegged.state.motorState[6].ddq_raw;
    message.temperature = udpLegged.state.motorState[6].temperature;

    // publish the message created above to the topic /legged_data/actuators/motor_6
    motor_6_publisher->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_motor_6;
  rclcpp::Publisher<unitree_ros2_cpp::msg::MotorState>::SharedPtr motor_6_publisher;

  /*
  * Motor 7 status Data publisher
  * Takes MotorStatus data and updates the msg with latest Motor 7 data
  */
  void motor_7_callback()
  {
    auto message = unitree_ros2_cpp::msg::MotorState();
    message.mode = udpLegged.state.motorState[7].mode;       
    message.q = udpLegged.state.motorState[7].q;
    message.dq = udpLegged.state.motorState[7].dq;              
    message.ddq = udpLegged.state.motorState[7].ddq;
    message.tau_est = udpLegged.state.motorState[7].tauEst;
    message.q_raw = udpLegged.state.motorState[7].q_raw;
    message.dq_raw = udpLegged.state.motorState[7].dq_raw;
    message.ddq_raw = udpLegged.state.motorState[7].ddq_raw;
    message.temperature = udpLegged.state.motorState[7].temperature;

    // publish the message created above to the topic /legged_data/actuators/motor_7
    motor_7_publisher->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_motor_7;
  rclcpp::Publisher<unitree_ros2_cpp::msg::MotorState>::SharedPtr motor_7_publisher;

  /*
  * Motor 8 status Data publisher
  * Takes MotorStatus data and updates the msg with latest Motor 8 data
  */
  void motor_8_callback()
  {
    auto message = unitree_ros2_cpp::msg::MotorState();
    message.mode = udpLegged.state.motorState[8].mode;       
    message.q = udpLegged.state.motorState[8].q;
    message.dq = udpLegged.state.motorState[8].dq;              
    message.ddq = udpLegged.state.motorState[8].ddq;
    message.tau_est = udpLegged.state.motorState[8].tauEst;
    message.q_raw = udpLegged.state.motorState[8].q_raw;
    message.dq_raw = udpLegged.state.motorState[8].dq_raw;
    message.ddq_raw = udpLegged.state.motorState[8].ddq_raw;
    message.temperature = udpLegged.state.motorState[8].temperature;

    // publish the message created above to the topic /legged_data/actuators/motor_8
    motor_8_publisher->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_motor_8;
  rclcpp::Publisher<unitree_ros2_cpp::msg::MotorState>::SharedPtr motor_8_publisher;

  /*
  * Motor 9 status Data publisher
  * Takes MotorStatus data and updates the msg with latest Motor 9 data
  */
  void motor_9_callback()
  {
    auto message = unitree_ros2_cpp::msg::MotorState();
    message.mode = udpLegged.state.motorState[9].mode;       
    message.q = udpLegged.state.motorState[9].q;
    message.dq = udpLegged.state.motorState[9].dq;              
    message.ddq = udpLegged.state.motorState[9].ddq;
    message.tau_est = udpLegged.state.motorState[9].tauEst;
    message.q_raw = udpLegged.state.motorState[9].q_raw;
    message.dq_raw = udpLegged.state.motorState[9].dq_raw;
    message.ddq_raw = udpLegged.state.motorState[9].ddq_raw;
    message.temperature = udpLegged.state.motorState[9].temperature;

    // publish the message created above to the topic /legged_data/actuators/motor_9
    motor_9_publisher->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_motor_9;
  rclcpp::Publisher<unitree_ros2_cpp::msg::MotorState>::SharedPtr motor_9_publisher;

  /*
  * Motor 10 status Data publisher
  * Takes MotorStatus data and updates the msg with latest Motor 10 data
  */
  void motor_10_callback()
  {
    auto message = unitree_ros2_cpp::msg::MotorState();
    message.mode = udpLegged.state.motorState[10].mode;       
    message.q = udpLegged.state.motorState[10].q;
    message.dq = udpLegged.state.motorState[10].dq;              
    message.ddq = udpLegged.state.motorState[10].ddq;
    message.tau_est = udpLegged.state.motorState[10].tauEst;
    message.q_raw = udpLegged.state.motorState[10].q_raw;
    message.dq_raw = udpLegged.state.motorState[10].dq_raw;
    message.ddq_raw = udpLegged.state.motorState[10].ddq_raw;
    message.temperature = udpLegged.state.motorState[10].temperature;

    // publish the message created above to the topic /legged_data/actuators/motor_10
    motor_10_publisher->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_motor_10;
  rclcpp::Publisher<unitree_ros2_cpp::msg::MotorState>::SharedPtr motor_10_publisher;

  /*
  * Motor 11 status Data publisher
  * Takes MotorStatus data and updates the msg with latest Motor 11 data
  */
  void motor_11_callback()
  {
    auto message = unitree_ros2_cpp::msg::MotorState();
    message.mode = udpLegged.state.motorState[11].mode;       
    message.q = udpLegged.state.motorState[11].q;
    message.dq = udpLegged.state.motorState[11].dq;              
    message.ddq = udpLegged.state.motorState[11].ddq;
    message.tau_est = udpLegged.state.motorState[11].tauEst;
    message.q_raw = udpLegged.state.motorState[11].q_raw;
    message.dq_raw = udpLegged.state.motorState[11].dq_raw;
    message.ddq_raw = udpLegged.state.motorState[11].ddq_raw;
    message.temperature = udpLegged.state.motorState[11].temperature;

    // publish the message created above to the topic /legged_data/actuators/motor_11
    motor_11_publisher->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_motor_11;
  rclcpp::Publisher<unitree_ros2_cpp::msg::MotorState>::SharedPtr motor_11_publisher;


  size_t count_;
};











// This is a break in the code to allow for easy navigation












// Declaring a new class as a subclass of the ROS 2 Node class
class LeggedControl : public rclcpp::Node
{
public:
  // Constructor specifying node name for superclass
  LeggedControl()
      : Node("LeggedDataTX")
  {

    // Create the instance of the twist subscriber that will receive messages
    // of type geometry_msgs/msg/Twist published to the topic "/cmd_vel"
    // a queue length of 10 is specified here and a reference is given
    subscription_twist = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10, std::bind(&LeggedControl::twist_callback, this, std::placeholders::_1));


    // Create the instance of the mode subscriber that will receive messages
    // of type unitree_ros2_cpp/msg/HighCmd published to the topic "/cmd_mode"
    // a queue length of 10 is specified here and a reference is given
    subscription_mode = this->create_subscription<unitree_ros2_cpp::msg::HighCmd>(
        "cmd_mode", 10, std::bind(&LeggedControl::cmdMode_callback, this, std::placeholders::_1));


    // Create the instance of the cmd_pos subscriber that will receive messages
    // of type geometry_msgs/msg/Twist published to the topic "/cmd_mode"
    // Only in Linear X & Y plane
    // a queue length of 10 is specified here and a reference is given
    subscription_twist_pos = this->create_subscription<unitree_ros2_cpp::msg::HighCmd>(
      "cmd_pos", 10, std::bind(&LeggedControl::cmdPos_callback, this, std::placeholders::_1));
    


    // Create the instance of the cmd_foot_raise_height subscriber that will receive messages
    // of type unitree_ros2_cpp/msg/HighCmd published to the topic "/cmd_foot_raise_height"
    // a queue length of 10 is specified here and a reference is given
    subscription_foot_raise_height = this->create_subscription<unitree_ros2_cpp::msg::HighCmd>(
      "cmd_foot_raise_height", 10, std::bind(&LeggedControl::cmdWalkHeight_callback, this, std::placeholders::_1));

      // Create the instance of the cmd_euler subscriber that will receive messages
    // of type unitree_ros2_cpp/msg/HighCmd published to the topic "/cmd_euler"
    // a queue length of 10 is specified here and a reference is given
    subscription_euler = this->create_subscription<unitree_ros2_cpp::msg::HighCmd>(
      "cmd_euler", 10, std::bind(&LeggedControl::cmdEuler_callback, this, std::placeholders::_1));


/*

CmdMotor feature currently disabled to do risks of damaging the robot. To be dicussed with Tomos Fearn

*/


    // // Create the instance of the cmd_actuators subscriber that will receive messages
    // // of type unitree_ros2_cpp/msg/MotorCmd published to the topic "/cmd_actuators/motor_0"
    // // a queue length of 10 is specified here and a reference is given
    // subscription_motor_0 = this->create_subscription<unitree_ros2_cpp::msg::MotorCmd>(
    //   "cmd_actuators/motor_0", 10, std::bind(&LeggedControl::cmdMotor_0_callback, this, std::placeholders::_1));


    // // Create the instance of the cmd_actuators subscriber that will receive messages
    // // of type unitree_ros2_cpp/msg/MotorCmd published to the topic "/cmd_actuators/motor_1"
    // // a queue length of 10 is specified here and a reference is given
    // subscription_motor_1 = this->create_subscription<unitree_ros2_cpp::msg::MotorCmd>(
    //   "cmd_actuators/motor_1", 10, std::bind(&LeggedControl::cmdMotor_1_callback, this, std::placeholders::_1));


    // // Create the instance of the cmd_actuators subscriber that will receive messages
    // // of type unitree_ros2_cpp/msg/MotorCmd published to the topic "/cmd_actuators/motor_2"
    // // a queue length of 10 is specified here and a reference is given
    // subscription_motor_2 = this->create_subscription<unitree_ros2_cpp::msg::MotorCmd>(
    //   "cmd_actuators/motor_2", 10, std::bind(&LeggedControl::cmdMotor_2_callback, this, std::placeholders::_1));


    // // Create the instance of the cmd_actuators subscriber that will receive messages
    // // of type unitree_ros2_cpp/msg/MotorCmd published to the topic "/cmd_actuators/motor_3"
    // // a queue length of 10 is specified here and a reference is given
    // subscription_motor_3 = this->create_subscription<unitree_ros2_cpp::msg::MotorCmd>(
    //   "cmd_actuators/motor_3", 10, std::bind(&LeggedControl::cmdMotor_3_callback, this, std::placeholders::_1));


    // // Create the instance of the cmd_actuators subscriber that will receive messages
    // // of type unitree_ros2_cpp/msg/MotorCmd published to the topic "/cmd_actuators/motor_4"
    // // a queue length of 10 is specified here and a reference is given
    // subscription_motor_4 = this->create_subscription<unitree_ros2_cpp::msg::MotorCmd>(
    //   "cmd_actuators/motor_4", 10, std::bind(&LeggedControl::cmdMotor_4_callback, this, std::placeholders::_1));

      
    // // Create the instance of the cmd_actuators subscriber that will receive messages
    // // of type unitree_ros2_cpp/msg/MotorCmd published to the topic "/cmd_actuators/motor_5"
    // // a queue length of 10 is specified here and a reference is given
    // subscription_motor_5 = this->create_subscription<unitree_ros2_cpp::msg::MotorCmd>(
    //   "cmd_actuators/motor_5", 10, std::bind(&LeggedControl::cmdMotor_5_callback, this, std::placeholders::_1));


    // // Create the instance of the cmd_actuators subscriber that will receive messages
    // // of type unitree_ros2_cpp/msg/MotorCmd published to the topic "/cmd_actuators/motor_6"
    // // a queue length of 10 is specified here and a reference is given
    // subscription_motor_6 = this->create_subscription<unitree_ros2_cpp::msg::MotorCmd>(
    //   "cmd_actuators/motor_6", 10, std::bind(&LeggedControl::cmdMotor_6_callback, this, std::placeholders::_1));


    // // Create the instance of the cmd_actuators subscriber that will receive messages
    // // of type unitree_ros2_cpp/msg/MotorCmd published to the topic "/cmd_actuators/motor_7"
    // // a queue length of 10 is specified here and a reference is given
    // subscription_motor_7 = this->create_subscription<unitree_ros2_cpp::msg::MotorCmd>(
    //   "cmd_actuators/motor_7", 10, std::bind(&LeggedControl::cmdMotor_7_callback, this, std::placeholders::_1));


    // // Create the instance of the cmd_actuators subscriber that will receive messages
    // // of type unitree_ros2_cpp/msg/MotorCmd published to the topic "/cmd_actuators/motor_8"
    // // a queue length of 10 is specified here and a reference is given
    // subscription_motor_8 = this->create_subscription<unitree_ros2_cpp::msg::MotorCmd>(
    //   "cmd_actuators/motor_8", 10, std::bind(&LeggedControl::cmdMotor_8_callback, this, std::placeholders::_1));


    // // Create the instance of the cmd_actuators subscriber that will receive messages
    // // of type unitree_ros2_cpp/msg/MotorCmd published to the topic "/cmd_actuators/motor_9"
    // // a queue length of 10 is specified here and a reference is given
    // subscription_motor_9 = this->create_subscription<unitree_ros2_cpp::msg::MotorCmd>(
    //   "cmd_actuators/motor_9", 10, std::bind(&LeggedControl::cmdMotor_9_callback, this, std::placeholders::_1));


    // // Create the instance of the cmd_actuators subscriber that will receive messages
    // // of type unitree_ros2_cpp/msg/MotorCmd published to the topic "/cmd_actuators/motor_10"
    // // a queue length of 10 is specified here and a reference is given
    // subscription_motor_10 = this->create_subscription<unitree_ros2_cpp::msg::MotorCmd>(
    //   "cmd_actuators/motor_10", 10, std::bind(&LeggedControl::cmdMotor_10_callback, this, std::placeholders::_1));


    // // Create the instance of the cmd_actuators subscriber that will receive messages
    // // of type unitree_ros2_cpp/msg/MotorCmd published to the topic "/cmd_actuators/motor_11"
    // // a queue length of 10 is specified here and a reference is given
    // subscription_motor_11 = this->create_subscription<unitree_ros2_cpp::msg::MotorCmd>(
    //   "cmd_actuators/motor_11", 10, std::bind(&LeggedControl::cmdMotor_11_callback, this, std::placeholders::_1));

  }

private:
  /*
   * twist Data subscriber
   * Takes twist msg data and sends it via UDP to the robot
   */
  void twist_callback(const geometry_msgs::msg::Twist &msg)
  {
    // Logger used to print details of the message received (printed to console).
    RCLCPP_INFO(this->get_logger(), "I heard: msg.data=%f  and  %f", msg.linear.x, msg.angular.z);
    udpLegged.cmd.velocity[0] = msg.linear.x; // Forward / backward motion
    udpLegged.cmd.velocity[1] = msg.linear.y; // Left / right motion
    udpLegged.cmd.bodyHeight = msg.linear.z; // Vertical / up & down

    

    udpLegged.cmd.yawSpeed = msg.angular.z; // Rotation in rad/s


    udpLegged.udp.SetSend(udpLegged.cmd);
    udpLegged.UDPSend();
  }
  // Declaration of private fields used for subscriber
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_twist;



  /*
   * mode Data subscriber
   * Takes HighCmd msg data and sends it via UDP to the robot
   */
  void cmdMode_callback(const unitree_ros2_cpp::msg::HighCmd &msg)
  {

    udpLegged.cmd.mode = msg.mode;

    udpLegged.udp.SetSend(udpLegged.cmd);
    udpLegged.UDPSend();
  }
  // Declaration of private fields used for subscriber
  rclcpp::Subscription<unitree_ros2_cpp::msg::HighCmd>::SharedPtr subscription_mode;



  /*
   * cmdPos Data subscriber
   * Takes HighCmd msg data and sends it via UDP to the robot
   */
  void cmdPos_callback(const unitree_ros2_cpp::msg::HighCmd &msg)
  {

    udpLegged.cmd.position[0] = msg.position[0];
    udpLegged.cmd.position[1] = msg.position[1];

    udpLegged.udp.SetSend(udpLegged.cmd);
    udpLegged.UDPSend();
  }
  // Declaration of private fields used for subscriber
  rclcpp::Subscription<unitree_ros2_cpp::msg::HighCmd>::SharedPtr subscription_twist_pos;



  /*
   * Walk height Data subscriber
   * Takes HighCmd msg data and sends it via UDP to the robot
   */
  void cmdWalkHeight_callback(const unitree_ros2_cpp::msg::HighCmd &msg)
  {

    udpLegged.cmd.footRaiseHeight = msg.foot_raise_height;

    udpLegged.udp.SetSend(udpLegged.cmd);
    udpLegged.UDPSend();
  }
  // Declaration of private fields used for subscriber
  rclcpp::Subscription<unitree_ros2_cpp::msg::HighCmd>::SharedPtr subscription_foot_raise_height;


  /*
   * cmdEuler Data subscriber
   * Takes HighCmd msg data and sends it via UDP to the robot
   */
  void cmdEuler_callback(const unitree_ros2_cpp::msg::HighCmd &msg)
  {

    udpLegged.cmd.euler[0] = msg.euler[0];
    udpLegged.cmd.euler[1] = msg.euler[1];
    udpLegged.cmd.euler[2] = msg.euler[2];

    udpLegged.udp.SetSend(udpLegged.cmd);
    udpLegged.UDPSend();
  }
  // Declaration of private fields used for subscriber
  rclcpp::Subscription<unitree_ros2_cpp::msg::HighCmd>::SharedPtr subscription_euler;


/*

CmdMotor feature currently disabled to do risks of damaging the robot. To be dicussed with Tomos Fearn

*/



  // /*
  // * MotorCmd Data subscriber
  // * Takes MotorCmd msg data and sends it via UDP to the robot
  // */
  // void cmdMotor_0_callback(const unitree_ros2_cpp::msg::MotorCmd &msg)
  // {
  //   udpLegged.cmd.motorCmd[0].mode = msg.mode;
  //   udpLegged.cmd.motorCmd[0].q = msg.q;
  //   udpLegged.cmd.motorCmd[0].dq = msg.dq;
  //   udpLegged.cmd.motorCmd[0].tau = msg.tau;
  //   udpLegged.cmd.motorCmd[0].kp = msg.kp;
  //   udpLegged.cmd.motorCmd[0].kd = msg.kd;

  //   udpLegged.udp.SetSend(udpLegged.cmd);
  //   udpLegged.UDPSend();
  // }
  // // Declaration of private fields used for subscriber
  // rclcpp::Subscription<unitree_ros2_cpp::msg::MotorCmd>::SharedPtr subscription_motor_0;



  // /*
  // * MotorCmd Data subscriber
  // * Takes MotorCmd msg data and sends it via UDP to the robot
  // */
  // void cmdMotor_1_callback(const unitree_ros2_cpp::msg::MotorCmd &msg)
  // {
  //   udpLegged.cmd.motorCmd[1].mode = msg.mode;
  //   udpLegged.cmd.motorCmd[1].q = msg.q;
  //   udpLegged.cmd.motorCmd[1].dq = msg.dq;
  //   udpLegged.cmd.motorCmd[1].tau = msg.tau;
  //   udpLegged.cmd.motorCmd[1].kp = msg.kp;
  //   udpLegged.cmd.motorCmd[1].kd = msg.kd;

  //   udpLegged.udp.SetSend(udpLegged.cmd);
  //   udpLegged.UDPSend();
  // }
  // // Declaration of private fields used for subscriber
  // rclcpp::Subscription<unitree_ros2_cpp::msg::MotorCmd>::SharedPtr subscription_motor_1;



  // /*
  // * MotorCmd Data subscriber
  // * Takes MotorCmd msg data and sends it via UDP to the robot
  // */
  // void cmdMotor_2_callback(const unitree_ros2_cpp::msg::MotorCmd &msg)
  // {
  //   udpLegged.cmd.motorCmd[2].mode = msg.mode;
  //   udpLegged.cmd.motorCmd[2].q = msg.q;
  //   udpLegged.cmd.motorCmd[2].dq = msg.dq;
  //   udpLegged.cmd.motorCmd[2].tau = msg.tau;
  //   udpLegged.cmd.motorCmd[2].kp = msg.kp;
  //   udpLegged.cmd.motorCmd[2].kd = msg.kd;

  //   udpLegged.udp.SetSend(udpLegged.cmd);
  //   udpLegged.UDPSend();
  // }
  // // Declaration of private fields used for subscriber
  // rclcpp::Subscription<unitree_ros2_cpp::msg::MotorCmd>::SharedPtr subscription_motor_2;



  // /*
  // * MotorCmd Data subscriber
  // * Takes MotorCmd msg data and sends it via UDP to the robot
  // */
  // void cmdMotor_3_callback(const unitree_ros2_cpp::msg::MotorCmd &msg)
  // {
  //   udpLegged.cmd.motorCmd[3].mode = msg.mode;
  //   udpLegged.cmd.motorCmd[3].q = msg.q;
  //   udpLegged.cmd.motorCmd[3].dq = msg.dq;
  //   udpLegged.cmd.motorCmd[3].tau = msg.tau;
  //   udpLegged.cmd.motorCmd[3].kp = msg.kp;
  //   udpLegged.cmd.motorCmd[3].kd = msg.kd;

  //   udpLegged.udp.SetSend(udpLegged.cmd);
  //   udpLegged.UDPSend();
  // }
  // // Declaration of private fields used for subscriber
  // rclcpp::Subscription<unitree_ros2_cpp::msg::MotorCmd>::SharedPtr subscription_motor_3;



  // /*
  // * MotorCmd Data subscriber
  // * Takes MotorCmd msg data and sends it via UDP to the robot
  // */
  // void cmdMotor_4_callback(const unitree_ros2_cpp::msg::MotorCmd &msg)
  // {
  //   udpLegged.cmd.motorCmd[4].mode = msg.mode;
  //   udpLegged.cmd.motorCmd[4].q = msg.q;
  //   udpLegged.cmd.motorCmd[4].dq = msg.dq;
  //   udpLegged.cmd.motorCmd[4].tau = msg.tau;
  //   udpLegged.cmd.motorCmd[4].kp = msg.kp;
  //   udpLegged.cmd.motorCmd[4].kd = msg.kd;

  //   udpLegged.udp.SetSend(udpLegged.cmd);
  //   udpLegged.UDPSend();
  // }
  // // Declaration of private fields used for subscriber
  // rclcpp::Subscription<unitree_ros2_cpp::msg::MotorCmd>::SharedPtr subscription_motor_4;



  // /*
  // * MotorCmd Data subscriber
  // * Takes MotorCmd msg data and sends it via UDP to the robot
  // */
  // void cmdMotor_5_callback(const unitree_ros2_cpp::msg::MotorCmd &msg)
  // {
  //   udpLegged.cmd.motorCmd[5].mode = msg.mode;
  //   udpLegged.cmd.motorCmd[5].q = msg.q;
  //   udpLegged.cmd.motorCmd[5].dq = msg.dq;
  //   udpLegged.cmd.motorCmd[5].tau = msg.tau;
  //   udpLegged.cmd.motorCmd[5].kp = msg.kp;
  //   udpLegged.cmd.motorCmd[5].kd = msg.kd;

  //   udpLegged.udp.SetSend(udpLegged.cmd);
  //   udpLegged.UDPSend();
  // }
  // // Declaration of private fields used for subscriber
  // rclcpp::Subscription<unitree_ros2_cpp::msg::MotorCmd>::SharedPtr subscription_motor_5;



  // /*
  // * MotorCmd Data subscriber
  // * Takes MotorCmd msg data and sends it via UDP to the robot
  // */
  // void cmdMotor_6_callback(const unitree_ros2_cpp::msg::MotorCmd &msg)
  // {
  //   udpLegged.cmd.motorCmd[6].mode = msg.mode;
  //   udpLegged.cmd.motorCmd[6].q = msg.q;
  //   udpLegged.cmd.motorCmd[6].dq = msg.dq;
  //   udpLegged.cmd.motorCmd[6].tau = msg.tau;
  //   udpLegged.cmd.motorCmd[6].kp = msg.kp;
  //   udpLegged.cmd.motorCmd[6].kd = msg.kd;

  //   udpLegged.udp.SetSend(udpLegged.cmd);
  //   udpLegged.UDPSend();
  // }
  // // Declaration of private fields used for subscriber
  // rclcpp::Subscription<unitree_ros2_cpp::msg::MotorCmd>::SharedPtr subscription_motor_6;



  // /*
  // * MotorCmd Data subscriber
  // * Takes MotorCmd msg data and sends it via UDP to the robot
  // */
  // void cmdMotor_7_callback(const unitree_ros2_cpp::msg::MotorCmd &msg)
  // {
  //   udpLegged.cmd.motorCmd[7].mode = msg.mode;
  //   udpLegged.cmd.motorCmd[7].q = msg.q;
  //   udpLegged.cmd.motorCmd[7].dq = msg.dq;
  //   udpLegged.cmd.motorCmd[7].tau = msg.tau;
  //   udpLegged.cmd.motorCmd[7].kp = msg.kp;
  //   udpLegged.cmd.motorCmd[7].kd = msg.kd;

  //   udpLegged.udp.SetSend(udpLegged.cmd);
  //   udpLegged.UDPSend();
  // }
  // // Declaration of private fields used for subscriber
  // rclcpp::Subscription<unitree_ros2_cpp::msg::MotorCmd>::SharedPtr subscription_motor_7;



  // /*
  // * MotorCmd Data subscriber
  // * Takes MotorCmd msg data and sends it via UDP to the robot
  // */
  // void cmdMotor_8_callback(const unitree_ros2_cpp::msg::MotorCmd &msg)
  // {
  //   udpLegged.cmd.motorCmd[8].mode = msg.mode;
  //   udpLegged.cmd.motorCmd[8].q = msg.q;
  //   udpLegged.cmd.motorCmd[8].dq = msg.dq;
  //   udpLegged.cmd.motorCmd[8].tau = msg.tau;
  //   udpLegged.cmd.motorCmd[8].kp = msg.kp;
  //   udpLegged.cmd.motorCmd[8].kd = msg.kd;

  //   udpLegged.udp.SetSend(udpLegged.cmd);
  //   udpLegged.UDPSend();
  // }
  // // Declaration of private fields used for subscriber
  // rclcpp::Subscription<unitree_ros2_cpp::msg::MotorCmd>::SharedPtr subscription_motor_8;



  // /*
  // * MotorCmd Data subscriber
  // * Takes MotorCmd msg data and sends it via UDP to the robot
  // */
  // void cmdMotor_9_callback(const unitree_ros2_cpp::msg::MotorCmd &msg)
  // {
  //   udpLegged.cmd.motorCmd[9].mode = msg.mode;
  //   udpLegged.cmd.motorCmd[9].q = msg.q;
  //   udpLegged.cmd.motorCmd[9].dq = msg.dq;
  //   udpLegged.cmd.motorCmd[9].tau = msg.tau;
  //   udpLegged.cmd.motorCmd[9].kp = msg.kp;
  //   udpLegged.cmd.motorCmd[9].kd = msg.kd;

  //   udpLegged.udp.SetSend(udpLegged.cmd);
  //   udpLegged.UDPSend();
  // }
  // // Declaration of private fields used for subscriber
  // rclcpp::Subscription<unitree_ros2_cpp::msg::MotorCmd>::SharedPtr subscription_motor_9;



  // /*
  // * MotorCmd Data subscriber
  // * Takes MotorCmd msg data and sends it via UDP to the robot
  // */
  // void cmdMotor_10_callback(const unitree_ros2_cpp::msg::MotorCmd &msg)
  // {
  //   udpLegged.cmd.motorCmd[10].mode = msg.mode;
  //   udpLegged.cmd.motorCmd[10].q = msg.q;
  //   udpLegged.cmd.motorCmd[10].dq = msg.dq;
  //   udpLegged.cmd.motorCmd[10].tau = msg.tau;
  //   udpLegged.cmd.motorCmd[10].kp = msg.kp;
  //   udpLegged.cmd.motorCmd[10].kd = msg.kd;

  //   udpLegged.udp.SetSend(udpLegged.cmd);
  //   udpLegged.UDPSend();
  // }
  // // Declaration of private fields used for subscriber
  // rclcpp::Subscription<unitree_ros2_cpp::msg::MotorCmd>::SharedPtr subscription_motor_10;



  // /*
  // * MotorCmd Data subscriber
  // * Takes MotorCmd msg data and sends it via UDP to the robot
  // */
  // void cmdMotor_11_callback(const unitree_ros2_cpp::msg::MotorCmd &msg)
  // {
  //   udpLegged.cmd. .motorCmd[11].mode = msg.mode;
  //   udpLegged.cmd.motorCmd[11].q = msg.q;
  //   udpLegged.cmd.motorCmd[11].dq = msg.dq;
  //   udpLegged.cmd.motorCmd[11].tau = msg.tau;
  //   udpLegged.cmd.motorCmd[11].kp = msg.kp;
  //   udpLegged.cmd.motorCmd[11].kd = msg.kd;

  //   udpLegged.udp.SetSend(udpLegged.cmd);
  //   udpLegged.UDPSend();
  // }
  // // Declaration of private fields used for subscriber
  // rclcpp::Subscription<unitree_ros2_cpp::msg::MotorCmd>::SharedPtr subscription_motor_11;

};

// Main method defining entry point for program
int main(int argc, char *argv[])
{
  // Initialise ROS 2 for this node
  rclcpp::init(argc, argv);

  /*
   * Start node as multithread process
   * Prevents any single callback blocking another. (Helps improve performance)
   * Source: https://docs.ros.org/en/humble/Concepts/Intermediate/About-Executors.html
   */
  rclcpp::executors::MultiThreadedExecutor executor;
  auto leggedDataNode = std::make_shared<LeggedDataRX>();
  auto leggedControlNode = std::make_shared<LeggedControl>();
  auto udpServer = std::make_shared<UDPLoopService>();

  executor.add_node(udpServer);
  executor.add_node(leggedDataNode);
  executor.add_node(leggedControlNode);
  executor.spin();

  // When the node is terminated, shut down ROS 2 for this node
  rclcpp::shutdown();
  return 0;
}
