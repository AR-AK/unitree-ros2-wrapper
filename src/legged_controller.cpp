// Copyright 2023 Open Source Robotics Foundation, Inc.
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

/*
 * Production Grade Unitree Go1 Controller for ROS 2
 * 
 * Improvements:
 * - Thread-safe memory sharing using Mutex.
 * - Centralized UDP handling.
 * - ROS Parameters for IP and Port configuration via Launch file.
 * - Consolidated timers for CPU efficiency.
 */

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <mutex>
#include <vector>
#include <cmath>

// ROS 2 Includes
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/temperature.hpp"
#include "nav_msgs/msg/odometry.hpp"

// Unitree ROS 2 Messages
#include "unitree_ros2_cpp/msg/bms_state.hpp"
#include "unitree_ros2_cpp/msg/high_state.hpp"
#include "unitree_ros2_cpp/msg/high_cmd.hpp"
#include "unitree_ros2_cpp/msg/motor_state.hpp"

// Unitree SDK
#include "unitree_legged_sdk/unitree_legged_sdk.h"

using namespace UNITREE_LEGGED_SDK;
using namespace std::chrono_literals;

// =================================================================================================
// CLASS: LeggedRobotInterface
// Description: A thread-safe wrapper for the Unitree SDK. 
// =================================================================================================
class LeggedRobotInterface
{
public:
  // Constructor now accepts ports as well
  LeggedRobotInterface(const std::string& robot_ip, uint16_t local_port, uint16_t remote_port) 
    : safe(LeggedType::Go1),
      udp(HIGHLEVEL, local_port, robot_ip.c_str(), remote_port)
  {
    udp.InitCmdData(cmd);
    // Initialize default safe commands
    cmd.mode = 0;
    cmd.gaitType = 0;
    cmd.speedLevel = 0;
    cmd.footRaiseHeight = 0.08;
    cmd.bodyHeight = 0.28;
    cmd.euler[0] = 0; cmd.euler[1] = 0; cmd.euler[2] = 0;
    cmd.velocity[0] = 0; cmd.velocity[1] = 0;
    cmd.yawSpeed = 0;
  }

  // --- UDP Communication Methods ---
  
  void udp_send() {
    std::lock_guard<std::mutex> lock(data_mutex_);
    udp.SetSend(cmd);
    udp.Send();
  }

  void udp_recv() {
    udp.Recv(); 
    
    std::lock_guard<std::mutex> lock(data_mutex_);
    udp.GetRecv(state);
  }

  // --- Thread-Safe Getters ---

  HighState get_state() {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return state;
  }

  // --- Thread-Safe Setters ---

  void set_velocity(float x, float y, float yaw, float body_h) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    cmd.velocity[0] = x;
    cmd.velocity[1] = y;
    cmd.yawSpeed = yaw;
    if(std::abs(body_h) > 0.001) cmd.bodyHeight = body_h;
  }

  void set_mode(uint8_t mode) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    cmd.mode = mode;
  }

  void set_gait_type(uint8_t gait) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    cmd.gaitType = gait;
  }

  void set_position(float x, float y) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    cmd.position[0] = x;
    cmd.position[1] = y;
  }

  void set_foot_raise_height(float height) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    cmd.footRaiseHeight = height;
  }

  void set_euler(float r, float p, float y) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    cmd.euler[0] = r;
    cmd.euler[1] = p;
    cmd.euler[2] = y;
  }

private:
  Safety safe;
  UDP udp;
  HighCmd cmd = {};
  HighState state = {};
  std::mutex data_mutex_; 
};


// =================================================================================================
// NODE: LeggedUDPLoop
// Description: Runs the High-Frequency loop to sync data with the robot.
// =================================================================================================
class LeggedUDPLoop : public rclcpp::Node
{
public:
  LeggedUDPLoop(std::shared_ptr<LeggedRobotInterface> interface)
      : Node("legged_udp_loop"), interface_(interface)
  {
    timer_udp_ = this->create_wall_timer(
        2ms, std::bind(&LeggedUDPLoop::udp_callback, this));
    
    RCLCPP_INFO(this->get_logger(), "Legged UDP Loop started.");
  }

private:
  void udp_callback()
  {
    interface_->udp_send();
    interface_->udp_recv();
  }

  std::shared_ptr<LeggedRobotInterface> interface_;
  rclcpp::TimerBase::SharedPtr timer_udp_;
};


// =================================================================================================
// NODE: LeggedDataRX (Publisher)
// Description: Reads data from the Interface and publishes to ROS topics.
// =================================================================================================
class LeggedDataRX : public rclcpp::Node
{
public:
  LeggedDataRX(std::shared_ptr<LeggedRobotInterface> interface)
      : Node("legged_data_rx"), interface_(interface)
  {
    pub_bms_ = this->create_publisher<unitree_ros2_cpp::msg::BmsState>("legged_data/sensors/bms", 10);
    pub_foot_force_ = this->create_publisher<unitree_ros2_cpp::msg::HighState>("legged_data/sensors/foot_force", 10);
    pub_imu_ = this->create_publisher<sensor_msgs::msg::Imu>("legged_data/sensors/imu", 10);
    pub_mode_ = this->create_publisher<unitree_ros2_cpp::msg::HighState>("legged_data/status/mode", 10);
    pub_temp_ = this->create_publisher<sensor_msgs::msg::Temperature>("legged_data/sensors/system_temperature", 10);
    pub_odom_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    pub_gait_ = this->create_publisher<unitree_ros2_cpp::msg::HighState>("legged_data/status/gait_type", 10);
    pub_about_ = this->create_publisher<unitree_ros2_cpp::msg::HighState>("legged_data/status/about_go1", 10);
    pub_foot_raise_ = this->create_publisher<unitree_ros2_cpp::msg::HighState>("legged_data/status/foot_raise_height", 10);

    for(int i=0; i<12; ++i) {
        std::string topic = "legged_data/actuators/motor_" + std::to_string(i);
        pub_motors_[i] = this->create_publisher<unitree_ros2_cpp::msg::MotorState>(topic, 10);
    }

    timer_slow_ = this->create_wall_timer(1000ms, std::bind(&LeggedDataRX::slow_callback, this));
    timer_medium_ = this->create_wall_timer(100ms, std::bind(&LeggedDataRX::medium_callback, this));
    timer_fast_ = this->create_wall_timer(20ms, std::bind(&LeggedDataRX::fast_callback, this));

    RCLCPP_INFO(this->get_logger(), "Legged Data RX (Publisher) started.");
  }

private:
  void slow_callback()
  {
    auto state = interface_->get_state();
    auto bms_msg = unitree_ros2_cpp::msg::BmsState();
    bms_msg.soc = state.bms.SOC;
    bms_msg.current = state.bms.current;
    for(int i=0; i<10; i++) bms_msg.cell_vol[i] = state.bms.cell_vol[i];
    bms_msg.version_h = state.bms.version_h;
    bms_msg.bms_status = state.bms.bms_status;
    bms_msg.cycle = state.bms.cycle;
    bms_msg.bq_ntc = state.bms.BQ_NTC;
    bms_msg.mcu_ntc = state.bms.MCU_NTC;
    
    if (bms_msg.soc == 0 && state.bms.version_h != 0) {
       RCLCPP_WARN_ONCE(this->get_logger(), "BMS SOC is 0. Check Battery Firmware.");
    }
    pub_bms_->publish(bms_msg);

    auto temp_msg = sensor_msgs::msg::Temperature();
    temp_msg.temperature = state.imu.temperature;
    pub_temp_->publish(temp_msg);

    auto about_msg = unitree_ros2_cpp::msg::HighState();
    about_msg.version = state.version;
    about_msg.bandwidth = state.bandWidth;
    pub_about_->publish(about_msg);
  }

  void medium_callback()
  {
    auto state = interface_->get_state();
    auto force_msg = unitree_ros2_cpp::msg::HighState();
    for(int i=0; i<4; i++) force_msg.foot_force[i] = state.footForce[i];
    pub_foot_force_->publish(force_msg);

    auto mode_msg = unitree_ros2_cpp::msg::HighState();
    mode_msg.mode = state.mode;
    pub_mode_->publish(mode_msg);

    auto gait_msg = unitree_ros2_cpp::msg::HighState();
    gait_msg.gait_type = state.gaitType;
    pub_gait_->publish(gait_msg);

    auto height_msg = unitree_ros2_cpp::msg::HighState();
    height_msg.foot_raise_height = state.footRaiseHeight;
    pub_foot_raise_->publish(height_msg);
  }

  void fast_callback()
  {
    auto state = interface_->get_state();
    auto current_time = this->get_clock()->now();

    auto imu_msg = sensor_msgs::msg::Imu();
    imu_msg.header.stamp = current_time;
    imu_msg.header.frame_id = "imu_link";
    imu_msg.orientation.w = state.imu.quaternion[0];
    imu_msg.orientation.x = state.imu.quaternion[1];
    imu_msg.orientation.y = state.imu.quaternion[2];
    imu_msg.orientation.z = state.imu.quaternion[3];
    imu_msg.angular_velocity.x = state.imu.gyroscope[0];
    imu_msg.angular_velocity.y = state.imu.gyroscope[1];
    imu_msg.angular_velocity.z = state.imu.gyroscope[2];
    imu_msg.linear_acceleration.x = state.imu.accelerometer[0];
    imu_msg.linear_acceleration.y = state.imu.accelerometer[1];
    imu_msg.linear_acceleration.z = state.imu.accelerometer[2];
    pub_imu_->publish(imu_msg);

    auto odom_msg = nav_msgs::msg::Odometry();
    odom_msg.header.stamp = current_time;
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_link";

    odom_msg.twist.twist.linear.x = state.velocity[0];
    odom_msg.twist.twist.linear.y = state.velocity[1];
    odom_msg.twist.twist.linear.z = state.velocity[2];
    odom_msg.twist.twist.angular.z = state.yawSpeed;

    odom_msg.pose.pose.orientation = imu_msg.orientation;
    odom_msg.pose.pose.position.x = state.position[0];
    odom_msg.pose.pose.position.y = state.position[1];
    odom_msg.pose.pose.position.z = state.position[2];
    pub_odom_->publish(odom_msg);

    for(int i=0; i<12; i++) {
        auto m_msg = unitree_ros2_cpp::msg::MotorState();
        m_msg.mode = state.motorState[i].mode;
        m_msg.q = state.motorState[i].q;
        m_msg.dq = state.motorState[i].dq;
        m_msg.ddq = state.motorState[i].ddq;
        m_msg.tau_est = state.motorState[i].tauEst;
        m_msg.q_raw = state.motorState[i].q_raw;
        m_msg.dq_raw = state.motorState[i].dq_raw;
        m_msg.ddq_raw = state.motorState[i].ddq_raw;
        m_msg.temperature = state.motorState[i].temperature;
        pub_motors_[i]->publish(m_msg);
    }
  }

  std::shared_ptr<LeggedRobotInterface> interface_;
  
  rclcpp::Publisher<unitree_ros2_cpp::msg::BmsState>::SharedPtr pub_bms_;
  rclcpp::Publisher<unitree_ros2_cpp::msg::HighState>::SharedPtr pub_foot_force_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_;
  rclcpp::Publisher<unitree_ros2_cpp::msg::HighState>::SharedPtr pub_mode_;
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr pub_temp_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
  rclcpp::Publisher<unitree_ros2_cpp::msg::HighState>::SharedPtr pub_gait_;
  rclcpp::Publisher<unitree_ros2_cpp::msg::HighState>::SharedPtr pub_about_;
  rclcpp::Publisher<unitree_ros2_cpp::msg::HighState>::SharedPtr pub_foot_raise_;
  rclcpp::Publisher<unitree_ros2_cpp::msg::MotorState>::SharedPtr pub_motors_[12];

  rclcpp::TimerBase::SharedPtr timer_slow_;
  rclcpp::TimerBase::SharedPtr timer_medium_;
  rclcpp::TimerBase::SharedPtr timer_fast_;
};


// =================================================================================================
// NODE: LeggedControl (Subscriber)
// Description: Listens to ROS commands and updates the Interface.
// =================================================================================================
class LeggedControl : public rclcpp::Node
{
public:
  LeggedControl(std::shared_ptr<LeggedRobotInterface> interface)
      : Node("legged_data_tx"), interface_(interface)
  {
    sub_twist_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10, std::bind(&LeggedControl::twist_callback, this, std::placeholders::_1));

    sub_mode_ = this->create_subscription<unitree_ros2_cpp::msg::HighCmd>(
        "cmd_mode", 10, std::bind(&LeggedControl::mode_callback, this, std::placeholders::_1));

    sub_pos_ = this->create_subscription<unitree_ros2_cpp::msg::HighCmd>(
        "cmd_pos", 10, std::bind(&LeggedControl::pos_callback, this, std::placeholders::_1));

    sub_height_ = this->create_subscription<unitree_ros2_cpp::msg::HighCmd>(
        "cmd_foot_raise_height", 10, std::bind(&LeggedControl::height_callback, this, std::placeholders::_1));

    sub_euler_ = this->create_subscription<unitree_ros2_cpp::msg::HighCmd>(
        "cmd_euler", 10, std::bind(&LeggedControl::euler_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Legged Control (Subscriber) started.");
  }

private:
  void twist_callback(const geometry_msgs::msg::Twist &msg)
  {
    interface_->set_velocity(msg.linear.x, msg.linear.y, msg.angular.z, msg.linear.z);
  }

  void mode_callback(const unitree_ros2_cpp::msg::HighCmd &msg)
  {
    interface_->set_mode(msg.mode);
  }

  void pos_callback(const unitree_ros2_cpp::msg::HighCmd &msg)
  {
    interface_->set_position(msg.position[0], msg.position[1]);
  }

  void height_callback(const unitree_ros2_cpp::msg::HighCmd &msg)
  {
    interface_->set_foot_raise_height(msg.foot_raise_height);
  }

  void euler_callback(const unitree_ros2_cpp::msg::HighCmd &msg)
  {
    interface_->set_euler(msg.euler[0], msg.euler[1], msg.euler[2]);
  }

  std::shared_ptr<LeggedRobotInterface> interface_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_twist_;
  rclcpp::Subscription<unitree_ros2_cpp::msg::HighCmd>::SharedPtr sub_mode_;
  rclcpp::Subscription<unitree_ros2_cpp::msg::HighCmd>::SharedPtr sub_pos_;
  rclcpp::Subscription<unitree_ros2_cpp::msg::HighCmd>::SharedPtr sub_height_;
  rclcpp::Subscription<unitree_ros2_cpp::msg::HighCmd>::SharedPtr sub_euler_;
};

// =================================================================================================
// MAIN
// =================================================================================================
int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  
  // Create a specific node to load configuration parameters.
  // The node name "legged_controller" must match the name defined in the launch file.
  auto config_node = std::make_shared<rclcpp::Node>("legged_controller");
  
  // Declare parameters with defaults
  config_node->declare_parameter("robot_ip", "192.168.12.1");
  config_node->declare_parameter("local_port", 8090);
  config_node->declare_parameter("remote_port", 8082);

  // Retrieve parameters
  std::string robot_ip = config_node->get_parameter("robot_ip").as_string();
  int local_port = config_node->get_parameter("local_port").as_int();
  int remote_port = config_node->get_parameter("remote_port").as_int();

  RCLCPP_INFO(config_node->get_logger(), "Connecting to Robot at %s (Local: %d, Remote: %d)", 
              robot_ip.c_str(), local_port, remote_port);

  // Create the shared interface with loaded configuration
  auto interface = std::make_shared<LeggedRobotInterface>(robot_ip, local_port, remote_port);

  // Create Worker Nodes
  auto udp_node = std::make_shared<LeggedUDPLoop>(interface);
  auto rx_node = std::make_shared<LeggedDataRX>(interface);
  auto tx_node = std::make_shared<LeggedControl>(interface);

  // Executor
  rclcpp::executors::MultiThreadedExecutor executor;
  // We add config_node to the executor so it can respond to parameter events if needed later
  executor.add_node(config_node); 
  executor.add_node(udp_node);
  executor.add_node(rx_node);
  executor.add_node(tx_node);

  executor.spin();

  rclcpp::shutdown();
  return 0;
}