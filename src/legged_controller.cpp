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
 * High-Performance Unitree Go1 Controller (Individual Topics Only)
 * 
 * Features:
 * - Publishes standard ROS 2 messages (Imu, Odometry, Temperature).
 * - Publishes Unitree specific data on individual topics.
 * - Dedicated UDP thread for Real-Time communication.
 * - Fixed: Removed incompatible Low-Level Safety checks for High-Level mode.
 */

#include <chrono>
#include <memory>
#include <string>
#include <mutex>
#include <thread>
#include <atomic>
#include <vector>
#include <cmath>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/temperature.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "unitree_ros2_cpp/msg/bms_state.hpp"
#include "unitree_ros2_cpp/msg/high_state.hpp"
#include "unitree_ros2_cpp/msg/high_cmd.hpp"
#include "unitree_ros2_cpp/msg/motor_state.hpp"

#include "unitree_legged_sdk/unitree_legged_sdk.h"

using namespace UNITREE_LEGGED_SDK;
using namespace std::chrono_literals;

class LeggedController : public rclcpp::Node
{
public:
  LeggedController() : Node("legged_controller")
  {
    // --- Parameters ---
    this->declare_parameter("robot_ip", "192.168.123.161");
    this->declare_parameter("local_port", 8090);
    this->declare_parameter("remote_port", 8082);

    std::string robot_ip = this->get_parameter("robot_ip").as_string();
    int local_port = this->get_parameter("local_port").as_int();
    int remote_port = this->get_parameter("remote_port").as_int();

    RCLCPP_INFO(this->get_logger(), "Connecting to Robot: %s (Local:%d -> Remote:%d)", 
        robot_ip.c_str(), local_port, remote_port);

    // --- Init Unitree SDK ---
    udp_ = std::make_shared<UDP>(HIGHLEVEL, local_port, robot_ip.c_str(), remote_port);
    
    // Initialize Command Struct with safe defaults
    udp_->InitCmdData(cmd_);
    cmd_.levelFlag = HIGHLEVEL; // Critical for HighLevel control
    cmd_.mode = 0;
    cmd_.gaitType = 0;
    cmd_.speedLevel = 0;
    cmd_.footRaiseHeight = 0.08;
    cmd_.bodyHeight = 0.28;
    cmd_.velocity[0] = 0; cmd_.velocity[1] = 0; cmd_.yawSpeed = 0;
    cmd_.euler[0] = 0; cmd_.euler[1] = 0; cmd_.euler[2] = 0;

    // --- Publishers ---
    // High Rate Publishers (Standard ROS Messages)
    pub_imu_  = this->create_publisher<sensor_msgs::msg::Imu>("legged_data/sensors/imu", 10);
    pub_odom_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    
    // Motor Publishers
    pub_motors_.resize(12);
    for(int i=0; i<12; ++i) {
        pub_motors_[i] = this->create_publisher<unitree_ros2_cpp::msg::MotorState>(
            "legged_data/actuators/motor_" + std::to_string(i), 10);
    }

    // Low Rate / Status Publishers
    pub_bms_        = this->create_publisher<unitree_ros2_cpp::msg::BmsState>("legged_data/sensors/bms", 10);
    pub_temp_       = this->create_publisher<sensor_msgs::msg::Temperature>("legged_data/sensors/system_temperature", 10);
    pub_foot_force_ = this->create_publisher<unitree_ros2_cpp::msg::HighState>("legged_data/sensors/foot_force", 10);
    pub_mode_       = this->create_publisher<unitree_ros2_cpp::msg::HighState>("legged_data/status/mode", 10);

    // --- Subscribers ---
    sub_twist_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10, std::bind(&LeggedController::twist_callback, this, std::placeholders::_1));
    sub_mode_ = this->create_subscription<unitree_ros2_cpp::msg::HighCmd>(
        "cmd_mode", 10, std::bind(&LeggedController::mode_callback, this, std::placeholders::_1));
    sub_pos_ = this->create_subscription<unitree_ros2_cpp::msg::HighCmd>(
        "cmd_pos", 10, std::bind(&LeggedController::pos_callback, this, std::placeholders::_1));
    sub_height_ = this->create_subscription<unitree_ros2_cpp::msg::HighCmd>(
        "cmd_foot_raise_height", 10, std::bind(&LeggedController::height_callback, this, std::placeholders::_1));
    sub_euler_ = this->create_subscription<unitree_ros2_cpp::msg::HighCmd>(
        "cmd_euler", 10, std::bind(&LeggedController::euler_callback, this, std::placeholders::_1));

    // --- Start Dedicated UDP Thread ---
    keep_running_ = true;
    udp_thread_ = std::thread(&LeggedController::udp_loop, this);
  }

  ~LeggedController()
  {
    keep_running_ = false;
    if (udp_thread_.joinable()) {
        udp_thread_.join();
    }
    RCLCPP_INFO(this->get_logger(), "Legged Controller Stopped.");
  }

private:
  // --- The High-Frequency Loop (Runs at Robot's Rate) ---
  void udp_loop()
  {
    while (rclcpp::ok() && keep_running_) {
        // 1. Receive Data (Blocking call usually)
        if (udp_->Recv() < 1) {
             // Polling delay if no packet received
             std::this_thread::sleep_for(std::chrono::milliseconds(2)); 
             continue;
        }

        udp_->GetRecv(state_); // Copy buffer to state struct

        // 2. Publish Data Immediately
        publish_data();

        // 3. Send Commands
        {
            std::lock_guard<std::mutex> lock(cmd_mutex_);
            // Note: SDK Safety::PositionLimit is only for LowCmd (Low Level Control).
            // In High Level (Sport Mode), the robot's internal controller handles safety limits.
            udp_->SetSend(cmd_);
        }
        udp_->Send();
    }
  }

  void publish_data()
  {
    auto current_time = this->get_clock()->now();

    // 1. IMU (Standard ROS Message)
    auto imu_msg = sensor_msgs::msg::Imu();
    imu_msg.header.stamp = current_time;
    imu_msg.header.frame_id = "imu_link";
    imu_msg.orientation.w = state_.imu.quaternion[0];
    imu_msg.orientation.x = state_.imu.quaternion[1];
    imu_msg.orientation.y = state_.imu.quaternion[2];
    imu_msg.orientation.z = state_.imu.quaternion[3];
    imu_msg.angular_velocity.x = state_.imu.gyroscope[0];
    imu_msg.angular_velocity.y = state_.imu.gyroscope[1];
    imu_msg.angular_velocity.z = state_.imu.gyroscope[2];
    imu_msg.linear_acceleration.x = state_.imu.accelerometer[0];
    imu_msg.linear_acceleration.y = state_.imu.accelerometer[1];
    imu_msg.linear_acceleration.z = state_.imu.accelerometer[2];
    pub_imu_->publish(imu_msg);

    // 2. Odometry (Standard ROS Message)
    auto odom_msg = nav_msgs::msg::Odometry();
    odom_msg.header.stamp = current_time;
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_link";
    odom_msg.twist.twist.linear.x = state_.velocity[0];
    odom_msg.twist.twist.linear.y = state_.velocity[1];
    odom_msg.twist.twist.linear.z = state_.velocity[2];
    odom_msg.twist.twist.angular.z = state_.yawSpeed;
    odom_msg.pose.pose.orientation = imu_msg.orientation; 
    odom_msg.pose.pose.position.x = state_.position[0];
    odom_msg.pose.pose.position.y = state_.position[1];
    odom_msg.pose.pose.position.z = state_.position[2];
    pub_odom_->publish(odom_msg);

    // 3. Motor States (12 Publishers)
    for(int i=0; i<12; i++) {
        auto m_msg = unitree_ros2_cpp::msg::MotorState();
        m_msg.q = state_.motorState[i].q;
        m_msg.dq = state_.motorState[i].dq;
        m_msg.tau_est = state_.motorState[i].tauEst;
        pub_motors_[i]->publish(m_msg);
    }

    // 4. Low Frequency / Status Data (Decimated to ~10Hz)
    // Assuming ~500Hz loop, publish every 50th tick
    static int low_freq_counter = 0;
    if (low_freq_counter++ >= 50) {
        low_freq_counter = 0;
        
        // BMS
        auto bms_msg = unitree_ros2_cpp::msg::BmsState();
        bms_msg.soc = state_.bms.SOC;
        bms_msg.current = state_.bms.current;
        bms_msg.cycle = state_.bms.cycle;
        pub_bms_->publish(bms_msg);

        // Temperature
        auto temp_msg = sensor_msgs::msg::Temperature();
        temp_msg.temperature = state_.imu.temperature;
        pub_temp_->publish(temp_msg);
        
        // Mode & Gait
        auto mode_msg = unitree_ros2_cpp::msg::HighState();
        mode_msg.mode = state_.mode;
        mode_msg.gait_type = state_.gaitType;
        pub_mode_->publish(mode_msg);

        // Foot Force
        auto force_msg = unitree_ros2_cpp::msg::HighState();
        std::copy(std::begin(state_.footForce), std::end(state_.footForce), std::begin(force_msg.foot_force));
        pub_foot_force_->publish(force_msg);
    }
  }

  // --- Callbacks (Thread-Safe Setters) ---
  void twist_callback(const geometry_msgs::msg::Twist &msg) {
    std::lock_guard<std::mutex> lock(cmd_mutex_);
    cmd_.velocity[0] = msg.linear.x;
    cmd_.velocity[1] = msg.linear.y;
    cmd_.yawSpeed = msg.angular.z;
  }

  void mode_callback(const unitree_ros2_cpp::msg::HighCmd &msg) {
    std::lock_guard<std::mutex> lock(cmd_mutex_);
    cmd_.mode = msg.mode;
  }

  void pos_callback(const unitree_ros2_cpp::msg::HighCmd &msg) {
    std::lock_guard<std::mutex> lock(cmd_mutex_);
    cmd_.position[0] = msg.position[0];
    cmd_.position[1] = msg.position[1];
  }

  void height_callback(const unitree_ros2_cpp::msg::HighCmd &msg) {
    std::lock_guard<std::mutex> lock(cmd_mutex_);
    cmd_.footRaiseHeight = msg.foot_raise_height;
  }

  void euler_callback(const unitree_ros2_cpp::msg::HighCmd &msg) {
    std::lock_guard<std::mutex> lock(cmd_mutex_);
    cmd_.euler[0] = msg.euler[0];
    cmd_.euler[1] = msg.euler[1];
    cmd_.euler[2] = msg.euler[2];
  }

  // Members
  std::shared_ptr<UDP> udp_;
  HighCmd cmd_ = {};
  HighState state_ = {};
  
  std::mutex cmd_mutex_;
  std::thread udp_thread_;
  std::atomic<bool> keep_running_;

  // Publishers
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
  rclcpp::Publisher<unitree_ros2_cpp::msg::BmsState>::SharedPtr pub_bms_;
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr pub_temp_;
  rclcpp::Publisher<unitree_ros2_cpp::msg::HighState>::SharedPtr pub_foot_force_;
  rclcpp::Publisher<unitree_ros2_cpp::msg::HighState>::SharedPtr pub_mode_;
  std::vector<rclcpp::Publisher<unitree_ros2_cpp::msg::MotorState>::SharedPtr> pub_motors_;

  // Subscribers
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_twist_;
  rclcpp::Subscription<unitree_ros2_cpp::msg::HighCmd>::SharedPtr sub_mode_;
  rclcpp::Subscription<unitree_ros2_cpp::msg::HighCmd>::SharedPtr sub_pos_;
  rclcpp::Subscription<unitree_ros2_cpp::msg::HighCmd>::SharedPtr sub_height_;
  rclcpp::Subscription<unitree_ros2_cpp::msg::HighCmd>::SharedPtr sub_euler_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LeggedController>());
  rclcpp::shutdown();
  return 0;
}