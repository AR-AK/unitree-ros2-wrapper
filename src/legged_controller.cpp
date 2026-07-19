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
 */

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <mutex>
#include <vector>
#include <cmath>
#include <thread>

// ROS 2 Includes
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/temperature.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/bool.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

// Unitree ROS 2 Messages
#include "unitree_ros2_cpp/msg/bms_state.hpp"
#include "unitree_ros2_cpp/msg/high_state.hpp"
#include "unitree_ros2_cpp/msg/high_cmd.hpp"
#include "unitree_ros2_cpp/msg/motor_state.hpp"
#include "unitree_ros2_cpp/msg/foot_contact.hpp"

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
  LeggedRobotInterface(const std::string& robot_ip, uint16_t local_port, uint16_t remote_port) 
    : safe(LeggedType::Go1),
      udp(HIGHLEVEL, local_port, robot_ip.c_str(), remote_port),
      has_received_(false)
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
    if (udp.udpState.RecvCount > 0) {
      has_received_ = true;
    }
  }

  // --- Thread-Safe Getters ---

  HighState get_state() {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return state;
  }

  bool is_data_ready() {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return has_received_;
  }

  // --- Thread-Safe Setters ---

  void set_velocity(float x, float y, float yaw, float body_h) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    cmd.velocity[0] = x;
    cmd.velocity[1] = y;
    cmd.yawSpeed = yaw;
    // Only update body height from Twist if it falls within a physically safe absolute range [0.15m, 0.40m]
    if (body_h >= 0.15f && body_h <= 0.40f) {
      cmd.bodyHeight = body_h;
    }
  }

  void set_body_height(float body_h) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (body_h >= 0.15f && body_h <= 0.40f) {
      cmd.bodyHeight = body_h;
    }
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
  bool has_received_;
};


// =================================================================================================
// CLASS: LeggedControllerNode
// Description: Unified ROS 2 Node handling parameter loading, thread-safe UDP communication,
//              best-effort telemetry publishing, dynamic TF broadcasting, and safe subscriber command dispatch.
// =================================================================================================
class LeggedControllerNode : public rclcpp::Node
{
public:
  LeggedControllerNode() : Node("legged_controller")
  {
    // --- Parameters ---
    this->declare_parameter<std::string>("robot_ip", "192.168.123.161");
    this->declare_parameter<int>("local_port", 8090);
    this->declare_parameter<int>("remote_port", 8082);
    this->declare_parameter<double>("cmd_watchdog_timeout", 0.5);
    this->declare_parameter<bool>("publish_tf", true);
    this->declare_parameter<std::string>("odom_frame", "odom");
    this->declare_parameter<std::string>("base_frame", "base_link");
    this->declare_parameter<int>("foot_contact_threshold", 40);

    std::string robot_ip = this->get_parameter("robot_ip").as_string();
    int local_port = this->get_parameter("local_port").as_int();
    int remote_port = this->get_parameter("remote_port").as_int();
    watchdog_timeout_ = this->get_parameter("cmd_watchdog_timeout").as_double();
    publish_tf_ = this->get_parameter("publish_tf").as_bool();
    odom_frame_ = this->get_parameter("odom_frame").as_string();
    base_frame_ = this->get_parameter("base_frame").as_string();
    foot_contact_threshold_ = this->get_parameter("foot_contact_threshold").as_int();

    // --- Standard Legged Robot Joint Names (matches official URDF representation) ---
    joint_names_ = {
      "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
      "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
      "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
      "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint"
    };

    // --- Robot Interface ---
    interface_ = std::make_shared<LeggedRobotInterface>(robot_ip, local_port, remote_port);
    last_cmd_time_ = this->now();

    // --- Callback Groups ---
    callback_group_udp_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    callback_group_telemetry_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    callback_group_commands_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    // --- TF Broadcaster ---
    if (publish_tf_) {
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
      RCLCPP_INFO(this->get_logger(), "TF Broadcast is ENABLED (%s -> %s)", odom_frame_.c_str(), base_frame_.c_str());
    } else {
      RCLCPP_INFO(this->get_logger(), "TF Broadcast is DISABLED (External EKF mode)");
    }

    // --- Publishers ---
    auto sensor_qos = rclcpp::SensorDataQoS(); // Best Effort, Depth 10, ideal for high-freq sensor feeds

    pub_bms_ = this->create_publisher<unitree_ros2_cpp::msg::BmsState>("legged_data/sensors/bms", 10);
    pub_foot_force_ = this->create_publisher<unitree_ros2_cpp::msg::HighState>("legged_data/sensors/foot_force", 10);
    pub_temp_ = this->create_publisher<sensor_msgs::msg::Temperature>("legged_data/sensors/system_temperature", 10);
    pub_mode_ = this->create_publisher<unitree_ros2_cpp::msg::HighState>("legged_data/status/mode", 10);
    pub_gait_ = this->create_publisher<unitree_ros2_cpp::msg::HighState>("legged_data/status/gait_type", 10);
    pub_about_ = this->create_publisher<unitree_ros2_cpp::msg::HighState>("legged_data/status/about_go1", 10);
    pub_foot_raise_ = this->create_publisher<unitree_ros2_cpp::msg::HighState>("legged_data/status/foot_raise_height", 10);
    
    // Switch high-frequency streams to standard SensorDataQoS (Best Effort)
    pub_imu_ = this->create_publisher<sensor_msgs::msg::Imu>("legged_data/sensors/imu", sensor_qos);
    pub_odom_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", sensor_qos);
    pub_joint_states_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", sensor_qos);

    // Unified contact publisher
    pub_foot_contacts_ = this->create_publisher<unitree_ros2_cpp::msg::FootContact>("legged_data/sensors/foot_contacts", 10);

    for (int i = 0; i < 12; ++i) {
        std::string topic = "legged_data/actuators/motor_" + std::to_string(i);
        pub_motors_[i] = this->create_publisher<unitree_ros2_cpp::msg::MotorState>(topic, sensor_qos);
    }

    // --- Subscriptions ---
    auto sub_options = rclcpp::SubscriptionOptions();
    sub_options.callback_group = callback_group_commands_;

    sub_twist_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10, std::bind(&LeggedControllerNode::twist_callback, this, std::placeholders::_1), sub_options);

    sub_mode_ = this->create_subscription<unitree_ros2_cpp::msg::HighCmd>(
        "cmd_mode", 10, std::bind(&LeggedControllerNode::mode_callback, this, std::placeholders::_1), sub_options);

    sub_pos_ = this->create_subscription<unitree_ros2_cpp::msg::HighCmd>(
        "cmd_pos", 10, std::bind(&LeggedControllerNode::pos_callback, this, std::placeholders::_1), sub_options);

    sub_height_ = this->create_subscription<unitree_ros2_cpp::msg::HighCmd>(
        "cmd_foot_raise_height", 10, std::bind(&LeggedControllerNode::height_callback, this, std::placeholders::_1), sub_options);

    sub_euler_ = this->create_subscription<unitree_ros2_cpp::msg::HighCmd>(
        "cmd_euler", 10, std::bind(&LeggedControllerNode::euler_callback, this, std::placeholders::_1), sub_options);

    sub_body_height_ = this->create_subscription<std_msgs::msg::Float32>(
        "cmd_body_height", 10, std::bind(&LeggedControllerNode::body_height_callback, this, std::placeholders::_1), sub_options);

    // --- Timers ---
    timer_udp_ = this->create_wall_timer(
        2ms, std::bind(&LeggedControllerNode::udp_callback, this), callback_group_udp_);

    timer_slow_ = this->create_wall_timer(
        1000ms, std::bind(&LeggedControllerNode::slow_callback, this), callback_group_telemetry_);
    timer_medium_ = this->create_wall_timer(
        100ms, std::bind(&LeggedControllerNode::medium_callback, this), callback_group_telemetry_);
    timer_fast_ = this->create_wall_timer(
        2ms, std::bind(&LeggedControllerNode::fast_callback, this), callback_group_telemetry_);

    // --- Dynamic Parameters Callback ---
    parameters_callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&LeggedControllerNode::parameters_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Legged Controller Node initialized successfully.");
  }

  // Destructor forces safe robot stand-down to prevent runaways or falling down hard
  ~LeggedControllerNode()
  {
    RCLCPP_INFO(this->get_logger(), "Shutting down Legged Controller Node. Sending safe stand-down commands...");
    if (interface_ && interface_->is_data_ready()) {
      interface_->set_velocity(0.0f, 0.0f, 0.0f, 0.0f);
      interface_->set_mode(0); // Set to default standing mode
      // Send UDP packets multiple times to guarantee arrival before socket is destroyed
      for (int i = 0; i < 5; ++i) {
        interface_->udp_send();
        std::this_thread::sleep_for(2ms);
      }
    }
  }

private:
  // --- UDP Loop Callback ---
  void udp_callback()
  {
    // Watchdog checking: if no command received for > watchdog_timeout_ seconds, force zero velocity
    double elapsed = (this->now() - last_cmd_time_).seconds();
    if (elapsed > watchdog_timeout_) {
      if (!watchdog_triggered_) {
        RCLCPP_WARN(this->get_logger(), "Command timeout! No cmd_vel received for %.2f seconds. Halting robot safely.", elapsed);
        watchdog_triggered_ = true;
      }
      interface_->set_velocity(0.0f, 0.0f, 0.0f, 0.0f); // Halt robot motion safely
    }

    // Execute hardware communication
    interface_->udp_send();
    interface_->udp_recv();
  }

  // --- Dynamic Parameter Validation & Updates ---
  rcl_interfaces::msg::SetParametersResult parameters_callback(const std::vector<rclcpp::Parameter> &parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "success";
    for (const auto &param : parameters) {
      if (param.get_name() == "cmd_watchdog_timeout") {
        double val = param.as_double();
        if (val >= 0.05 && val <= 10.0) {
          watchdog_timeout_ = val;
          RCLCPP_INFO(this->get_logger(), "Dynamic Parameter Updated: cmd_watchdog_timeout = %.2f s", val);
        } else {
          result.successful = false;
          result.reason = "cmd_watchdog_timeout must be between 0.05 and 10.0 seconds";
        }
      } else if (param.get_name() == "publish_tf") {
        publish_tf_ = param.as_bool();
        if (publish_tf_ && !tf_broadcaster_) {
          tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        }
        RCLCPP_INFO(this->get_logger(), "Dynamic Parameter Updated: publish_tf = %s", publish_tf_ ? "true" : "false");
      } else if (param.get_name() == "foot_contact_threshold") {
        int val = param.as_int();
        if (val >= 5 && val <= 200) {
          foot_contact_threshold_ = val;
          RCLCPP_INFO(this->get_logger(), "Dynamic Parameter Updated: foot_contact_threshold = %d", val);
        } else {
          result.successful = false;
          result.reason = "foot_contact_threshold must be between 5 and 200";
        }
      }
    }
    return result;
  }

  // --- Subscriber Callbacks ---
  void twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    last_cmd_time_ = this->now();
    if (watchdog_triggered_) {
      RCLCPP_INFO(this->get_logger(), "Command link restored. Control active.");
      watchdog_triggered_ = false;
    }
    interface_->set_velocity(msg->linear.x, msg->linear.y, msg->angular.z, msg->linear.z);
  }

  void body_height_callback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    interface_->set_body_height(msg->data);
  }

  void mode_callback(const unitree_ros2_cpp::msg::HighCmd::SharedPtr msg)
  {
    interface_->set_mode(msg->mode);
  }

  void pos_callback(const unitree_ros2_cpp::msg::HighCmd::SharedPtr msg)
  {
    interface_->set_position(msg->position[0], msg->position[1]);
  }

  void height_callback(const unitree_ros2_cpp::msg::HighCmd::SharedPtr msg)
  {
    interface_->set_foot_raise_height(msg->foot_raise_height);
  }

  void euler_callback(const unitree_ros2_cpp::msg::HighCmd::SharedPtr msg)
  {
    interface_->set_euler(msg->euler[0], msg->euler[1], msg->euler[2]);
  }

  // --- Telemetry Timer Callbacks ---
  void slow_callback()
  {
    if (!interface_->is_data_ready()) {
       RCLCPP_INFO_ONCE(this->get_logger(), "Waiting for first valid UDP telemetry packet from robot...");
       return;
    }
    RCLCPP_INFO_ONCE(this->get_logger(), "First valid UDP telemetry packet received! Starting telemetry publishers.");

    auto state = interface_->get_state();
    
    // --- Core Telemetry Message Packing ---
    auto bms_msg = unitree_ros2_cpp::msg::BmsState();
    bms_msg.soc = state.bms.SOC;
    bms_msg.current = state.bms.current;
    for (int i = 0; i < 10; i++) bms_msg.cell_vol[i] = state.bms.cell_vol[i];
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

    // --- Embedded Safety Diagnostics Monitoring ---
    // 1. Motor Thermal Checks
    bool motor_hot = false;
    int hottest_motor = -1;
    int max_temp = -128;
    for (int i = 0; i < 12; i++) {
      if (state.motorState[i].temperature > max_temp) {
        max_temp = state.motorState[i].temperature;
        hottest_motor = i;
      }
      if (state.motorState[i].temperature > 55) { // 55°C is hot threshold for brushless actuator cores
        motor_hot = true;
      }
    }
    if (motor_hot) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
                           "MOTOR OVERHEATING! Motor %d is at %d°C. Please let the robot cool down.", 
                           hottest_motor, max_temp);
    }

    // 2. Battery SOC Warnings
    if (state.bms.SOC < 15 && state.bms.SOC > 0) {
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 10000, 
                            "BATTERY CRITICALLY LOW! SOC is %d%%. Please recharge or swap battery immediately.", 
                            state.bms.SOC);
    }
  }

  void medium_callback()
  {
    if (!interface_->is_data_ready()) {
      return;
    }

    auto state = interface_->get_state();
    
    auto force_msg = unitree_ros2_cpp::msg::HighState();
    for (int i = 0; i < 4; i++) force_msg.foot_force[i] = state.footForce[i];
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

    // --- Foot Contact Monitoring (Consolidated single message) ---
    auto contact_msg = unitree_ros2_cpp::msg::FootContact();
    contact_msg.fr = state.footForce[0] > foot_contact_threshold_;
    contact_msg.fl = state.footForce[1] > foot_contact_threshold_;
    contact_msg.rr = state.footForce[2] > foot_contact_threshold_;
    contact_msg.rl = state.footForce[3] > foot_contact_threshold_;
    pub_foot_contacts_->publish(contact_msg);
  }

  void fast_callback()
  {
    if (!interface_->is_data_ready()) {
      return;
    }

    auto state = interface_->get_state();
    auto current_time = this->get_clock()->now();

    // --- IMU Telemetry Packing (Best-Effort SensorDataQoS) ---
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

    // Standard high-quality IMU covariance values (crucial for EKF state estimation/robot_localization)
    for (int i = 0; i < 9; i += 4) {
      imu_msg.orientation_covariance[i] = 1e-5;
      imu_msg.angular_velocity_covariance[i] = 1e-6;
      imu_msg.linear_acceleration_covariance[i] = 1e-4;
    }
    pub_imu_->publish(imu_msg);

    // --- Odometry Telemetry Packing (Best-Effort SensorDataQoS) ---
    auto odom_msg = nav_msgs::msg::Odometry();
    odom_msg.header.stamp = current_time;
    odom_msg.header.frame_id = odom_frame_;
    odom_msg.child_frame_id = base_frame_;

    odom_msg.twist.twist.linear.x = state.velocity[0];
    odom_msg.twist.twist.linear.y = state.velocity[1];
    odom_msg.twist.twist.linear.z = state.velocity[2];
    odom_msg.twist.twist.angular.z = state.yawSpeed;

    odom_msg.pose.pose.orientation = imu_msg.orientation;
    odom_msg.pose.pose.position.x = state.position[0];
    odom_msg.pose.pose.position.y = state.position[1];
    odom_msg.pose.pose.position.z = state.position[2];

    // Standard high-quality robot odometry covariance values (essential for robot_localization)
    for (int i = 0; i < 36; i += 7) {
      odom_msg.pose.covariance[i] = 1e-3;
      odom_msg.twist.covariance[i] = 1e-3;
    }
    // Set stabilized dimensions (z, roll, pitch) to have minimal/negligible variance
    odom_msg.pose.covariance[14] = 1e-5;  // z pose
    odom_msg.pose.covariance[21] = 1e-5;  // roll pose
    odom_msg.pose.covariance[28] = 1e-5;  // pitch pose
    odom_msg.twist.covariance[14] = 1e-5; // z twist
    odom_msg.twist.covariance[21] = 1e-5; // roll twist
    odom_msg.twist.covariance[28] = 1e-5; // pitch twist

    pub_odom_->publish(odom_msg);

    // --- Dynamic JointState Publisher (URDF / robot_state_publisher compliant) ---
    auto joint_state_msg = sensor_msgs::msg::JointState();
    joint_state_msg.header.stamp = current_time;
    joint_state_msg.name = joint_names_;
    joint_state_msg.position.resize(12);
    joint_state_msg.velocity.resize(12);
    joint_state_msg.effort.resize(12);

    for (int i = 0; i < 12; i++) {
      joint_state_msg.position[i] = state.motorState[i].q;
      joint_state_msg.velocity[i] = state.motorState[i].dq;
      joint_state_msg.effort[i] = state.motorState[i].tauEst;
    }
    pub_joint_states_->publish(joint_state_msg);

    // --- Dynamic TF odom -> base_link Broadcast ---
    if (publish_tf_ && tf_broadcaster_) {
      geometry_msgs::msg::TransformStamped tf_msg;
      tf_msg.header.stamp = current_time;
      tf_msg.header.frame_id = odom_frame_;
      tf_msg.child_frame_id = base_frame_;

      tf_msg.transform.translation.x = state.position[0];
      tf_msg.transform.translation.y = state.position[1];
      tf_msg.transform.translation.z = state.position[2];
      tf_msg.transform.rotation = imu_msg.orientation;

      tf_broadcaster_->sendTransform(tf_msg);
    }

    for (int i = 0; i < 12; i++) {
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

  // --- Node Members ---
  std::shared_ptr<LeggedRobotInterface> interface_;
  double watchdog_timeout_;
  rclcpp::Time last_cmd_time_;
  bool watchdog_triggered_ = false;

  bool publish_tf_;
  std::string odom_frame_;
  std::string base_frame_;
  int foot_contact_threshold_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  std::vector<std::string> joint_names_;

  // Callback Groups
  rclcpp::CallbackGroup::SharedPtr callback_group_udp_;
  rclcpp::CallbackGroup::SharedPtr callback_group_telemetry_;
  rclcpp::CallbackGroup::SharedPtr callback_group_commands_;

  // Publishers
  rclcpp::Publisher<unitree_ros2_cpp::msg::BmsState>::SharedPtr pub_bms_;
  rclcpp::Publisher<unitree_ros2_cpp::msg::HighState>::SharedPtr pub_foot_force_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_;
  rclcpp::Publisher<unitree_ros2_cpp::msg::HighState>::SharedPtr pub_mode_;
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr pub_temp_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_joint_states_;
  rclcpp::Publisher<unitree_ros2_cpp::msg::HighState>::SharedPtr pub_gait_;
  rclcpp::Publisher<unitree_ros2_cpp::msg::HighState>::SharedPtr pub_about_;
  rclcpp::Publisher<unitree_ros2_cpp::msg::HighState>::SharedPtr pub_foot_raise_;
  rclcpp::Publisher<unitree_ros2_cpp::msg::MotorState>::SharedPtr pub_motors_[12];

  // Unified contact publisher
  rclcpp::Publisher<unitree_ros2_cpp::msg::FootContact>::SharedPtr pub_foot_contacts_;

  // Subscriptions
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_twist_;
  rclcpp::Subscription<unitree_ros2_cpp::msg::HighCmd>::SharedPtr sub_mode_;
  rclcpp::Subscription<unitree_ros2_cpp::msg::HighCmd>::SharedPtr sub_pos_;
  rclcpp::Subscription<unitree_ros2_cpp::msg::HighCmd>::SharedPtr sub_height_;
  rclcpp::Subscription<unitree_ros2_cpp::msg::HighCmd>::SharedPtr sub_euler_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_body_height_;

  // Timers
  rclcpp::TimerBase::SharedPtr timer_udp_;
  rclcpp::TimerBase::SharedPtr timer_slow_;
  rclcpp::TimerBase::SharedPtr timer_medium_;
  rclcpp::TimerBase::SharedPtr timer_fast_;

  // Dynamic Parameter Handle
  OnSetParametersCallbackHandle::SharedPtr parameters_callback_handle_;
};


// =================================================================================================
// MAIN
// =================================================================================================
int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<LeggedControllerNode>();

  // Use a MultiThreadedExecutor to allow callbacks from separate groups to run in parallel
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
