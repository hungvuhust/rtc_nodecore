// Copyright (c) 2024 RTC Technology JSC
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

#include "rtc_nodecore/node_core.hpp"
#include <chrono>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <random>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class NavigationNode : public rclcpp::Node, public rtcrobot_core::NodeCore {
public:
  NavigationNode()
      : rclcpp::Node("rtc_navigation_node"), rtcrobot_core::NodeCore(
                                                 "navigation") // Slave node
        ,
        gen_(rd_()), dis_x_(8.0, 12.0), // Random X position 8-12m
        dis_y_(18.0, 22.0),             // Random Y position 18-22m
        dis_theta_(-3.14, 3.14),        // Random orientation
        current_x_(10.0), current_y_(20.0), current_theta_(0.0) {
    RCLCPP_INFO(this->get_logger(), "Initializing Navigation Node...");

    // Wait for master node to initialize shared memory
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // Initialize NodeCore (connects to shared memory)
    rtcrobot_core::NodeCore::initialize();

    // Setup ROS2 subscribers
    cmd_vel_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", rclcpp::QoS(10),
        std::bind(&NavigationNode::cmdVelCallback, this,
                  std::placeholders::_1));

    odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", rclcpp::QoS(10),
        std::bind(&NavigationNode::odomCallback, this, std::placeholders::_1));

    initial_pose_subscriber_ = this->create_subscription<
        geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/initialpose", rclcpp::QoS(10),
        std::bind(&NavigationNode::initialPoseCallback, this,
                  std::placeholders::_1));

    // Setup ROS2 publishers
    navigation_status_publisher_ =
        this->create_publisher<std_msgs::msg::String>("/navigation/status",
                                                      rclcpp::QoS(10));

    // Timer để update navigation state định kỳ
    navigation_update_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100), // 10Hz
        std::bind(&NavigationNode::updateNavigationState, this));

    // Timer để simulate movement
    movement_simulation_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500), // 2Hz
        std::bind(&NavigationNode::simulateMovement, this));

    RCLCPP_INFO(this->get_logger(), "Navigation Node initialized successfully");
  }

  ~NavigationNode() {
    RCLCPP_INFO(this->get_logger(), "Shutting down Navigation Node...");
    rtcrobot_core::NodeCore::shutdown();
  }

protected:
  void onStateReceived(const vda5050_msgs::msg::State &state) override {
    RCLCPP_DEBUG(this->get_logger(), "Navigation received state update");

    // Process navigation logic based on overall AGV state
    if (state.driving && state.operating_mode == "AUTOMATIC") {
      // Start automatic navigation
      is_navigating_ = true;
    } else if (state.paused || !state.driving) {
      // Stop navigation
      is_navigating_           = false;
      target_linear_velocity_  = 0.0;
      target_angular_velocity_ = 0.0;
    }
  }

private:
  // ROS2 Publishers
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr
      navigation_status_publisher_;

  // ROS2 Subscribers
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr
                                                           cmd_vel_subscriber_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      initial_pose_subscriber_;

  // Timers
  rclcpp::TimerBase::SharedPtr navigation_update_timer_;
  rclcpp::TimerBase::SharedPtr movement_simulation_timer_;

  // Navigation state
  bool   is_navigating_            = false;
  double target_linear_velocity_   = 0.0;
  double target_angular_velocity_  = 0.0;
  double current_linear_velocity_  = 0.0;
  double current_angular_velocity_ = 0.0;

  // Position simulation
  std::random_device               rd_;
  std::mt19937                     gen_;
  std::uniform_real_distribution<> dis_x_;
  std::uniform_real_distribution<> dis_y_;
  std::uniform_real_distribution<> dis_theta_;

  double current_x_;
  double current_y_;
  double current_theta_;

  void updateNavigationState() {
    // Update AGV position trong shared memory
    vda5050_msgs::msg::AGVPosition position;
    position.x                    = current_x_;
    position.y                    = current_y_;
    position.theta                = current_theta_;
    position.map_id               = "warehouse_map_v1";
    position.position_initialized = true;
    position.localization_score   = 0.95; // High confidence

    setAGVPosition(position);

    // Update velocity
    vda5050_msgs::msg::Velocity velocity;
    velocity.vx    = current_linear_velocity_;
    velocity.vy    = 0.0; // Assume differential drive
    velocity.omega = current_angular_velocity_;

    setVelocity(velocity);

    // Update navigation-specific state
    if (is_navigating_) {
      // Add navigation action state
      vda5050_msgs::msg::ActionState nav_action;
      nav_action.action_id          = "navigate_001";
      nav_action.action_type        = "navigate";
      nav_action.action_status      = "RUNNING";
      nav_action.result_description = "Navigating to target position";

      addActionState(nav_action);
    } else {
      // Remove navigation action if not navigating
      removeActionState("navigate_001");
    }

    // Publish navigation status
    std_msgs::msg::String status_msg;
    std::string           status_str = is_navigating_ ? "ACTIVE" : "IDLE";
    status_msg.data = "Navigation Status: " + status_str + ", Position: (" +
                      std::to_string(current_x_) + ", " +
                      std::to_string(current_y_) + ", " +
                      std::to_string(current_theta_) + ")"; // NOLINT
    navigation_status_publisher_->publish(status_msg);
  }

  void simulateMovement() {
    if (!is_navigating_) {
      return;
    }

    // Simulate some movement towards random target
    static double target_x = dis_x_(gen_);
    static double target_y = dis_y_(gen_);

    double dx       = target_x - current_x_;
    double dy       = target_y - current_y_;
    double distance = std::sqrt(dx * dx + dy * dy);

    if (distance < 0.5) {
      // Reached target, choose new target
      target_x = dis_x_(gen_);
      target_y = dis_y_(gen_);
      RCLCPP_INFO(this->get_logger(), "Navigation: New target (%.2f, %.2f)",
                  target_x, target_y);
      return;
    }

    // Calculate desired velocity
    double desired_linear = std::min(1.0, distance * 0.5); // Max 1 m/s
    double desired_angle  = std::atan2(dy, dx);
    double angle_diff     = desired_angle - current_theta_;

    // Normalize angle difference
    while (angle_diff > M_PI) angle_diff -= 2 * M_PI;
    while (angle_diff < -M_PI) angle_diff += 2 * M_PI;

    double desired_angular =
        std::max(-0.5, std::min(0.5, angle_diff * 2.0)); // Max 0.5 rad/s

    // Update velocities (simple simulation)
    target_linear_velocity_  = desired_linear;
    target_angular_velocity_ = desired_angular;

    // Smooth velocity changes
    current_linear_velocity_ =
        current_linear_velocity_ * 0.8 + target_linear_velocity_ * 0.2;
    current_angular_velocity_ =
        current_angular_velocity_ * 0.8 + target_angular_velocity_ * 0.2;

    // Update position based on velocity
    double dt = 0.5; // 500ms
    current_x_ += current_linear_velocity_ * std::cos(current_theta_) * dt;
    current_y_ += current_linear_velocity_ * std::sin(current_theta_) * dt;
    current_theta_ += current_angular_velocity_ * dt;

    // Normalize theta
    while (current_theta_ > M_PI) current_theta_ -= 2 * M_PI;
    while (current_theta_ < -M_PI) current_theta_ += 2 * M_PI;
  }

  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    RCLCPP_DEBUG(this->get_logger(),
                 "Received cmd_vel: linear=%.2f, angular=%.2f", msg->linear.x,
                 msg->angular.z);

    // Update target velocities from manual control
    if (!is_navigating_) { // Only in manual mode
      target_linear_velocity_  = msg->linear.x;
      target_angular_velocity_ = msg->angular.z;
    }
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr /*msg*/) {
    RCLCPP_DEBUG(this->get_logger(), "Received odometry update");

    // Update position from odometry (in real system)
    // For simulation, we use our simulated position
  }

  void initialPoseCallback(
      const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Received initial pose");

    // Update position from initial pose
    current_x_ = msg->pose.pose.position.x;
    current_y_ = msg->pose.pose.position.y;

    // Convert quaternion to yaw
    auto &q        = msg->pose.pose.orientation;
    current_theta_ = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                                1.0 - 2.0 * (q.y * q.y + q.z * q.z));

    RCLCPP_INFO(this->get_logger(),
                "Navigation: Position reset to (%.2f, %.2f, %.2f)", current_x_,
                current_y_, current_theta_);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto navigation_node = std::make_shared<NavigationNode>();

  RCLCPP_INFO(navigation_node->get_logger(),
              "Navigation Node started, spinning...");

  try {
    rclcpp::spin(navigation_node);
  } catch (const std::exception &e) {
    RCLCPP_ERROR(navigation_node->get_logger(),
                 "Exception in navigation node: %s", e.what());
  }

  rclcpp::shutdown();
  return 0;
}