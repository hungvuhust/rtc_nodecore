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
#include "rtc_nodecore/signal_handler.hpp"
#include <chrono>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <random>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

class SafetyNode : public rclcpp::Node, public rtcrobot_core::NodeCore {
public:
  SafetyNode()
      : rclcpp::Node("rtc_safety_node"), rtcrobot_core::NodeCore(
                                             "safety_controller") // Slave node
        ,
        emergency_stop_active_(false), safety_zone_violated_(false),
        min_obstacle_distance_(5.0), // Start with no obstacles
        gen_(rd_()),
        obstacle_distance_dist_(0.3, 3.0) // Random obstacle distance 0.3-3.0m

  {
    RCLCPP_INFO(this->get_logger(), "Initializing Safety Node...");

    // Wait for master node to initialize shared memory
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // Initialize NodeCore (connects to shared memory)
    rtcrobot_core::NodeCore::initialize();

    // Setup ROS2 subscribers
    laser_scan_subscriber_ =
        this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", rclcpp::QoS(10),
            std::bind(&SafetyNode::laserScanCallback, this,
                      std::placeholders::_1));

    emergency_stop_subscriber_ = this->create_subscription<std_msgs::msg::Bool>(
        "/emergency_stop", rclcpp::QoS(10).reliable(),
        std::bind(&SafetyNode::emergencyStopCallback, this,
                  std::placeholders::_1));

    reset_safety_subscriber_ = this->create_subscription<std_msgs::msg::Bool>(
        "/safety/reset", rclcpp::QoS(10).reliable(),
        std::bind(&SafetyNode::resetSafetyCallback, this,
                  std::placeholders::_1));

    // Setup ROS2 publishers
    safety_status_publisher_ = this->create_publisher<std_msgs::msg::String>(
        "/safety/status", rclcpp::QoS(10));

    emergency_stop_publisher_ = this->create_publisher<std_msgs::msg::Bool>(
        "/safety/emergency_stop_active", rclcpp::QoS(10).reliable());

    safety_cmd_vel_publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>(
            "/safety/cmd_vel_override", rclcpp::QoS(10));

    // Timer để update safety state định kỳ
    safety_update_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100), // 10Hz - high frequency for safety
        std::bind(&SafetyNode::updateSafetyState, this));

    // Timer để simulate laser scan
    laser_simulation_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(200), // 5Hz
        std::bind(&SafetyNode::simulateLaserScan, this));

    RCLCPP_INFO(this->get_logger(), "Safety Node initialized successfully");
  }

  ~SafetyNode() {
    RCLCPP_INFO(this->get_logger(), "Shutting down Safety Node...");
    rtcrobot_core::NodeCore::shutdown();
  }

protected:
  void onStateReceived(const vda5050_msgs::msg::State &state) override {
    RCLCPP_DEBUG(this->get_logger(), "Safety received state update");

    // Monitor safety based on AGV state
    if (state.driving && !emergency_stop_active_) {
      // Check if safe to drive
      if (safety_zone_violated_ && state.velocity.vx > 0.1) {
        // Force emergency stop if moving into obstacle
        triggerEmergencyStop("Obstacle in safety zone while driving");
      }
    }

    // Update safety state in shared memory if emergency stop is active
    if (emergency_stop_active_) {
      setPaused(true);
      setDriving(false);
    }
  }

private:
  // ROS2 Publishers
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr safety_status_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr   emergency_stop_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr
      safety_cmd_vel_publisher_;

  // ROS2 Subscribers
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
      laser_scan_subscriber_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr
      emergency_stop_subscriber_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr reset_safety_subscriber_;

  // Timers
  rclcpp::TimerBase::SharedPtr safety_update_timer_;
  rclcpp::TimerBase::SharedPtr laser_simulation_timer_;

  // Safety state
  bool         emergency_stop_active_;
  bool         safety_zone_violated_;
  double       min_obstacle_distance_;
  const double SAFETY_DISTANCE  = 0.5; // 50cm safety zone
  const double WARNING_DISTANCE = 1.0; // 1m warning zone

  // Random number generation for simulation
  std::random_device               rd_;
  std::mt19937                     gen_;
  std::uniform_real_distribution<> obstacle_distance_dist_;

  void updateSafetyState() {
    // Create VDA5050 safety state
    vda5050_msgs::msg::SafetyState safety_state;
    safety_state.e_stop          = emergency_stop_active_;
    safety_state.field_violation = safety_zone_violated_;

    // Set safety state trong NodeCore
    setSafetyState(safety_state);

    // Publish safety status
    std_msgs::msg::String status_msg;
    std::string           status_str =
        emergency_stop_active_ ? "E-STOP ACTIVE" : "NORMAL";
    std::string safety_zone_violated_str = safety_zone_violated_ ? "YES" : "NO";
    status_msg.data = "Safety Status: " + status_str + ", Min Distance: " +
                      std::to_string(min_obstacle_distance_) + "m" +
                      ", Zone Violated: " + safety_zone_violated_str;
    safety_status_publisher_->publish(status_msg);

    // Publish emergency stop status
    std_msgs::msg::Bool estop_msg;
    estop_msg.data = emergency_stop_active_;
    emergency_stop_publisher_->publish(estop_msg);

    // If emergency stop is active, publish zero velocity
    if (emergency_stop_active_) {
      geometry_msgs::msg::Twist stop_cmd;
      stop_cmd.linear.x  = 0.0;
      stop_cmd.linear.y  = 0.0;
      stop_cmd.angular.z = 0.0;
      safety_cmd_vel_publisher_->publish(stop_cmd);
    }

    // Update action states for safety
    if (emergency_stop_active_) {
      vda5050_msgs::msg::ActionState safety_action;
      safety_action.action_id     = "emergency_stop_001";
      safety_action.action_type   = "emergency_stop";
      safety_action.action_status = "RUNNING";
      safety_action.result_description =
          "Emergency stop active - safety violation detected";

      addActionState(safety_action);
    } else {
      removeActionState("emergency_stop_001");
    }
  }

  void simulateLaserScan() {
    // Simulate random obstacle detection
    static int simulation_counter = 0;
    simulation_counter++;

    // Every 50 cycles (10 seconds), randomly add/remove obstacles
    if (simulation_counter % 50 == 0) {
      if (gen_() % 3 == 0) { // 33% chance
        min_obstacle_distance_ = obstacle_distance_dist_(gen_);
        RCLCPP_INFO(this->get_logger(), "Safety: Simulated obstacle at %.2fm",
                    min_obstacle_distance_);
      } else {
        min_obstacle_distance_ = 5.0; // Clear path
      }
    }

    // Check safety zones
    bool previous_violation = safety_zone_violated_;
    safety_zone_violated_   = (min_obstacle_distance_ < SAFETY_DISTANCE);

    if (safety_zone_violated_ && !previous_violation) {
      RCLCPP_WARN(this->get_logger(),
                  "Safety zone violation! Obstacle at %.2fm (limit: %.2fm)",
                  min_obstacle_distance_, SAFETY_DISTANCE);

      raiseError("SAFETY_ZONE_VIOLATION", "WARNING",
                 "Obstacle detected in safety zone",
                 "Reduce speed or stop until obstacle is clear");

    } else if (!safety_zone_violated_ && previous_violation) {
      RCLCPP_INFO(this->get_logger(), "Safety zone clear");
      releaseError("SAFETY_ZONE_VIOLATION");
    }

    // Warning zone check
    if (min_obstacle_distance_ < WARNING_DISTANCE &&
        min_obstacle_distance_ >= SAFETY_DISTANCE) {
      raiseError("OBSTACLE_WARNING", "INFO",
                 "Obstacle detected in warning zone", "Proceed with caution");
    } else {
      releaseError("OBSTACLE_WARNING");
    }

    // Auto emergency stop if obstacle too close while moving
    vda5050_msgs::msg::State current_state;
    if (getLatestState(current_state)) {
      if (current_state.driving && current_state.velocity.vx > 0.1 &&
          min_obstacle_distance_ < SAFETY_DISTANCE) {
        triggerEmergencyStop("Automatic emergency stop - obstacle too close");
      }
    }
  }

  void laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    RCLCPP_DEBUG(this->get_logger(), "Received laser scan");

    // Find minimum distance in laser scan
    double min_dist = std::numeric_limits<double>::max();

    for (size_t i = 0; i < msg->ranges.size(); ++i) {
      if (msg->ranges[i] >= msg->range_min &&
          msg->ranges[i] <= msg->range_max) {
        min_dist = std::min(min_dist, static_cast<double>(msg->ranges[i]));
      }
    }

    if (min_dist != std::numeric_limits<double>::max()) {
      min_obstacle_distance_ = min_dist;
    }
  }

  void emergencyStopCallback(const std_msgs::msg::Bool::SharedPtr msg) {
    if (msg->data && !emergency_stop_active_) {
      triggerEmergencyStop("Manual emergency stop activated");
    } else if (!msg->data && emergency_stop_active_) {
      RCLCPP_INFO(this->get_logger(),
                  "Emergency stop reset requested via topic");
      // Note: For safety, manual reset might require additional confirmation
    }
  }

  void resetSafetyCallback(const std_msgs::msg::Bool::SharedPtr msg) {
    if (msg->data && emergency_stop_active_) {
      resetEmergencyStop();
    }
  }

  void triggerEmergencyStop(const std::string &reason) {
    if (emergency_stop_active_) {
      return; // Already in emergency stop
    }

    emergency_stop_active_ = true;

    RCLCPP_ERROR(this->get_logger(), "EMERGENCY STOP TRIGGERED: %s",
                 reason.c_str());

    // Raise critical error
    raiseError("EMERGENCY_STOP", "FATAL", "Emergency stop activated: " + reason,
               "Reset safety system to resume operation");

    // Immediately stop the AGV
    setPaused(true);
    setDriving(false);

    // Zero velocity command
    vda5050_msgs::msg::Velocity zero_vel;
    zero_vel.vx    = 0.0;
    zero_vel.vy    = 0.0;
    zero_vel.omega = 0.0;
    setVelocity(zero_vel);
  }

  void resetEmergencyStop() {
    if (!emergency_stop_active_) {
      return; // Not in emergency stop
    }

    // Check if it's safe to reset
    if (safety_zone_violated_) {
      RCLCPP_WARN(this->get_logger(),
                  "Cannot reset emergency stop - safety zone still violated");
      return;
    }

    emergency_stop_active_ = false;

    RCLCPP_INFO(this->get_logger(),
                "Emergency stop RESET - AGV ready for operation");

    // Release emergency stop error
    releaseError("EMERGENCY_STOP");

    // Allow normal operation
    setPaused(false);
    // Note: Don't automatically set driving=true, let operator decide
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto safety_node = std::make_shared<SafetyNode>();

  RCLCPP_INFO(safety_node->get_logger(), "🛡️ Safety Node started!");

  // Use SignalHandler for graceful shutdown
  auto &signal_handler = rtcrobot_core::SignalHandler::getInstance();
  int   exit_code      = signal_handler.spinWithSignalHandling(safety_node);

  RCLCPP_INFO(safety_node->get_logger(), "🏁 Safety Node shutdown complete");
  rclcpp::shutdown();
  return exit_code;
}