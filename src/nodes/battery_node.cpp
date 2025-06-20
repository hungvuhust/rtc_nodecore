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
#include <memory>
#include <random>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

class BatteryNode : public rclcpp::Node, public rtcrobot_core::NodeCore {
public:
  BatteryNode()
      : rclcpp::Node("rtc_battery_node"), rtcrobot_core::NodeCore(
                                              "battery_monitor") // Slave node
        ,
        gen_(rd_()), battery_noise_(-0.5, 0.5) // Small random variation
        ,
        current_battery_charge_(85.0), is_charging_(false),
        battery_voltage_(24.0), battery_current_(0.0) {
    RCLCPP_INFO(this->get_logger(), "Initializing Battery Node...");

    // Wait for master node to initialize shared memory
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // Initialize NodeCore (connects to shared memory)
    rtcrobot_core::NodeCore::initialize();

    // Setup ROS2 subscribers
    charging_command_subscriber_ =
        this->create_subscription<std_msgs::msg::Bool>(
            "/battery/start_charging", rclcpp::QoS(10),
            std::bind(&BatteryNode::chargingCommandCallback, this,
                      std::placeholders::_1));

    // Setup ROS2 publishers
    battery_status_publisher_ =
        this->create_publisher<sensor_msgs::msg::BatteryState>("/battery/state",
                                                               rclcpp::QoS(10));

    battery_info_publisher_ = this->create_publisher<std_msgs::msg::String>(
        "/battery/info", rclcpp::QoS(10));

    // Timer để update battery state định kỳ
    battery_update_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500), // 2Hz
        std::bind(&BatteryNode::updateBatteryState, this));

    // Timer để simulate battery discharge/charge
    battery_simulation_timer_ = this->create_wall_timer(
        std::chrono::seconds(2), // 0.5Hz - slower simulation
        std::bind(&BatteryNode::simulateBatteryChanges, this));

    RCLCPP_INFO(this->get_logger(), "Battery Node initialized successfully");
  }

  ~BatteryNode() {
    RCLCPP_INFO(this->get_logger(), "Shutting down Battery Node...");
    rtcrobot_core::NodeCore::shutdown();
  }

protected:
  void onStateReceived(const vda5050_msgs::msg::State &state) override {
    RCLCPP_DEBUG(this->get_logger(), "Battery received state update");

    // Adjust battery consumption based on AGV activity
    if (state.driving) {
      // Higher consumption when driving
      base_discharge_rate_ = 0.1; // 0.1% per 2 seconds

      // Even higher consumption with high velocity
      if (state.velocity.vx > 0.5 || std::abs(state.velocity.omega) > 0.3) {
        base_discharge_rate_ = 0.2; // 0.2% per 2 seconds
      }

      battery_current_ = -2.5; // Negative current (discharge)
    } else {
      // Lower consumption when idle
      base_discharge_rate_ = 0.02; // 0.02% per 2 seconds
      battery_current_     = -0.5; // Low idle current
    }
  }

private:
  // ROS2 Publishers
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr
                                                      battery_status_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr battery_info_publisher_;

  // ROS2 Subscribers
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr
      charging_command_subscriber_;

  // Timers
  rclcpp::TimerBase::SharedPtr battery_update_timer_;
  rclcpp::TimerBase::SharedPtr battery_simulation_timer_;

  // Battery state
  double current_battery_charge_; // Percentage 0-100
  bool   is_charging_;
  double battery_voltage_;
  double battery_current_;
  double base_discharge_rate_ = 0.05; // Default discharge rate

  // Random number generation for noise
  std::random_device               rd_;
  std::mt19937                     gen_;
  std::uniform_real_distribution<> battery_noise_;

  void updateBatteryState() {
    // Create VDA5050 battery state
    vda5050_msgs::msg::BatteryState vda_battery_state;
    vda_battery_state.battery_charge  = current_battery_charge_;
    vda_battery_state.charging        = is_charging_;
    vda_battery_state.reach           = 120; // Estimated minutes remaining
    vda_battery_state.battery_voltage = battery_voltage_;

    // Update NodeCore với battery state
    setBatteryState(vda_battery_state);

    // Create ROS2 sensor_msgs::BatteryState
    sensor_msgs::msg::BatteryState ros_battery_msg;
    ros_battery_msg.header.stamp    = this->now();
    ros_battery_msg.header.frame_id = "battery";

    ros_battery_msg.voltage = battery_voltage_;
    ros_battery_msg.current = battery_current_;
    ros_battery_msg.charge =
        current_battery_charge_ / 100.0; // Convert to 0-1 range
    ros_battery_msg.percentage      = current_battery_charge_;
    ros_battery_msg.capacity        = 100.0; // Assume 100Ah capacity
    ros_battery_msg.design_capacity = 100.0;

    ros_battery_msg.power_supply_status =
        is_charging_
            ? sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_CHARGING
            : sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;

    ros_battery_msg.power_supply_health =
        sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_GOOD;

    ros_battery_msg.power_supply_technology =
        sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_LION;

    ros_battery_msg.present = true;

    // Publish ROS2 battery state
    battery_status_publisher_->publish(ros_battery_msg);

    // Publish battery info
    std_msgs::msg::String info_msg;
    info_msg.data = "Battery: " + std::to_string(current_battery_charge_) +
                    "%, " + (is_charging_ ? "CHARGING" : "DISCHARGING") +
                    ", Voltage: " + std::to_string(battery_voltage_) + "V" +
                    ", Current: " + std::to_string(battery_current_) + "A";
    battery_info_publisher_->publish(info_msg);

    // Check for low battery warning
    if (current_battery_charge_ < 20.0 && !is_charging_) {
      raiseError("BATTERY_LOW", "WARNING",
                 "Battery level below 20%: " +
                     std::to_string(current_battery_charge_) + "%",
                 "Please charge the AGV soon");
    } else if (current_battery_charge_ < 10.0 && !is_charging_) {
      raiseError("BATTERY_CRITICAL", "FATAL",
                 "Battery level critical: " +
                     std::to_string(current_battery_charge_) + "%",
                 "AGV must be charged immediately");
    } else {
      // Release battery warnings if charging or battery is good
      releaseError("BATTERY_LOW");
      releaseError("BATTERY_CRITICAL");
    }
  }

  void simulateBatteryChanges() {
    if (is_charging_) {
      // Simulate charging
      if (current_battery_charge_ < 100.0) {
        double charge_rate = 1.0; // 1% per 2 seconds when charging
        current_battery_charge_ =
            std::min(100.0, current_battery_charge_ + charge_rate);
        battery_current_ = 5.0; // Positive current (charging)
        battery_voltage_ =
            25.0 + (current_battery_charge_ / 100.0) * 2.0; // 25-27V

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 10000,
                             "Battery charging: %.1f%%",
                             current_battery_charge_);
      } else {
        // Fully charged
        is_charging_     = false;
        battery_current_ = 0.0;
        RCLCPP_INFO(this->get_logger(), "Battery fully charged: 100%%");
      }
    } else {
      // Simulate discharge
      if (current_battery_charge_ > 0.0) {
        double discharge_rate = base_discharge_rate_ + battery_noise_(gen_);
        current_battery_charge_ =
            std::max(0.0, current_battery_charge_ - discharge_rate);

        // Update voltage based on charge level
        battery_voltage_ =
            20.0 + (current_battery_charge_ / 100.0) * 4.0; // 20-24V

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 30000,
                             "Battery discharging: %.1f%%, Rate: %.3f%%/2s",
                             current_battery_charge_, discharge_rate);
      } else {
        // Battery empty - critical error
        raiseError("BATTERY_EMPTY", "FATAL", "Battery completely discharged",
                   "AGV cannot operate, immediate charging required");
      }
    }
  }

  void chargingCommandCallback(const std_msgs::msg::Bool::SharedPtr msg) {
    bool start_charging = msg->data;

    if (start_charging && !is_charging_) {
      is_charging_ = true;
      RCLCPP_INFO(this->get_logger(), "Starting battery charging...");

      // Add charging action state
      vda5050_msgs::msg::ActionState charging_action;
      charging_action.action_id          = "battery_charge_001";
      charging_action.action_type        = "charge";
      charging_action.action_status      = "RUNNING";
      charging_action.result_description = "Battery charging in progress";

      addActionState(charging_action);

    } else if (!start_charging && is_charging_) {
      is_charging_ = false;
      RCLCPP_INFO(this->get_logger(), "Stopping battery charging...");

      // Remove charging action state
      removeActionState("battery_charge_001");
    }
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto battery_node = std::make_shared<BatteryNode>();

  RCLCPP_INFO(battery_node->get_logger(), "Battery Node started, spinning...");

  try {
    rclcpp::spin(battery_node);
  } catch (const std::exception &e) {
    RCLCPP_ERROR(battery_node->get_logger(), "Exception in battery node: %s",
                 e.what());
  }

  rclcpp::shutdown();
  return 0;
}