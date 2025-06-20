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

#include "rtc_nodecore/message.hpp"
#include "rtc_nodecore/node_core.hpp"
#include <bond/msg/status.hpp>
#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <vda5050_msgs/msg/order.hpp>
#include <vda5050_msgs/msg/state.hpp>

class MasterNode : public rclcpp::Node, public rtcrobot_core::NodeCore {
public:
  MasterNode()
      : rclcpp::Node("rtc_master_node"),
        rtcrobot_core::NodeCore() // Master node constructor
        ,
        active_modules_() {
    RCLCPP_INFO(this->get_logger(), "Initializing RTC Master Node...");

    // Initialize NodeCore (creates shared memory)
    rtcrobot_core::NodeCore::initialize();

    // Setup ROS2 publishers
    state_publisher_ = this->create_publisher<vda5050_msgs::msg::State>(
        "/agv/state", rclcpp::QoS(10).reliable());

    heartbeat_status_publisher_ = this->create_publisher<std_msgs::msg::String>(
        "/agv/heartbeat_status", rclcpp::QoS(10).reliable());

    // Setup ROS2 subscribers
    command_subscriber_ = this->create_subscription<vda5050_msgs::msg::Order>(
        "/agv/order", rclcpp::QoS(10).reliable(),
        std::bind(&MasterNode::commandCallback, this, std::placeholders::_1));

    // Timer để publish state định kỳ
    state_publish_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(200), // 5Hz
        std::bind(&MasterNode::publishCurrentState, this));

    // Timer để check heartbeat status
    heartbeat_check_timer_ = this->create_wall_timer(
        std::chrono::seconds(2),
        std::bind(&MasterNode::checkHeartbeatStatus, this));

    RCLCPP_INFO(this->get_logger(), "Master Node initialized successfully");
  }

  ~MasterNode() {
    RCLCPP_INFO(this->get_logger(), "Shutting down Master Node...");
    rtcrobot_core::NodeCore::shutdown();
  }

protected:
  // Override callback để handle state từ modules
  void onStateReceived(const vda5050_msgs::msg::State & /*state*/) override {
    RCLCPP_DEBUG(this->get_logger(), "Received state update from module");

    // State đã được update trong NodeCore, không cần xử lý thêm
    // Will be published by publishCurrentState()
  }

  // Override callback để handle heartbeat từ modules
  void onHeartbeatReceived(const std::string &sender_id) override {
    RCLCPP_DEBUG(this->get_logger(), "Received heartbeat from: %s",
                 sender_id.c_str());

    // Update last seen time cho module
    active_modules_[sender_id] = this->now();

    // Log active modules periodically
    if (active_modules_.size() > last_logged_module_count_) {
      RCLCPP_INFO(this->get_logger(), "Active modules: %zu",
                  active_modules_.size());
      for (const auto &[module_name, last_seen] : active_modules_) {
        RCLCPP_INFO(this->get_logger(), "  - %s", module_name.c_str());
      }
      last_logged_module_count_ = active_modules_.size();
    }
  }

private:
  // ROS2 Publishers
  rclcpp::Publisher<vda5050_msgs::msg::State>::SharedPtr state_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr
      heartbeat_status_publisher_;

  // ROS2 Subscribers
  rclcpp::Subscription<vda5050_msgs::msg::Order>::SharedPtr command_subscriber_;

  // Timers
  rclcpp::TimerBase::SharedPtr state_publish_timer_;
  rclcpp::TimerBase::SharedPtr heartbeat_check_timer_;

  // Module tracking
  std::map<std::string, rclcpp::Time> active_modules_;
  size_t                              last_logged_module_count_ = 0;

  void publishCurrentState() {
    vda5050_msgs::msg::State current_state;
    if (getLatestState(current_state)) {
      // Add timestamp
      current_state.timestamp = rtcrobot_core::NodeCore::getCurrentTimestamp();

      // Publish state
      state_publisher_->publish(current_state);

      RCLCPP_DEBUG(this->get_logger(),
                   "Published state - Driving: %s, Mode: %s, Battery: %.1f%%",
                   current_state.driving ? "true" : "false",
                   current_state.operating_mode.c_str(),
                   current_state.battery_state.battery_charge);
    }
  }

  void checkHeartbeatStatus() {
    auto                     now = this->now();
    std::vector<std::string> inactive_modules;

    // Check for inactive modules (no heartbeat in last 5 seconds)
    for (auto it = active_modules_.begin(); it != active_modules_.end();) {
      auto time_since_last_heartbeat = (now - it->second).seconds();

      if (time_since_last_heartbeat > 5.0) {
        RCLCPP_WARN(this->get_logger(),
                    "Module '%s' inactive (last heartbeat: %.1f seconds ago)",
                    it->first.c_str(), time_since_last_heartbeat);

        inactive_modules.push_back(it->first);

        // Raise error for inactive module
        raiseError("MODULE_INACTIVE_" + it->first, "WARNING",
                   "Module " + it->first + " has not sent heartbeat",
                   "Check module status and restart if necessary");

        it = active_modules_.erase(it);
      } else {
        ++it;
      }
    }

    // Publish heartbeat status
    std_msgs::msg::String status_msg;
    status_msg.data =
        "Active modules: " + std::to_string(active_modules_.size());
    if (!inactive_modules.empty()) {
      status_msg.data += ", Inactive: ";
      for (size_t i = 0; i < inactive_modules.size(); ++i) {
        status_msg.data += inactive_modules[i];
        if (i < inactive_modules.size() - 1)
          status_msg.data += ", ";
      }
    }
    heartbeat_status_publisher_->publish(status_msg);
  }

  void commandCallback(const vda5050_msgs::msg::Order::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Received order: %s",
                msg->order_id.c_str());

    // Process order - update AGV state accordingly
    // This is where you would implement order processing logic

    // Example: Set driving state based on order
    if (!msg->nodes.empty()) {
      setDriving(true);
      setOperatingMode("AUTOMATIC");

      RCLCPP_INFO(this->get_logger(),
                  "Started automatic operation for order: %s",
                  msg->order_id.c_str());
    }
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  // Clean up any existing shared memory
  ShmManager::cleanup_shared_memory();

  auto master_node = std::make_shared<MasterNode>();

  RCLCPP_INFO(master_node->get_logger(), "Master Node started, spinning...");

  try {
    rclcpp::spin(master_node);
  } catch (const std::exception &e) {
    RCLCPP_ERROR(master_node->get_logger(), "Exception in master node: %s",
                 e.what());
  }

  rclcpp::shutdown();
  return 0;
}