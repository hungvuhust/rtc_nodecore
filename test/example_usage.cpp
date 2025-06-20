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

/**
 * @file example_usage.cpp
 * @brief Example demonstrating how to use the rtc_nodecore library
 *
 * This example shows how to:
 * 1. Create master and module nodes
 * 2. Initialize and manage VDA5050 state
 * 3. Handle errors and action states
 * 4. Implement heartbeat communication
 */

#include "rtc_nodecore/node_core.hpp"
#include <chrono>
#include <iostream>
#include <memory>
#include <thread>

// Example custom module extending NodeCore
class ExampleAGVModule : public rtcrobot_core::NodeCore {
public:
  explicit ExampleAGVModule(const std::string &module_name)
      : rtcrobot_core::NodeCore(module_name) {}

protected:
  // Override callback to handle received state messages
  void onStateReceived(const vda5050_msgs::msg::State &state) override {
    std::cout << "[" << getModuleName() << "] Received state update - "
              << "Driving: " << (state.driving ? "true" : "false")
              << ", Operating Mode: " << state.operating_mode
              << ", Battery: " << state.battery_state.battery_charge << "%"
              << std::endl;
  }

  // Override callback to handle heartbeat messages
  void onHeartbeatReceived(const std::string &sender_id) override {
    std::cout << "[" << getModuleName()
              << "] Received heartbeat from: " << sender_id << std::endl;
  }
};

void demonstrateBasicUsage() {
  std::cout << "\n=== Basic NodeCore Usage Demo ===" << std::endl;

  // 1. Create master node (manages shared memory)
  auto master_node = std::make_unique<rtcrobot_core::NodeCore>();
  std::cout << "Created master node: " << master_node->getModuleName()
            << std::endl;

  // 2. Initialize master node
  master_node->initialize();
  std::cout << "Master node initialized and active: " << master_node->isActive()
            << std::endl;

  // Give time for initialization
  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  // 3. Create and initialize module nodes
  auto navigation_module = std::make_unique<ExampleAGVModule>("navigation");
  auto battery_module = std::make_unique<ExampleAGVModule>("battery_monitor");

  navigation_module->initialize();
  battery_module->initialize();

  std::cout << "Created modules: " << navigation_module->getModuleName()
            << " and " << battery_module->getModuleName() << std::endl;

  // Give time for modules to connect
  std::this_thread::sleep_for(std::chrono::milliseconds(300));

  // 4. Demonstrate state management
  std::cout << "\n--- State Management Demo ---" << std::endl;

  // Set initial AGV state
  vda5050_msgs::msg::State initial_state;
  initial_state.driving        = false;
  initial_state.paused         = false;
  initial_state.operating_mode = "MANUAL";

  // Set battery state
  vda5050_msgs::msg::BatteryState battery_state;
  battery_state.battery_charge = 85.5;
  battery_state.charging       = false;
  initial_state.battery_state  = battery_state;

  // Set AGV position
  vda5050_msgs::msg::AGVPosition position;
  position.x                 = 10.5;
  position.y                 = 20.3;
  position.theta             = 1.57; // 90 degrees
  position.map_id            = "warehouse_map_v1";
  initial_state.agv_position = position;

  master_node->setStateData(initial_state);
  std::cout << "Set initial AGV state" << std::endl;

  // Give time for state propagation
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  // Cleanup
  navigation_module->shutdown();
  battery_module->shutdown();
  master_node->shutdown();

  std::cout << "Basic usage demo completed!" << std::endl;
}

void demonstrateErrorHandling() {
  std::cout << "\n=== Error Handling Demo ===" << std::endl;

  auto master_node = std::make_unique<rtcrobot_core::NodeCore>();
  master_node->initialize();

  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  // Demonstrate raising and managing errors
  std::cout << "Initial error count: " << master_node->getErrors().size()
            << std::endl;

  // Raise some errors
  master_node->raiseError("BATTERY_LOW", "WARNING", "Battery level below 20%",
                          "Please charge the AGV soon");

  master_node->raiseError("SENSOR_FAULT", "FATAL",
                          "LIDAR sensor not responding",
                          "Check sensor connections and restart");

  auto errors = master_node->getErrors();
  std::cout << "After raising errors, count: " << errors.size() << std::endl;

  for (const auto &error : errors) {
    std::cout << "  - " << error.error_type << " (" << error.error_level
              << "): " << error.error_description << std::endl;
  }

  // Release specific error
  master_node->releaseError("BATTERY_LOW");
  std::cout << "After releasing BATTERY_LOW, count: "
            << master_node->getErrors().size() << std::endl;

  // Release all errors
  master_node->releaseAllErrors();
  std::cout << "After releasing all errors, count: "
            << master_node->getErrors().size() << std::endl;

  master_node->shutdown();
  std::cout << "Error handling demo completed!" << std::endl;
}

void demonstrateActionStates() {
  std::cout << "\n=== Action States Demo ===" << std::endl;

  auto master_node = std::make_unique<rtcrobot_core::NodeCore>();
  master_node->initialize();

  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  // Add some action states
  vda5050_msgs::msg::ActionState move_action;
  move_action.action_id          = "move_001";
  move_action.action_type        = "move";
  move_action.action_status      = "INITIALIZING";
  move_action.result_description = "Preparing to move to target position";

  master_node->addActionState(move_action);
  std::cout << "Added move action" << std::endl;

  vda5050_msgs::msg::ActionState pick_action;
  pick_action.action_id          = "pick_002";
  pick_action.action_type        = "pick";
  pick_action.action_status      = "WAITING";
  pick_action.result_description = "Waiting for move action to complete";

  master_node->addActionState(pick_action);
  std::cout << "Added pick action" << std::endl;

  // Simulate action progress
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  master_node->updateActionState("move_001", "RUNNING",
                                 "Moving to position (x:15.2, y:30.8)");
  std::cout << "Updated move action to RUNNING" << std::endl;

  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  master_node->updateActionState("move_001", "FINISHED",
                                 "Successfully reached target position");
  master_node->updateActionState("pick_002", "RUNNING", "Picking up item");
  std::cout << "Move completed, pick started" << std::endl;

  std::this_thread::sleep_for(std::chrono::milliseconds(800));

  master_node->updateActionState("pick_002", "FINISHED",
                                 "Item picked successfully");
  std::cout << "Pick completed" << std::endl;

  // Clean up finished actions
  master_node->removeActionState("move_001");
  master_node->removeActionState("pick_002");
  std::cout << "Removed completed actions" << std::endl;

  master_node->shutdown();
  std::cout << "Action states demo completed!" << std::endl;
}

void demonstrateMultiModuleCommunication() {
  std::cout << "\n=== Multi-Module Communication Demo ===" << std::endl;

  // Create master and multiple modules
  auto master_node    = std::make_unique<rtcrobot_core::NodeCore>();
  auto nav_module     = std::make_unique<ExampleAGVModule>("navigation");
  auto battery_module = std::make_unique<ExampleAGVModule>("battery_monitor");
  auto safety_module  = std::make_unique<ExampleAGVModule>("safety_controller");

  // Initialize all modules
  master_node->initialize();
  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  nav_module->initialize();
  battery_module->initialize();
  safety_module->initialize();

  std::cout << "Initialized 4 modules (1 master + 3 regular)" << std::endl;
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  // Simulate some state changes
  std::cout << "\nSimulating AGV operation..." << std::endl;

  // Start driving
  master_node->setDriving(true);
  master_node->setOperatingMode("AUTOMATIC");
  std::this_thread::sleep_for(std::chrono::milliseconds(300));

  // Update battery level
  vda5050_msgs::msg::BatteryState battery;
  battery.battery_charge = 78.2;
  battery.charging       = false;
  master_node->setBatteryState(battery);
  std::this_thread::sleep_for(std::chrono::milliseconds(300));

  // Set velocity
  vda5050_msgs::msg::Velocity velocity;
  velocity.vx    = 1.5; // 1.5 m/s forward
  velocity.vy    = 0.0;
  velocity.omega = 0.2; // slight turn
  master_node->setVelocity(velocity);
  std::this_thread::sleep_for(std::chrono::milliseconds(300));

  // Emergency stop
  master_node->setPaused(true);
  master_node->setDriving(false);
  velocity.vx    = 0.0;
  velocity.vy    = 0.0;
  velocity.omega = 0.0;
  master_node->setVelocity(velocity);
  std::cout << "Emergency stop activated!" << std::endl;

  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  // Show message counts
  std::cout << "\nMessage counts:" << std::endl;
  std::cout << "  Master: " << master_node->getMessageCount() << std::endl;
  std::cout << "  Navigation: " << nav_module->getMessageCount() << std::endl;
  std::cout << "  Battery: " << battery_module->getMessageCount() << std::endl;
  std::cout << "  Safety: " << safety_module->getMessageCount() << std::endl;

  // Cleanup
  safety_module->shutdown();
  battery_module->shutdown();
  nav_module->shutdown();
  master_node->shutdown();

  std::cout << "Multi-module communication demo completed!" << std::endl;
}

int main() {
  try {
    std::cout << "=== RTC NodeCore Library Usage Examples ===" << std::endl;

    // Clean up any existing shared memory
    ShmManager::cleanup_shared_memory();

    // Run all demos
    demonstrateBasicUsage();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    demonstrateErrorHandling();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    demonstrateActionStates();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    demonstrateMultiModuleCommunication();

    std::cout << "\n=== All examples completed successfully! ===" << std::endl;

  } catch (const std::exception &e) {
    std::cerr << "Error during demo: " << e.what() << std::endl;
    return 1;
  }

  return 0;
}