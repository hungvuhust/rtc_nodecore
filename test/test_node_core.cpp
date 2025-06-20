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
#include <gtest/gtest.h>
#include <memory>
#include <thread>

// Helper class to access protected methods for testing
class TestableNodeCore : public rtcrobot_core::NodeCore {
public:
  TestableNodeCore() : rtcrobot_core::NodeCore() {}
  explicit TestableNodeCore(const std::string &module_name)
      : rtcrobot_core::NodeCore(module_name) {}

  // Expose protected methods for testing
  uint64_t testGetCurrentTimestamp() { return getCurrentTimestamp(); }
  uint64_t testGetNextSequenceId() { return getNextSequenceId(); }
};

class TestNodeCore : public ::testing::Test {
protected:
  void SetUp() override {
    // Clean up any existing shared memory before each test
    ShmManager::cleanup_shared_memory();
  }

  void TearDown() override {
    // Clean up shared memory after each test
    ShmManager::cleanup_shared_memory();
  }
};

// Test NodeCore master node creation
TEST_F(TestNodeCore, MasterNodeCreation) {
  EXPECT_NO_THROW({
    auto master_node = std::make_unique<rtcrobot_core::NodeCore>();
    EXPECT_TRUE(master_node->isMasterNode());
    EXPECT_EQ(master_node->getModuleName(), "master");
    EXPECT_FALSE(
        master_node->isActive()); // Should not be active until initialized
  });
}

// Test NodeCore regular module creation
TEST_F(TestNodeCore, RegularModuleCreation) {
  // First create master node
  auto master_node = std::make_unique<rtcrobot_core::NodeCore>();
  master_node->initialize();

  // Give master time to initialize
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  EXPECT_NO_THROW({
    auto module_node = std::make_unique<rtcrobot_core::NodeCore>("test_module");
    EXPECT_FALSE(module_node->isMasterNode());
    EXPECT_EQ(module_node->getModuleName(), "test_module");
    EXPECT_FALSE(
        module_node->isActive()); // Should not be active until initialized
  });

  master_node->shutdown();
}

// Test NodeCore initialization and shutdown
TEST_F(TestNodeCore, InitializationAndShutdown) {
  auto master_node = std::make_unique<rtcrobot_core::NodeCore>();

  // Test initialization
  EXPECT_NO_THROW(master_node->initialize());
  EXPECT_TRUE(master_node->isActive());

  // Give time for threads to start
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // Test shutdown
  EXPECT_NO_THROW(master_node->shutdown());
  EXPECT_FALSE(master_node->isActive());
}

// Test VDA5050 State operations
TEST_F(TestNodeCore, VDA5050StateOperations) {
  auto master_node = std::make_unique<rtcrobot_core::NodeCore>();
  master_node->initialize();

  // Give time for initialization
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // Test state data setting
  vda5050_msgs::msg::State test_state;
  test_state.driving        = true;
  test_state.paused         = false;
  test_state.operating_mode = "AUTOMATIC";

  EXPECT_NO_THROW(master_node->setStateData(test_state));

  // Test state getters
  vda5050_msgs::msg::State retrieved_state;
  EXPECT_TRUE(master_node->getLatestState(retrieved_state));
  EXPECT_EQ(retrieved_state.driving, true);
  EXPECT_EQ(retrieved_state.paused, false);
  EXPECT_EQ(retrieved_state.operating_mode, "AUTOMATIC");

  master_node->shutdown();
}

// Test Battery State setting
TEST_F(TestNodeCore, BatteryStateOperations) {
  auto master_node = std::make_unique<rtcrobot_core::NodeCore>();
  master_node->initialize();

  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  vda5050_msgs::msg::BatteryState battery_state;
  battery_state.battery_charge = 85.5;
  battery_state.charging       = false;

  EXPECT_NO_THROW(master_node->setBatteryState(battery_state));

  vda5050_msgs::msg::State current_state;
  master_node->getLatestState(current_state);
  EXPECT_DOUBLE_EQ(current_state.battery_state.battery_charge, 85.5);
  EXPECT_FALSE(current_state.battery_state.charging);

  master_node->shutdown();
}

// Test Error Management
TEST_F(TestNodeCore, ErrorManagement) {
  auto master_node = std::make_unique<rtcrobot_core::NodeCore>();
  master_node->initialize();

  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // Test raising an error
  EXPECT_NO_THROW(master_node->raiseError("TEST_ERROR", "WARNING",
                                          "This is a test error",
                                          "Check test conditions"));

  // Check that error was added
  auto errors = master_node->getErrors();
  EXPECT_EQ(errors.size(), 1);
  EXPECT_EQ(errors[0].error_type, "TEST_ERROR");
  EXPECT_EQ(errors[0].error_level, "WARNING");

  // Test releasing specific error
  EXPECT_NO_THROW(master_node->releaseError("TEST_ERROR"));
  errors = master_node->getErrors();
  EXPECT_EQ(errors.size(), 0);

  // Test multiple errors and clear all
  master_node->raiseError("ERROR_1", "WARNING", "First error");
  master_node->raiseError("ERROR_2", "FATAL", "Second error");
  errors = master_node->getErrors();
  EXPECT_EQ(errors.size(), 2);

  EXPECT_NO_THROW(master_node->releaseAllErrors());
  errors = master_node->getErrors();
  EXPECT_EQ(errors.size(), 0);

  master_node->shutdown();
}

// Test Action State operations
TEST_F(TestNodeCore, ActionStateOperations) {
  auto master_node = std::make_unique<rtcrobot_core::NodeCore>();
  master_node->initialize();

  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // Test adding action state
  vda5050_msgs::msg::ActionState action_state;
  action_state.action_id          = "test_action_001";
  action_state.action_type        = "move";
  action_state.action_status      = "RUNNING";
  action_state.result_description = "Moving to target";

  EXPECT_NO_THROW(master_node->addActionState(action_state));

  vda5050_msgs::msg::State current_state;
  master_node->getLatestState(current_state);
  EXPECT_EQ(current_state.action_states.size(), 1);
  EXPECT_EQ(current_state.action_states[0].action_id, "test_action_001");

  // Test updating action state
  EXPECT_NO_THROW(master_node->updateActionState("test_action_001", "FINISHED",
                                                 "Completed successfully"));

  master_node->getLatestState(current_state);
  EXPECT_EQ(current_state.action_states[0].action_status, "FINISHED");
  EXPECT_EQ(current_state.action_states[0].result_description,
            "Completed successfully");

  // Test removing action state
  EXPECT_NO_THROW(master_node->removeActionState("test_action_001"));
  master_node->getLatestState(current_state);
  EXPECT_EQ(current_state.action_states.size(), 0);

  master_node->shutdown();
}

// Test Heartbeat functionality
TEST_F(TestNodeCore, HeartbeatFunctionality) {
  auto master_node = std::make_unique<rtcrobot_core::NodeCore>();
  master_node->initialize();

  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // Test manual heartbeat send
  EXPECT_TRUE(master_node->sendHeartbeat());

  // Create a second node to receive heartbeats from master
  auto module_node = std::make_unique<rtcrobot_core::NodeCore>("test_module");
  module_node->initialize();

  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // Test heartbeat communication between nodes
  auto initial_count = module_node->getMessageCount();
  std::this_thread::sleep_for(
      std::chrono::milliseconds(1500)); // Wait more than heartbeat interval
  auto final_count = module_node->getMessageCount();

  // Module should receive heartbeats from master
  EXPECT_GT(final_count, initial_count);

  module_node->shutdown();
  master_node->shutdown();
}

// Test Multi-module communication
TEST_F(TestNodeCore, MultiModuleCommunication) {
  // Create master node
  auto master_node = std::make_unique<rtcrobot_core::NodeCore>();
  master_node->initialize();

  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  // Create regular module
  auto module_node = std::make_unique<rtcrobot_core::NodeCore>("test_module");
  module_node->initialize();

  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  // Both nodes should be active
  EXPECT_TRUE(master_node->isActive());
  EXPECT_TRUE(module_node->isActive());

  // Test state sharing between modules
  vda5050_msgs::msg::State test_state;
  test_state.driving        = true;
  test_state.operating_mode = "MANUAL";

  master_node->setStateData(test_state);

  // Give time for message propagation
  std::this_thread::sleep_for(std::chrono::milliseconds(300));

  // Module should receive state updates
  EXPECT_GT(module_node->getMessageCount(), 0);

  module_node->shutdown();
  master_node->shutdown();
}

// Test Utility functions
TEST_F(TestNodeCore, UtilityFunctions) {
  auto testable_node = std::make_unique<TestableNodeCore>();

  // Test timestamp generation
  auto timestamp1 = testable_node->testGetCurrentTimestamp();
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  auto timestamp2 = testable_node->testGetCurrentTimestamp();

  EXPECT_GT(timestamp2, timestamp1);

  // Test sequence ID generation
  auto seq1 = testable_node->testGetNextSequenceId();
  auto seq2 = testable_node->testGetNextSequenceId();

  EXPECT_EQ(seq2, seq1 + 1);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}