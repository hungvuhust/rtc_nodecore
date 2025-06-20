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
#include <bond/msg/status.hpp>
#include <chrono>
#include <gtest/gtest.h>
#include <memory>
#include <thread>
#include <vda5050_msgs/msg/state.hpp>

class TestSharedMemory : public ::testing::Test {
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

// Test shared memory creation and cleanup
TEST_F(TestSharedMemory, MemoryCreationAndCleanup) {
  EXPECT_FALSE(ShmManager::shared_memory_exists());

  // Create shared memory
  auto shm_segment = std::make_unique<ipc::managed_shared_memory>(
      ipc::create_only, ShmConfig::SHM_NAME, ShmConfig::MAX_SIZE);

  EXPECT_TRUE(ShmManager::shared_memory_exists());

  // Clean up
  shm_segment.reset();
  EXPECT_TRUE(ShmManager::cleanup_shared_memory());
  EXPECT_FALSE(ShmManager::shared_memory_exists());
}

// Test VDA5050 State serialization/deserialization
TEST_F(TestSharedMemory, VDA5050StateSerialization) {
  // Create shared memory
  auto shm_segment = std::make_unique<ipc::managed_shared_memory>(
      ipc::create_only, ShmConfig::SHM_NAME, ShmConfig::MAX_SIZE);

  const ShmAllocator alloc(shm_segment->get_segment_manager());
  auto               shared_data =
      shm_segment->find_or_construct<SharedData>("test_data")(alloc);

  // Create test state
  vda5050_msgs::msg::State original_state;
  original_state.driving                      = true;
  original_state.paused                       = false;
  original_state.operating_mode               = "AUTOMATIC";
  original_state.battery_state.battery_charge = 75.5;
  original_state.battery_state.charging       = false;

  // Test serialization
  EXPECT_TRUE(serialize_message_safe(original_state, *shared_data));
  EXPECT_TRUE(shared_data->has_new_data);
  EXPECT_GT(shared_data->timestamp, 0);

  // Test deserialization
  vda5050_msgs::msg::State deserialized_state;
  EXPECT_TRUE(deserialize_message_safe(deserialized_state, *shared_data,
                                       std::chrono::milliseconds(100)));

  // Verify data integrity
  EXPECT_EQ(deserialized_state.driving, original_state.driving);
  EXPECT_EQ(deserialized_state.paused, original_state.paused);
  EXPECT_EQ(deserialized_state.operating_mode, original_state.operating_mode);
  EXPECT_DOUBLE_EQ(deserialized_state.battery_state.battery_charge,
                   original_state.battery_state.battery_charge);
  EXPECT_EQ(deserialized_state.battery_state.charging,
            original_state.battery_state.charging);
}

// Test Bond Status serialization/deserialization
TEST_F(TestSharedMemory, BondStatusSerialization) {
  // Create shared memory
  auto shm_segment = std::make_unique<ipc::managed_shared_memory>(
      ipc::create_only, ShmConfig::SHM_NAME, ShmConfig::MAX_SIZE);

  const ShmAllocator alloc(shm_segment->get_segment_manager());
  auto               shared_data =
      shm_segment->find_or_construct<SharedData>("test_data")(alloc);

  // Create test bond status
  bond::msg::Status original_status;
  original_status.header.frame_id   = "test_frame";
  original_status.id                = "test_module";
  original_status.instance_id       = "test_instance_123";
  original_status.active            = true;
  original_status.heartbeat_timeout = 5.0;
  original_status.heartbeat_period  = 1.0;

  // Test serialization
  EXPECT_TRUE(serialize_message_safe(original_status, *shared_data));
  EXPECT_TRUE(shared_data->has_new_data);

  // Test deserialization
  bond::msg::Status deserialized_status;
  EXPECT_TRUE(deserialize_message_safe(deserialized_status, *shared_data,
                                       std::chrono::milliseconds(100)));

  // Verify data integrity
  EXPECT_EQ(deserialized_status.header.frame_id,
            original_status.header.frame_id);
  EXPECT_EQ(deserialized_status.id, original_status.id);
  EXPECT_EQ(deserialized_status.instance_id, original_status.instance_id);
  EXPECT_EQ(deserialized_status.active, original_status.active);
  EXPECT_DOUBLE_EQ(deserialized_status.heartbeat_timeout,
                   original_status.heartbeat_timeout);
  EXPECT_DOUBLE_EQ(deserialized_status.heartbeat_period,
                   original_status.heartbeat_period);
}

// Test multiple messages in sequence
TEST_F(TestSharedMemory, MultipleMessagesSerialization) {
  // Create shared memory
  auto shm_segment = std::make_unique<ipc::managed_shared_memory>(
      ipc::create_only, ShmConfig::SHM_NAME, ShmConfig::MAX_SIZE);

  const ShmAllocator alloc(shm_segment->get_segment_manager());
  auto               shared_data =
      shm_segment->find_or_construct<SharedData>("test_data")(alloc);

  // Send multiple states
  for (int i = 0; i < 5; ++i) {
    vda5050_msgs::msg::State state;
    state.driving                      = (i % 2 == 0);
    state.operating_mode               = "MODE_" + std::to_string(i);
    state.battery_state.battery_charge = 50.0 + i * 10.0;

    EXPECT_TRUE(serialize_message_safe(state, *shared_data));

    // Read back immediately
    vda5050_msgs::msg::State read_state;
    EXPECT_TRUE(deserialize_message_safe(read_state, *shared_data,
                                         std::chrono::milliseconds(100)));

    EXPECT_EQ(read_state.driving, state.driving);
    EXPECT_EQ(read_state.operating_mode, state.operating_mode);
    EXPECT_DOUBLE_EQ(read_state.battery_state.battery_charge,
                     state.battery_state.battery_charge);
  }
}

// Test deserialization timeout
TEST_F(TestSharedMemory, DeserializationTimeout) {
  // Create shared memory
  auto shm_segment = std::make_unique<ipc::managed_shared_memory>(
      ipc::create_only, ShmConfig::SHM_NAME, ShmConfig::MAX_SIZE);

  const ShmAllocator alloc(shm_segment->get_segment_manager());
  auto               shared_data =
      shm_segment->find_or_construct<SharedData>("test_data")(alloc);

  // Try to deserialize without any data - should timeout
  vda5050_msgs::msg::State state;
  auto                     start_time = std::chrono::steady_clock::now();

  EXPECT_FALSE(deserialize_message_safe(state, *shared_data,
                                        std::chrono::milliseconds(100)));

  auto end_time = std::chrono::steady_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
      end_time - start_time);

  // Should take approximately the timeout duration
  EXPECT_GE(duration.count(), 95);  // Allow some tolerance
  EXPECT_LE(duration.count(), 200); // But not too much more
}

// Test large message handling
TEST_F(TestSharedMemory, LargeMessageHandling) {
  // Create shared memory
  auto shm_segment = std::make_unique<ipc::managed_shared_memory>(
      ipc::create_only, ShmConfig::SHM_NAME, ShmConfig::MAX_SIZE);

  const ShmAllocator alloc(shm_segment->get_segment_manager());
  auto               shared_data =
      shm_segment->find_or_construct<SharedData>("test_data")(alloc);

  // Create state with many action states to make it large
  vda5050_msgs::msg::State large_state;
  large_state.operating_mode = "AUTOMATIC";

  // Add many action states
  for (int i = 0; i < 100; ++i) {
    vda5050_msgs::msg::ActionState action;
    action.action_id   = "action_" + std::to_string(i);
    action.action_type = "test_action_type_with_long_name_" + std::to_string(i);
    action.action_status      = "RUNNING";
    action.result_description = "Long description for action " +
                                std::to_string(i) +
                                " with lots of text to make the message larger";
    large_state.action_states.push_back(action);
  }

  // Test serialization of large message
  EXPECT_TRUE(serialize_message_safe(large_state, *shared_data));

  // Test deserialization
  vda5050_msgs::msg::State deserialized_state;
  EXPECT_TRUE(deserialize_message_safe(deserialized_state, *shared_data,
                                       std::chrono::milliseconds(500)));

  // Verify data integrity
  EXPECT_EQ(deserialized_state.action_states.size(),
            large_state.action_states.size());
  EXPECT_EQ(deserialized_state.operating_mode, large_state.operating_mode);

  // Check a few action states
  for (size_t i = 0;
       i < std::min(size_t(10), deserialized_state.action_states.size()); ++i) {
    EXPECT_EQ(deserialized_state.action_states[i].action_id,
              large_state.action_states[i].action_id);
    EXPECT_EQ(deserialized_state.action_states[i].action_type,
              large_state.action_states[i].action_type);
  }
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}