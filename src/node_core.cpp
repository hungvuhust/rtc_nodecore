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
#include <iomanip>
#include <iostream>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <sstream>
#include <sys/types.h>
#include <unistd.h>

// Global shutdown flag
std::atomic<bool> g_shutdown_requested{false};

// Signal handler implementation
void signal_handler(int signal) {
  std::cout << "\nReceived signal " << signal
            << ". Initiating graceful shutdown..." << std::endl;
  g_shutdown_requested = true;
}

void setup_signal_handling() {
  std::signal(SIGINT, signal_handler);
  std::signal(SIGTERM, signal_handler);
}

namespace rtcrobot_core {

// Constructor for regular modules
NodeCore::NodeCore(const std::string &module_name)
    : module_name_(module_name), is_master_(false), is_active_(false),
      shutdown_requested_(false), sequence_counter_(0), message_count_(0),
      shared_data_state_(nullptr), shared_data_heartbeat_(nullptr) {

  std::cout << "[NodeCore] Starting module: " << module_name_ << std::endl;

  try {
    // Connect to existing shared memory
    shm_segment_ = std::make_unique<ipc::managed_shared_memory>(
        ipc::open_only, ShmConfig::SHM_NAME);

    setupMessageChannels();

  } catch (const std::exception &e) {
    std::cerr << "[NodeCore] Error connecting to shared memory: " << e.what()
              << std::endl;
    throw;
  }
}

// Constructor for master node
NodeCore::NodeCore()
    : module_name_("master"), is_master_(true), is_active_(false),
      shutdown_requested_(false), sequence_counter_(0), message_count_(0),
      shared_data_state_(nullptr), shared_data_heartbeat_(nullptr) {

  std::cout << "[NodeCore] Starting master node" << std::endl;

  try {
    initializeSharedMemory();
    setupMessageChannels();

    // Initialize default VDA5050 state - will be set by derived classes
    // VDA5050 messages will be properly initialized by the application layer

  } catch (const std::exception &e) {
    std::cerr << "[NodeCore] Error creating shared memory: " << e.what()
              << std::endl;
    throw;
  }
}

NodeCore::~NodeCore() {
  std::cout << "[NodeCore] Shutting down: " << module_name_ << std::endl;
  shutdown();

  if (is_master_) {
    ShmManager::cleanup_shared_memory();
  }
}

void NodeCore::initialize() {
  std::cout << "[NodeCore] Initializing: " << module_name_ << std::endl;

  // Setup signal handling
  setup_signal_handling();

  // Start processing threads
  processing_thread_ =
      std::make_unique<std::thread>(&NodeCore::messageProcessingLoop, this);
  heartbeat_thread_ =
      std::make_unique<std::thread>(&NodeCore::heartbeatLoop, this);

  is_active_      = true;
  last_heartbeat_ = std::chrono::steady_clock::now();

  std::cout << "[NodeCore] Initialized successfully: " << module_name_
            << std::endl;
}

void NodeCore::shutdown() {
  if (!is_active_)
    return;

  std::cout << "[NodeCore] Shutting down: " << module_name_ << std::endl;
  shutdown_requested_ = true;

  // Wait for threads to finish
  if (processing_thread_ && processing_thread_->joinable()) {
    processing_thread_->join();
  }
  if (heartbeat_thread_ && heartbeat_thread_->joinable()) {
    heartbeat_thread_->join();
  }

  is_active_ = false;
  std::cout << "[NodeCore] Shutdown complete: " << module_name_ << std::endl;
}

void NodeCore::release() { shutdown(); }

// VDA5050 State operations
bool NodeCore::publishState(const vda5050_msgs::msg::State &state) {
  if (!is_active_ || !shared_data_state_)
    return false;

  try {
    // Use message.hpp serialization for VDA5050 State
    return serialize_message_safe(state, *shared_data_state_);

  } catch (const std::exception &e) {
    std::cerr << "[NodeCore] Error publishing state: " << e.what() << std::endl;
    return false;
  }
}

bool NodeCore::getLatestState(vda5050_msgs::msg::State &state) {
  std::lock_guard<std::mutex> lock(data_mutex_);
  state = current_state_;
  return true;
}

void NodeCore::setStateData(const vda5050_msgs::msg::State &state) {
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    current_state_ = state;
    // Update sequence and timestamp - field names will depend on actual VDA5050
    // structure current_state_.header_id = getNextSequenceId();
    // current_state_.timestamp = getCurrentTimestamp();
  }

  publishState(current_state_);
}

// VDA5050 State Setter Methods
void NodeCore::setBatteryState(
    const vda5050_msgs::msg::BatteryState &battery_state) {
  std::lock_guard<std::mutex> lock(data_mutex_);
  current_state_.battery_state = battery_state;
  publishState(current_state_);
}

void NodeCore::setAGVPosition(
    const vda5050_msgs::msg::AGVPosition &agv_position) {
  std::lock_guard<std::mutex> lock(data_mutex_);
  current_state_.agv_position = agv_position;
  publishState(current_state_);
}

void NodeCore::setVelocity(const vda5050_msgs::msg::Velocity &velocity) {
  std::lock_guard<std::mutex> lock(data_mutex_);
  current_state_.velocity = velocity;
  publishState(current_state_);
}

void NodeCore::setOperatingMode(const std::string &operating_mode) {
  std::lock_guard<std::mutex> lock(data_mutex_);
  current_state_.operating_mode = operating_mode;
  publishState(current_state_);
}

void NodeCore::setDriving(bool driving) {
  std::lock_guard<std::mutex> lock(data_mutex_);
  current_state_.driving = driving;
  publishState(current_state_);
}

void NodeCore::setPaused(bool paused) {
  std::lock_guard<std::mutex> lock(data_mutex_);
  current_state_.paused = paused;
  publishState(current_state_);
}

void NodeCore::setSafetyState(
    const vda5050_msgs::msg::SafetyState &safety_state) {
  std::lock_guard<std::mutex> lock(data_mutex_);
  current_state_.safety_state = safety_state;
  publishState(current_state_);
}

void NodeCore::addActionState(
    const vda5050_msgs::msg::ActionState &action_state) {
  std::lock_guard<std::mutex> lock(data_mutex_);

  // Check if action already exists, if so update it
  auto it = std::find_if(
      current_state_.action_states.begin(), current_state_.action_states.end(),
      [&action_state](const vda5050_msgs::msg::ActionState &existing) {
        return existing.action_id == action_state.action_id;
      });

  if (it != current_state_.action_states.end()) {
    *it = action_state; // Update existing action
  } else {
    current_state_.action_states.push_back(action_state); // Add new action
  }

  publishState(current_state_);
}

void NodeCore::updateActionState(const std::string &action_id,
                                 const std::string &action_status,
                                 const std::string &result_description) {
  std::lock_guard<std::mutex> lock(data_mutex_);

  auto it = std::find_if(
      current_state_.action_states.begin(), current_state_.action_states.end(),
      [&action_id](const vda5050_msgs::msg::ActionState &action) {
        return action.action_id == action_id;
      });

  if (it != current_state_.action_states.end()) {
    it->action_status = action_status;
    if (!result_description.empty()) {
      it->result_description = result_description;
    }
    publishState(current_state_);
  }
}

void NodeCore::removeActionState(const std::string &action_id) {
  std::lock_guard<std::mutex> lock(data_mutex_);

  current_state_.action_states.erase(
      std::remove_if(
          current_state_.action_states.begin(),
          current_state_.action_states.end(),
          [&action_id](const vda5050_msgs::msg::ActionState &action) {
            return action.action_id == action_id;
          }),
      current_state_.action_states.end());

  publishState(current_state_);
}

void NodeCore::clearActionStates() {
  std::lock_guard<std::mutex> lock(data_mutex_);
  current_state_.action_states.clear();
  publishState(current_state_);
}

// VDA5050 Error Management
void NodeCore::raiseError(
    const std::string &error_type, const std::string &error_level,
    const std::string &error_description, const std::string &error_hint,
    const std::vector<vda5050_msgs::msg::ErrorReference> &error_references) {
  std::lock_guard<std::mutex> lock(data_mutex_);

  // Check if error already exists
  auto it =
      std::find_if(current_state_.errors.begin(), current_state_.errors.end(),
                   [&error_type](const vda5050_msgs::msg::Error &error) {
                     return error.error_type == error_type;
                   });

  if (it != current_state_.errors.end()) {
    return; // Error already exists, don't add duplicate
  }

  // Create new error
  vda5050_msgs::msg::Error new_error;
  new_error.error_type        = error_type;
  new_error.error_level       = error_level;
  new_error.error_description = error_description;
  new_error.error_hint        = error_hint;
  new_error.error_references  = error_references;

  current_state_.errors.push_back(new_error);
  publishState(current_state_);

  std::cout << "[NodeCore] Error raised: " << error_type << " (" << error_level
            << ") - " << error_description << std::endl;
}

void NodeCore::releaseError(const std::string &error_type) {
  std::lock_guard<std::mutex> lock(data_mutex_);

  current_state_.errors.erase(
      std::remove_if(current_state_.errors.begin(), current_state_.errors.end(),
                     [&error_type](const vda5050_msgs::msg::Error &error) {
                       return error.error_type == error_type;
                     }),
      current_state_.errors.end());

  publishState(current_state_);

  std::cout << "[NodeCore] Released error: " << error_type << std::endl;
}

void NodeCore::releaseAllErrors() {
  std::lock_guard<std::mutex> lock(data_mutex_);
  current_state_.errors.clear();
  publishState(current_state_);

  std::cout << "[NodeCore] All errors released" << std::endl;
}

std::vector<vda5050_msgs::msg::Error> NodeCore::getErrors() const {
  std::lock_guard<std::mutex> lock(const_cast<std::mutex &>(data_mutex_));
  return current_state_.errors;
}

bool NodeCore::sendHeartbeat() {
  if (!is_active_)
    return false;

  try {
    // Create bond::msg::Status message for heartbeat
    bond::msg::Status heartbeat_msg;
    heartbeat_msg.header.stamp.sec =
        static_cast<int32_t>(getCurrentTimestamp() / 1000000000);
    heartbeat_msg.header.stamp.nanosec =
        static_cast<uint32_t>(getCurrentTimestamp() % 1000000000);
    heartbeat_msg.header.frame_id = module_name_;
    heartbeat_msg.id              = module_name_;
    heartbeat_msg.instance_id = module_name_ + "_" + std::to_string(getpid());
    heartbeat_msg.active      = is_active_;
    heartbeat_msg.heartbeat_timeout = 5.0; // 5 seconds timeout
    heartbeat_msg.heartbeat_period  = 1.0; // 1 second period

    // Send heartbeat using serialize_message_safe on heartbeat channel
    if (shared_data_heartbeat_) {
      return serialize_message_safe(heartbeat_msg, *shared_data_heartbeat_);
    }

  } catch (const std::exception &e) {
    std::cerr << "[NodeCore] Heartbeat error: " << e.what() << std::endl;
  }

  return false;
}

// Protected methods
void NodeCore::messageProcessingLoop() {
  std::cout << "[NodeCore] Starting message processing loop for: "
            << module_name_ << std::endl;

  while (!shutdown_requested_) {
    try {
      // Process state messages
      if (shared_data_state_) {
        vda5050_msgs::msg::State state;
        if (deserialize_message_safe(state, *shared_data_state_,
                                     MESSAGE_TIMEOUT)) {
          handleStateMessage("");
          message_count_++;

          // Update local state and call callback
          {
            std::lock_guard<std::mutex> lock(data_mutex_);
            current_state_ = state;
          }
          onStateReceived(state);
        }

        // Also check for heartbeat messages (bond::msg::Status) on separate
        // channel
        if (shared_data_heartbeat_) {
          bond::msg::Status heartbeat_status;
          if (deserialize_message_safe(
                  heartbeat_status, *shared_data_heartbeat_, MESSAGE_TIMEOUT)) {
            message_count_++; // Count heartbeat messages too

            // Check if this is a heartbeat from another module
            if (heartbeat_status.id != module_name_) {
              handleHeartbeatMessage(heartbeat_status);
            }
          }
        }
      }

    } catch (const std::exception &e) {
      std::cerr << "[NodeCore] Message processing error: " << e.what()
                << std::endl;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  std::cout << "[NodeCore] Message processing loop ended for: " << module_name_
            << std::endl;
}

void NodeCore::heartbeatLoop() {
  std::cout << "[NodeCore] Starting heartbeat loop for: " << module_name_
            << std::endl;

  while (!shutdown_requested_) {
    try {
      auto now = std::chrono::steady_clock::now();
      if (now - last_heartbeat_ >= HEARTBEAT_INTERVAL) {
        sendHeartbeat();
        last_heartbeat_ = now;
      }

    } catch (const std::exception &e) {
      std::cerr << "[NodeCore] Heartbeat error: " << e.what() << std::endl;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }

  std::cout << "[NodeCore] Heartbeat loop ended for: " << module_name_
            << std::endl;
}

// Utility functions
uint64_t NodeCore::getCurrentTimestamp() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

uint64_t NodeCore::getNextSequenceId() { return ++sequence_counter_; }

bool NodeCore::writeToSharedMemory(const std::string &key,
                                   const std::string &data) {
  (void)key;
  (void)data;
  // Custom implementation if needed
  return true;
}

bool NodeCore::readFromSharedMemory(const std::string &key, std::string &data) {
  (void)key;
  (void)data;
  // Custom implementation if needed
  return true;
}

// Private methods
void NodeCore::handleStateMessage(const std::string &message_data) {
  (void)message_data;
  // State message handling is done in messageProcessingLoop
  std::cout << "[NodeCore] State message received by: " << module_name_
            << std::endl;
}

void NodeCore::handleHeartbeatMessage(
    const bond::msg::Status &heartbeat_status) {
  try {
    // Process bond::msg::Status heartbeat message
    if (heartbeat_status.id != module_name_) {
      onHeartbeatReceived(heartbeat_status.id);

      std::cout << "[NodeCore] Heartbeat from: " << heartbeat_status.id
                << " (instance: " << heartbeat_status.instance_id
                << ", active: " << (heartbeat_status.active ? "true" : "false")
                << ", timeout: " << heartbeat_status.heartbeat_timeout << "s)"
                << std::endl;
    }

  } catch (const std::exception &e) {
    std::cerr << "[NodeCore] Error handling heartbeat: " << e.what()
              << std::endl;
  }
}

void NodeCore::initializeSharedMemory() {
  // Clean up any existing shared memory
  ShmManager::cleanup_shared_memory();

  // Create new shared memory
  shm_segment_ = std::make_unique<ipc::managed_shared_memory>(
      ipc::create_only, ShmConfig::SHM_NAME, ShmConfig::MAX_SIZE);

  std::cout << "[NodeCore] Created shared memory segment" << std::endl;
}

void NodeCore::setupMessageChannels() {
  const ShmAllocator alloc(shm_segment_->get_segment_manager());

  // Setup state channel
  shared_data_state_ =
      shm_segment_->find_or_construct<SharedData>(STATE_SHM_KEY)(alloc);

  // Setup heartbeat channel
  shared_data_heartbeat_ =
      shm_segment_->find_or_construct<SharedData>(HEARTBEAT_SHM_KEY)(alloc);

  std::cout << "[NodeCore] Setup message channels for: " << module_name_
            << std::endl;
}

} // namespace rtcrobot_core