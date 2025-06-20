#pragma once

#include "rosidl_runtime_cpp/message_initialization.hpp"
#include "rosidl_runtime_cpp/message_type_support_decl.hpp"
#include <atomic>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/interprocess/allocators/allocator.hpp>
#include <boost/interprocess/containers/vector.hpp>
#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/sync/interprocess_condition.hpp>
#include <boost/interprocess/sync/interprocess_mutex.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>
#include <chrono>
#include <csignal>
#include <cstring>
#include <iostream>
#include <memory>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <string>
#include <unistd.h>

namespace ipc = boost::interprocess;

// Shared memory configuration
struct ShmConfig {
  static constexpr const char *SHM_NAME       = "ROSSharedMemoryImproved";
  static constexpr const char *VECTOR_NAME    = "DataVector";
  static constexpr const char *MUTEX_NAME     = "DataMutex";
  static constexpr const char *CONDITION_NAME = "DataCondition";
  static constexpr const char *CONTROL_NAME   = "ControlBlock";
  static constexpr size_t      INITIAL_SIZE   = 4096; // Tăng size ban đầu
  static constexpr size_t      MAX_SIZE       = 1024 * 1024; // 1MB max
};

// Control block để quản lý state
struct ControlBlock {
  std::atomic<bool>     publisher_active{false};
  std::atomic<bool>     subscriber_active{false};
  std::atomic<uint64_t> message_count{0};
  std::atomic<uint64_t> sequence_number{0};
};

// Custom allocators và containers
typedef ipc::allocator<uint8_t, ipc::managed_shared_memory::segment_manager>
                                           ShmAllocator;
typedef ipc::vector<uint8_t, ShmAllocator> ShmVector;

// Shared data structure
struct SharedData {
  ipc::interprocess_mutex     mutex;
  ipc::interprocess_condition condition;
  ShmVector                   data_vector;
  ControlBlock                control;
  bool                        has_new_data{false};
  uint64_t                    timestamp{0};

  SharedData(const ShmAllocator &alloc) : data_vector(alloc) {}
};

// Global signal handler flag
extern std::atomic<bool> g_shutdown_requested;

// Signal handler
void signal_handler(int signal);

// Setup signal handling
void setup_signal_handling();

// Improved serialize function with error handling
template <typename MessageRos>
bool serialize_message_safe(const MessageRos &ros_msg,
                            SharedData       &shared_data) {
  try {
    rclcpp::Serialization<MessageRos> serializer;
    rclcpp::SerializedMessage         serialized_msg;

    // Serialize ROS2 message
    serializer.serialize_message(&ros_msg, &serialized_msg);

    // Lock để thread safety
    ipc::scoped_lock<ipc::interprocess_mutex> lock(shared_data.mutex);

    // Resize vector if needed
    size_t required_size = serialized_msg.size();
    if (required_size > ShmConfig::MAX_SIZE) {
      std::cerr << "Message size " << required_size
                << " exceeds maximum allowed size " << ShmConfig::MAX_SIZE
                << std::endl;
      return false;
    }

    shared_data.data_vector.resize(required_size);

    // Copy data
    std::memcpy(shared_data.data_vector.data(),
                serialized_msg.get_rcl_serialized_message().buffer,
                required_size);

    // Update metadata
    shared_data.has_new_data = true;
    shared_data.timestamp =
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch())
            .count();
    shared_data.control.sequence_number++;

    // Notify waiting subscribers
    shared_data.condition.notify_all();

    return true;

  } catch (const std::exception &e) {
    std::cerr << "Serialization error: " << e.what() << std::endl;
    return false;
  }
}

// Improved deserialize function with timeout
template <typename MessageRos>
bool deserialize_message_safe(
    MessageRos &ros_msg, SharedData &shared_data,
    std::chrono::milliseconds timeout = std::chrono::milliseconds(100)) {
  try {
    rclcpp::Serialization<MessageRos> serializer;
    rclcpp::SerializedMessage         serialized_msg;

    // Lock với timeout
    ipc::scoped_lock<ipc::interprocess_mutex> lock(shared_data.mutex);

    // Wait for new data với timeout
    if (!shared_data.has_new_data) {
      auto wait_result = shared_data.condition.timed_wait(
          lock, boost::posix_time::microsec_clock::universal_time() +
                    boost::posix_time::milliseconds(timeout.count()));

      if (!wait_result || !shared_data.has_new_data) {
        return false; // Timeout hoặc không có data mới
      }
    }

    if (shared_data.data_vector.empty()) {
      return false;
    }

    // Prepare serialized message
    size_t data_size = shared_data.data_vector.size();
    serialized_msg.reserve(data_size);

    std::memcpy(serialized_msg.get_rcl_serialized_message().buffer,
                shared_data.data_vector.data(), data_size);
    serialized_msg.get_rcl_serialized_message().buffer_length = data_size;

    // Mark data as read
    shared_data.has_new_data = false;

    // Unlock before deserializing
    lock.unlock();

    // Deserialize ROS2 message
    serializer.deserialize_message(&serialized_msg, &ros_msg);

    return true;

  } catch (const std::exception &e) {
    std::cerr << "Deserialization error: " << e.what() << std::endl;
    return false;
  }
}

// Utility functions
class ShmManager {
public:
  static bool cleanup_shared_memory() {
    try {
      return ipc::shared_memory_object::remove(ShmConfig::SHM_NAME);
    } catch (const std::exception &e) {
      std::cerr << "Cleanup error: " << e.what() << std::endl;
      return false;
    }
  }

  static bool shared_memory_exists() {
    try {
      ipc::managed_shared_memory shm(ipc::open_only, ShmConfig::SHM_NAME);
      return true;
    } catch (const ipc::interprocess_exception &) {
      return false;
    }
  }

  static void print_memory_info(const ipc::managed_shared_memory &shm) {
    std::cout << "Shared Memory Info:" << std::endl;
    std::cout << "  Size: " << shm.get_size() << " bytes" << std::endl;
    std::cout << "  Free: " << shm.get_free_memory() << " bytes" << std::endl;
    std::cout << "  Used: " << (shm.get_size() - shm.get_free_memory())
              << " bytes" << std::endl;
  }
};