#ifndef RTCROBOT_CORE__NODE_CORE_HPP_
#define RTCROBOT_CORE__NODE_CORE_HPP_

#include "rtc_nodecore/message.hpp"
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <unordered_map>

#include <bond/msg/status.hpp>
#include <vda5050_msgs/msg/action_state.hpp>
#include <vda5050_msgs/msg/battery_state.hpp>
#include <vda5050_msgs/msg/edge_state.hpp>
#include <vda5050_msgs/msg/node_state.hpp>
#include <vda5050_msgs/msg/order.hpp>
#include <vda5050_msgs/msg/safety_state.hpp>
#include <vda5050_msgs/msg/state.hpp>

#include <bond/msg/status.hpp>

namespace rtcrobot_core {

// Message types for VDA5050
enum class VDA5050MessageType { STATE = 0, HEARTBEAT = 1 };

// Base NodeCore class for inheritance
class NodeCore {
public:
  // Constructor for regular modules
  explicit NodeCore(const std::string &module_name);

  // Constructor for master node
  NodeCore();

  // Virtual destructor for inheritance
  virtual ~NodeCore();

  // Core functionality
  virtual void initialize();
  virtual void shutdown();
  virtual void release();

  // VDA5050 State operations
  bool publishState(const vda5050_msgs::msg::State &state);
  bool getLatestState(vda5050_msgs::msg::State &state);
  void setStateData(const vda5050_msgs::msg::State &state);

  // VDA5050 State Setter Methods
  void setBatteryState(const vda5050_msgs::msg::BatteryState &battery_state);
  void setAGVPosition(const vda5050_msgs::msg::AGVPosition &agv_position);
  void setVelocity(const vda5050_msgs::msg::Velocity &velocity);
  void setOperatingMode(const std::string &operating_mode);
  void setDriving(bool driving);
  void setPaused(bool paused);
  void setSafetyState(const vda5050_msgs::msg::SafetyState &safety_state);
  void addActionState(const vda5050_msgs::msg::ActionState &action_state);
  void updateActionState(const std::string &action_id,
                         const std::string &action_status,
                         const std::string &result_description = "");
  void removeActionState(const std::string &action_id);
  void clearActionStates();

  // VDA5050 Error Management
  void raiseError(const std::string &error_type,
                  const std::string &error_level       = "WARNING",
                  const std::string &error_description = "",
                  const std::string &error_hint        = "",
                  const std::vector<vda5050_msgs::msg::ErrorReference>
                      &error_references = {});
  void releaseError(const std::string &error_type);
  void releaseAllErrors();
  std::vector<vda5050_msgs::msg::Error> getErrors() const;

  // Heartbeat
  bool sendHeartbeat();

  // Status information
  std::string getModuleName() const { return module_name_; }
  bool        isMasterNode() const { return is_master_; }
  bool        isActive() const { return is_active_; }
  uint64_t    getMessageCount() const { return message_count_; }

  // Virtual callbacks for derived classes to override
  virtual void onStateReceived(const vda5050_msgs::msg::State &state) {
    (void)state;
  }
  virtual void onHeartbeatReceived(const std::string &sender_id) {
    (void)sender_id;
  }

protected:
  // Message processing loops (can be overridden)
  virtual void messageProcessingLoop();
  virtual void heartbeatLoop();

  // Utility functions
  uint64_t getCurrentTimestamp();
  uint64_t getNextSequenceId();

  // Shared memory access helpers
  bool writeToSharedMemory(const std::string &key, const std::string &data);
  bool readFromSharedMemory(const std::string &key, std::string &data);

  // Thread-safe data access
  std::mutex               data_mutex_;
  vda5050_msgs::msg::State current_state_;

private:
  // Core member variables
  std::string           module_name_;
  bool                  is_master_;
  std::atomic<bool>     is_active_;
  std::atomic<bool>     shutdown_requested_;
  std::atomic<uint64_t> sequence_counter_;
  std::atomic<uint64_t> message_count_;

  // Shared memory management
  std::unique_ptr<ipc::managed_shared_memory> shm_segment_;
  SharedData                                 *shared_data_state_;
  SharedData                                 *shared_data_heartbeat_;

  // Processing threads
  std::unique_ptr<std::thread> processing_thread_;
  std::unique_ptr<std::thread> heartbeat_thread_;

  // Internal message handling
  void handleStateMessage(const std::string &message_data);
  void handleHeartbeatMessage(const bond::msg::Status &heartbeat_status);

  // Internal initialization
  void initializeSharedMemory();
  void setupMessageChannels();

  // Timing
  std::chrono::steady_clock::time_point last_heartbeat_;
  static constexpr auto HEARTBEAT_INTERVAL = std::chrono::seconds(1);
  static constexpr auto MESSAGE_TIMEOUT    = std::chrono::milliseconds(100);

  // Shared memory keys
  static constexpr const char *STATE_SHM_KEY     = "VDA5050_STATE_CHANNEL";
  static constexpr const char *HEARTBEAT_SHM_KEY = "HEARTBEAT_CHANNEL";
};

} // namespace rtcrobot_core

#endif // RTCROBOT_CORE__NODE_CORE_HPP_