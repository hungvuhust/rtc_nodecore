# RTC NodeCore

[![ROS2](https://img.shields.io/badge/ros2-humble-blue.svg)](https://docs.ros.org/en/humble/)
[![C++](https://img.shields.io/badge/c++-17-blue.svg)](https://en.cppreference.com/)
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](LICENSE)

**RTC NodeCore** là một thư viện C++ cho ROS2 cung cấp khung làm việc để phát triển các module giao tiếp với nhau thông qua shared memory, tuân thủ chuẩn VDA5050 cho AGV (Automated Guided Vehicle).

## 🚀 Tính năng chính

- ✅ **VDA5050 Compliance**: Hỗ trợ đầy đủ VDA5050 state management
- ✅ **Shared Memory Communication**: Giao tiếp hiệu suất cao giữa các modules
- ✅ **Master-Slave Architecture**: Quản lý tập trung với multiple modules
- ✅ **Heartbeat Monitoring**: Giám sát trạng thái modules qua Bond messages
- ✅ **Error Management**: Hệ thống quản lý lỗi theo chuẩn VDA5050
- ✅ **Action State Tracking**: Theo dõi và quản lý trạng thái các actions
- ✅ **Thread-Safe**: Đảm bảo an toàn khi sử dụng multi-threading
- ✅ **Type-Safe Serialization**: Sử dụng ROS2 serialization cho type safety

## 🛠️ Dependencies

### Bắt buộc:
- **ROS2 Humble** (hoặc tương tự)
- **vda5050_msgs** - VDA5050 message definitions
- **bond** - ROS2 bond package cho heartbeat
- **Boost.Interprocess** - Shared memory management
- **rclcpp** - ROS2 C++ client library

### Để development:
- **ament_cmake** - Build system
- **ament_cmake_gtest** - Testing framework


## 🎯 Cách sử dụng cơ bản

### 1. Tạo Master Node

```cpp
#include "rtc_nodecore/node_core.hpp"

// Tạo master node (quản lý shared memory)
auto master_node = std::make_unique<rtcrobot_core::NodeCore>();
master_node->initialize();
```

### 2. Tạo Module Node

```cpp
// Tạo module node
auto navigation_module = std::make_unique<rtcrobot_core::NodeCore>("navigation");
navigation_module->initialize();
```

### 3. Quản lý VDA5050 State

```cpp
// Thiết lập battery state
vda5050_msgs::msg::BatteryState battery;
battery.battery_charge = 85.5;
battery.charging = false;
master_node->setBatteryState(battery);

// Thiết lập driving state
master_node->setDriving(true);
master_node->setOperatingMode("AUTOMATIC");

// Lấy state hiện tại
vda5050_msgs::msg::State current_state;
master_node->getLatestState(current_state);
```

### 4. Quản lý Errors

```cpp
// Raise một error
master_node->raiseError("BATTERY_LOW", "WARNING", 
                       "Battery level below 20%", 
                       "Please charge the AGV soon");

// Release error
master_node->releaseError("BATTERY_LOW");

// Release tất cả errors
master_node->releaseAllErrors();
```

### 5. Quản lý Action States

```cpp
// Thêm action state
vda5050_msgs::msg::ActionState action;
action.action_id = "move_001";
action.action_type = "move";
action.action_status = "RUNNING";
master_node->addActionState(action);

// Update action state
master_node->updateActionState("move_001", "FINISHED", "Task completed");

// Remove action state
master_node->removeActionState("move_001");
```

## 🏗️ Architecture

### Master-Slave Pattern

```
┌─────────────────┐    Shared Memory    ┌─────────────────┐
│   Master Node   │◄──────────────────►│  Module Node 1  │
│                 │                     │   (Navigation)  │
└─────────────────┘                     └─────────────────┘
         ▲                                        ▲
         │                                        │
         ▼                                        ▼
┌─────────────────┐                     ┌─────────────────┐
│  Module Node 2  │                     │  Module Node 3  │
│   (Battery)     │                     │    (Safety)     │
└─────────────────┘                     └─────────────────┘
```

### Communication Channels

- **State Channel**: VDA5050 State messages
- **Heartbeat Channel**: Bond Status messages để monitoring

### Shared Memory Layout

```
┌─────────────────────────────────────────┐
│            Shared Memory                │
├─────────────────────────────────────────┤
│  VDA5050_STATE_CHANNEL                 │
│  ├─ Mutex & Condition Variable          │
│  ├─ Serialized VDA5050 State Data      │
│  └─ Control Block (metadata)           │
├─────────────────────────────────────────┤
│  HEARTBEAT_CHANNEL                     │
│  ├─ Mutex & Condition Variable          │
│  ├─ Serialized Bond Status Data        │
│  └─ Control Block (metadata)           │
└─────────────────────────────────────────┘
```

## 📚 API Reference

### NodeCore Class

#### Constructors
```cpp
// Tạo master node
NodeCore();

// Tạo module node
explicit NodeCore(const std::string& module_name);
```

#### Core Methods
```cpp
void initialize();                    // Khởi tạo node
void shutdown();                      // Shutdown node
bool isActive() const;               // Kiểm tra trạng thái active
std::string getModuleName() const;   // Lấy tên module
bool isMasterNode() const;           // Kiểm tra có phải master node
```

#### VDA5050 State Management
```cpp
// State operations
bool publishState(const vda5050_msgs::msg::State& state);
bool getLatestState(vda5050_msgs::msg::State& state);
void setStateData(const vda5050_msgs::msg::State& state);

// Individual state setters
void setBatteryState(const vda5050_msgs::msg::BatteryState& battery_state);
void setAGVPosition(const vda5050_msgs::msg::AGVPosition& agv_position);
void setVelocity(const vda5050_msgs::msg::Velocity& velocity);
void setOperatingMode(const std::string& operating_mode);
void setDriving(bool driving);
void setPaused(bool paused);
void setSafetyState(const vda5050_msgs::msg::SafetyState& safety_state);
```

#### Error Management
```cpp
void raiseError(const std::string& error_type,
               const std::string& error_level = "WARNING",
               const std::string& error_description = "",
               const std::string& error_hint = "");
void releaseError(const std::string& error_type);
void releaseAllErrors();
std::vector<vda5050_msgs::msg::Error> getErrors() const;
```

#### Action State Management
```cpp
void addActionState(const vda5050_msgs::msg::ActionState& action_state);
void updateActionState(const std::string& action_id,
                      const std::string& action_status,
                      const std::string& result_description = "");
void removeActionState(const std::string& action_id);
void clearActionStates();
```

#### Heartbeat
```cpp
bool sendHeartbeat();                // Gửi heartbeat manual
```

#### Virtual Callbacks (để override)
```cpp
virtual void onStateReceived(const vda5050_msgs::msg::State& state);
virtual void onHeartbeatReceived(const std::string& sender_id);
```

## 📝 Examples

### Custom Module với Callbacks

```cpp
class NavigationModule : public rtcrobot_core::NodeCore {
public:
  explicit NavigationModule() : rtcrobot_core::NodeCore("navigation") {}

protected:
  void onStateReceived(const vda5050_msgs::msg::State& state) override {
    std::cout << "Navigation received state update - "
              << "Driving: " << (state.driving ? "true" : "false")
              << ", Mode: " << state.operating_mode << std::endl;
    
    // Process navigation logic based on state
    if (state.driving && state.operating_mode == "AUTOMATIC") {
      startAutomaticNavigation();
    }
  }

  void onHeartbeatReceived(const std::string& sender_id) override {
    std::cout << "Navigation: Heartbeat from " << sender_id << std::endl;
  }

private:
  void startAutomaticNavigation() {
    // Implementation navigation logic
  }
};
```

### Multi-Module Application

```cpp
int main() {
  // Tạo master node
  auto master = std::make_unique<rtcrobot_core::NodeCore>();
  master->initialize();

  // Tạo các modules
  auto navigation = std::make_unique<NavigationModule>();
  auto battery_monitor = std::make_unique<BatteryModule>();
  auto safety_controller = std::make_unique<SafetyModule>();

  navigation->initialize();
  battery_monitor->initialize();
  safety_controller->initialize();

  // Main application loop
  while (rclcpp::ok()) {
    // Update states, handle business logic
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  // Cleanup
  safety_controller->shutdown();
  battery_monitor->shutdown();
  navigation->shutdown();
  master->shutdown();

  return 0;
}
```

## 🧪 Testing

### Chạy Tests

```bash
# Build với tests
colcon build --packages-select rtc_nodecore

# Chạy tất cả tests
colcon test --packages-select rtc_nodecore

# Xem kết quả tests
colcon test-result --verbose
```

### Chạy Example

```bash
# Chạy example demo
ros2 run rtc_nodecore rtc_nodecore_example
```

### Test Coverage

Package bao gồm comprehensive tests cho:
- ✅ NodeCore creation và initialization
- ✅ VDA5050 state operations
- ✅ Error management
- ✅ Action state management
- ✅ Heartbeat communication
- ✅ Multi-module communication
- ✅ Shared memory operations
- ✅ Serialization/deserialization
- ✅ Timeout handling

## ⚙️ Configuration

### Shared Memory Settings

Có thể tùy chỉnh trong `include/rtc_nodecore/message.hpp`:

```cpp
struct ShmConfig {
  static constexpr const char* SHM_NAME = "ROSSharedMemoryImproved";
  static constexpr size_t INITIAL_SIZE = 4096;     // 4KB initial
  static constexpr size_t MAX_SIZE = 1024 * 1024;  // 1MB max
};
```

### Heartbeat Settings

```cpp
// Trong NodeCore class
static constexpr auto HEARTBEAT_INTERVAL = std::chrono::seconds(1);
static constexpr auto MESSAGE_TIMEOUT = std::chrono::milliseconds(100);
```

## 🔧 Troubleshooting

### Common Issues

**1. Shared memory permission errors:**
```bash
# Clean up shared memory
sudo ipcs -m | grep ROSSharedMemoryImproved | awk '{print $2}' | xargs sudo ipcrm -m
```

**2. Build errors với vda5050_msgs:**
```bash
# Ensure vda5050_msgs được build trước
colcon build --packages-select vda5050_msgs
source install/setup.bash
colcon build --packages-select rtc_nodecore
```

**3. Test failures:**
```bash
# Chạy tests với verbose output
colcon test --packages-select rtc_nodecore --event-handlers console_direct+
```

### Debug Mode

```cpp
// Enable debug output trong code
std::cout.setf(std::ios::unitbuf); // Flush immediately
```

## 🤝 Contributing

1. Fork repository
2. Tạo feature branch: `git checkout -b feature/amazing-feature`
3. Commit changes: `git commit -m 'Add amazing feature'`
4. Push branch: `git push origin feature/amazing-feature`
5. Tạo Pull Request

### Coding Standards

- Sử dụng C++17 standards
- Follow ROS2 naming conventions
- Thêm comprehensive tests cho new features
- Document public APIs
- Sử dụng meaningful commit messages

## 📄 License

Distributed under the Apache License 2.0. See `LICENSE` for more information.

## 👥 Authors

- **RTC Technology JSC** - *Initial work* - [rtc-agv](mailto:agv04@rtc.edu.vn)

## 🔗 Related Projects

- [vda5050_msgs](https://github.com/vda5050/vda5050_msgs) - VDA5050 message definitions
- [ROS2 Bond](https://github.com/ros/bond_core) - Bond library cho node monitoring

## 📈 Roadmap

- [ ] Support cho multiple shared memory segments
- [ ] Performance metrics và monitoring
- [ ] Configuration file support
- [ ] Docker containerization
- [ ] Integration với ROS2 lifecycle nodes
- [ ] Support cho distributed systems

---

Để biết thêm thông tin, vui lòng liên hệ team development tại [agv04@rtc.edu.vn](mailto:agv04@rtc.edu.vn)
