# RTC NodeCore Testing

Thư mục này chứa các test files và example code cho package `rtc_nodecore`.

## Cấu trúc Files

- `test_node_core.cpp` - Test chính cho class NodeCore
- `test_shared_memory.cpp` - Test cho shared memory functionality
- `example_usage.cpp` - Example demonstrating cách sử dụng library

## Chạy Tests

### Build package với tests:
```bash
cd /path/to/workspace
colcon build --packages-select rtc_nodecore
```

### Chạy tất cả tests:
```bash
colcon test --packages-select rtc_nodecore
```

### Chạy tests cụ thể:
```bash
# Trong build directory
./test/test_node_core
./test/test_shared_memory
```

### Chạy example:
```bash
ros2 run rtc_nodecore rtc_nodecore_example
```

## Test Cases

### test_node_core.cpp
- **MasterNodeCreation**: Test tạo master node
- **RegularModuleCreation**: Test tạo regular module node
- **InitializationAndShutdown**: Test khởi tạo và shutdown
- **VDA5050StateOperations**: Test VDA5050 state management
- **BatteryStateOperations**: Test battery state operations
- **ErrorManagement**: Test error handling
- **ActionStateOperations**: Test action state management
- **HeartbeatFunctionality**: Test heartbeat communication
- **MultiModuleCommunication**: Test communication giữa nhiều modules
- **UtilityFunctions**: Test utility functions

### test_shared_memory.cpp
- **MemoryCreationAndCleanup**: Test shared memory creation/cleanup
- **VDA5050StateSerialization**: Test VDA5050 message serialization
- **BondStatusSerialization**: Test Bond status message serialization
- **MultipleMessagesSerialization**: Test multiple messages
- **DeserializationTimeout**: Test timeout functionality
- **LargeMessageHandling**: Test large message handling

## Example Usage

File `example_usage.cpp` demonstrate:

1. **Basic Usage**: Tạo master node và modules
2. **Error Handling**: Quản lý errors
3. **Action States**: Quản lý action states
4. **Multi-Module Communication**: Communication giữa nhiều modules

### Chạy Example:
```bash
# Sau khi build
ros2 run rtc_nodecore rtc_nodecore_example
```

Example sẽ hiển thị:
- Tạo và quản lý nodes
- State updates
- Error handling
- Action state management
- Heartbeat communication

## Debugging Tests

### Với verbose output:
```bash
colcon test --packages-select rtc_nodecore --event-handlers console_direct+
```

### Xem test results:
```bash
colcon test-result --verbose
```

### Manual test với gdb:
```bash
cd build/rtc_nodecore
gdb ./test/test_node_core
(gdb) run
```

## Requirements

- ROS2 Humble
- vda5050_msgs package
- bond package (cho heartbeat)
- GoogleTest (ament_cmake_gtest)
- Boost.Interprocess

## Notes

- Tests tự động cleanup shared memory trước và sau mỗi test
- Các tests có thể chạy song song vì sử dụng isolated shared memory segments
- Example code có thể dùng như reference implementation cho applications thực 