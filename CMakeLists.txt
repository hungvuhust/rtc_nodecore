cmake_minimum_required(VERSION 3.8)
project(rtc_nodecore)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

set(THIS_PACKAGES_DEPENDENCIES
  rclcpp
  std_srvs
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  vda5050_msgs
  rosidl_runtime_cpp
  rosidl_typesupport_cpp
  rosidl_default_runtime
  bond
)

foreach(DEP ${THIS_PACKAGES_DEPENDENCIES})
  find_package(${DEP} REQUIRED)
endforeach()

find_package(Boost REQUIRED COMPONENTS system thread)

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}_lib SHARED
  src/node_core.cpp
)
target_link_libraries(${PROJECT_NAME}_lib ${Boost_LIBRARIES})
ament_target_dependencies(${PROJECT_NAME}_lib ${THIS_PACKAGES_DEPENDENCIES})

add_executable(${PROJECT_NAME}
 src/main.cpp
)
target_link_libraries(${PROJECT_NAME} ${PROJECT_NAME}_lib)
ament_target_dependencies(${PROJECT_NAME} ${THIS_PACKAGES_DEPENDENCIES})

# Example executable
add_executable(${PROJECT_NAME}_example
  test/example_usage.cpp
)
target_link_libraries(${PROJECT_NAME}_example ${PROJECT_NAME}_lib)
ament_target_dependencies(${PROJECT_NAME}_example ${THIS_PACKAGES_DEPENDENCIES})

# ROS2 Nodes
# Master Node
add_executable(rtc_master_node src/nodes/master_node.cpp)
target_link_libraries(rtc_master_node ${PROJECT_NAME}_lib)
ament_target_dependencies(rtc_master_node ${THIS_PACKAGES_DEPENDENCIES})

# Navigation Node
add_executable(rtc_navigation_node src/nodes/navigation_node.cpp)
target_link_libraries(rtc_navigation_node ${PROJECT_NAME}_lib)
ament_target_dependencies(rtc_navigation_node ${THIS_PACKAGES_DEPENDENCIES})

# Battery Node
add_executable(rtc_battery_node src/nodes/battery_node.cpp)
target_link_libraries(rtc_battery_node ${PROJECT_NAME}_lib)
ament_target_dependencies(rtc_battery_node ${THIS_PACKAGES_DEPENDENCIES})

# Safety Node
add_executable(rtc_safety_node src/nodes/safety_node.cpp)
target_link_libraries(rtc_safety_node ${PROJECT_NAME}_lib)
ament_target_dependencies(rtc_safety_node ${THIS_PACKAGES_DEPENDENCIES})

install(TARGETS
  ${PROJECT_NAME}_lib
  DESTINATION lib
)

install(TARGETS
  ${PROJECT_NAME}
  ${PROJECT_NAME}_example
  rtc_master_node
  rtc_navigation_node
  rtc_battery_node
  rtc_safety_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  FILES_MATCHING PATTERN "*.py"
)

# Install scripts
install(PROGRAMS
  scripts/demo_rtc_agv.py
  DESTINATION lib/${PROJECT_NAME}
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  
  ament_lint_auto_find_test_dependencies()

  # Add test for NodeCore
  ament_add_gtest(test_node_core
    test/test_node_core.cpp
  )
  target_link_libraries(test_node_core ${PROJECT_NAME}_lib)
  ament_target_dependencies(test_node_core ${THIS_PACKAGES_DEPENDENCIES})

  # Add test for Shared Memory
  ament_add_gtest(test_shared_memory
    test/test_shared_memory.cpp
  )
  target_link_libraries(test_shared_memory ${PROJECT_NAME}_lib)
  ament_target_dependencies(test_shared_memory ${THIS_PACKAGES_DEPENDENCIES})
endif()

ament_package()
