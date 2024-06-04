cmake_minimum_required(VERSION 3.5)
project(mobile_robot_systems)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Include directories
include_directories(inc)

# Generate messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/NextOrder.msg")

# Add include directory
include_directories(
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
)

# Add executable
add_executable(OrderOptimizer src/order_optimizer.cpp src/main.cpp)
ament_target_dependencies(
OrderOptimizer
rclcpp
std_msgs
)

# Install executable
install(TARGETS
  OrderOptimizer
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
