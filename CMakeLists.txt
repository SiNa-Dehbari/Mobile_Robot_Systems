cmake_minimum_required(VERSION 3.5)
project(mobile_robot_systems)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# Include directories
include_directories(include)

# Generate messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/NextOrder.msg")


# Add executable
add_executable(OrderOptimizer src/order_optimizer.cpp src/main.cpp)
ament_target_dependencies(OrderOptimizer
rclcpp
std_msgs
geometry_msgs
visualization_msgs
)

# Link against the generated message types
rosidl_target_interfaces(OrderOptimizer
${PROJECT_NAME} "rosidl_typesupport_cpp")

target_link_libraries(OrderOptimizer ${cpp_typesupport_target} yaml-cpp)

# Install executable
install(TARGETS
  OrderOptimizer
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
