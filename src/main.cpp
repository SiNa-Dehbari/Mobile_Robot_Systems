#include "rclcpp/rclcpp.hpp"
#include "mobile_robot_systems/order_optimizer.h"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OrderOptimizer>());
    rclcpp::shutdown();
    return 0;
}