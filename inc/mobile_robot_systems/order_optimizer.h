#ifndef ORDER_OPTIMIZER_PUBLISHER__ORDER_OPTIMIZER_PUBLISHER_H_
#define ORDER_OPTIMIZER_PUBLISHER__ORDER_OPTIMIZER_PUBLISHER_H_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "mobile_robot_systems/msg/next_order.hpp"
#include <chrono>

using namespace std::chrono_literals;

class OrderOptimizer : public rclcpp::Node {
public:
    OrderOptimizer();

private:
    void DummyPublisher();

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    size_t count_;
};

#endif  // ORDER_OPTIMIZER_PUBLISHER__ORDER_OPTIMIZER_PUBLISHER_H_