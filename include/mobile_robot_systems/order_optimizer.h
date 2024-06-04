#ifndef ORDER_OPTIMIZER_PUBLISHER__ORDER_OPTIMIZER_PUBLISHER_H_
#define ORDER_OPTIMIZER_PUBLISHER__ORDER_OPTIMIZER_PUBLISHER_H_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "mobile_robot_systems/msg/next_order.hpp"
#include <chrono>

using namespace std::chrono_literals;

class OrderOptimizer : public rclcpp::Node {
public:
    OrderOptimizer();

private:

    void DummyPublisher();

    void PoseSubscriber(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    size_t count_;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscription_;

    rclcpp::Subscription<mobile_robot_systems::msg::NextOrder>::SharedPtr subscription_;
    void NextOrderSubscriber(const mobile_robot_systems::msg::NextOrder::SharedPtr msg);
};

#endif  // ORDER_OPTIMIZER_PUBLISHER__ORDER_OPTIMIZER_PUBLISHER_H_