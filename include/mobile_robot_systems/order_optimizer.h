#ifndef ORDER_OPTIMIZER_PUBLISHER__ORDER_OPTIMIZER_PUBLISHER_H_
#define ORDER_OPTIMIZER_PUBLISHER__ORDER_OPTIMIZER_PUBLISHER_H_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "mobile_robot_systems/msg/next_order.hpp"
#include <unistd.h>
#include <limits.h>
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

    void logDirectoryPath();
    std::string directory_path_;

    void PoseSubscriber(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscription_;

    void NextOrderSubscriber(const mobile_robot_systems::msg::NextOrder::SharedPtr msg);
    rclcpp::Subscription<mobile_robot_systems::msg::NextOrder>::SharedPtr subscription_;
};

#endif  // ORDER_OPTIMIZER_PUBLISHER__ORDER_OPTIMIZER_PUBLISHER_H_