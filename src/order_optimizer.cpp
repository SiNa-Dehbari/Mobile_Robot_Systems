#include "mobile_robot_systems/order_optimizer.h"

OrderOptimizer::OrderOptimizer() : Node("OrderOptimizer"), count_(0)
{
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    timer_ = this->create_wall_timer(
        500ms, std::bind(&OrderOptimizer::DummyPublisher, this));
}

//TODO CleanUp at the end
void OrderOptimizer::DummyPublisher()
{
    auto message = std_msgs::msg::String();
    message.data = "Hello, world! " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
};