#include "mobile_robot_systems/order_optimizer.h"

OrderOptimizer::OrderOptimizer() : Node("OrderOptimizer"), count_(0)
{
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    timer_ = this->create_wall_timer(
        500ms, std::bind(&OrderOptimizer::DummyPublisher, this));

    pose_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "currentPosition", 10, std::bind(&OrderOptimizer::PoseSubscriber, this, std::placeholders::_1));
}

//TODO CleanUp at the end
void OrderOptimizer::DummyPublisher()
{
    auto message = std_msgs::msg::String();
    message.data = "Hello, world! " + std::to_string(count_++);
    //RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
};

void OrderOptimizer::PoseSubscriber(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received pose: x=%.2f, y=%.2f, z=%.2f",
                msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
  }

