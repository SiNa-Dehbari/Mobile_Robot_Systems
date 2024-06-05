#include "mobile_robot_systems/order_optimizer.h"

OrderOptimizer::OrderOptimizer() : Node("OrderOptimizer"), count_(0)
{
    // Dummy Publisher_ to be cleaned
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    timer_ = this->create_wall_timer(
        500ms, std::bind(&OrderOptimizer::DummyPublisher, this));

    // File path arguments
    this->declare_parameter<std::string>("directory_path", "");

    std::string param_directory_path;
    if (this->get_parameter("directory_path", param_directory_path) && !param_directory_path.empty()) {
        directory_path_ = param_directory_path;
    } else {
        // Get the current working directory
        char cwd[PATH_MAX];
        getcwd(cwd, sizeof(cwd));
        directory_path_ = std::string(cwd) + "/src/mobile_robot_systems/files";
        RCLCPP_WARN(this->get_logger(), "Directory path parameter not set, using default: %s", directory_path_.c_str());
    }
    logDirectoryPath();
    //
    pose_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "currentPosition", 10, std::bind(&OrderOptimizer::PoseSubscriber, this, std::placeholders::_1));

    subscription_ = this->create_subscription<mobile_robot_systems::msg::NextOrder>(
      "nextOrder", 10, std::bind(&OrderOptimizer::NextOrderSubscriber, this, std::placeholders::_1));
}

//TODO CleanUp at the end
void OrderOptimizer::DummyPublisher()
{
    auto message = std_msgs::msg::String();
    message.data = "Hello, world! " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
};

void OrderOptimizer::logDirectoryPath() {
    RCLCPP_INFO(this->get_logger(), "Using directory path: %s", directory_path_.c_str());
}

void OrderOptimizer::PoseSubscriber(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received pose: x=%.2f, y=%.2f, z=%.2f",
                msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
  }

void OrderOptimizer::NextOrderSubscriber(const mobile_robot_systems::msg::NextOrder::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Received order_id: %d, description: %s", msg->order_id, msg->description.c_str());
}