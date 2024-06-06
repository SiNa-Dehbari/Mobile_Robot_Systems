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

    OrderFilesReader(directory_path_);

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

void OrderOptimizer::OrderFilesReader(const std::string directory_path_){
    std::filesystem::path  orders_path = std::filesystem::path(directory_path_) / "orders";

    // Read all .yaml files in the orders directory
    for (const auto &entry : std::filesystem::directory_iterator(orders_path))
    {
      if (entry.path().extension() == ".yaml")
      {
        //RCLCPP_INFO(this->get_logger(), "orders files: %s", entry.path().string().c_str());
        threads_.emplace_back(&OrderOptimizer::OrderFilesParser, this, entry.path().string());
      }
    }
}

void OrderOptimizer::OrderFilesParser(const std::string &order_file_path)
{
  RCLCPP_INFO(this->get_logger(), "Parsed file: %s", order_file_path.c_str());

  // Process the YAML file and store the orders
  try
    {
      YAML::Node yaml_file = YAML::LoadFile(order_file_path);
      RCLCPP_INFO(this->get_logger(), "Parsed Order file: %s", order_file_path.c_str());
      // Process the YAML file and store the orders
      for (const auto& entry : yaml_file)
      {
        uint32_t order_id = entry["order"].as<uint32_t>();
        OrderData order_data;
        order_data.cx = entry["cx"].as<double>();
        order_data.cy = entry["cy"].as<double>();
        order_data.products = entry["products"].as<std::vector<uint32_t>>();
        std::lock_guard<std::mutex> lock(orders_mutex_);
        orders_[order_id] = order_data;
      }
    }
    catch (const YAML::Exception &e)
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to parse YAML file: %s. Error: %s", order_file_path.c_str(), e.what());
    }
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