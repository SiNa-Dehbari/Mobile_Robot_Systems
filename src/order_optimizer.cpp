#include "mobile_robot_systems/order_optimizer.h"

OrderOptimizer::OrderOptimizer() : Node("OrderOptimizer")
{
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


    ConfigFileParser(directory_path_);

    //
    pose_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "currentPosition", 10, std::bind(&OrderOptimizer::PoseSubscriber, this, std::placeholders::_1));

    subscription_ = this->create_subscription<mobile_robot_systems::msg::NextOrder>(
      "nextOrder", 10, std::bind(&OrderOptimizer::NextOrderSubscriber, this, std::placeholders::_1));

    marker_array_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("order_path", 10);

}

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
  //RCLCPP_INFO(this->get_logger(), "Parsed file: %s", order_file_path.c_str());

  // Process the YAML file and store the orders
  try
    {
      YAML::Node yaml_file = YAML::LoadFile(order_file_path);
      //RCLCPP_INFO(this->get_logger(), "Parsed Order file: %s", order_file_path.c_str());
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
      RCLCPP_ERROR(this->get_logger(), "Unknow YAML file Format");
    }
}

void OrderOptimizer::ConfigFileParser(const std::string &directory_path)
{
  std::filesystem::path config_path = std::filesystem::path(directory_path) / "configuration/products.yaml";
  try
    {
      YAML::Node config_file = YAML::LoadFile(config_path);
      RCLCPP_INFO(this->get_logger(), "Parsed configuration file: %s", config_path.c_str());

      for (const auto& entry : config_file)
      {
        Product product;
        product.name = entry["product"].as<std::string>();
        for (const auto& part : entry["parts"])
        {
          Part part_data;
          part_data.name = part["part"].as<std::string>();
          part_data.cx = part["cx"].as<double>();
          part_data.cy = part["cy"].as<double>();
          product.parts.push_back(part_data);
        }
        products_[entry["id"].as<uint32_t>()] = product;
      }
    }
    catch (const YAML::Exception &e)
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to parse configuration file: %s. Error: %s", config_path.c_str(), e.what());
    }
}

void OrderOptimizer::PoseSubscriber(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received pose: x=%.2f, y=%.2f, z=%.2f",
                msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    MarkerVisualizer(msg->pose.position.x, msg->pose.position.y, "Robot");
  }

void OrderOptimizer::NextOrderSubscriber(const mobile_robot_systems::msg::NextOrder::SharedPtr msg)
{
  uint32_t order_id = msg->order_id;

  RCLCPP_INFO(this->get_logger(), "Working on order: %u , %s", order_id, msg->description.c_str());
  OrderFilesReader(directory_path_);
  //std::lock_guard<std::mutex> lock(orders_mutex_);
  std::lock_guard<std::mutex> lock(products_mutex_);

  /*
  for (const auto &order_pair : orders_) {
      RCLCPP_INFO(this->get_logger(), "Order ID: %u", order_pair.first);
  }
  */


  auto orders = orders_.find(order_id);
  if (orders != orders_.end())
  {
    const OrderData &order_data = orders ->second;
    //RCLCPP_WARN(this->get_logger(), "Order ID: %u  found", order_id);

    OrderContainer order_container;
    order_container.order_id = order_id;

    for (const auto &products : order_data.products)
    {
        //RCLCPP_INFO(this->get_logger(), "Product ID: %u", products);
        auto product = products_.find(products);
        const Product &product_data = product ->second;

        order_container.products.push_back(product_data);
    RouteOptimizer(order_container);
    }
  }     else {
        RCLCPP_WARN(this->get_logger(), "unspecified Order ID %u .", order_id);
    }
}

void OrderOptimizer::RouteOptimizer(const OrderContainer &order_container)
{
  const auto& order = orders_[order_container.order_id];
  std::vector<std::pair<Part, std::string>> parts_to_fetch;
  for (const auto &product : order_container.products) {
      //RCLCPP_INFO(this->get_logger(), "Product: %s", product.name.c_str());
      for (const auto &part : product.parts) {
          parts_to_fetch.emplace_back(part, product.name);
          //RCLCPP_INFO(this->get_logger(), "Part: %s, cx: %f, cy: %f", part.name.c_str(), part.cx, part.cy);
      }
  }
      // Sort parts by proximity
      std::vector<bool> picked(parts_to_fetch.size(), false);
      double current_x = 0.0, current_y = 0.0;
      size_t count = 0;

      while (count < parts_to_fetch.size())
      {
        // Find the closest part
        double min_distance = std::numeric_limits<double>::max();
        size_t closest_index = 0;

        for (size_t i = 0; i < parts_to_fetch.size(); ++i)
        {
          if (!picked[i])
          {
            // Finding the closest part
            double distance = std::sqrt(std::pow(parts_to_fetch[i].first.cx - current_x, 2) + std::pow(parts_to_fetch[i].first.cy - current_y, 2));
            if (distance < min_distance)
            {
              min_distance = distance;
              closest_index = i;
            }
          }
        }
        // Pick all parts at the closest location
        for (size_t i = 0; i < parts_to_fetch.size(); ++i)
        {
          if (!picked[i] &&
            parts_to_fetch[i].first.cx == parts_to_fetch[closest_index].first.cx &&
            parts_to_fetch[i].first.cy == parts_to_fetch[closest_index].first.cy)

          {
            RCLCPP_INFO(this->get_logger(), "%zu. Fetching part '%s' for '%s' at x: %f, y: %f",
            count + 1, parts_to_fetch[i].first.name.c_str(), parts_to_fetch[i].second.c_str(),
            parts_to_fetch[i].first.cx, parts_to_fetch[i].first.cy);
            MarkerVisualizer(parts_to_fetch[i].first.cx, parts_to_fetch[i].first.cy, "Part");
            picked[i] = true;
            ++count;
          }
        }

        // Update the current location to the location of the picked parts
        current_x = parts_to_fetch[closest_index].first.cx;
        current_y = parts_to_fetch[closest_index].first.cy;
      }

      RCLCPP_INFO(this->get_logger(), "%zu. Delivering to destination x: %f, y: %f",
                  count + 1, order.cx, order.cy);
}

void OrderOptimizer::MarkerVisualizer(const double x, double y, std::string Type)
{
    auto marker_array = visualization_msgs::msg::MarkerArray();
    static int id = 0;

    // Create a CUBE marker for single (x, y) coordinate
    visualization_msgs::msg::Marker cube_marker;
    cube_marker.header.frame_id = "map";
    cube_marker.header.stamp = this->now();
    cube_marker.ns = "basic_shapes";
    cube_marker.id = id++;
    if (Type == "Part"){
      cube_marker.type = visualization_msgs::msg::Marker::CYLINDER;
    } else if (Type == "Robot"){
      cube_marker.type = visualization_msgs::msg::Marker::CUBE;
    }
    cube_marker.action = visualization_msgs::msg::Marker::ADD;
    cube_marker.pose.position.x = x;
    cube_marker.pose.position.y = y;
    cube_marker.pose.position.z = 0.0;
    cube_marker.pose.orientation.x = 0.0;
    cube_marker.pose.orientation.y = 0.0;
    cube_marker.pose.orientation.z = 0.0;
    cube_marker.pose.orientation.w = 1.0;
    cube_marker.scale.x = 1.0;
    cube_marker.scale.y = 1.0;
    cube_marker.scale.z = 1.0;
    cube_marker.color.r = 0.0f;
    cube_marker.color.g = 1.0f;
    cube_marker.color.b = 0.0f;
    cube_marker.color.a = 1.0f;
    marker_array.markers.push_back(cube_marker);
    marker_array_publisher_->publish(marker_array);
}