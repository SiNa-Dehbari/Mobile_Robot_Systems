#ifndef ORDER_OPTIMIZER_PUBLISHER__ORDER_OPTIMIZER_PUBLISHER_H_
#define ORDER_OPTIMIZER_PUBLISHER__ORDER_OPTIMIZER_PUBLISHER_H_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <visualization_msgs/msg/marker_array.hpp>
#include "visualization_msgs/msg/marker.hpp"
#include "mobile_robot_systems/msg/next_order.hpp"
#include <unordered_map>
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <unistd.h>
#include <limits.h>
#include <chrono>
#include <vector>
#include <mutex>

/*
structure of configuration file
*/
struct Part
    {
        std::string name;
        double cx;
        double cy;
    };

/*
structure of each product
*/
struct Product
    {
        std::string name;
        std::vector<Part> parts;
    };

/*
structure of Order files
*/
struct OrderData
    {
        double cx;
        double cy;
        std::vector<uint32_t> products;
    };

struct OrderContainer {
    uint32_t order_id;
    std::vector<Product> products;
};

class OrderOptimizer : public rclcpp::Node {
public:
    OrderOptimizer();
    /*
    Subscriber: Next order
    */
    void NextOrderSubscriber(const mobile_robot_systems::msg::NextOrder::SharedPtr msg);
    /*
    Subscriber position of the robot
    */
    void PoseSubscriber(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    /*
    Marker Visualizer
    */
   void MarkerVisualizer(const double x, double y, std::string Type);

private:

    /*
    Directory of folder (arguments)
    */
    void logDirectoryPath();

    /*
    listing  order files in the directory (one thread per each file)
    */
    void OrderFilesReader(const std::string directory_path);

    /*
    Parsing order files
    */
    void OrderFilesParser(const std::string &order_file_path);

    /*
    parsing config file
    */
    void ConfigFileParser(const std::string &directory_path);
    /*
    Optmizing the route to find the shortest path
    */
    void RouteOptimizer(const OrderContainer &order_container);

    std::string directory_path_;
    std::vector<std::thread> threads_;
    std::unordered_map<uint32_t, OrderData> orders_;
    std::unordered_map<uint32_t, Product> products_;
    std::mutex orders_mutex_;
    std::mutex products_mutex_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscriber_;
    rclcpp::Subscription<mobile_robot_systems::msg::NextOrder>::SharedPtr order_subscriber_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_publisher_;
};

#endif  // ORDER_OPTIMIZER_PUBLISHER__ORDER_OPTIMIZER_PUBLISHER_H_