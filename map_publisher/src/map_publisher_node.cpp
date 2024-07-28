#include "rclcpp/rclcpp.hpp"
#include "map_publisher/Mapper.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MapPublisherNode>(); // Example: 10x10x4 grid
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
