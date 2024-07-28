#include "rclcpp/rclcpp.hpp"

#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <my_robot_interfaces/srv/get_map.hpp>
#include <my_robot_interfaces/srv/update_map.hpp>
#include <chrono>
#include <vector>

using namespace std::chrono_literals; // for time literals like 1s
using namespace std;

class MapPublisherNode : public rclcpp::Node
{
    public:
        MapPublisherNode() : Node("map_publisher_node")
        {
            RCLCPP_INFO(this->get_logger(), "Map Publisher Node initialized.");

            // /getmap service call that returns the map
            get_service_ = this->create_service<my_robot_interfaces::srv::GetMap>(
                "/get_map", std::bind(&MapPublisherNode::get_map , this, std::placeholders::_1, std::placeholders::_2)
            );
            // update map service call that updates the real map 
            update_service_ = this->create_service<my_robot_interfaces::srv::UpdateMap>(
                "/update_map", std::bind(&MapPublisherNode::update_map , this, std::placeholders::_1, std::placeholders::_2)
            );
            // obstacle publisher for the rviz visualizer
            pub_obstacle_marker_ = this->create_publisher<visualization_msgs::msg::Marker>("/Rviz_marker_topic/obstacles", 10);
            // timer to publish every 500 ms
            timer_ = this->create_wall_timer(500ms, std::bind(&MapPublisherNode::timer_callback, this));

            // Initialize the 3D grid
            init_map();

        }

    private:

        //service to get the map and update it 
        rclcpp::Service<my_robot_interfaces::srv::GetMap>::SharedPtr get_service_;
        rclcpp::Service<my_robot_interfaces::srv::UpdateMap>::SharedPtr update_service_;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_obstacle_marker_;
        rclcpp::TimerBase::SharedPtr timer_;

        std::vector<int> global_map;

        // Initialize the grid map
        void init_map()
        {
            vector<vector<int>> data1 = {
                {1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
                {1, 1, 1, 1, 1, 0, 0, 1, 1, 1},
                {1, 1, 1, 1, 1, 0, 0, 1, 1, 1},
                {1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
                {1, 1, 1, 1, 1, 1, 1, 0, 0, 1},
                {1, 1, 0, 1, 1, -1, 1, 1, 1, 1},
                {1, 1, 0, 1, 1, 1, 1, 1, 1, 1},
                {1, 1, 1, 1, 1, 1, 1, -1, 1, 1},
                {1, 1, 1, 0, 0, 1, 0, 1, 1, 1},
                {1, 1, 1, 1, 0, 1, 0, 1, 1, 1}
            };

            vector<vector<int>> data2 = {
                {1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
                {1, 1, 1, 0, 1, 1, 1, 0, 1, 1},
                {1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
                {1, 0, 0, 0, 1, 1, 1, 1, 1, 1},
                {1, 0, 0, 0, 1, 1, 1, 1, 1, 1},
                {1, 1, 1, 1, 1, -1, 1, 1, 1, 1},
                {1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
                {1, 1, 0, 1, 1, 1, 1, -1, 1, 1},
                {1, 1, 0, 0, 0, 1, 1, 1, 1, 1},
                {1, 1, 1, 1, 1, 1, 1, 1, 1, 1}
            };

            vector<vector<int>> data3 = {
                {1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
                {1, 1, 1, 1, 1, 1, 0, 0, 0, 1},
                {1, 0, 0, 1, 1, 1, 1, 1, 1, 1},
                {1, 1, 0, 1, 1, 1, 0, 1, 1, 1},
                {1, 1, 0, 0, 1, 1, 1, 1, 1, 1},
                {1, 1, 1, 1, 1, -1, 1, 1, 1, 1},
                {1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
                {1, 1, 0, 0, 0, 1, 1, -1, 1, 1},
                {1, 1, 0, 1, 1, 1, 1, 1, 1, 1},
                {1, 1, 1, 1, 1, 1, 1, 1, 1, 1}
            };
            
            // Append data1
            for (int i = 0; i < 10; ++i) {
                for (int j = 0; j < 10; ++j) {
                    global_map.push_back(data1[i][j]);
                }
            }
            
            // Append data2
            for (int i = 0; i < 10; ++i) {
                for (int j = 0; j < 10; ++j) {
                    global_map.push_back(data2[i][j]);
                }
            }
            
            // Append data3
            for (int i = 0; i < 10; ++i) {
                for (int j = 0; j < 10; ++j) {
                    global_map.push_back(data3[i][j]);
                }
            }

            // Append the dimensions of the matrices
            global_map.push_back(3);
            global_map.push_back(10);
            global_map.push_back(10);
        }

        // service to get map 
        void get_map(const std::shared_ptr<my_robot_interfaces::srv::GetMap::Request> request,
            std::shared_ptr<my_robot_interfaces::srv::GetMap::Response> response)
        {
            RCLCPP_INFO(this->get_logger(), "Incoming request");

            // Send the map
            response->map = global_map;

            RCLCPP_INFO(this->get_logger(), "Map sent");
        }

        //service to update it 
        void update_map(const std::shared_ptr<my_robot_interfaces::srv::UpdateMap::Request> request,
            std::shared_ptr<my_robot_interfaces::srv::UpdateMap::Response> response)
        {
            RCLCPP_INFO(this->get_logger(), "Incoming request");

            // Update the map
            global_map = request->map;

            // Send response
            response->result = true;
            RCLCPP_INFO(this->get_logger(), "Map updated");
        }

        // a callback that runs every 500 ms that can be used to execute any function calls
        void timer_callback()
        {
            obstacle_publisher();
        }


        void obstacle_publisher(){
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "world";
            marker.header.stamp = this->now();
            marker.ns = "obstacles";
            marker.id = 0;
            marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = 1.0;
            marker.scale.y = 1.0;
            marker.scale.z = 0.5;
            marker.color.a = 1.0; // Don't forget to set the alpha!
            marker.color.r = 228;
            marker.color.g = 147;
            marker.color.b = 76;

            geometry_msgs::msg::Point point;

            for (int i = 0; i < 3; ++i) {
                for (int j = 0; j < 10; ++j) {
                    for (int k = 0; k < 10; ++k) {
                        if (global_map[i * 100 + j * 10 + k] == 0) {
                            point.x = j;
                            point.y = k;
                            point.z = i;
                            marker.points.push_back(point);
                        }
                    }
                }
            }

            pub_obstacle_marker_->publish(marker);
        }
};