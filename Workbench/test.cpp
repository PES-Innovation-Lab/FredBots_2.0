// #include<iostream>
// #include<vector>
// using namespace std;


// int main (){
//     // Define the three 10x10 matrices
//     int data1[10][10] = {
//         {1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
//         {1, 1, 1, 1, -1, 1, 1, 0, 1, 1},
//         {1, 1, 0, 1, 1, 1, 1, 0, 1, 1},
//         {1, 1, 0, 1, 1, 1, 1, 0, 1, 1},
//         {1, 1, 0, 1, -1, 1, 1, 1, 1, 1},
//         {1, 1, 1, 1, 0, 1, 1, 0, 1, 1},
//         {1, 1, 0, 1, 1, 1, 1, 1, 1, 0},
//         {0, 1, 0, 1, -1, 1, 1, -1, 1, 1},
//         {0, 0, 0, 0, 1, 0, 1, 1, 1, 1},
//         {1, 1, 1, 1, 1, 1, 1, 0, 0, 1}
//     };

//     int data2[10][10] = {
//         {1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
//         {1, 1, 1, 0, -1, 0, 1, 1, 1, 0},
//         {1, 1, 1, 0, 1, 1, 1, 1, 1, 0},
//         {1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
//         {1, 1, 1, 1, -1, 1, 1, 1, 1, 0},
//         {1, 1, 1, 0, 1, 0, 1, 1, 1, 0},
//         {1, 1, 1, 0, 1, 0, 0, 0, 0, 1},
//         {1, 1, 1, 0, -1, 0, 0, -1, 0, 1},
//         {1, 1, 1, 1, 1, 1, 0, 1, 0, 1},
//         {1, 1, 1, 1, 1, 1, 1, 1, 1, 1}
//     };

//     int data3[10][10] = {
//         {0, 0, 1, 1, 1, 1, 1, 1, 1, 1},
//         {1, 1, 1, 1, -1, 1, 1, 1, 0, 0},
//         {0, 1, 1, 1, 1, 1, 1, 0, 1, 1},
//         {0, 1, 1, 1, 0, 0, 0, 0, 1, 1},
//         {1, 1, 1, 1, -1, 1, 1, 1, 1, 1},
//         {1, 1, 1, 1, 1, 0, 1, 1, 1, 1},
//         {1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
//         {1, 1, 1, 1, -1, 1, 0, -1, 0, 1},
//         {1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
//         {0, 0, 1, 1, 1, 1, 1, 1, 1, 1}
//     };

//     // Concatenate the matrices into a single vector
//     std::vector<int> mapp;
    
//     // Append data1
//     for (int i = 0; i < 10; ++i) {
//         for (int j = 0; j < 10; ++j) {
//             mapp.push_back(data1[i][j]);
//         }
//     }
    
//     // Append data2
//     for (int i = 0; i < 10; ++i) {
//         for (int j = 0; j < 10; ++j) {
//             mapp.push_back(data2[i][j]);
//         }
//     }
    
//     // Append data3
//     for (int i = 0; i < 10; ++i) {
//         for (int j = 0; j < 10; ++j) {
//             mapp.push_back(data3[i][j]);
//         }
//     }


//     // Append the dimensions of the matrices
//     mapp.push_back(10);
//     mapp.push_back(10);
//     mapp.push_back(3);


//     cout<<mapp.size()<<endl;

// }


#include <vector>
#include <queue>
#include <cmath>
#include <limits>
#include <algorithm>
#include <optional>
#include <bits/stdc++.h>
#include <iostream>
#include <geometry_msgs/msg/point.hpp>
#include <PathPlanners/PathPlanningLibv2.hpp>
#include <PathPlanners/A_star.hpp>
#include <PathPlanners/Bidirectional_A_Star.hpp>
#include <PathPlanners/Dijkstra.hpp>

struct Path_Node
{
    geometry_msgs::msg::Point point;
    geometry_msgs::msg::Point parent;
    int G_cost;
    bool operator<(const Path_Node& other) const
    {
        return G_cost > other.G_cost; // Use '>' for min-heap
    }
};

int main(){
    std::string logging_message = "Hello from main";
    std::vector<std::vector<std::vector<int>>> map = {
        {
            {1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
            {1, 1, 1, 1, -1, 1, 1, 0, 1, 1},
            {1, 1, 0, 1, 1, 1, 1, 0, 1, 1},
            {1, 1, 0, 1, 1, 1, 1, 0, 1, 1},
            {1, 1, 0, 1, -1, 1, 1, 1, 1, 1},
            {1, 1, 1, 1, 0, 1, 1, 0, 1, 1},
            {1, 1, 0, 1, 1, 1, 1, 1, 1, 0},
            {0, 1, 0, 1, -1, 1, 1, -1, 1, 1},
            {0, 0, 0, 0, 1, 0, 1, 1, 1, 1},
            {1, 1, 1, 1, 1, 1, 1, 0, 0, 1}
        },
        {
            {1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
            {1, 1, 1, 0, -1, 0, 1, 1, 1, 0},
            {1, 1, 1, 0, 1, 1, 1, 1, 1, 0},
            {1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
            {1, 1, 1, 1, -1, 1, 1, 1, 1, 0},
            {1, 1, 1, 0, 1, 0, 1, 1, 1, 0},
            {1, 1, 1, 0, 1, 0, 0, 0, 0, 1},
            {1, 1, 1, 0, -1, 0, 0, -1, 0, 1},
            {1, 1, 1, 1, 1, 1, 0, 1, 1, 1},
            {1, 1, 1, 1, 1, 1, 1, 1, 1, 1}
        },
        {
            {1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
            {1, 1, 1, 0, -1, 0, 1, 1, 1, 0},
            {1, 1, 1, 0, 1, 1, 1, 1, 1, 0},
            {1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
            {1, 1, 1, 1, -1, 1, 1, 1, 1, 0},
            {1, 1, 1, 0, 1, 0, 1, 1, 1, 0},
            {1, 1, 1, 0, 1, 0, 0, 0, 0, 1},
            {1, 1, 1, 0, -1, 0, 0, -1, 0, 1},
            {1, 1, 1, 1, 1, 1, 0, 1, 0, 1},
            {1, 1, 1, 1, 1, 1, 1, 1, 1, 1}
        }
    };

    AStar astar;
    BID_A_Star bid_astar;
    Dijkstra dijkstra;


    // std::vector<int> start = {0, 0, 0};
    // std::vector<int> goal = {9, 9, 1};
    geometry_msgs::msg::Point start , goal;

    for(int i=0; i<3; i++){
        start.x = 0+i;
        start.y = 0+i;
        start.z = 0+i;
        goal.x = 9-i;
        goal.y = 9-i;
        goal.z = 2-i;

        logging_message += "Start: " + std::to_string(start.x) + " " + std::to_string(start.y) + " " + std::to_string(start.z) + "\n";
        logging_message += "Goal: " + std::to_string(goal.x) + " " + std::to_string(goal.y) + " " + std::to_string(goal.z) + "\n";

        int start_time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

        std::vector<geometry_msgs::msg::Point> path = astar.get_plan(map, start, goal);
        int end_time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

        logging_message += "A* Time: " + std::to_string(end_time - start_time) + "\n";

        logging_message += "Path length: " + std::to_string(path.size()) + "\n";
        for(auto point : path){
            logging_message += "Path: " + std::to_string(point.x) + " " + std::to_string(point.y) + " " + std::to_string(point.z) + "\n";
        }

        //dijkstra 
        start_time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

        path = dijkstra.get_plan(map, start, goal);
        end_time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

        logging_message += "A* Time: " + std::to_string(end_time - start_time) + "\n";

        logging_message += "Path length: " + std::to_string(path.size()) + "\n";
        for(auto point : path){
            logging_message += "Path: " + std::to_string(point.x) + " " + std::to_string(point.y) + " " + std::to_string(point.z) + "\n";
        }

        //bidirectional A*
        start_time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

        path = bid_astar.get_plan(map, start, goal);
        end_time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

        logging_message += "A* Time: " + std::to_string(end_time - start_time) + "\n";

        logging_message += "Path length: " + std::to_string(path.size()) + "\n";
        for(auto point : path){
            logging_message += "Path: " + std::to_string(point.x) + " " + std::to_string(point.y) + " " + std::to_string(point.z) + "\n";
        }

    }

    //write it to a file
    std::ofstream file;
    file.open("/home/neo/Robotics/ros2_ws/Logs/logv2.txt");
    file << logging_message;
    file.close();

}