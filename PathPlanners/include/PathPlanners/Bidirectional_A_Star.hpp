#ifndef BID_A_STAR_HPP
#define BID_A_STAR_HPP

#include <bits/stdc++.h>
#include <cmath>
#include <limits>
#include <algorithm>
#include <iostream>
#include <geometry_msgs/msg/point.hpp>
#include <PathPlanners/PathPlanningLibv2.hpp>

class BID_A_Star : public PathPlanners {
public:
    BID_A_Star() {
        std::cout << "BID_A_Star obj init" << std::endl;
    }

    std::vector<geometry_msgs::msg::Point> get_plan(std::vector<std::vector<std::vector<int>>> map, geometry_msgs::msg::Point start, geometry_msgs::msg::Point goal) override {
        this->map = map;
        this->goal = goal;
        this->start = start;
        this->diagonal_traversal = false; // Set diagonal traversal if needed
        this->print_map();
        return find_path(start, goal);
    }

private:
    std::vector<std::vector<std::vector<int>>> map;
    geometry_msgs::msg::Point start;
    geometry_msgs::msg::Point goal;
    bool diagonal_traversal;

    std::vector<geometry_msgs::msg::Point> find_path(geometry_msgs::msg::Point start, geometry_msgs::msg::Point goal) {
        if (start.z == goal.z) {
            return bidirectional_a_star(start, goal);
        } else {
            geometry_msgs::msg::Point start_elevator = find_nearest_elevator(start);
            if (start_elevator.y == -1)
                return {};

            geometry_msgs::msg::Point dest_elevator = start_elevator;
            dest_elevator.z = goal.z;

            auto source_to_elevator = bidirectional_a_star(start, start_elevator);
            auto elevator_to_dest = bidirectional_a_star(dest_elevator, goal);

            if (!source_to_elevator.empty() && !elevator_to_dest.empty()) {
                source_to_elevator.insert(source_to_elevator.end(), elevator_to_dest.begin(), elevator_to_dest.end());
                return source_to_elevator;
            } else {
                std::cout << "Error - 03 : failed to get source to destination, try a different map" << std::endl;
                return {};
            }
        }
    }

    std::vector<geometry_msgs::msg::Point> bidirectional_a_star(geometry_msgs::msg::Point start, geometry_msgs::msg::Point goal) {
        std::priority_queue<std::shared_ptr<Path_Node>, std::vector<std::shared_ptr<Path_Node>>, CompareNode> forward_pq, backward_pq;
        std::unordered_map<std::string, std::shared_ptr<Path_Node>> forward_visited, backward_visited;

        auto start_node = std::make_shared<Path_Node>();
        start_node->point = start;
        start_node->parent = nullptr;
        start_node->G_cost = 0;
        start_node->F_cost = heuristic(start, goal);

        auto goal_node = std::make_shared<Path_Node>();
        goal_node->point = goal;
        goal_node->parent = nullptr;
        goal_node->G_cost = 0;
        goal_node->F_cost = heuristic(goal, start);

        forward_pq.push(start_node);
        backward_pq.push(goal_node);

        forward_visited[node_key(start_node)] = start_node;
        backward_visited[node_key(goal_node)] = goal_node;

        while (!forward_pq.empty() && !backward_pq.empty()) {
            auto forward_result = process_node(forward_pq, forward_visited, backward_visited, goal, false);
            if (!forward_result.empty()) return forward_result;

            auto backward_result = process_node(backward_pq, backward_visited, forward_visited, start, true);
            if (!backward_result.empty()) return backward_result;
        }

        return {};
    }

    std::vector<geometry_msgs::msg::Point> process_node(std::priority_queue<std::shared_ptr<Path_Node>, std::vector<std::shared_ptr<Path_Node>>, CompareNode>& pq,
                                                        std::unordered_map<std::string, std::shared_ptr<Path_Node>>& visited,
                                                        std::unordered_map<std::string, std::shared_ptr<Path_Node>>& other_visited,
                                                        geometry_msgs::msg::Point target,
                                                        bool reverse) {
        auto current = pq.top();
        pq.pop();

        int x = static_cast<int>(current->point.x);
        int y = static_cast<int>(current->point.y);
        int z = static_cast<int>(current->point.z);

        std::string key = node_key(current);
        if (other_visited.find(key) != other_visited.end()) {
            return reconstruct_bidirectional_path(current, other_visited[key], reverse);
        }

        std::vector<std::pair<int, int>> directions;
        if (this->diagonal_traversal) {
            directions = {{0, 1}, {0, -1}, {1, 0}, {-1, 0}, {-1, 1}, {1, 1}, {1, -1}, {-1, -1}};
        } else {
            directions = {{0, 1}, {0, -1}, {1, 0}, {-1, 0}};
        }

        for (const auto& [dy, dx] : directions) {
            int new_y = y + dy;
            int new_x = x + dx;

            if (new_y >= 0 && new_y < static_cast<int>(map[0].size()) && new_x >= 0 && new_x < static_cast<int>(map[0][0].size()) && map[z][new_y][new_x] != 0) {
                int new_G_cost = current->G_cost + 1;

                if (diagonal_traversal && dx != 0 && dy != 0) {
                    new_G_cost += 1;
                }
                
                if (map[z][new_y][new_x] > 1) {
                    new_G_cost += 2*map[z][new_y][new_x];
                }

                auto new_node = std::make_shared<Path_Node>();
                new_node->point.z = static_cast<double>(z);
                new_node->point.y = static_cast<double>(new_y);
                new_node->point.x = static_cast<double>(new_x);
                new_node->parent = current;
                new_node->G_cost = new_G_cost;
                new_node->F_cost = new_G_cost + heuristic(new_node->point, target);

                std::string new_key = node_key(new_node);
                if (visited.find(new_key) == visited.end() || new_node->G_cost < visited[new_key]->G_cost) {
                    visited[new_key] = new_node;
                    pq.push(new_node);
                }
            }
        }

        return {};
    }

    std::string node_key(const std::shared_ptr<Path_Node>& node) {
        return std::to_string(static_cast<int>(node->point.z)) + "," +
               std::to_string(static_cast<int>(node->point.y)) + "," +
               std::to_string(static_cast<int>(node->point.x));
    }

    geometry_msgs::msg::Point find_nearest_elevator(geometry_msgs::msg::Point start) {
        int min_dist = INT_MAX;
        geometry_msgs::msg::Point elev;
        elev.z = start.z;
        elev.y = -1;
        elev.x = -1;

        for (int i = 0; i < static_cast<int>(map[0].size()); ++i) {
            for (int j = 0; j < static_cast<int>(map[0][0].size()); ++j) {
                if (map[start.z][i][j] == -1) {
                    int dist = std::abs(i - start.y) + std::abs(start.x - j);
                    if (min_dist > dist) {
                        min_dist = dist;
                        elev.y = i;
                        elev.x = j;
                    }
                }
            }
        }
        return elev;
    }

    inline int heuristic(geometry_msgs::msg::Point start, geometry_msgs::msg::Point goal) {
        return std::abs(start.x - goal.x) + std::abs(start.y - goal.y) + std::abs(start.z - goal.z);
    }

    std::vector<geometry_msgs::msg::Point> reconstruct_bidirectional_path(const std::shared_ptr<Path_Node>& forward_node, const std::shared_ptr<Path_Node>& backward_node, bool reverse) {
        std::vector<geometry_msgs::msg::Point> forward_path = reconstruct_path(forward_node);
        std::vector<geometry_msgs::msg::Point> backward_path = reconstruct_path(backward_node);

        // Remove the duplicate meeting point
        forward_path.pop_back();

        // Always reverse the backward path
        std::reverse(backward_path.begin(), backward_path.end());

        // Combine the paths
        forward_path.insert(forward_path.end(), backward_path.begin(), backward_path.end());

        // If the search that found the meeting point was the reverse search, reverse the entire path
        if (reverse) {
            std::reverse(forward_path.begin(), forward_path.end());
        }

        return forward_path;
    }

    std::vector<geometry_msgs::msg::Point> reconstruct_path(const std::shared_ptr<Path_Node>& end_node) {
        std::vector<geometry_msgs::msg::Point> path;
        std::shared_ptr<Path_Node> current = end_node;

        while (current != nullptr) {
            path.push_back(current->point);
            current = current->parent;
        }

        std::reverse(path.begin(), path.end());
        return path;
    }

    void print_map(){
        for(int i=0 ; i<static_cast<int>(map.size()) ; ++i){
            for(int j = 0 ; j<static_cast<int>(map[0].size()) ; ++j){
                for(int k = 0 ; k<static_cast<int>(map[0][0].size()) ; ++k){
                    std::cout<<map[i][j][k]<<" ";
                }
                std::cout<<std::endl;
            }
            std::cout<<std::endl;
        }
        std::cout<<std::endl;
    }
};

#endif // BID_A_STAR_HPP