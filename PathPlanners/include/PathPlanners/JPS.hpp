#ifndef JPS_HPP
#define JPS_HPP

#include <bits/stdc++.h>
#include <cmath>
#include <limits>
#include <algorithm>
#include <iostream>
#include <geometry_msgs/msg/point.hpp>
#include <PathPlanners/PathPlanningLibv2.hpp>
using namespace std;

class JPS : public PathPlanners {
public:
    JPS() {
        std::cout << "JPS obj init" << std::endl;
    }

    std::vector<geometry_msgs::msg::Point> get_plan(std::vector<std::vector<std::vector<int>>> map, geometry_msgs::msg::Point start, geometry_msgs::msg::Point goal) override {
        this->map = map;
        this->goal = goal;
        this->start = start;
        this->print_map();
        return find_path(start, goal);
    }

private:
    std::vector<std::vector<std::vector<int>>> map;
    geometry_msgs::msg::Point start;
    geometry_msgs::msg::Point goal;

    std::vector<geometry_msgs::msg::Point> find_path(geometry_msgs::msg::Point start, geometry_msgs::msg::Point goal) {
        if (start.z == goal.z) {
            return find_path_on_level_jps(start, goal);
        } else {
            geometry_msgs::msg::Point start_elevator = find_nearest_elevator(start);

            if (start_elevator.y == -1)
                return {};

            geometry_msgs::msg::Point dest_elevator = start_elevator;
            dest_elevator.z = goal.z;

            auto source_to_elevator = find_path_on_level_jps(start, start_elevator);
            auto elevator_to_dest = find_path_on_level_jps(dest_elevator, goal);

            if (!source_to_elevator.empty() && !elevator_to_dest.empty()) {
                source_to_elevator.insert(source_to_elevator.end(), elevator_to_dest.begin(), elevator_to_dest.end());
                return source_to_elevator;
            } else {
                std::cout << "Error - 03 : failed to get source to destination, try a different map" << std::endl;
                return {};
            }
        }
    }

    std::vector<geometry_msgs::msg::Point> find_path_on_level_jps(geometry_msgs::msg::Point start, geometry_msgs::msg::Point goal) {
        std::priority_queue<std::shared_ptr<Path_Node>, std::vector<std::shared_ptr<Path_Node>>, CompareNode> open_list;
        std::unordered_map<std::string, std::shared_ptr<Path_Node>> open_set;
        std::unordered_set<std::string> closed_set;

        auto start_node = std::make_shared<Path_Node>();
        start_node->point = start;
        start_node->parent = nullptr;
        start_node->G_cost = 0;
        start_node->F_cost = heuristic(start, goal);

        open_list.push(start_node);
        open_set[node_key(start_node)] = start_node;

        while (!open_list.empty()) {
            auto current = open_list.top();
            open_list.pop();
            open_set.erase(node_key(current));

            if (current->point == goal) {
                std::cout << "Goal Reached at (" << goal.z << " " << goal.y << " " << goal.x << ")" << std::endl;
                auto path = reconstruct_path(current);
                std::cout << "path length: " << path.size() << std::endl;
                return path;
            }

            closed_set.insert(node_key(current));

            auto successors = identify_successors(current, goal);
            for (const auto& successor : successors) {
                if (closed_set.find(node_key(successor)) != closed_set.end()) {
                    continue;
                }

                if (open_set.find(node_key(successor)) == open_set.end()) {
                    open_list.push(successor);
                    open_set[node_key(successor)] = successor;
                } else {
                    auto existing = open_set[node_key(successor)];
                    if (successor->G_cost < existing->G_cost) {
                        existing->G_cost = successor->G_cost;
                        existing->F_cost = successor->F_cost;
                        existing->parent = successor->parent;
                    }
                }
            }
        }

        return {};
    }

    std::vector<std::shared_ptr<Path_Node>> identify_successors(const std::shared_ptr<Path_Node>& node, const geometry_msgs::msg::Point& goal) {
        std::vector<std::shared_ptr<Path_Node>> successors;
        auto neighbors = get_pruned_neighbors(node);

        for (const auto& neighbor : neighbors) {
            auto jump_point = jump(neighbor, node->point, goal);
            if (jump_point) {
                auto new_node = std::make_shared<Path_Node>();
                new_node->point = jump_point->point;
                new_node->parent = node;
                new_node->G_cost = node->G_cost + distance(node->point, jump_point->point);
                new_node->F_cost = new_node->G_cost + heuristic(jump_point->point, goal);
                successors.push_back(new_node);
            }
        }

        return successors;
    }


    std::vector<geometry_msgs::msg::Point> get_pruned_neighbors(const std::shared_ptr<Path_Node>& node) {
        std::vector<geometry_msgs::msg::Point> neighbors;
        int x = node->point.x;
        int y = node->point.y;
        int z = node->point.z;

        if (!node->parent) {
            // If it's the start node, consider all neighbors
            for (int dx = -1; dx <= 1; dx++) {
                for (int dy = -1; dy <= 1; dy++) {
                    if (dx == 0 && dy == 0) continue;
                    if (is_walkable(x + dx, y + dy, z)) {
                        geometry_msgs::msg::Point p;
                        p.x = static_cast<double>(x + dx);
                        p.y = static_cast<double>(y + dy);
                        p.z = static_cast<double>(z);
                        neighbors.push_back(p);
                    }
                }
            }
        } else {
            int px = node->parent->point.x;
            int py = node->parent->point.y;
            int dx = (x - px) / std::max(std::abs(x - px), 1);
            int dy = (y - py) / std::max(std::abs(y - py), 1);

            // Diagonal move
            if (dx != 0 && dy != 0) {
                if (is_walkable(x, y + dy, z)) neighbors.push_back(createPoint(x, y + dy, z));
                if (is_walkable(x + dx, y, z)) neighbors.push_back(createPoint(x + dx, y, z));
                if (is_walkable(x + dx, y + dy, z)) neighbors.push_back(createPoint(x + dx, y + dy, z));
                if (!is_walkable(x - dx, y, z)) neighbors.push_back(createPoint(x - dx, y + dy, z));
                if (!is_walkable(x, y - dy, z)) neighbors.push_back(createPoint(x + dx, y - dy, z));
            } else {
                // Straight move
                if (dx == 0) {
                    if (is_walkable(x, y + dy, z)) {
                        neighbors.push_back(createPoint(x, y + dy, z));
                        if (!is_walkable(x + 1, y, z)) neighbors.push_back(createPoint(x + 1, y + dy, z));
                        if (!is_walkable(x - 1, y, z)) neighbors.push_back(createPoint(x - 1, y + dy, z));
                    }
                } else {
                    if (is_walkable(x + dx, y, z)) {
                        neighbors.push_back(createPoint(x + dx, y, z));
                        if (!is_walkable(x, y + 1, z)) neighbors.push_back(createPoint(x + dx, y + 1, z));
                        if (!is_walkable(x, y - 1, z)) neighbors.push_back(createPoint(x + dx, y - 1, z));
                    }
                }
            }
        }

        return neighbors;
    }

    // Helper function to create a Point
    geometry_msgs::msg::Point createPoint(double x, double y, double z) {
        geometry_msgs::msg::Point p;
        p.x = x;
        p.y = y;
        p.z = z;
        return p;
    }

    std::shared_ptr<Path_Node> jump(const geometry_msgs::msg::Point& p, const geometry_msgs::msg::Point& parent, const geometry_msgs::msg::Point& goal) {
        int x = p.x;
        int y = p.y;
        int z = p.z;
        int dx = x - parent.x;
        int dy = y - parent.y;

        if (!is_walkable(x, y, z)) return nullptr;

        if (p == goal) {
            auto node = std::make_shared<Path_Node>();
            node->point = p;
            return node;
        }

        // Check for forced neighbors
        if (dx != 0 && dy != 0) {
            if ((is_walkable(x - dx, y + dy, z) && !is_walkable(x - dx, y, z)) ||
                (is_walkable(x + dx, y - dy, z) && !is_walkable(x, y - dy, z))) {
                auto node = std::make_shared<Path_Node>();
                node->point = p;
                return node;
            }
        } else {
            if (dx != 0) {
                if ((is_walkable(x + dx, y + 1, z) && !is_walkable(x, y + 1, z)) ||
                    (is_walkable(x + dx, y - 1, z) && !is_walkable(x, y - 1, z))) {
                    auto node = std::make_shared<Path_Node>();
                    node->point = p;
                    return node;
                }
            } else {
                if ((is_walkable(x + 1, y + dy, z) && !is_walkable(x + 1, y, z)) ||
                    (is_walkable(x - 1, y + dy, z) && !is_walkable(x - 1, y, z))) {
                    auto node = std::make_shared<Path_Node>();
                    node->point = p;
                    return node;
                }
            }
        }

        // Recursively apply jump
        if (dx != 0 && dy != 0) {
            if (jump(createPoint(x + dx, y, z), p, goal) ||
                jump(createPoint(x, y + dy, z), p, goal)) {
                auto node = std::make_shared<Path_Node>();
                node->point = p;
                return node;
            }
        }

        // Continue jumping
        return jump(createPoint(x + dx, y + dy, z), p, goal);
    }

    bool is_walkable(int x, int y, int z) {
        return x >= 0 && x < static_cast<int>(map[0][0].size()) &&
               y >= 0 && y < static_cast<int>(map[0].size()) &&
               z >= 0 && z < static_cast<int>(map.size()) &&
               map[z][y][x] != 0;
    }

    inline int heuristic(geometry_msgs::msg::Point start, geometry_msgs::msg::Point goal) {
        return std::abs(start.x - goal.x) + std::abs(start.y - goal.y) + std::abs(start.z - goal.z);
    }

    inline int distance(geometry_msgs::msg::Point a, geometry_msgs::msg::Point b) {
        return std::abs(a.x - b.x) + std::abs(a.y - b.y) + std::abs(a.z - b.z);
    }

    inline std::string node_key(const std::shared_ptr<Path_Node>& node) {
        return std::to_string(node->point.x) + "," + std::to_string(node->point.y) + "," + std::to_string(node->point.z);
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

#endif // JPS_HPP