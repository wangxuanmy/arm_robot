#include "AStar3D.h"
#include <queue>
#include <functional>
#include <cmath>
#include <limits>
#include <algorithm>
#include <set>
#include <map>
#include <unordered_map>
#include <unordered_set>

AStar3D::AStar3D(std::shared_ptr<Map3D> map) : map_(map) {}

std::vector<std::tuple<double, double, double>> AStar3D::plan(
    std::tuple<double, double, double> start,
    std::tuple<double, double, double> goal) {
    
    // No need to convert start and goal to grid coordinates here
    // We'll work directly with world coordinates and use Map3D's conversion methods when needed
    
    // Check if start and goal are valid
    if (!map_->isValidPosition(std::get<0>(start), 
                               std::get<1>(start), 
                               std::get<2>(start)) ||
        !map_->isValidPosition(std::get<0>(goal), 
                               std::get<1>(goal), 
                               std::get<2>(goal)) ||
        map_->isObstacle(std::get<0>(start), 
                         std::get<1>(start), 
                         std::get<2>(start)) ||
        map_->isObstacle(std::get<0>(goal), 
                         std::get<1>(goal), 
                         std::get<2>(goal))) {
        return {}; // Return empty path if start or goal is invalid
    }
    
    // Initialize open and closed sets
    std::priority_queue<Node, std::vector<Node>, CompareNodes> open_set;
    std::unordered_set<Node> closed_set;  // Using custom hash now
    
    // Initialize cost maps
    std::unordered_map<Node, double> g_costs;  // Using custom hash
    std::unordered_map<Node, Node> came_from;  // Using custom hash
    
    // Create start node in world coordinates
    Node start_node(std::get<0>(start), 
                    std::get<1>(start), 
                    std::get<2>(start), 
                    0, 
                    heuristic(std::get<0>(start), 
                              std::get<1>(start), 
                              std::get<2>(start),
                              std::get<0>(goal), 
                              std::get<1>(goal), 
                              std::get<2>(goal)));
    
    open_set.push(start_node);
    g_costs[start_node] = 0;
    
    // Define the 26 neighbors (3D connectivity)
    std::vector<std::tuple<int, int, int>> directions = {
        {-1, -1, -1}, {-1, -1, 0}, {-1, -1, 1},
        {-1, 0, -1},  {-1, 0, 0},  {-1, 0, 1},
        {-1, 1, -1},  {-1, 1, 0},  {-1, 1, 1},
        {0, -1, -1},  {0, -1, 0},  {0, -1, 1},
        {0, 0, -1},              {0, 0, 1},
        {0, 1, -1},   {0, 1, 0},   {0, 1, 1},
        {1, -1, -1},  {1, -1, 0},  {1, -1, 1},
        {1, 0, -1},   {1, 0, 0},   {1, 0, 1},
        {1, 1, -1},   {1, 1, 0},   {1, 1, 1}
    };
    
    while (!open_set.empty()) {
        Node current = open_set.top();
        open_set.pop();
        
        // Check if we reached the goal using tolerance for floating point precision
        const double tolerance = 0.001;
        if (std::abs(current.x - std::get<0>(goal)) < tolerance &&
            std::abs(current.y - std::get<1>(goal)) < tolerance &&
            std::abs(current.z - std::get<2>(goal)) < tolerance) {
            // Reconstruct path
            std::vector<std::tuple<double, double, double>> path;
            Node current_node = current;
            
            // Backtrack to find the complete path
            while(true) {
                path.push_back(std::make_tuple(current_node.x, current_node.y, current_node.z));
                
                // Check if we've reached the start node
                if (std::abs(current_node.x - std::get<0>(start)) < tolerance && 
                    std::abs(current_node.y - std::get<1>(start)) < tolerance && 
                    std::abs(current_node.z - std::get<2>(start)) < tolerance) {
                    break;
                }
                
                // Get the parent of the current node
                if (came_from.find(current_node) != came_from.end()) {
                    current_node = came_from[current_node];
                } else {
                    // If we can't find a parent, we've reached the start
                    break;
                }
            }
            
            std::reverse(path.begin(), path.end());
            return path;
        }
        
        // Add current to closed set
        closed_set.insert(current);
        
        // Explore neighbors
        for (const auto& dir : directions) {
            int new_grid_x = 0, new_grid_y = 0, new_grid_z = 0;
            
            // Calculate new grid coordinates based on current position
            auto [current_grid_x, current_grid_y, current_grid_z] = map_->worldToGrid(current.x, current.y, current.z);
            new_grid_x = current_grid_x + std::get<0>(dir);
            new_grid_y = current_grid_y + std::get<1>(dir);
            new_grid_z = current_grid_z + std::get<2>(dir);
            
            // Check if neighbor is valid and not in closed set
            if (new_grid_x < 0 || new_grid_x >= map_->getWidth() ||
                new_grid_y < 0 || new_grid_y >= map_->getHeight() ||
                new_grid_z < 0 || new_grid_z >= map_->getDepth()) {
                continue;
            }
            
            // Convert back to world coordinates
            auto [nx, ny, nz] = map_->gridToWorld(new_grid_x, new_grid_y, new_grid_z);
            
            Node neighbor_node(nx, ny, nz);
            if (closed_set.count(neighbor_node)) {
                continue;
            }
            
            if (map_->isObstacle(nx, ny, nz)) {
                continue;
            }
            
            // Calculate tentative g_cost
            double dx = nx - current.x;
            double dy = ny - current.y;
            double dz = nz - current.z;
            double move_cost = std::sqrt(dx*dx + dy*dy + dz*dz);
            double tentative_g = current.g + move_cost;
            
            // Check if this path to neighbor is better
            if (g_costs.find(neighbor_node) == g_costs.end() || 
                tentative_g < g_costs[neighbor_node]) {
                
                // Update costs and add to open set
                came_from[neighbor_node] = current;  // Store the current node as parent
                g_costs[neighbor_node] = tentative_g;
                
                Node neighbor(nx, ny, nz,
                              tentative_g,
                              heuristic(nx, ny, nz,
                                        std::get<0>(goal), 
                                        std::get<1>(goal), 
                                        std::get<2>(goal)));
                
                open_set.push(neighbor);
            }
        }
    }
    
    // No path found
    return {};
}

double AStar3D::heuristic(double x1, double y1, double z1, double x2, double y2, double z2) const {
    // Euclidean distance heuristic
    double dx = x2 - x1;
    double dy = y2 - y1;
    double dz = z2 - z1;
    
    return std::sqrt(dx*dx + dy*dy + dz*dz);
}

