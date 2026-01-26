#ifndef ASTAR3D_H
#define ASTAR3D_H

#include "Map3D.h"
#include <vector>
#include <tuple>
#include <memory>
#include <unordered_map>
#include <functional>
#include <cmath>

struct Node {
    double x, y, z;  // World coordinates
    double g;        // Cost from start to current node
    double h;        // Heuristic cost from current node to goal
    double f;        // Total cost (f = g + h)

    Node() : x(0), y(0), z(0), g(0), h(0), f(0) {}
    Node(double x_, double y_, double z_, double g_ = 0, double h_ = 0) 
        : x(x_), y(y_), z(z_), g(g_), h(h_), f(g_ + h_) {}
    
    bool operator<(const Node& other) const {
        return f > other.f;  // For min-heap behavior in priority queue
    }
    
    bool operator==(const Node& other) const {
        // Compare nodes using world coordinates with tolerance for floating point precision
        const double tolerance = 0.001; // Small tolerance value
        return std::abs(this->x - other.x) < tolerance &&
               std::abs(this->y - other.y) < tolerance &&
               std::abs(this->z - other.z) < tolerance;
    }
};

// Custom hash function for Node
namespace std {
    template <>
    struct hash<Node> {
        size_t operator()(const Node& n) const {
            // Hash using world coordinates rounded to prevent floating point precision issues
            int x_hash = static_cast<int>(std::round(n.x / 0.01)); // Round to nearest 0.01
            int y_hash = static_cast<int>(std::round(n.y / 0.01));
            int z_hash = static_cast<int>(std::round(n.z / 0.01));

            // Combine the hash values using standard approach
            size_t h1 = std::hash<int>{}(x_hash);
            size_t h2 = std::hash<int>{}(y_hash);
            size_t h3 = std::hash<int>{}(z_hash);

            // Boost's hash_combine equivalent
            size_t seed = h1;
            seed ^= h2 + 0x9e3779b9 + (seed << 6) + (seed >> 2);
            seed ^= h3 + 0x9e3779b9 + (seed << 6) + (seed >> 2);
            return seed;
        }
    };
}

struct CompareNodes {
    bool operator()(const Node& a, const Node& b) const {
        return a.f > b.f;  // Min-heap based on f cost
    }
};

class AStar3D {
private:
    std::shared_ptr<Map3D> map_;

public:
    AStar3D(std::shared_ptr<Map3D> map);

    /**
     * 执行路径规划
     * @param start 起点坐标 (x, y, z)
     * @param goal 终点坐标 (x, y, z)
     * @return 路径点列表，如果无法找到路径则返回空列表
     */
    std::vector<std::tuple<double, double, double>> plan(
        std::tuple<double, double, double> start,
        std::tuple<double, double, double> goal);

private:
    double heuristic(double x1, double y1, double z1, double x2, double y2, double z2) const;
};

#endif // ASTAR3D_H