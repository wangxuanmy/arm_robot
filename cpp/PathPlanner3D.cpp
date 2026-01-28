#include "PathPlanner3D.h"
#include <memory>

PathPlanner3D::PathPlanner3D(int width, int height, int depth, double resolution, 
                             double origin_x, double origin_y, double origin_z) {
    map3d_ = std::make_shared<Map3D>(width, height, depth, resolution, 
                                     origin_x, origin_y, origin_z);
    astar3d_ = std::make_shared<AStar3D>(map3d_);
    pathSmoother_ = std::make_shared<PathSmoother>();
}

std::pair<std::vector<std::tuple<double, double, double>>, std::vector<std::tuple<double, double, double>>> 
PathPlanner3D::planWithObstacles(
    std::tuple<double, double, double> start,
    std::tuple<double, double, double> goal,
    const std::vector<std::tuple<double, double, double>>& obstacles,
    bool useSmoothing) {
    
    // 添加所有障碍物到地图
    for (const auto& obs : obstacles) {
        map3d_->addObstacle(std::get<0>(obs), std::get<1>(obs), std::get<2>(obs));
    }
    
    // 执行A*路径规划
    auto rawPath = astar3d_->plan(start, goal);
    
    if (rawPath.empty()) {
        return std::make_pair(rawPath, rawPath); // 如果原始路径为空，则返回两个空路径
    }
    
    // 如果需要平滑，则对路径进行平滑处理
    auto smoothedPath = rawPath;
    if (useSmoothing) {
        smoothedPath = pathSmoother_->smoothPathWithObstacleAvoidance(rawPath, map3d_, 0.5, 10);
    }
    
    return std::make_pair(rawPath, smoothedPath);
}