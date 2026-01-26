#include "PathSmoother.h"
#include "AStar3D.h"
#include <iostream>
#include <memory>

int main() {
    try {
        // 创建一个较小的3D地图实例
        auto map3d = std::make_shared<Map3D>(10, 10, 10, 1.0, 0.0, 0.0, 0.0);
        
        // 设置膨胀半径为0，避免过度膨胀
        map3d->setInflationRadius(0.0);
        
        // 添加一些障碍物
        map3d->addObstacle(2.0, 2.0, 2.0);
        map3d->addObstacle(2.0, 2.0, 3.0);
        map3d->addObstacle(2.0, 3.0, 2.0);
        
        // 创建A*算法实例
        AStar3D astar(map3d);
        
        // 设置起始点和目标点
        std::tuple<double, double, double> start(0.0, 0.0, 0.0);
        std::tuple<double, double, double> goal(5.0, 5.0, 5.0);
        
        std::cout << "开始A*路径规划..." << std::endl;
        std::cout << "起点: (" << std::get<0>(start) << ", " << std::get<1>(start) << ", " << std::get<2>(start) << ")" << std::endl;
        std::cout << "终点: (" << std::get<0>(goal) << ", " << std::get<1>(goal) << ", " << std::get<2>(goal) << ")" << std::endl;
        
        // 执行路径规划
        auto path = astar.plan(start, goal);
        
        if (!path.empty()) {
            std::cout << "找到路径，路径长度: " << path.size() << " 个节点" << std::endl;
            
            // 创建路径平滑器
            PathSmoother smoother;
            
            std::cout << "开始路径平滑..." << std::endl;
            
            // 使用路径平滑器
            auto smoothed_path = smoother.smooth_path(path, 0.5, 20);
            
            std::cout << "平滑后路径长度: " << smoothed_path.size() << " 个节点" << std::endl;
            
            // 显示原始路径的前几个点和平滑后路径的前几个点
            std::cout << "\n原始路径前5个点:" << std::endl;
            for (size_t i = 0; i < std::min(static_cast<size_t>(5), path.size()); ++i) {
                auto [x, y, z] = path[i];
                std::cout << "  [" << i << "] (" << x << ", " << y << ", " << z << ")" << std::endl;
            }
            
            std::cout << "\n平滑后路径前5个点:" << std::endl;
            for (size_t i = 0; i < std::min(static_cast<size_t>(5), smoothed_path.size()); ++i) {
                auto [x, y, z] = smoothed_path[i];
                std::cout << "  [" << i << "] (" << x << ", " << y << ", " << z << ")" << std::endl;
            }
            
            std::cout << "\n平滑后路径后5个点:" << std::endl;
            for (size_t i = std::max(static_cast<size_t>(0), smoothed_path.size() - 5); i < smoothed_path.size(); ++i) {
                auto [x, y, z] = smoothed_path[i];
                std::cout << "  [" << i << "] (" << x << ", " << y << ", " << z << ")" << std::endl;
            }
            
            std::cout << "\n开始带障碍物避让的路径平滑..." << std::endl;
            
            // 使用带障碍物避让的路径平滑
            auto obstacle_avoiding_path = smoother.smooth_path_with_obstacle_avoidance(path, map3d, 0.5, 20, 1.0, 5);
            
            std::cout << "避障平滑后路径长度: " << obstacle_avoiding_path.size() << " 个节点" << std::endl;
            
            std::cout << "\n避障平滑后路径前5个点:" << std::endl;
            for (size_t i = 0; i < std::min(static_cast<size_t>(5), obstacle_avoiding_path.size()); ++i) {
                auto [x, y, z] = obstacle_avoiding_path[i];
                std::cout << "  [" << i << "] (" << x << ", " << y << ", " << z << ")" << std::endl;
            }
            
            std::cout << "\n避障平滑后路径后5个点:" << std::endl;
            for (size_t i = std::max(static_cast<size_t>(0), obstacle_avoiding_path.size() - 5); i < obstacle_avoiding_path.size(); ++i) {
                auto [x, y, z] = obstacle_avoiding_path[i];
                std::cout << "  [" << i << "] (" << x << ", " << y << ", " << z << ")" << std::endl;
            }
        } else {
            std::cout << "未找到路径!" << std::endl;
        }
        
    } catch (const std::exception& e) {
        std::cerr << "发生错误: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}