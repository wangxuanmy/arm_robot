#include "AStar3D.h"
#include <iostream>
#include <memory>

int main() {
    try {
        // 创建一个更紧凑的3D地图实例
        auto map3d = std::make_shared<Map3D>(8, 8, 8, 1.0, 0.0, 0.0, 0.0);
        
        // 关闭膨胀以简化测试
        map3d->setInflationRadius(0.0);
        
        // 添加少量障碍物
        map3d->addObstacle(2.0, 2.0, 2.0);
        
        // 创建A*算法实例
        AStar3D astar(map3d);
        
        // 设置起始点和目标点，避开障碍物区域
        std::tuple<double, double, double> start(0.0, 0.0, 0.0);
        std::tuple<double, double, double> goal(5.0, 5.0, 5.0);
        
        std::cout << "开始A*路径规划..." << std::endl;
        std::cout << "起点: (" << std::get<0>(start) << ", " << std::get<1>(start) << ", " << std::get<2>(start) << ")" << std::endl;
        std::cout << "终点: (" << std::get<0>(goal) << ", " << std::get<1>(goal) << ", " << std::get<2>(goal) << ")" << std::endl;
        std::cout << "地图尺寸: " << map3d->getWidth() << "x" << map3d->getHeight() << "x" << map3d->getDepth() << std::endl;
        
        // 执行路径规划
        auto path = astar.plan(start, goal);
        
        if (!path.empty()) {
            std::cout << "找到路径，路径长度: " << path.size() << " 个节点" << std::endl;
            std::cout << "路径:" << std::endl;
            
            // 打印全部路径，但限制最多打印50个节点
            int maxPrint = 50;
            int count = 0;
            for (size_t i = 0; i < path.size() && count < maxPrint; ++i, ++count) {
                auto [x, y, z] = path[i];
                std::cout << "  [" << i << "] (" << x << ", " << y << ", " << z << ")" << std::endl;
            }
            if (path.size() > maxPrint) {
                std::cout << "  ... (还有 " << (path.size() - maxPrint) << " 个节点) ..." << std::endl;
                // 打印最后5个节点
                for (size_t i = path.size() - 5; i < path.size(); ++i) {
                    auto [x, y, z] = path[i];
                    std::cout << "  [" << i << "] (" << x << ", " << y << ", " << z << ")" << std::endl;
                }
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