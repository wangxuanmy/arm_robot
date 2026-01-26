#ifndef PATH_PLANNER_3D_H
#define PATH_PLANNER_3D_H

#include <vector>
#include <tuple>
#include <memory>
#include "Map3D.h"
#include "AStar3D.h"
#include "PathSmoother.h"

class PathPlanner3D {
private:
    std::shared_ptr<Map3D> map3d_;
    std::shared_ptr<AStar3D> astar3d_;
    std::shared_ptr<PathSmoother> pathSmoother_;

public:
    PathPlanner3D(int width, int height, int depth, double resolution, 
                  double origin_x = 0.0, double origin_y = 0.0, double origin_z = 0.0);
    
    // 返回平滑前后的路径对
    std::pair<std::vector<std::tuple<double, double, double>>, std::vector<std::tuple<double, double, double>>> 
    planWithObstacles(
        std::tuple<double, double, double> start,
        std::tuple<double, double, double> goal,
        const std::vector<std::tuple<double, double, double>>& obstacles,
        bool useSmoothing = true
    );
};

#endif // PATH_PLANNER_3D_H