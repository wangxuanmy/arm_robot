#ifndef PATHSMOOTHER_H
#define PATHSMOOTHER_H

#include <vector>
#include <tuple>
#include <memory>
#include "Map3D.h"

class PathSmoother {
public:
    PathSmoother();

    /**
     * 使用B样条对路径进行平滑处理
     * 
     * @param path 原始路径点列表 [(x, y, z), ...]
     * @param smoothing_factor 平滑因子，值越大越平滑，但可能偏离原路径越远
     * @param num_points 输出路径点的数量，如果为None则自动计算
     * @return 平滑后的路径点列表
     */
    std::vector<std::tuple<double, double, double>> smoothPath(
        const std::vector<std::tuple<double, double, double>>& path,
        double smoothing_factor = 0.5,
        int num_points = -1);

    /**
     * 在考虑障碍物的情况下平滑路径
     * 
     * @param path 原始路径点列表
     * @param map3d 3D地图对象
     * @param smoothing_factor 平滑因子
     * @param num_points 输出路径点的数量
     * @param safety_distance 安全距离，如果为None则使用地图的膨胀半径
     * @param max_iterations 最大迭代次数
     * @return 平滑且避开障碍物的路径点列表
     */
    std::vector<std::tuple<double, double, double>> smoothPathWithObstacleAvoidance(
        const std::vector<std::tuple<double, double, double>>& path,
        std::shared_ptr<Map3D> map3d,
        double smoothing_factor = 0.5,
        int num_points = -1,
        double safety_distance = -1,
        int max_iterations = 5);

private:
    /**
     * 检测路径中的障碍物冲突
     *
     * @param path 路径点列表
     * @param map3d 3D地图对象
     * @return 冲突信息列表，包含(索引, 冲突点, 原始点)
     */
    std::vector<std::tuple<int, std::tuple<double, double, double>, std::tuple<double, double, double>>> 
    detectObstacleConflicts(
        const std::vector<std::tuple<double, double, double>>& path,
        std::shared_ptr<Map3D> map3d);

    /**
     * 检测路径中的直线段并优化为直线
     * 
     * @param path 输入路径点列表
     * @param map3d 3D地图对象
     * @return 优化后的路径点列表
     */
    std::vector<std::tuple<double, double, double>> optimizeStraightLines(
        const std::vector<std::tuple<double, double, double>>& path,
        std::shared_ptr<Map3D> map3d);

    /**
     * 检查两点之间的直线是否无障碍
     * 
     * @param start 起点坐标
     * @param end 终点坐标
     * @param map3d 3D地图对象
     * @param num_check_points 检查点的数量
     * @return 如果直线上无障碍物返回True，否则返回False
     */
    bool IsLineClear(
        const std::tuple<double, double, double>& start,
        const std::tuple<double, double, double>& end,
        std::shared_ptr<Map3D> map3d,
        int num_check_points = 10);

    /**
     * 使用线性插值生成平滑路径（备用方法）
     * 
     * @param path 原始路径点列表
     * @param num_points 输出路径点的数量
     * @return 线性插值后的路径点列表
     */
    std::vector<std::tuple<double, double, double>> LinearInterpolatePath(
        const std::vector<std::tuple<double, double, double>>& path,
        int num_points);

    /**
     * 使用启发式方法在给定点附近查找一个无障碍的点，朝着原始点方向搜索
     * 
     * @param point 冲突点坐标
     * @param original_point 对应的原始路径点坐标
     * @param map3d 3D地图对象
     * @param safety_distance 安全距离
     * @return 无障碍点坐标
     */
    std::tuple<double, double, double> findFreePointHeuristic(
        const std::tuple<double, double, double>& point,
        const std::tuple<double, double, double>& original_point,
        std::shared_ptr<Map3D> map3d,
        double safety_distance);

    /**
     * 在给定点附近查找一个无障碍的点
     * 
     * @param point 原始点坐标
     * @param map3d 3D地图对象
     * @param safety_distance 安全距离
     * @return 无障碍点坐标
     */
    std::tuple<double, double, double> findFreePoint(
        const std::tuple<double, double, double>& point,
        std::shared_ptr<Map3D> map3d,
        double safety_distance);

    /**
     * 对冲突点周围的区域进行局部再平滑
     * 
     * @param path 当前路径
     * @param conflict_points_info 冲突点信息列表，包含(索引, 冲突点, 原始点)
     * @param map3d 地图对象
     * @return 局部再平滑后的路径
     */
    std::vector<std::tuple<double, double, double>> LocalResmooth(
        const std::vector<std::tuple<double, double, double>>& path,
        const std::vector<std::tuple<int, std::tuple<double, double, double>, std::tuple<double, double, double>>>& conflict_points_info,
        std::shared_ptr<Map3D> map3d);
};

#endif // PATHSMOOTHER_H