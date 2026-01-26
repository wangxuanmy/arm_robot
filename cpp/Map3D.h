#ifndef MAP3D_H
#define MAP3D_H

#include <vector>
#include <tuple>
#include <memory>

class Map3D {
private:
    int width_, height_, depth_;
    double resolution_;
    double origin_x_, origin_y_, origin_z_;
    std::vector<std::vector<std::vector<int>>> grid_; // 0表示自由空间，1表示障碍物
    int inflation_radius_;

public:
    /**
     * 初始化3D地图
     * @param width 地图宽度(X轴)m
     * @param height 地图高度(Y轴)m
     * @param depth 地图深度(Z轴)m
     * @param resolution 地图分辨率(每个网格代表的实际距离，单位：米)
     * @param origin_x 地图原点X坐标（世界坐标）
     * @param origin_y 地图原点Y坐标（世界坐标）
     * @param origin_z 地图原点Z坐标（世界坐标）
     */
    Map3D(int width, int height, int depth, double resolution = 1.0,
          double origin_x = 0.0, double origin_y = 0.0, double origin_z = 0.0);

    /**
     * 设置障碍物膨胀半径
     * @param radius 膨胀半径（实际单位，米）
     */
    void setInflationRadius(double radius);

    /**
     * 对障碍物进行膨胀处理
     */
    void inflateObstacles();

    /**
     * 清除所有障碍物
     */
    void clearObstacles();

    /**
     * 在指定位置添加障碍物
     * @param x, y, z 障碍物的世界坐标
     * @return 添加成功返回true，否则返回false
     */
    bool addObstacle(double x, double y, double z);

    /**
     * 从点云数据添加障碍物
     * @param points 点云数据列表，每个元素为(x, y, z)世界坐标元组
     * @return 成功添加的障碍物数量
     */
    int addObstaclesFromPointCloud(const std::vector<std::tuple<double, double, double>>& points);

    /**
     * 移除指定位置的障碍物
     * @param x, y, z 障碍物的世界坐标
     * @return 移除成功返回true，否则返回false
     */
    bool removeObstacle(double x, double y, double z);

    /**
     * 检查指定位置是否为障碍物
     * @param x, y, z 世界坐标
     * @return 是障碍物返回true，否则返回false
     */
    bool isObstacle(double x, double y, double z) const;

    /**
     * 检查位置是否有效（在地图范围内）
     * @param x, y, z 世界坐标
     * @return 有效返回true，否则返回false
     */
    bool isValidPosition(double x, double y, double z) const;

    /**
     * 获取指定位置的邻居节点（6个直接相邻的节点）
     * @param x, y, z 当前世界坐标
     * @return 邻居节点列表（世界坐标）
     */
    std::vector<std::tuple<double, double, double>> getNeighbors(double x, double y, double z) const;

    /**
     * 将世界坐标转换为网格坐标
     * @param x, y, z 世界坐标
     * @return 对应的网格坐标
     */
    std::tuple<int, int, int> worldToGrid(double x, double y, double z) const;

    /**
     * 将网格坐标转换为世界坐标
     * @param x, y, z 网格坐标
     * @return 对应的世界坐标
     */
    std::tuple<double, double, double> gridToWorld(int x, int y, int z) const;

    /**
     * 获取分辨率
     * @return 分辨率值
     */
    double getResolution() const { return resolution_; }

    /**
     * 获取地图尺寸
     */
    int getWidth() const { return width_; }
    int getHeight() const { return height_; }
    int getDepth() const { return depth_; }
    
    /**
     * 获取地图原点坐标
     */
    double getOriginX() const { return origin_x_; }
    double getOriginY() const { return origin_y_; }
    double getOriginZ() const { return origin_z_; }
};

using Map3DPtr = std::shared_ptr<Map3D>;

#endif // MAP3D_H