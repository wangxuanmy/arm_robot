#include "Map3D.h"
#include <cmath>
#include <algorithm>

Map3D::Map3D(int width, int height, int depth, double resolution,
             double origin_x, double origin_y, double origin_z)
    : resolution_(resolution), origin_x_(origin_x), origin_y_(origin_y), origin_z_(origin_z),
      inflation_radius_(5) {
    width_ = static_cast<int>(width / resolution_) + 2;
    height_ = static_cast<int>(height / resolution_) + 2;
    depth_ = static_cast<int>(depth / resolution_) + 2;
    
    // 初始化三维网格
    grid_.resize(width_);
    for (int i = 0; i < width_; ++i) {
        grid_[i].resize(height_);
        for (int j = 0; j < height_; ++j) {
            grid_[i][j].resize(depth_, 0);
        }
    }
}

void Map3D::setInflationRadius(double radius) {
    inflation_radius_ = static_cast<int>(std::ceil(radius / resolution_));
}

void Map3D::inflateObstacles() {
    if (inflation_radius_ <= 0) {
        return;
    }

    // 创建一个新的膨胀后的地图
    auto inflated_grid = grid_;

    // 遍历所有网格点，找到障碍物位置
    for (int x = 0; x < width_; ++x) {
        for (int y = 0; y < height_; ++y) {
            for (int z = 0; z < depth_; ++z) {
                if (grid_[x][y][z] == 1) {
                    // 在膨胀半径范围内标记为障碍物
                    for (int dx = -inflation_radius_; dx <= inflation_radius_; ++dx) {
                        for (int dy = -inflation_radius_; dy <= inflation_radius_; ++dy) {
                            for (int dz = -inflation_radius_; dz <= inflation_radius_; ++dz) {
                                // 检查是否在膨胀范围内（使用球形膨胀）
                                double distance = std::sqrt(dx*dx + dy*dy + dz*dz);
                                if (distance <= inflation_radius_) {
                                    int nx = x + dx, ny = y + dy, nz = z + dz;
                                    if (nx >= 0 && nx < width_ && ny >= 0 && ny < height_ && nz >= 0 && nz < depth_) {
                                        inflated_grid[nx][ny][nz] = 1;
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    grid_ = inflated_grid;
}

void Map3D::clearObstacles() {
    for (int i = 0; i < width_; ++i) {
        for (int j = 0; j < height_; ++j) {
            std::fill(grid_[i][j].begin(), grid_[i][j].end(), 0);
        }
    }
}

bool Map3D::addObstacle(double x, double y, double z) {
    auto coords = worldToGrid(x, y, z);
    int grid_x = std::get<0>(coords);
    int grid_y = std::get<1>(coords);
    int grid_z = std::get<2>(coords);
    
    if (grid_x >= 0 && grid_x < width_ && grid_y >= 0 && grid_y < height_ && grid_z >= 0 && grid_z < depth_) {
        grid_[grid_x][grid_y][grid_z] = 1;
        return true;
    }
    return false;
}

int Map3D::addObstaclesFromPointCloud(const std::vector<std::tuple<double, double, double>>& points) {
    int count = 0;
    for (const auto& point : points) {
        double x, y, z;
        std::tie(x, y, z) = point;
        if (addObstacle(x, y, z)) {
            count++;
        }
    }

    // 应用膨胀
    inflateObstacles();
    return count;
}

bool Map3D::removeObstacle(double x, double y, double z) {
    auto coords = worldToGrid(x, y, z);
    int grid_x = std::get<0>(coords);
    int grid_y = std::get<1>(coords);
    int grid_z = std::get<2>(coords);
    
    if (grid_x >= 0 && grid_x < width_ && grid_y >= 0 && grid_y < height_ && grid_z >= 0 && grid_z < depth_) {
        grid_[grid_x][grid_y][grid_z] = 0;
        return true;
    }
    return false;
}

bool Map3D::isObstacle(double x, double y, double z) const {
    auto coords = worldToGrid(x, y, z);
    int grid_x = std::get<0>(coords);
    int grid_y = std::get<1>(coords);
    int grid_z = std::get<2>(coords);
    
    if (grid_x >= 0 && grid_x < width_ && grid_y >= 0 && grid_y < height_ && grid_z >= 0 && grid_z < depth_) {
        return grid_[grid_x][grid_y][grid_z] == 1;
    }
    return true; // 超出边界也视为障碍物
}

bool Map3D::isValidPosition(double x, double y, double z) const {
    auto coords = worldToGrid(x, y, z);
    int grid_x = std::get<0>(coords);
    int grid_y = std::get<1>(coords);
    int grid_z = std::get<2>(coords);
    
    return grid_x >= 0 && grid_x < width_ && grid_y >= 0 && grid_y < height_ && grid_z >= 0 && grid_z < depth_;
}

std::vector<std::tuple<double, double, double>> Map3D::getNeighbors(double x, double y, double z) const {
    auto coords = worldToGrid(x, y, z);
    int grid_x = std::get<0>(coords);
    int grid_y = std::get<1>(coords);
    int grid_z = std::get<2>(coords);
    
    std::vector<std::tuple<double, double, double>> neighbors;
    
    // 6个方向: 上下左右前后
    int directions[6][3] = {{1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1}};
    
    for (const auto& dir : directions) {
        int nx = grid_x + dir[0];
        int ny = grid_y + dir[1];
        int nz = grid_z + dir[2];
        
        double world_x, world_y, world_z;
        std::tie(world_x, world_y, world_z) = gridToWorld(nx, ny, nz);
        
        if (isValidPosition(world_x, world_y, world_z) && !isObstacle(world_x, world_y, world_z)) {
            neighbors.push_back(gridToWorld(nx, ny, nz));
        }
    }
                
    return neighbors;
}

std::tuple<int, int, int> Map3D::worldToGrid(double x, double y, double z) const {
    int grid_x = static_cast<int>(std::round((x - origin_x_) / resolution_));
    int grid_y = static_cast<int>(std::round((y - origin_y_) / resolution_));
    int grid_z = static_cast<int>(std::round((z - origin_z_) / resolution_));
    return std::make_tuple(grid_x, grid_y, grid_z);
}

std::tuple<double, double, double> Map3D::gridToWorld(int x, int y, int z) const {
    double world_x = x * resolution_ + origin_x_;
    double world_y = y * resolution_ + origin_y_;
    double world_z = z * resolution_ + origin_z_;
    return std::make_tuple(world_x, world_y, world_z);
}