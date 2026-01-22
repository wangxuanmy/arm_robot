
import numpy as np
from typing import List, Tuple, Set
import matplotlib.pyplot as plt

class Map3D:
    """
    3D地图类，用于管理障碍物和空间信息
    """
    
    def __init__(self, width: int, height: int, depth: int, resolution: float = 1.0, 
                 origin_x: float = 0.0, origin_y: float = 0.0, origin_z: float = 0.0):
        """
        初始化3D地图
        
        Args:
            width: 地图宽度(X轴)m
            height: 地图高度(Y轴)m
            depth: 地图深度(Z轴)m
            resolution: 地图分辨率(每个网格代表的实际距离，单位：米)
            origin_x: 地图原点X坐标（世界坐标）
            origin_y: 地图原点Y坐标（世界坐标）
            origin_z: 地图原点Z坐标（世界坐标）
        """
        self.width = int(width / resolution) + 2
        self.height = int(height / resolution) + 2
        self.depth = int(depth / resolution) + 2
        self.resolution = resolution
        self.origin_x = origin_x
        self.origin_y = origin_y
        self.origin_z = origin_z
        # 0表示自由空间，1表示障碍物
        self.grid = np.zeros((self.width, self.height, self.depth), dtype=int)
        self.inflation_radius = 5  # 膨胀半径（网格单位）
        
        
    def set_inflation_radius(self, radius: float):
        """
        设置障碍物膨胀半径
        
        Args:
            radius: 膨胀半径（实际单位，米）
        """
        self.inflation_radius = int(np.ceil(radius / self.resolution))
        
    def _inflate_obstacles(self):
        """
        对障碍物进行膨胀处理
        """
        if self.inflation_radius <= 0:
            return
            
        # 创建一个新的膨胀后的地图
        inflated_grid = self.grid.copy()
        
        # 获取所有障碍物的位置
        obstacle_positions = np.where(self.grid == 1)
        
        # 对每个障碍物进行膨胀
        for x, y, z in zip(obstacle_positions[0], obstacle_positions[1], obstacle_positions[2]):
            # 在膨胀半径范围内标记为障碍物
            # for dx in range(-self.inflation_radius, self.inflation_radius + 1):
            #     for dy in range(-self.inflation_radius, self.inflation_radius + 1):
            #         for dz in range(-self.inflation_radius, self.inflation_radius + 1):
                        # # 检查是否在膨胀范围内（使用球形膨胀）
                        # distance = np.sqrt(dx**2 + dy**2 + dz**2)
                        # if distance <= self.inflation_radius:
                        #     nx, ny, nz = x + dx, y + dy, z + dz
                        #     if 0 <= nx < self.width and 0 <= ny < self.height and 0 <= nz < self.depth:
                        #         inflated_grid[nx, ny, nz] = 1

            # 使用立方体膨胀，加速
            inflated_grid[x - self.inflation_radius:x + self.inflation_radius + 1,
                          y - self.inflation_radius:y + self.inflation_radius + 1,
                          z - self.inflation_radius:z + self.inflation_radius + 1] = 1
        
        self.grid = inflated_grid
        

    def cleran_obstacles(self):
        """
        清除所有障碍物
        """
        self.grid = np.zeros((self.width, self.height, self.depth), dtype=int)
        
    def add_obstacle(self, x: float, y: float, z: float) -> bool:
        """
        在指定位置添加障碍物
        
        Args:
            x, y, z: 障碍物的世界坐标
            
        Returns:
            bool: 添加成功返回True，否则返回False
        """
        grid_x, grid_y, grid_z = self._world_to_grid(x, y, z)
        if 0 <= grid_x < self.width and 0 <= grid_y < self.height and 0 <= grid_z < self.depth:
            self.grid[grid_x, grid_y, grid_z] = 1
            return True
        return False
        
    def add_obstacles_from_point_cloud(self, points: List[Tuple[float, float, float]]) -> int:
        """
        从点云数据添加障碍物
        
        Args:
            points: 点云数据列表，每个元素为(x, y, z)世界坐标元组
            
        Returns:
            int: 成功添加的障碍物数量
        """
        count = 0
        for point in points:
            x, y, z = point
            if self.add_obstacle(x, y, z):
                count += 1
        
        # 应用膨胀
        self._inflate_obstacles()
        return count
        
    def remove_obstacle(self, x: float, y: float, z: float) -> bool:
        """
        移除指定位置的障碍物
        
        Args:
            x, y, z: 障碍物的世界坐标
            
        Returns:
            bool: 移除成功返回True，否则返回False
        """
        grid_x, grid_y, grid_z = self._world_to_grid(x, y, z)
        if 0 <= grid_x < self.width and 0 <= grid_y < self.height and 0 <= grid_z < self.depth:
            self.grid[grid_x, grid_y, grid_z] = 0
            return True
        return False
        
    def is_obstacle(self, x: float, y: float, z: float) -> bool:
        """
        检查指定位置是否为障碍物
        
        Args:
            x, y, z: 世界坐标
            
        Returns:
            bool: 是障碍物返回True，否则返回False
        """
        grid_x, grid_y, grid_z = self._world_to_grid(x, y, z)
        if 0 <= grid_x < self.width and 0 <= grid_y < self.height and 0 <= grid_z < self.depth:
            return self.grid[grid_x, grid_y, grid_z] == 1
        return True  # 超出边界也视为障碍物
        
    def is_valid_position(self, x: float, y: float, z: float) -> bool:
        """
        检查位置是否有效（在地图范围内）
        
        Args:
            x, y, z: 世界坐标
            
        Returns:
            bool: 有效返回True，否则返回False
        """
        grid_x, grid_y, grid_z = self._world_to_grid(x, y, z)
        return 0 <= grid_x < self.width and 0 <= grid_y < self.height and 0 <= grid_z < self.depth
        
    def get_neighbors(self, x: float, y: float, z: float) -> List[Tuple[float, float, float]]:
        """
        获取指定位置的邻居节点（6个直接相邻的节点）
        
        Args:
            x, y, z: 当前世界坐标
            
        Returns:
            List[Tuple[float, float, float]]: 邻居节点列表（世界坐标）
        """
        grid_x, grid_y, grid_z = self._world_to_grid(x, y, z)
        neighbors = []
        # 6个方向: 上下左右前后
        directions = [(1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1), (0, 0, -1)]
        
        for dx, dy, dz in directions:
            nx, ny, nz = grid_x + dx, grid_y + dy, grid_z + dz
            if self.is_valid_position(nx * self.resolution + self.origin_x, 
                                      ny * self.resolution + self.origin_y, 
                                      nz * self.resolution + self.origin_z) and \
               not self.is_obstacle(nx * self.resolution + self.origin_x, 
                                    ny * self.resolution + self.origin_y, 
                                    nz * self.resolution + self.origin_z):
                neighbors.append(self._grid_to_world(nx, ny, nz))
                
        return neighbors
        
    def _world_to_grid(self, x: float, y: float, z: float) -> Tuple[int, int, int]:
        """
        将世界坐标转换为网格坐标
        
        Args:
            x, y, z: 世界坐标
            
        Returns:
            Tuple[int, int, int]: 对应的网格坐标
        """
        grid_x = int(round((x - self.origin_x) / self.resolution))
        grid_y = int(round((y - self.origin_y) / self.resolution))
        grid_z = int(round((z - self.origin_z) / self.resolution))
        return grid_x, grid_y, grid_z
        
    def _grid_to_world(self, x: int, y: int, z: int) -> Tuple[float, float, float]:
        """
        将网格坐标转换为世界坐标
        
        Args:
            x, y, z: 网格坐标
            
        Returns:
            Tuple[float, float, float]: 对应的世界坐标
        """
        world_x = float(x * self.resolution + self.origin_x)
        world_y = float(y * self.resolution + self.origin_y)
        world_z = float(z * self.resolution + self.origin_z)
        return world_x, world_y, world_z
        
    def visualize(self, path: List[Tuple[float, float, float]] = None, 
                  smoothed_path: List[Tuple[float, float, float]] = None,
                  detailed_obstacles=False):
        """
        使用matplotlib可视化地图和路径
        
        Args:
            path: 原始路径点列表（世界坐标）
            smoothed_path: 平滑路径点列表（世界坐标）
            detailed_obstacles: 是否显示详细的障碍物体素（仅适用于少量障碍物）
        """
        try:
            from mpl_toolkits.mplot3d import Axes3D
        except ImportError:
            print("警告：无法导入3D绘图工具，跳过可视化")
            return
            
        fig = plt.figure(figsize=(12, 8))
        ax = fig.add_subplot(111, projection='3d')
        
        # 绘制障碍物（优化显示性能）
        obstacle_grid_x, obstacle_grid_y, obstacle_grid_z = np.where(self.grid == 1)
        
        if len(obstacle_grid_x) > 0:
            if detailed_obstacles and len(obstacle_grid_x) <= 200:
                # 对于少量障碍物，绘制详细的立方体
                voxel_size = self.resolution
                # 合并所有障碍物坐标，一次性绘制
                obstacle_coords = np.column_stack((
                    obstacle_grid_x * voxel_size + self.origin_x,
                    obstacle_grid_y * voxel_size + self.origin_y, 
                    obstacle_grid_z * voxel_size + self.origin_z
                ))
                
                # 绘制立方体顶点
                ax.scatter(
                    obstacle_coords[:, 0] + voxel_size/2,
                    obstacle_coords[:, 1] + voxel_size/2,
                    obstacle_coords[:, 2] + voxel_size/2,
                    c='red', s=200, alpha=0.7, marker='s', label='Obstacles'
                )
            else:
                # 根据障碍物数量选择不同的显示方式
                obstacle_x = obstacle_grid_x * self.resolution + self.origin_x
                obstacle_y = obstacle_grid_y * self.resolution + self.origin_y
                obstacle_z = obstacle_grid_z * self.resolution + self.origin_z
                
                if len(obstacle_grid_x) <= 100:
                    # 障碍物较少时，使用较大的标记
                    ax.scatter(obstacle_x, obstacle_y, obstacle_z, c='red', marker='s', s=100, alpha=0.7, label='Obstacles')
                elif len(obstacle_grid_x) <= 1000:
                    # 障碍物中等数量时，使用适中大小的标记
                    ax.scatter(obstacle_x, obstacle_y, obstacle_z, c='red', marker='s', s=50, alpha=0.7, label='Obstacles')
                else:
                    # 障碍物很多时，使用较小的标记和较低的透明度以提高性能
                    ax.scatter(obstacle_x, obstacle_y, obstacle_z, c='red', marker='s', s=20, alpha=0.5, label='Obstacles')
        
        # 绘制原始路径
        if path:
            path_x = [p[0] for p in path]
            path_y = [p[1] for p in path]
            path_z = [p[2] for p in path]
            ax.plot(path_x, path_y, path_z, c='blue', linewidth=1, linestyle='--', marker='o', markersize=2, label='Original Path')
            if path:
                ax.scatter(path_x[0], path_y[0], path_z[0], c='green', s=100, marker='o', label='Start')
                ax.scatter(path_x[-1], path_y[-1], path_z[-1], c='purple', s=100, marker='o', label='Goal')
        
        # 绘制平滑路径
        if smoothed_path:
            smoothed_x = [p[0] for p in smoothed_path]
            smoothed_y = [p[1] for p in smoothed_path]
            smoothed_z = [p[2] for p in smoothed_path]
            ax.plot(smoothed_x, smoothed_y, smoothed_z, c='orange', linewidth=2, label='Smoothed Path')
        
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_title('3D Map Visualization')
        ax.legend()
        
        # 设置坐标轴比例
        ax.set_xlim([self.origin_x, self.origin_x + self.width * self.resolution])
        ax.set_ylim([self.origin_y, self.origin_y + self.height * self.resolution])
        ax.set_zlim([self.origin_z, self.origin_z + self.depth * self.resolution])
        
        plt.show()