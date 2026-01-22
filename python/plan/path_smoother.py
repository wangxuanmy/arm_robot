import numpy as np
from typing import List, Tuple
from scipy.interpolate import splprep, splev


class PathSmoother:
    """
    路径平滑器，使用B样条对路径进行平滑处理
    """
    
    def __init__(self):
        pass
    
    def _optimize_straight_lines(self, path: List[Tuple[float, float, float]], 
                                map3d) -> List[Tuple[float, float, float]]:
        """
        检测路径中的直线段并优化为直线
        
        Args:
            path: 输入路径点列表
            map3d: 3D地图对象
            
        Returns:
            优化后的路径点列表
        """
        if len(path) < 3:
            return path
            
        optimized_path = [path[0]]  # 添加起点
        i = 0
        
        while i < len(path) - 1:
            # 从当前点开始尝试找到最长的直线段
            start_point = path[i]
            best_j = i + 1
            best_point = path[i + 1]
            
            # 尝试连接到更远的点
            for j in range(i + 2, len(path)):
                end_point = path[j]
                
                # 检查从start_point到end_point的直线是否无障碍
                if self._is_line_clear(start_point, end_point, map3d):
                    best_j = j
                    best_point = end_point
                else:
                    # 一旦遇到有障碍物阻挡的线段，就停止搜索
                    break
            
            # 添加找到的最佳点
            # 使用值比较而不是直接比较元组，避免numpy数组比较问题
            point_exists = False
            for p in optimized_path:
                if (abs(p[0] - best_point[0]) < 1e-6 and 
                    abs(p[1] - best_point[1]) < 1e-6 and 
                    abs(p[2] - best_point[2]) < 1e-6):
                    point_exists = True
                    break
            
            if not point_exists:
                optimized_path.append(best_point)
            
            # 移动到下一个检查点
            i = best_j
        
        return optimized_path
    
    def _is_line_clear(self, start: Tuple[float, float, float], 
                      end: Tuple[float, float, float], 
                      map3d, 
                      num_check_points: int = 10) -> bool:
        """
        检查两点之间的直线是否无障碍
        
        Args:
            start: 起点坐标
            end: 终点坐标
            map3d: 3D地图对象
            num_check_points: 检查点的数量
            
        Returns:
            bool: 如果直线上无障碍物返回True，否则返回False
        """
        start_x, start_y, start_z = start
        end_x, end_y, end_z = end
        
        # 在两点之间插值检查
        for i in range(1, num_check_points):
            ratio = i / num_check_points
            check_x = start_x + ratio * (end_x - start_x)
            check_y = start_y + ratio * (end_y - start_y)
            check_z = start_z + ratio * (end_z - start_z)
            
            # 如果检查点是障碍物，则直线不畅通
            if map3d.is_obstacle(check_x, check_y, check_z):
                return False
                
        return True
    
    def smooth_path(self, path: List[Tuple[float, float, float]], 
                    smoothing_factor: float = 0.5, 
                    num_points: int = None) -> List[Tuple[float, float, float]]:
        """
        使用B样条对路径进行平滑处理
        
        Args:
            path: 原始路径点列表 [(x, y, z), ...]
            smoothing_factor: 平滑因子，值越大越平滑，但可能偏离原路径越远
            num_points: 输出路径点的数量，如果为None则自动计算
            
        Returns:
            平滑后的路径点列表
        """
        if len(path) < 2:
            return path
            
        # 提取坐标
        x_coords = [point[0] for point in path]
        y_coords = [point[1] for point in path]
        z_coords = [point[2] for point in path]
        
        # 如果路径点太少，添加一些中间点
        if len(path) < 4:
            # 线性插值增加点
            new_path = []
            for i in range(len(path) - 1):
                new_path.append(path[i])
                # 在每两个点之间插入一个中间点
                mid_x = (path[i][0] + path[i+1][0]) / 2
                mid_y = (path[i][1] + path[i+1][1]) / 2
                mid_z = (path[i][2] + path[i+1][2]) / 2
                new_path.append((mid_x, mid_y, mid_z))
            new_path.append(path[-1])
            
            x_coords = [point[0] for point in new_path]
            y_coords = [point[1] for point in new_path]
            z_coords = [point[2] for point in new_path]
        
        # 设置默认输出点数
        if num_points is None:
            # 根据路径长度确定输出点数
            path_length = 0
            for i in range(len(path) - 1):
                dx = path[i+1][0] - path[i][0]
                dy = path[i+1][1] - path[i][1]
                dz = path[i+1][2] - path[i][2]
                path_length += np.sqrt(dx*dx + dy*dy + dz*dz)
            
            # 每单位长度大约5个点
            num_points = max(int(path_length / 0.5), 20)
        
        # 确保至少有20个点
        num_points = max(num_points, 20)
        
        try:
            # 构造参数点
            points = np.array([x_coords, y_coords, z_coords])
            
            # 使用B样条拟合路径
            # k=3 表示使用三次B样条
            tck, u = splprep(points, s=smoothing_factor, k=3, nest=-1)
            
            # 生成平滑路径点
            u_new = np.linspace(0, 1, num_points)
            smoothed_points = splev(u_new, tck)
            
            # 转换为元组列表
            smoothed_path = []
            for i in range(len(u_new)):
                smoothed_path.append((float(smoothed_points[0][i]), 
                                    float(smoothed_points[1][i]), 
                                    float(smoothed_points[2][i])))
            
            return smoothed_path
            
        except Exception as e:
            # 如果B样条拟合失败，返回线性插值的路径
            print(f"B样条拟合失败: {e}，使用线性插值")
            return self._linear_interpolate_path(path, num_points)
    
    def _linear_interpolate_path(self, path: List[Tuple[float, float, float]], 
                                num_points: int) -> List[Tuple[float, float, float]]:
        """
        使用线性插值生成平滑路径（备用方法）
        
        Args:
            path: 原始路径点列表
            num_points: 输出路径点的数量
            
        Returns:
            线性插值后的路径点列表
        """
        if len(path) < 2:
            return path
            
        # 计算路径总长度
        total_length = 0
        segment_lengths = [0]
        for i in range(len(path) - 1):
            dx = path[i+1][0] - path[i][0]
            dy = path[i+1][1] - path[i][1]
            dz = path[i+1][2] - path[i][2]
            segment_length = np.sqrt(dx*dx + dy*dy + dz*dz)
            total_length += segment_length
            segment_lengths.append(total_length)
        
        # 生成均匀分布的参数值
        target_lengths = np.linspace(0, total_length, num_points)
        
        # 插值生成新路径点
        smoothed_path = []
        path_index = 0
        
        for target_length in target_lengths:
            # 找到目标长度所在的线段
            while (path_index < len(segment_lengths) - 2 and 
                   segment_lengths[path_index + 1] < target_length):
                path_index += 1
            
            # 在线段内进行插值
            if path_index >= len(path) - 1:
                smoothed_path.append(path[-1])
            else:
                segment_start = segment_lengths[path_index]
                segment_end = segment_lengths[path_index + 1]
                segment_length = segment_end - segment_start
                
                if segment_length > 0:
                    ratio = (target_length - segment_start) / segment_length
                else:
                    ratio = 0
                
                # 线性插值
                x = path[path_index][0] + ratio * (path[path_index + 1][0] - path[path_index][0])
                y = path[path_index][1] + ratio * (path[path_index + 1][1] - path[path_index][1])
                z = path[path_index][2] + ratio * (path[path_index + 1][2] - path[path_index][2])
                
                smoothed_path.append((x, y, z))
        
        return smoothed_path
    
    def smooth_path_with_obstacle_avoidance(self, 
                                          path: List[Tuple[float, float, float]], 
                                          map3d,
                                          smoothing_factor: float = 0.5,
                                          num_points: int = None,
                                          safety_distance: float = None,
                                          max_iterations: int = 5) -> List[Tuple[float, float, float]]:
        """
        在考虑障碍物的情况下平滑路径
        
        Args:
            path: 原始路径点列表
            map3d: 3D地图对象
            smoothing_factor: 平滑因子
            num_points: 输出路径点的数量
            safety_distance: 安全距离，如果为None则使用地图的膨胀半径
            max_iterations: 最大迭代次数
            
        Returns:
            平滑且避开障碍物的路径点列表
        """
        if len(path) < 2:
            return path
            
        # 首先优化直线段
        optimized_path = self._optimize_straight_lines(path, map3d)
        
        # 然后进行基本平滑
        smoothed_path = self.smooth_path(optimized_path, smoothing_factor, num_points)
        
        # 如果没有提供安全距离，使用地图的膨胀半径
        if safety_distance is None:
            safety_distance = max(map3d.inflation_radius * map3d.resolution, map3d.resolution)
        
        # 存储冲突点的索引和原始点信息，用于启发式搜索
        conflict_points_info = []
        
        # 检查并修正与障碍物冲突的点
        for iteration in range(max_iterations):
            conflict_points_info = []
            for i, point in enumerate(smoothed_path):
                if map3d.is_obstacle(point[0], point[1], point[2]):
                    # 记录冲突点和对应的原始路径点
                    # 计算在原始路径中的对应点
                    original_point_index = int((i / len(smoothed_path)) * (len(path) - 1))
                    original_point = path[original_point_index]
                    conflict_points_info.append((i, point, original_point))
            
            # 如果没有冲突，停止迭代
            if not conflict_points_info:
                break
                
            # 修正冲突点
            for i, point, original_point in conflict_points_info:
                # 使用启发式方法查找附近的无障碍点
                corrected_point = self._find_free_point_heuristic(point, original_point, map3d, safety_distance)
                smoothed_path[i] = corrected_point
        
        # 如果进行了冲突点修正，对受影响的区域进行局部再平滑
        if conflict_points_info:
            smoothed_path = self._local_resmooth(smoothed_path, conflict_points_info, map3d)
        
        return smoothed_path
    
    def _find_free_point_heuristic(self, point: Tuple[float, float, float], 
                                  original_point: Tuple[float, float, float],
                                  map3d, 
                                  safety_distance: float) -> Tuple[float, float, float]:
        """
        使用启发式方法在给定点附近查找一个无障碍的点，朝着原始点方向搜索
        
        Args:
            point: 冲突点坐标
            original_point: 对应的原始路径点坐标
            map3d: 3D地图对象
            safety_distance: 安全距离
            
        Returns:
            无障碍点坐标
        """
        x, y, z = point
        orig_x, orig_y, orig_z = original_point
        
        # 计算从冲突点到原始点的方向向量
        dx = orig_x - x
        dy = orig_y - y
        dz = orig_z - z
        
        # 计算方向向量的长度
        distance = np.sqrt(dx*dx + dy*dy + dz*dz)
        
        # 如果两点重合，则使用随机方向
        if distance < 1e-6:
            # 回退到原来的搜索方法
            return self._find_free_point(point, map3d, safety_distance)
        
        # 归一化方向向量
        dx /= distance
        dy /= distance
        dz /= distance
        
        # 沿着这个方向逐步搜索无障碍点
        search_step = safety_distance / 5.0
        max_search_distance = safety_distance
        
        # 沿着方向搜索
        for i in range(1, int(max_search_distance / search_step) + 1):
            search_distance = i * search_step
            new_x = x + dx * search_distance
            new_y = y + dy * search_distance
            new_z = z + dz * search_distance
            
            # 检查新点是否有效且无障碍
            if (map3d.is_valid_position(new_x, new_y, new_z) and 
                not map3d.is_obstacle(new_x, new_y, new_z)):
                return (new_x, new_y, new_z)
        
        # 如果在原始点方向上找不到，尝试在周围搜索
        return self._find_free_point(point, map3d, safety_distance)
    
    def _find_free_point(self, point: Tuple[float, float, float], 
                        map3d, 
                        safety_distance: float) -> Tuple[float, float, float]:
        """
        在给定点附近查找一个无障碍的点
        
        Args:
            point: 原始点坐标
            map3d: 3D地图对象
            safety_distance: 安全距离
            
        Returns:
            无障碍点坐标
        """
        x, y, z = point
        
        # 在安全距离范围内搜索无障碍点
        search_radius = safety_distance
        search_step = safety_distance / 5.0
        
        # 检查原始点周围的一系列点
        for dx in np.arange(-search_radius, search_radius + search_step, search_step):
            for dy in np.arange(-search_radius, search_radius + search_step, search_step):
                for dz in np.arange(-search_radius, search_radius + search_step, search_step):
                    new_x, new_y, new_z = x + dx, y + dy, z + dz
                    # 检查新点是否有效且无障碍
                    if (map3d.is_valid_position(new_x, new_y, new_z) and 
                        not map3d.is_obstacle(new_x, new_y, new_z)):
                        # 检查是否比当前点更接近原始路径
                        distance = np.sqrt(dx*dx + dy*dy + dz*dz)
                        if distance <= search_radius:
                            return (new_x, new_y, new_z)
        
        # 如果找不到附近的无障碍点，返回原始点（这是最后的选择）
        return point
    
    def _local_resmooth(self, path: List[Tuple[float, float, float]], 
                       conflict_points_info: List[Tuple[int, Tuple[float, float, float], Tuple[float, float, float]]],
                       map3d) -> List[Tuple[float, float, float]]:
        """
        对冲突点周围的区域进行局部再平滑
        
        Args:
            path: 当前路径
            conflict_points_info: 冲突点信息列表，包含(索引, 冲突点, 原始点)
            map3d: 地图对象
            
        Returns:
            局部再平滑后的路径
        """
        if not conflict_points_info:
            return path
            
        # 创建路径副本
        resmoothed_path = path.copy()
        
        # 确定需要重新平滑的区域
        window_size = min(10, len(path) // 4)  # 窗口大小，至少为10但不超过路径长度的1/4
        
        # 对每个冲突点进行局部处理
        for index, _, _ in conflict_points_info:
            # 确定局部窗口范围
            start_idx = max(0, index - window_size)
            end_idx = min(len(path), index + window_size + 1)
            
            # 提取局部路径段
            local_path_segment = path[start_idx:end_idx]
            
            # 对局部路径段进行轻度平滑
            if len(local_path_segment) >= 2:
                try:
                    local_smoothed = self.smooth_path(local_path_segment, smoothing_factor=0.1, num_points=len(local_path_segment))
                    
                    # 将平滑后的局部路径段替换回原路径
                    resmoothed_path[start_idx:end_idx] = local_smoothed
                except Exception as e:
                    # 如果局部平滑失败，保持原样
                    print(f"局部平滑失败: {e}")
        
        return resmoothed_path