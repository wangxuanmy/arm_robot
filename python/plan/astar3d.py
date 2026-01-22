import heapq
import math
from typing import List, Tuple, Dict, Set
from map3d import Map3D

class Node:
    """
    A*算法中的节点类
    """
    def __init__(self, x: float, y: float, z: float, g: float = 0, h: float = 0, parent = None):
        self.x = x
        self.y = y
        self.z = z
        self.g = g  # 从起点到当前节点的实际代价
        self.h = h  # 启发式函数估算的从当前节点到终点的代价
        self.f = g + h  # 总代价
        self.parent = parent  # 父节点
        
    def __lt__(self, other):
        # 优化比较操作，首先比较f值，然后比较h值（更接近目标的优先）
        if self.f == other.f:
            return self.h < other.h
        return self.f < other.f
        
    def __eq__(self, other):
        return isinstance(other, Node) and self.x == other.x and self.y == other.y and self.z == other.z
        
    def __hash__(self):
        # 使用坐标哈希，但需要处理浮点数精度问题
        return hash((round(self.x, 6), round(self.y, 6), round(self.z, 6)))

class AStar3D:
    """
    3D A*路径规划算法
    """
    
    def __init__(self, map3d: Map3D):
        self.map = map3d
        
    def heuristic(self, x1: float, y1: float, z1: float, x2: float, y2: float, z2: float) -> float:
        """
        启发式函数，计算两个点之间的估计代价
        
        Args:
            x1, y1, z1: 起点世界坐标
            x2, y2, z2: 终点世界坐标
            
        Returns:
            float: 估计代价
        """
        # 使用欧几里得距离作为启发式函数
        return math.sqrt((x1 - x2)**2 + (y1 - y2)**2 + (z1 - z2)**2)
        
    def _get_key(self, x: float, y: float, z: float) -> Tuple[int, int, int]:
        """
        将世界坐标转换为网格坐标作为键值，用于快速查找
        
        Args:
            x, y, z: 世界坐标
            
        Returns:
            Tuple[int, int, int]: 网格坐标
        """
        grid_x, grid_y, grid_z = self.map._world_to_grid(x, y, z)
        return (grid_x, grid_y, grid_z)
        
    def plan(self, start: Tuple[float, float, float], goal: Tuple[float, float, float]) -> List[Tuple[float, float, float]]:
        """
        执行A*路径规划
        
        Args:
            start: 起点世界坐标 (x, y, z)
            goal: 终点世界坐标 (x, y, z)
            
        Returns:
            List[Tuple[float, float, float]]: 路径点列表（世界坐标），如果找不到路径则返回空列表
        """
        start_x, start_y, start_z = start
        goal_x, goal_y, goal_z = goal
        
        # 检查起点和终点是否有效
        if not self.map.is_valid_position(start_x, start_y, start_z):
            raise ValueError("Invalid start position")
            
        if not self.map.is_valid_position(goal_x, goal_y, goal_z):
            raise ValueError("Invalid goal position")
            
        if self.map.is_obstacle(start_x, start_y, start_z):
            raise ValueError("Start position is an obstacle")
            
        if self.map.is_obstacle(goal_x, goal_y, goal_z):
            raise ValueError("Goal position is an obstacle")
            
        # 初始化开放列表和关闭列表
        # 使用优先队列存储 (f_score, insertion_index, Node)
        open_list = []
        insertion_index = 0  # 用于保持堆的稳定性
        
        # 使用网格坐标作为键值的字典，提高查找效率
        open_dict: Dict[Tuple[int, int, int], Tuple[float, int, Node]] = {}
        closed_set: Set[Tuple[int, int, int]] = set()
        
        # 创建起点节点
        start_node = Node(start_x, start_y, start_z, 0, self.heuristic(start_x, start_y, start_z, goal_x, goal_y, goal_z))
        heapq.heappush(open_list, (start_node.f, insertion_index, start_node))
        insertion_index += 1
        start_key = self._get_key(start_x, start_y, start_z)
        open_dict[start_key] = (0, 0, start_node)
        
        while open_list:
            # 取出f值最小的节点
            _, _, current_node = heapq.heappop(open_list)
            current_key = self._get_key(current_node.x, current_node.y, current_node.z)
            
            # 检查是否已经在关闭列表中
            if current_key in closed_set:
                continue
            
            # 从开放字典中移除
            if current_key in open_dict:
                del open_dict[current_key]
            
            # 如果到达目标节点（检查是否足够接近目标）
            distance_to_goal = math.sqrt((current_node.x - goal_x)**2 + 
                                       (current_node.y - goal_y)**2 + 
                                       (current_node.z - goal_z)**2)
            if distance_to_goal <= self.map.resolution:
                # 重构路径
                path = []
                while current_node:
                    path.append((current_node.x, current_node.y, current_node.z))
                    current_node = current_node.parent
                return path[::-1]  # 反转路径
                
            # 将当前节点加入关闭列表
            closed_set.add(current_key)
            
            # 检查所有邻居节点
            for neighbor_pos in self.map.get_neighbors(current_node.x, current_node.y, current_node.z):
                # 获取邻居节点的世界坐标
                nx, ny, nz = neighbor_pos
                # 计算邻居的网格键
                neighbor_key = self._get_key(nx, ny, nz)
                
                # 如果在关闭列表中，跳过
                if neighbor_key in closed_set:
                    continue
                
                # 计算到邻居节点的代价
                # 移动代价（欧几里得距离）
                move_cost = math.sqrt((current_node.x - nx)**2 + (current_node.y - ny)**2 + (current_node.z - nz)**2)
                tentative_g_score = current_node.g + move_cost
                
                # 如果我们已经找到了到达这个节点的更好路径，跳过
                if neighbor_key in open_dict and tentative_g_score >= open_dict[neighbor_key][0]:
                    continue
                    
                # 这条路径是到达该节点的最佳路径
                neighbor_node = Node(nx, ny, nz, tentative_g_score, self.heuristic(nx, ny, nz, goal_x, goal_y, goal_z), current_node)
                
                # 添加到开放列表
                heapq.heappush(open_list, (neighbor_node.f, insertion_index, neighbor_node))
                open_dict[neighbor_key] = (tentative_g_score, insertion_index, neighbor_node)
                insertion_index += 1
                        
        # 未找到路径
        return []