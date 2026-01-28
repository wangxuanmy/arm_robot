import json
import numpy as np
from typing import List, Tuple, Dict, Optional

# 导入arm_utils模块
from arm_utils.darwin import Human
from arm_utils.coord import euler_rpy_to_homogeneous_matrix
from plan.map3d import Map3D
from plan.astar3d import AStar3D
from plan.path_smoother import PathSmoother


class RobotCtrl:
    """
    机器人控制器类，整合了机器人模型、路径规划和控制功能
    """
    
    def __init__(self, json_config_path: str):
        """
        初始化机器人控制器
        
        Args:
            json_config_path: JSON配置文件路径，如darwin_02.json
        """
        self.config_path = json_config_path
        self.robot = None
        self.map3d = None
        self.path_planner = None
        self.path_smoother = PathSmoother()
        self.current_path = []
        self.smoothed_path = []
        
        # 加载配置并初始化机器人
        self.load_config(json_config_path)
        self.init_robot()
    
    def load_config(self, json_config_path: str):
        """
        从JSON文件加载机器人配置
        
        Args:
            json_config_path: JSON配置文件路径
        """
        with open(json_config_path, 'r') as f:
            self.config = json.load(f)
        
        # 解析DH参数
        self.dh_params = self.config['dh_params']
        self.joint_min = self.config['joint_min']
        self.joint_max = self.config['joint_max']
        self.enable_joint = self.config['enable_joint']
        self.joint_names = self.config['joint_names']
        self.init_offset = self.config['init_offset']
        
        # 计算关节限制（弧度制）
        self.joint_limits = []
        for min_val, max_val in zip(self.joint_min, self.joint_max):
            self.joint_limits.append([min_val, max_val])
        self.joint_limits = np.array(self.joint_limits)
    
    def init_robot(self):
        """
        根据配置初始化机器人模型
        """
        # 获取DH参数
        body_dh = self.dh_params['body']
        left_hand_dh = self.dh_params['left_hand']
        right_hand_dh = self.dh_params['right_hand']
        
        # 创建机器人实例
        self.robot = Human(body_dh, left_hand_dh, right_hand_dh, self.joint_limits)
        
        # 设置初始关节角度
        self.reset_to_initial_position()
    
    def reset_to_initial_position(self):
        """
        重置机器人到初始位置
        """
        if self.robot and self.init_offset:
            # 根据init_offset设置初始关节角度
            # 注意：init_offset的结构可能需要根据实际情况调整
            # 这里假设init_offset是3组初始值：[body, left_hand, right_hand]
            if len(self.init_offset) >= 1:
                # 设置body部分的初始角度
                body_theta_count = self.robot.body.num_joints
                for i in range(min(len(self.init_offset[0]), body_theta_count)):
                    self.robot.body.theta[i] = self.init_offset[0][i]
            
            if len(self.init_offset) >= 2:
                # 设置left_hand部分的初始角度
                left_hand_theta_count = self.robot.left_hand.num_joints
                for i in range(min(len(self.init_offset[1]), left_hand_theta_count)):
                    self.robot.left_hand.theta[i] = self.init_offset[1][i]
            
            if len(self.init_offset) >= 3:
                # 设置right_hand部分的初始角度
                right_hand_theta_count = self.robot.right_hand.num_joints
                for i in range(min(len(self.init_offset[2]), right_hand_theta_count)):
                    self.robot.right_hand.theta[i] = self.init_offset[2][i]
    
    def init_map(self, width: float, height: float, depth: float, resolution: float = 1.0,
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
        self.map3d = Map3D(width, height, depth, resolution, origin_x, origin_y, origin_z)
        self.path_planner = AStar3D(self.map3d)
        
        return self.map3d
    
    def set_inflation_radius(self, radius: float):
        """
        设置障碍物膨胀半径
        
        Args:
            radius: 膨胀半径（实际单位，米）
        """
        if self.map3d:
            self.map3d.set_inflation_radius(radius)
    
    def add_obstacle(self, x: float, y: float, z: float) -> bool:
        """
        在指定位置添加障碍物
        
        Args:
            x, y, z: 障碍物的世界坐标
            
        Returns:
            bool: 添加成功返回True，否则返回False
        """
        if self.map3d:
            return self.map3d.add_obstacle(x, y, z)
        return False
    
    def add_obstacles_from_point_cloud(self, points: List[Tuple[float, float, float]]) -> int:
        """
        从点云数据添加障碍物
        
        Args:
            points: 点云数据列表，每个元素为(x, y, z)世界坐标元组
            
        Returns:
            int: 成功添加的障碍物数量
        """
        if self.map3d:
            return self.map3d.add_obstacles_from_point_cloud(points)
        return 0
    
    def remove_obstacle(self, x: float, y: float, z: float) -> bool:
        """
        移除指定位置的障碍物
        
        Args:
            x, y, z: 障碍物的世界坐标
            
        Returns:
            bool: 移除成功返回True，否则返回False
        """
        if self.map3d:
            return self.map3d.remove_obstacle(x, y, z)
        return False
    
    def plan_path(self, start: Tuple[float, float, float], 
                  goal: Tuple[float, float, float]) -> List[Tuple[float, float, float]]:
        """
        规划从起点到终点的路径
        
        Args:
            start: 起点世界坐标 (x, y, z)
            goal: 终点世界坐标 (x, y, z)
            
        Returns:
            List[Tuple[float, float, float]]: 路径点列表（世界坐标），如果找不到路径则返回空列表
        """
        if self.path_planner:
            self.current_path = self.path_planner.plan(start, goal)
            return self.current_path
        return []
    
    def smooth_path(self, path: List[Tuple[float, float, float]] = None,
                    smoothing_factor: float = 0.5,
                    num_points: int = None,
                    with_obstacle_avoidance: bool = True) -> List[Tuple[float, float, float]]:
        """
        对路径进行平滑处理
        
        Args:
            path: 要平滑的路径，如果为None则使用最近一次规划的路径
            smoothing_factor: 平滑因子，值越大越平滑
            num_points: 输出路径点的数量
            with_obstacle_avoidance: 是否考虑避障进行平滑
            
        Returns:
            List[Tuple[float, float, float]]: 平滑后的路径点列表
        """
        if path is None:
            path = self.current_path
        
        if not path:
            return []
        
        if with_obstacle_avoidance and self.map3d:
            self.smoothed_path = self.path_smoother.smooth_path_with_obstacle_avoidance(
                path, self.map3d, smoothing_factor, num_points)
        else:
            self.smoothed_path = self.path_smoother.smooth_path(path, smoothing_factor, num_points)
        
        return self.smoothed_path
    
    def visualize_map(self, show_original_path: bool = True, show_smoothed_path: bool = True):
        """
        可视化地图和路径
        
        Args:
            show_original_path: 是否显示原始路径
            show_smoothed_path: 是否显示平滑路径
        """
        if self.map3d:
            path = self.current_path if show_original_path else None
            smoothed_path = self.smoothed_path if show_smoothed_path else None
            self.map3d.visualize(path=path, smoothed_path=smoothed_path)
    
    def control_part(self, aim_mat: np.ndarray, part_name: str = 'body', ignore_angle: bool = False):
        """
        控制机器人特定部分到目标位姿
        
        Args:
            aim_mat: 目标位姿齐次变换矩阵
            part_name: 要控制的部分 ('body', 'left', 'right')
            ignore_angle: 是否忽略角度限制
            
        Returns:
            tuple: (theta, theta_can, get_aim) 分别是最终关节角度、过程角度序列、是否达到目标
        """
        if self.robot:
            return self.robot.control_part(aim_mat, part_name, ignore_angle)
        return [], [], False
    
    def control_hand(self, aim_mat: np.ndarray, hand_name: str = 'left', ignore_angle: bool = False):
        """
        控制机器人手部到目标位姿（身体和手作为一个整体）
        
        Args:
            aim_mat: 目标位姿齐次变换矩阵
            hand_name: 要控制的手部 ('left', 'right')
            ignore_angle: 是否忽略角度限制
            
        Returns:
            tuple: (theta, theta_can, get_aim) 分别是最终关节角度、过程角度序列、是否达到目标
        """
        if self.robot:
            return self.robot.control_hand(aim_mat, hand_name, ignore_angle)
        return [], [], False
    
    def get_robot_state(self) -> Dict:
        """
        获取机器人当前状态
        
        Returns:
            Dict: 包含机器人各种状态信息的字典
        """
        if self.robot:
            return {
                'body_theta': self.robot.body.theta,
                'left_hand_theta': self.robot.left_hand.theta,
                'right_hand_theta': self.robot.right_hand.theta,
                'all_theta': self.robot.get_all_theta(),
                'eef_poses': self.robot.get_eef(),
                'all_tf': self.robot.get_all_tf()
            }
        return {}
    
    def update_robot_theta(self, theta: List[float]):
        """
        更新机器人所有关节的角度
        
        Args:
            theta: 关节角度列表
        """
        if self.robot:
            self.robot.flash_theta(theta)
    
    def get_part_base(self, part_name: str) -> np.ndarray:
        """
        获取机器人某部分的基座矩阵
        
        Args:
            part_name: 部件名称 ('body', 'left', 'right')
            
        Returns:
            np.ndarray: 基座变换矩阵
        """
        if self.robot:
            return self.robot.get_part_base(part_name)
        return np.eye(4)
    
    def is_collision_at_pose(self, pose: Tuple[float, float, float], safety_margin: float = 0.1) -> bool:
        """
        检查给定位姿是否与障碍物碰撞
        
        Args:
            pose: 要检查的位姿 (x, y, z)
            safety_margin: 安全边距
            
        Returns:
            bool: 如果碰撞返回True，否则返回False
        """
        if self.map3d:
            x, y, z = pose
            # 检查当前位置是否有障碍物
            if self.map3d.is_obstacle(x, y, z):
                return True
            
            # 检查周边是否有障碍物（考虑安全边距）
            for dx in np.arange(-safety_margin, safety_margin, self.map3d.resolution):
                for dy in np.arange(-safety_margin, safety_margin, self.map3d.resolution):
                    for dz in np.arange(-safety_margin, safety_margin, self.map3d.resolution):
                        if self.map3d.is_obstacle(x + dx, y + dy, z + dz):
                            return True
            return False
        return False


if __name__ == "__main__":
    # 示例用法
    robot_ctrl = RobotCtrl("/home/std/arm_robot/cpp/darwin_02.json")
    
    # 初始化地图
    robot_ctrl.init_map(10, 10, 5, resolution=0.1)
    
    # 添加一些障碍物
    robot_ctrl.add_obstacle(5, 5, 2)
    robot_ctrl.add_obstacle(5, 6, 2)
    robot_ctrl.add_obstacle(5, 7, 2)
    
    # 规划路径
    start = (0, 0, 1)
    goal = (10, 10, 2)
    path = robot_ctrl.plan_path(start, goal)
    
    # 平滑路径
    smoothed_path = robot_ctrl.smooth_path()
    
    # 可视化
    robot_ctrl.visualize_map()
    
    print(f"路径点数量: {len(path)} -> {len(smoothed_path)}")
    print(f"机器人状态: {robot_ctrl.get_robot_state()['body_theta'][:3]}...")  # 显示前3个关节角度