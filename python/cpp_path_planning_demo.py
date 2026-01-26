#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
C++ Backend 3D Path Planning Algorithm Demo
"""
import sys
import os
import traceback

# ========== 核心修改：用脚本自身路径计算cpp目录的绝对路径 ==========
# 获取当前脚本（visualize_cpp_backend.py）的绝对路径
script_dir = os.path.abspath(os.path.dirname(__file__))
# 计算cpp目录的绝对路径（无论在哪运行，路径都正确）
cpp_dir = os.path.abspath(os.path.join(script_dir, "../cpp"))
# 添加到Python路径（优先添加绝对路径）
sys.path.insert(0, cpp_dir)

# ========== 增强版导入：带详细错误日志 ==========
try:
    import arm_robot_cpp
except ImportError as e:
    print("="*50)
    print("C++模块导入失败，错误详情：")
    print(f"1. 脚本所在目录：{script_dir}")
    print(f"2. 期望的C++模块目录：{cpp_dir}")
    print(f"3. 该目录是否存在：{os.path.exists(cpp_dir)}")
    # 列出cpp目录下的文件（排查.so是否存在）
    if os.path.exists(cpp_dir):
        print(f"4. C++目录下的文件：{os.listdir(cpp_dir)}")
    else:
        print("4. C++目录不存在！")
    print(f"5. ImportError原始信息：{str(e)}")
    print("="*50)
    print("解决步骤：")
    print("1. 检查cpp目录是否存在：ls", cpp_dir)
    print("2. 重新编译模块：cd", cpp_dir, "&& make clean && make pybind")
    print("3. 确认模块名是否为arm_robot_cpp.so（不是arm_robot.so）")
    print("="*50)
    exit(1)
except Exception as e:
    # 捕获其他异常（比如.so加载失败、依赖库缺失）
    print("C++模块加载异常（非导入找不到）：")
    traceback.print_exc()
    exit(1)

# 验证导入成功（可选）
print(f"✅ C++模块导入成功！模块路径：{arm_robot_cpp.__file__}")

import numpy as np
import random
import matplotlib
matplotlib.use('TkAgg')  # 设置matplotlib后端
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


def create_random_obstacles(width, height, depth, resolution, num_obstacles, start, goal):
    """
    在地图上随机生成障碍物
    
    Args:
        width, height, depth: 地图尺寸（格子数）
        resolution: 分辨率
        num_obstacles: 障碍物数量
        start, goal: 起点和终点
    Returns:
        obstacles: 障碍物列表
    """
    obstacles = []
    added = 0
    
    while added < num_obstacles:
        x = random.uniform(0, (width - 1) * resolution)
        y = random.uniform(0, (height - 1) * resolution)
        z = random.uniform(0, (depth - 1) * resolution)
        
        # 避免在起点和终点附近添加障碍物
        distance_to_start = np.sqrt((x-start[0])**2 + (y-start[1])**2 + (z-start[2])**2)
        distance_to_goal = np.sqrt((x-goal[0])**2 + (y-goal[1])**2 + (z-goal[2])**2)
        
        if distance_to_start > resolution * 2 and distance_to_goal > resolution * 2:
            obstacles.append((x, y, z))
            added += 1
    
    return obstacles

def demo_basic():
    """
    基础演示
    """
    print("=== C++后端 3D A*算法演示 ===")
    
    # 创建20x20x20的地图，分辨率为0.2米
    width, height, depth = 20, 20, 20
    resolution = 0.2
    
    # 创建障碍物
    obstacles = []
    # 创建一个简单的墙
    for i in range(10):
        obstacles.append((2.0, i * 0.2, 2.0))
    
    # 创建路径规划器
    planner = arm_robot_cpp.PathPlanner3D(width, height, depth, resolution)
    
    # 定义起点和终点（使用实际坐标）
    start = (0.0, 0.0, 0.0)
    goal = (3.8, 3.8, 3.8)
    
    print(f"地图尺寸: {width} x {height} x {depth}")
    print(f"地图分辨率: {resolution} 米/格")
    print(f"实际地图大小: {width*resolution} x {height*resolution} x {depth*resolution} 米")
    print(f"起点: {start}")
    print(f"终点: {goal}")
    print(f"障碍物数量: {len(obstacles)}")
    
    # 执行路径规划 - 现在返回原始路径和平滑后路径两个结果
    raw_path, smoothed_path = planner.planWithObstacles(start, goal, obstacles, useSmoothing=True)
    
    if raw_path:
        print(f"原始路径规划成功，路径长度: {len(raw_path)}")
    else:
        print("原始路径规划失败，未找到可行路径")
        return None, None
    
    if smoothed_path:
        print(f"平滑后路径长度: {len(smoothed_path)}")
        
        # 检查平滑路径质量
        is_good_quality = check_path_quality(smoothed_path)
        if not is_good_quality:
            print("警告: 平滑路径可能存在折线问题")
    else:
        print("路径平滑失败")
        return raw_path, None
    
    # 可视化结果
    print("显示可视化结果...")
    visualize_path(start, goal, obstacles, raw_path, smoothed_path)
    
    return raw_path, smoothed_path

def demo_point_cloud():
    """
    点云数据演示
    """
    print("\n=== 点云数据演示 ===")
    
    # 创建30x30x30的地图，分辨率为0.1米
    width, height, depth = 30, 30, 30
    resolution = 0.1
    
    # 生成模拟的点云数据
    obstacles = []
    # 创建一个球形障碍物
    center = (1.5, 1.5, 1.5)
    radius = 0.8
    for x in range(30):
        for y in range(30):
            for z in range(30):
                world_x = x * 0.1
                world_y = y * 0.1
                world_z = z * 0.1
                distance = np.sqrt((world_x-center[0])**2 + (world_y-center[1])**2 + (world_z-center[2])**2)
                if abs(distance - radius) < 0.15:  # 球壳
                    obstacles.append((world_x, world_y, world_z))
    
    print(f"从点云数据添加了 {len(obstacles)} 个障碍物")
    
    # 创建路径规划器
    planner = arm_robot_cpp.PathPlanner3D(width, height, depth, resolution)
    
    # 定义起点和终点（使用实际坐标）
    start = (0.0, 0.0, 0.0)
    goal = (2.9, 2.9, 2.9)
    
    print(f"地图尺寸: {width} x {height} x {depth}")
    print(f"地图分辨率: {resolution} 米/格")
    print(f"实际地图大小: {width*resolution} x {height*resolution} x {depth*resolution} 米")
    print(f"起点: {start}")
    print(f"终点: {goal}")
    
    # 执行路径规划
    raw_path, smoothed_path = planner.planWithObstacles(start, goal, obstacles, useSmoothing=True)
    
    if raw_path:
        print(f"原始路径规划成功，路径长度: {len(raw_path)}")
    else:
        print("原始路径规划失败，未找到可行路径")
        return None, None
    
    if smoothed_path:
        print(f"平滑后路径长度: {len(smoothed_path)}")

        # 检查平滑路径质量
        is_good_quality = check_path_quality(smoothed_path)
        if not is_good_quality:
            print("警告: 平滑路径可能存在折线问题")
    else:
        print("路径平滑失败")
        return raw_path, None
    
    # 可视化结果
    print("显示可视化结果...")
    visualize_path(start, goal, obstacles, raw_path, smoothed_path)
    
    return raw_path, smoothed_path

def demo_random_obstacles():
    """
    随机障碍物演示
    """
    print("\n=== 随机障碍物演示 ===")
    
    # 创建25x25x25的地图，分辨率为0.15米
    width, height, depth = 25, 25, 25
    resolution = 0.15
    
    # 添加随机障碍物
    start = (0.0, 0.0, 0.0)
    goal = (3.6, 3.6, 3.6)
    obstacles = create_random_obstacles(width, height, depth, resolution, 100, start, goal)
    print(f"添加了{len(obstacles)}个随机障碍物")
    
    # 创建路径规划器
    planner = arm_robot_cpp.PathPlanner3D(width, height, depth, resolution)
    
    print(f"地图尺寸: {width} x {height} x {depth}")
    print(f"地图分辨率: {resolution} 米/格")
    print(f"实际地图大小: {width*resolution} x {height*resolution} x {depth*resolution} 米")
    print(f"起点: {start}")
    print(f"终点: {goal}")
    
    # 执行路径规划
    raw_path, smoothed_path = planner.planWithObstacles(start, goal, obstacles, useSmoothing=True)
    
    if raw_path:
        print(f"原始路径规划成功，路径长度: {len(raw_path)}")
    else:
        print("原始路径规划失败，未找到可行路径")
        return None, None
    
    if smoothed_path:
        print(f"平滑后路径长度: {len(smoothed_path)}")

        # 检查平滑路径质量
        is_good_quality = check_path_quality(smoothed_path)
        if not is_good_quality:
            print("警告: 平滑路径可能存在折线问题")
    else:
        print("路径平滑失败")
        return raw_path, None
    
    # 可视化结果
    print("显示可视化结果...")
    visualize_path(start, goal, obstacles, raw_path, smoothed_path)
    
    return raw_path, smoothed_path

def demo_negative_coordinates():
    """
    负坐标范围演示
    """
    print("\n=== 负坐标范围演示 ===")
    
    # 创建一个支持负坐标的地图
    # 地图范围: x[-5, 5], y[-5, 5], z[0, 10] (单位: 米)
    width, height, depth = 50, 50, 50
    resolution = 0.2
    origin_x, origin_y, origin_z = -5.0, -5.0, 0.0
    
    # 创建障碍物
    obstacles = []
    # 在负坐标区域添加障碍物
    for i in range(10):
        obstacles.append((-3.0, -4.0 + i * 0.8, 2.0))
    
    # 在跨越0点的区域添加障碍物
    for i in range(15):
        obstacles.append((-1.0 + i * 0.6, 1.0, 3.0))
    
    # 创建路径规划器
    planner = arm_robot_cpp.PathPlanner3D(width, height, depth, resolution, 
                                          origin_x, origin_y, origin_z)
    
    # 定义起点和终点，其中终点在负坐标区域
    start = (3.0, 3.0, 1.0)          # 正坐标区域
    goal = (-3.0, -3.0, 5.0)         # 负坐标区域
    
    print(f"地图尺寸: {width} x {height} x {depth}")
    print(f"地图分辨率: {resolution} 米/格")
    print(f"地图原点: ({origin_x}, {origin_y}, {origin_z})")
    print(f"实际地图范围: X[{origin_x}, {origin_x + (width-1)*resolution}], "
          f"Y[{origin_y}, {origin_y + (height-1)*resolution}], "
          f"Z[{origin_z}, {origin_z + (depth-1)*resolution}]")
    print(f"起点: {start}")
    print(f"终点: {goal}")
    
    # 执行路径规划
    raw_path, smoothed_path = planner.planWithObstacles(start, goal, obstacles, useSmoothing=True)
    
    if raw_path:
        print(f"原始路径规划成功，路径长度: {len(raw_path)}")
    else:
        print("原始路径规划失败，未找到可行路径")
        return None, None
    
    if smoothed_path:
        print(f"平滑后路径长度: {len(smoothed_path)}")

        # 检查平滑路径质量
        is_good_quality = check_path_quality(smoothed_path)
        if not is_good_quality:
            print("警告: 平滑路径可能存在折线问题")
    else:
        print("路径平滑失败")
        return raw_path, None
    
    # 可视化结果
    print("显示可视化结果...")
    visualize_path(start, goal, obstacles, raw_path, smoothed_path)
    
    return raw_path, smoothed_path

def demo_maze():
    """
    3D迷宫演示
    """
    print("\n=== 3D迷宫演示 ===")
    
    # 创建一个30x30x15的地图，分辨率为0.15米
    width, height, depth = 30, 30, 15
    resolution = 0.15
    
    # 生成3D迷宫
    obstacles = generate_3d_maze(width, height, depth, resolution)
    
    # 创建路径规划器
    planner = arm_robot_cpp.PathPlanner3D(width, height, depth, resolution)
    
    # 定义起点和终点
    start = (0.0, 0.0, 0.0)  # 迷宫入口
    goal = ((width-1) * resolution, (height-1) * resolution, (depth-1) * resolution)  # 迷宫出口
    
    print(f"地图尺寸: {width} x {height} x {depth}")
    print(f"地图分辨率: {resolution} 米/格")
    print(f"实际地图大小: {width*resolution} x {height*resolution} x {depth*resolution} 米")
    print(f"起点: {start}")
    print(f"终点: {goal}")
    print(f"障碍物数量: {len(obstacles)}")
    
    # 执行路径规划
    raw_path, smoothed_path = planner.planWithObstacles(start, goal, obstacles, useSmoothing=True)
    
    if raw_path:
        print(f"原始路径规划成功，路径长度: {len(raw_path)}")
    else:
        print("原始路径规划失败，未找到可行路径")
    
    if smoothed_path:
        print(f"平滑后路径长度: {len(smoothed_path)}")

        # 检查平滑路径质量
        is_good_quality = check_path_quality(smoothed_path)
        if not is_good_quality:
            print("警告: 平滑路径可能存在折线问题")
    else:
        print("路径平滑失败")
    
    # 可视化结果（即使路径规划失败也会显示迷宫结构）
    print("显示可视化结果...")
    visualize_path(start, goal, obstacles, raw_path, smoothed_path)
    
    return raw_path, smoothed_path

def generate_3d_maze(width, height, depth, resolution):
    """
    生成一个3D迷宫
    使用类似真实迷宫的设计，只创建墙壁而不填满空间
    """
    print("正在生成3D迷宫...")
    
    obstacles = []
    
    # 创建外部边框
    # 底部和顶部平面
    for x in range(width):
        for y in range(height):
            obstacles.append((x * resolution, y * resolution, 0))
            obstacles.append((x * resolution, y * resolution, (depth-1) * resolution))
    
    # 四周墙壁
    for x in range(width):
        for z in range(1, depth-1):
            obstacles.append((x * resolution, 0, z * resolution))
            obstacles.append((x * resolution, (height-1) * resolution, z * resolution))
            
    for y in range(1, height-1):
        for z in range(1, depth-1):
            obstacles.append((0, y * resolution, z * resolution))
            obstacles.append(((width-1) * resolution, y * resolution, z * resolution))
    
    # 创建迷宫内墙，采用网格状布局
    wall_spacing_x = 6  # X方向墙体间隔
    wall_spacing_y = 6  # Y方向墙体间隔
    wall_spacing_z = 5  # Z方向墙体间隔
    
    # X方向的纵向墙（沿Y轴延伸）
    for x in range(wall_spacing_x, width, wall_spacing_x):
        for y in range(1, height-1, 2):  # 隔一行放一行，留下通道
            for z in range(1, depth-1):
                obstacles.append((x * resolution, y * resolution, z * resolution))
    
    # Y方向的横向墙（沿X轴延伸）
    for y in range(wall_spacing_y, height, wall_spacing_y):
        for x in range(1, width-1, 2):  # 隔一列放一列，留下通道
            for z in range(1, depth-1):
                obstacles.append((x * resolution, y * resolution, z * resolution))
    
    # Z方向的水平墙（创建层次间的连接）
    # 在某些位置创建"楼梯"效果，允许在Z层之间移动
    for z in range(2, depth-1, 3):  # 每三层创建一层"楼梯"
        for x in range(2, width-2, 4):
            for y in range(2, height-2, 4):
                # 创建一个开放的平台
                for dx in range(2):
                    for dy in range(2):
                        if (x+dx < width-1 and y+dy < height-1):
                            obstacles.append(((x+dx) * resolution, (y+dy) * resolution, z * resolution))
    
    # 移除一些墙体来创建通道
    # 为X方向墙开洞
    for x in range(wall_spacing_x, width, wall_spacing_x):
        # 在每面墙上随机开1-2个洞
        open_y_positions = [
            random.randint(1, wall_spacing_x-1),
            random.randint(wall_spacing_x, 2*wall_spacing_x-1)
        ]
        for y_pos in open_y_positions:
            if y_pos < height:
                for z in range(1, depth-1):
                    obstacle = (x * resolution, y_pos * resolution, z * resolution)
                    if obstacle in obstacles:
                        obstacles.remove(obstacle)
    
    # 为Y方向墙开洞
    for y in range(wall_spacing_y, height, wall_spacing_y):
        # 在每面墙上随机开1-2个洞
        open_x_positions = [
            random.randint(1, wall_spacing_y-1),
            random.randint(wall_spacing_y, 2*wall_spacing_y-1)
        ]
        for x_pos in open_x_positions:
            if x_pos < width:
                for z in range(1, depth-1):
                    obstacle = (x_pos * resolution, y * resolution, z * resolution)
                    if obstacle in obstacles:
                        obstacles.remove(obstacle)
    
    # 为Z方向墙开洞（创建Z轴上的通道）
    for z in range(2, depth-1, 3):
        for x in range(2, width-2, 4):
            for y in range(2, height-2, 4):
                # 在平台的一些位置留出空隙
                gap_x = random.randint(x, min(x+1, width-2))
                gap_y = random.randint(y, min(y+1, height-2))
                gap_z = z
                obstacle = (gap_x * resolution, gap_y * resolution, gap_z * resolution)
                if obstacle in obstacles:
                    obstacles.remove(obstacle)
    
    # 确保起点和终点附近是开放的
    start_area = [(0, 0, 0), (1, 0, 0), (0, 1, 0), (1, 1, 0)]
    end_area = [
        ((width-1), (height-1), (depth-1)), 
        ((width-2), (height-1), (depth-1)),
        ((width-1), (height-2), (depth-1)),
        ((width-2), (height-2), (depth-1))
    ]
    
    for pos in start_area:
        obstacle = (pos[0] * resolution, pos[1] * resolution, pos[2] * resolution)
        if obstacle in obstacles:
            obstacles.remove(obstacle)
    
    for pos in end_area:
        obstacle = (pos[0] * resolution, pos[1] * resolution, pos[2] * resolution)
        if obstacle in obstacles:
            obstacles.remove(obstacle)
    
    print(f"3D迷宫生成完成，包含 {len(obstacles)} 个障碍物")
    return obstacles

def check_path_quality(path):
    """
    检查路径质量，检测是否有突然的方向变化
    """
    if len(path) < 3:
        return True  # 路径太短无法评估
    
    sudden_changes = 0
    for i in range(1, len(path)-1):
        p0 = np.array(path[i-1])
        p1 = np.array(path[i])
        p2 = np.array(path[i+1])
        
        # 计算相邻段之间的夹角
        v1 = p1 - p0
        v2 = p2 - p1
        
        # 归一化向量
        v1_norm = v1 / np.linalg.norm(v1)
        v2_norm = v2 / np.linalg.norm(v2)
        
        # 计算夹角（弧度）
        dot_product = np.clip(np.dot(v1_norm, v2_norm), -1.0, 1.0)
        angle = np.arccos(dot_product)
        
        # 如果角度接近180度（直线），则认为是平滑的
        # 如果角度接近0度（折返），或非常尖锐的角度，则认为是突变
        if angle > np.pi / 3:  # 大于60度认为是突变
            sudden_changes += 1

    return sudden_changes == 0  # 突变点不超过总点数的30%

def visualize_path(start, goal, obstacles, raw_path, smoothed_path):
    """
    可视化路径（包含原始路径和平滑后路径）
    """
    fig = plt.figure(figsize=(14, 10))
    ax = fig.add_subplot(111, projection='3d')
    
    # 绘制障碍物
    if obstacles:
        obs_x, obs_y, obs_z = zip(*obstacles)
        ax.scatter(obs_x, obs_y, obs_z, c='red', marker='s', alpha=0.3, label='Obstacles')
    
    # 绘制原始路径
    if raw_path:
        px, py, pz = zip(*raw_path)
        ax.plot(px, py, pz, 'r-', linewidth=1, label='Raw Path', alpha=0.6)
        ax.scatter([px[0]], [py[0]], [pz[0]], color='green', s=100, label='Start', depthshade=False)
        ax.scatter([px[-1]], [py[-1]], [pz[-1]], color='red', s=100, label='Goal', depthshade=False)
    
    # 绘制平滑后路径
    if smoothed_path and len(smoothed_path) > 0:
        spx, spy, spz = zip(*smoothed_path)
        ax.plot(spx, spy, spz, 'b-', linewidth=2, label='Smoothed Path')
    
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.legend()
    
    plt.title('3D Path Planning Visualization - Raw vs Smoothed Path')
    plt.show()

if __name__ == "__main__":
    print("C++后端 3D A* 路径规划算法演示程序")
    print("=" * 40)
    
    try:
        path1, smoothed_path1 = demo_basic()
        path2, smoothed_path2 = demo_point_cloud()
        path3, smoothed_path3 = demo_random_obstacles()
        path4, smoothed_path4 = demo_negative_coordinates()
        path5, smoothed_path5 = demo_maze()  # 添加3D迷宫测试
        
        print("\n所有演示完成!")
        print("\n效果对比总结:")
        if path1 and smoothed_path1:
            print(f"基础演示: 原始路径 {len(path1)} 点 -> 平滑路径 {len(smoothed_path1)} 点")
        if path2 and smoothed_path2:
            print(f"点云演示: 原始路径 {len(path2)} 点 -> 平滑路径 {len(smoothed_path2)} 点")
        if path3 and smoothed_path3:
            print(f"随机障碍物演示: 原始路径 {len(path3)} 点 -> 平滑路径 {len(smoothed_path3)} 点")
        if path4 and smoothed_path4:
            print(f"负坐标演示: 原始路径 {len(path4)} 点 -> 平滑路径 {len(smoothed_path4)} 点")
        if path5 and smoothed_path5:
            print(f"3D迷宫演示: 原始路径 {len(path5)} 点 -> 平滑路径 {len(smoothed_path5)} 点")
    except Exception as e:
        print(f"演示过程中出现错误: {e}")
        import traceback
        traceback.print_exc()