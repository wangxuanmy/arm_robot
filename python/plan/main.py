#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
3D A* Path Planning Algorithm Demo
"""

import numpy as np
import random
from map3d import Map3D
from astar3d import AStar3D
from path_smoother import PathSmoother

def create_random_obstacles(map3d, num_obstacles):
    """
    在地图上随机生成障碍物
    
    Args:
        map3d: 3D地图对象
        num_obstacles: 障碍物数量
    """
    added = 0
    while added < num_obstacles:
        x = random.uniform(0, (map3d.width - 1) * map3d.resolution)
        y = random.uniform(0, (map3d.height - 1) * map3d.resolution)
        z = random.uniform(0, (map3d.depth - 1) * map3d.resolution)
        
        # 避免在起点和终点添加障碍物
        start = (0, 0, 0)
        goal = ((map3d.width-1) * map3d.resolution, 
                (map3d.height-1) * map3d.resolution, 
                (map3d.depth-1) * map3d.resolution)
                
        distance_to_start = np.sqrt((x-start[0])**2 + (y-start[1])**2 + (z-start[2])**2)
        distance_to_goal = np.sqrt((x-goal[0])**2 + (y-goal[1])**2 + (z-goal[2])**2)
        
        if distance_to_start > map3d.resolution * 2 and distance_to_goal > map3d.resolution * 2:
            map3d.add_obstacle(x, y, z)
            added += 1

def demo_basic():
    """
    基础演示
    """
    print("=== 基础3D A*算法演示 ===")
    
    # 创建20x20x20的地图，分辨率为0.2米
    map3d = Map3D(20, 20, 20, 0.2)
    
    # 添加一些障碍物（使用实际坐标）
    # 创建一个简单的墙
    for i in range(10):
        map3d.add_obstacle(2.0, i * 0.2, 2.0)
    
    # 创建A*算法实例
    astar = AStar3D(map3d)
    
    # 定义起点和终点（使用实际坐标）
    start = (0.0, 0.0, 0.0)
    goal = (3.8, 3.8, 3.8)
    
    print(f"地图尺寸: {map3d.width} x {map3d.height} x {map3d.depth}")
    print(f"地图分辨率: {map3d.resolution} 米/格")
    print(f"实际地图大小: {map3d.width*map3d.resolution} x {map3d.height*map3d.resolution} x {map3d.depth*map3d.resolution} 米")
    print(f"起点: {start}")
    print(f"终点: {goal}")
    
    # 执行路径规划
    path = astar.plan(start, goal)
    
    if path:
        print(f"路径规划成功，路径长度: {len(path)}")
        # print(f"路径: {path}")
    else:
        print("路径规划失败，未找到可行路径")
        return None, None
    
    # 路径平滑（带障碍物避让）
    print("进行路径平滑处理（带障碍物避让）...")
    smoother = PathSmoother()
    smoothed_path = smoother.smooth_path_with_obstacle_avoidance(path, map3d, smoothing_factor=0.5)
    print(f"平滑后路径点数: {len(smoothed_path)}")
    
    # 检查冲突
    conflict_count = 0
    for point in smoothed_path:
        if map3d.is_obstacle(point[0], point[1], point[2]):
            conflict_count += 1
    print(f"平滑路径与障碍物冲突点数: {conflict_count}")
        
    # 可视化结果
    print("显示可视化结果...")
    map3d.visualize(path, smoothed_path)
    
    return path, smoothed_path

def demo_point_cloud():
    """
    点云数据演示
    """
    print("\n=== 点云数据演示 ===")
    
    # 创建30x30x30的地图，分辨率为0.1米
    map3d = Map3D(30, 30, 30, 0.1)
    
    # 生成模拟的点云数据
    point_cloud = []
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
                    point_cloud.append((world_x, world_y, world_z))
    
    # 添加点云障碍物
    added_count = map3d.add_obstacles_from_point_cloud(point_cloud)
    print(f"从点云数据添加了 {added_count} 个障碍物")
    
    # 创建A*算法实例
    astar = AStar3D(map3d)
    
    # 定义起点和终点（使用实际坐标）
    start = (0.0, 0.0, 0.0)
    goal = (2.9, 2.9, 2.9)
    
    print(f"地图尺寸: {map3d.width} x {map3d.height} x {map3d.depth}")
    print(f"地图分辨率: {map3d.resolution} 米/格")
    print(f"实际地图大小: {map3d.width*map3d.resolution} x {map3d.height*map3d.resolution} x {map3d.depth*map3d.resolution} 米")
    print(f"起点: {start}")
    print(f"终点: {goal}")
    
    # 执行路径规划
    path = astar.plan(start, goal)
    
    if path:
        print(f"路径规划成功，路径长度: {len(path)}")
    else:
        print("路径规划失败，未找到可行路径")
        return None, None
    
    # 路径平滑（带障碍物避让）
    print("进行路径平滑处理（带障碍物避让）...")
    smoother = PathSmoother()
    smoothed_path = smoother.smooth_path_with_obstacle_avoidance(path, map3d, smoothing_factor=1.0, num_points=100)
    print(f"平滑后路径点数: {len(smoothed_path)}")
    
    # 检查冲突
    conflict_count = 0
    for point in smoothed_path:
        if map3d.is_obstacle(point[0], point[1], point[2]):
            conflict_count += 1
    print(f"平滑路径与障碍物冲突点数: {conflict_count}")
        
    # 可视化结果
    print("显示可视化结果...")
    map3d.visualize(path, smoothed_path)
    
    return path, smoothed_path

def demo_random_obstacles():
    """
    随机障碍物演示
    """
    print("\n=== 随机障碍物演示 ===")
    
    # 创建25x25x25的地图，分辨率为0.15米
    map3d = Map3D(25, 25, 25, 0.15)
    
    # 添加随机障碍物
    create_random_obstacles(map3d, 100)
    print(f"添加了100个随机障碍物")
    
    # 创建A*算法实例
    astar = AStar3D(map3d)
    
    # 定义起点和终点（使用实际坐标）
    start = (0.0, 0.0, 0.0)
    goal = (3.6, 3.6, 3.6)
    
    print(f"地图尺寸: {map3d.width} x {map3d.height} x {map3d.depth}")
    print(f"地图分辨率: {map3d.resolution} 米/格")
    print(f"实际地图大小: {map3d.width*map3d.resolution} x {map3d.height*map3d.resolution} x {map3d.depth*map3d.resolution} 米")
    print(f"起点: {start}")
    print(f"终点: {goal}")
    
    # 执行路径规划
    path = astar.plan(start, goal)
    
    if path:
        print(f"路径规划成功，路径长度: {len(path)}")
    else:
        print("路径规划失败，未找到可行路径")
        return None, None
    
    # 路径平滑（带障碍物避让）
    print("进行路径平滑处理（带障碍物避让）...")
    smoother = PathSmoother()
    smoothed_path = smoother.smooth_path_with_obstacle_avoidance(path, map3d, smoothing_factor=0.8)
    print(f"平滑后路径点数: {len(smoothed_path)}")
    
    # 检查冲突
    conflict_count = 0
    for point in smoothed_path:
        if map3d.is_obstacle(point[0], point[1], point[2]):
            conflict_count += 1
    print(f"平滑路径与障碍物冲突点数: {conflict_count}")
        
    # 可视化结果
    print("显示可视化结果...")
    map3d.visualize(path, smoothed_path)
    
    return path, smoothed_path

def demo_negative_coordinates():
    """
    负坐标范围演示
    """
    print("\n=== 负坐标范围演示 ===")
    
    # 创建一个支持负坐标的地图
    # 地图范围: x[-5, 5], y[-5, 5], z[0, 10] (单位: 米)
    map_width, map_height, map_depth = 50, 50, 50
    resolution = 0.2
    map3d = Map3D(map_width, map_height, map_depth, resolution, 
                  origin_x=-5.0, origin_y=-5.0, origin_z=0.0)
    
    # 添加一些障碍物
    # 在负坐标区域添加障碍物
    for i in range(10):
        map3d.add_obstacle(-3.0, -4.0 + i * 0.8, 2.0)
    
    # 在跨越0点的区域添加障碍物
    for i in range(15):
        map3d.add_obstacle(-1.0 + i * 0.6, 1.0, 3.0)
    
    # 创建A*算法实例
    astar = AStar3D(map3d)
    
    # 定义起点和终点，其中终点在负坐标区域
    start = (3.0, 3.0, 1.0)          # 正坐标区域
    goal = (-3.0, -3.0, 5.0)         # 负坐标区域
    
    print(f"地图尺寸: {map3d.width} x {map3d.height} x {map3d.depth}")
    print(f"地图分辨率: {map3d.resolution} 米/格")
    print(f"地图原点: ({map3d.origin_x}, {map3d.origin_y}, {map3d.origin_z})")
    print(f"实际地图范围: X[{map3d.origin_x}, {map3d.origin_x + (map3d.width-1)*map3d.resolution}], "
          f"Y[{map3d.origin_y}, {map3d.origin_y + (map3d.height-1)*map3d.resolution}], "
          f"Z[{map3d.origin_z}, {map3d.origin_z + (map3d.depth-1)*map3d.resolution}]")
    print(f"起点: {start}")
    print(f"终点: {goal}")
    
    # 执行路径规划
    path = astar.plan(start, goal)
    
    if path:
        print(f"路径规划成功，路径长度: {len(path)}")
    else:
        print("路径规划失败，未找到可行路径")
        return None, None
    
    # 路径平滑（带障碍物避让）
    print("进行路径平滑处理（带障碍物避让）...")
    smoother = PathSmoother()
    smoothed_path = smoother.smooth_path_with_obstacle_avoidance(path, map3d, smoothing_factor=0.8)
    print(f"平滑后路径点数: {len(smoothed_path)}")
    
    # 检查冲突
    conflict_count = 0
    for point in smoothed_path:
        if map3d.is_obstacle(point[0], point[1], point[2]):
            conflict_count += 1
    print(f"平滑路径与障碍物冲突点数: {conflict_count}")
        
    # 可视化结果
    print("显示可视化结果...")
    map3d.visualize(path, smoothed_path)
    
    return path, smoothed_path

if __name__ == "__main__":
    print("3D A* 路径规划算法演示程序")
    print("=" * 40)
    
    try:
        path1, smoothed_path1 = demo_basic()
        path2, smoothed_path2 = demo_point_cloud()
        path3, smoothed_path3 = demo_random_obstacles()
        path4, smoothed_path4 = demo_negative_coordinates()
        
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
    except Exception as e:
        print(f"演示过程中出现错误: {e}")