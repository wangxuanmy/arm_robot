#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
路径平滑演示程序
展示从A*路径规划到B样条平滑的完整流程
"""

import numpy as np
from map3d import Map3D
from astar3d import AStar3D
from path_smoother import PathSmoother


def demo_basic_smoothing():
    """
    基本路径平滑演示
    """
    print("=== 基本路径平滑演示 ===")
    
    # 创建地图
    map3d = Map3D(20, 20, 20, 0.5)
    
    # 添加一些障碍物形成通道
    for i in range(5, 15):
        map3d.add_obstacle(2.5, i * 0.5, 2.5)  # 左墙
        map3d.add_obstacle(7.5, i * 0.5, 2.5)  # 右墙
    
    # 创建A*算法实例
    astar = AStar3D(map3d)
    
    # 规划路径
    start = (0.0, 0.0, 0.0)
    goal = (9.5, 9.5, 9.5)
    
    print(f"路径规划: 从 {start} 到 {goal}")
    path = astar.plan(start, goal)
    
    if not path:
        print("未找到路径")
        return
    
    print(f"原始路径点数: {len(path)}")
    
    # 路径平滑
    smoother = PathSmoother()
    smoothed_path = smoother.smooth_path(path, smoothing_factor=1.0, num_points=100)
    
    print(f"平滑后路径点数: {len(smoothed_path)}")
    
    # 可视化结果
    print("显示可视化结果...")
    map3d.visualize(path, smoothed_path)


def demo_complex_environment():
    """
    复杂环境路径平滑演示
    """
    print("\n=== 复杂环境路径平滑演示 ===")
    
    # 创建较大地图
    map3d = Map3D(30, 30, 30, 0.3)
    
    # 添加一些柱子（避免在起点和终点添加）
    for i in range(10, 25, 3):
        for j in range(10, 25, 3):
            # 避免在起点和终点附近添加障碍物
            if not (i < 7 and j < 7) and not (i > 22 and j > 22):
                map3d.add_obstacle(i * 0.3, 1.5, j * 0.3)
    
    # 创建A*算法实例
    astar = AStar3D(map3d)
    
    # 规划路径
    start = (1.5, 0.0, 1.5)
    goal = (8.4, 0.0, 8.4)
    
    print(f"路径规划: 从 {start} 到 {goal}")
    path = astar.plan(start, goal)
    
    if not path:
        print("未找到路径")
        return
    
    print(f"原始路径点数: {len(path)}")
    
    # 路径平滑
    smoother = PathSmoother()
    smoothed_path = smoother.smooth_path(path, smoothing_factor=0.8, num_points=80)
    
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


def demo_different_smoothing_factors():
    """
    不同平滑因子效果演示
    """
    print("\n=== 不同平滑因子效果演示 ===")
    
    # 创建地图
    map3d = Map3D(25, 25, 25, 0.4)
    
    # 添加一些障碍物
    for i in range(10, 20):
        map3d.add_obstacle(5.0, i * 0.4, 5.0)
        map3d.add_obstacle(8.0, i * 0.4, 5.0)
    
    # 创建A*算法实例
    astar = AStar3D(map3d)
    
    # 规划路径
    start = (0.0, 0.0, 0.0)
    goal = (9.6, 9.6, 9.6)
    
    print(f"路径规划: 从 {start} 到 {goal}")
    path = astar.plan(start, goal)
    
    if not path:
        print("未找到路径")
        return
    
    print(f"原始路径点数: {len(path)}")
    
    # 使用不同的平滑因子
    smoother = PathSmoother()
    factors = [0.1, 0.5, 1.0, 2.0]
    
    for factor in factors:
        smoothed_path = smoother.smooth_path(path, smoothing_factor=factor, num_points=60)
        print(f"平滑因子 {factor}: 生成 {len(smoothed_path)} 个点")


if __name__ == "__main__":
    print("3D 路径平滑演示程序")
    print("=" * 40)
    
    try:
        demo_basic_smoothing()
        demo_complex_environment()
        demo_different_smoothing_factors()
        
        print("\n所有演示完成!")
    except Exception as e:
        print(f"演示过程中出现错误: {e}")