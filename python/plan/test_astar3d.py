import numpy as np
import random
from map3d import Map3D
from astar3d import AStar3D

def test_simple_path():
    """
    测试简单路径规划
    """
    print("测试1: 简单路径规划")
    
    # 创建10x10x10的地图，分辨率为0.5米
    map3d = Map3D(10, 10, 10, 0.5)
    
    # 添加一些障碍物（使用实际坐标）
    map3d.add_obstacle(2.5, 2.5, 2.5)  # 对应网格(5,5,5)
    map3d.add_obstacle(2.5, 3.0, 2.5)  # 对应网格(5,6,5)
    map3d.add_obstacle(2.5, 3.5, 2.5)  # 对应网格(5,7,5)
    map3d.add_obstacle(2.5, 4.0, 2.5)  # 对应网格(5,8,5)
    
    # 创建A*算法实例
    astar = AStar3D(map3d)
    
    # 规划路径（使用实际坐标）
    start = (0.0, 0.0, 0.0)
    goal = (4.5, 4.5, 4.5)
    
    path = astar.plan(start, goal)
    
    print(f"地图分辨率: {map3d.resolution} 米/格")
    print(f"起点: {start}")
    print(f"终点: {goal}")
    print(f"路径: {path}")
    print(f"路径长度: {len(path)}")
    
    # 可视化结果（如果需要）
    # map3d.visualize(path)
    
    return path

def test_point_cloud_obstacles():
    """
    测试从点云数据添加障碍物
    """
    print("\n测试2: 点云障碍物")
    
    # 创建20x20x20的地图，分辨率为0.2米
    map3d = Map3D(20, 20, 20, 0.2)
    
    # 生成随机点云数据作为障碍物（使用实际坐标）
    point_cloud = []
    for _ in range(100):
        x = random.uniform(0.4, 3.6)
        y = random.uniform(0.4, 3.6)
        z = random.uniform(0.4, 3.6)
        point_cloud.append((x, y, z))
    
    # 添加点云障碍物到地图
    added_count = map3d.add_obstacles_from_point_cloud(point_cloud)
    print(f"从点云添加了 {added_count} 个障碍物")
    
    # 创建A*算法实例
    astar = AStar3D(map3d)
    
    # 规划路径
    start = (0.0, 0.0, 0.0)
    goal = (3.8, 3.8, 3.8)
    
    path = astar.plan(start, goal)
    
    print(f"地图分辨率: {map3d.resolution} 米/格")
    print(f"起点: {start}")
    print(f"终点: {goal}")
    print(f"路径点数: {len(path)}")
    
    return map3d, path

def test_complex_obstacles():
    """
    测试复杂障碍物环境
    """
    print("\n测试3: 复杂障碍物环境")
    
    # 创建15x15x15的地图，分辨率为0.3米
    map3d = Map3D(15, 15, 15, 0.3)
    
    # 创建一个"墙"障碍物（使用实际坐标）
    for x in range(5, 10):
        for z in range(5, 10):
            map3d.add_obstacle(x * 0.3, 2.1, z * 0.3)
    
    # 创建一个"柱子"障碍物
    for y in range(3, 12):
        map3d.add_obstacle(0.9, y * 0.3, 0.9)
    
    # 创建A*算法实例
    astar = AStar3D(map3d)
    
    # 规划路径
    start = (0.3, 0.3, 0.3)
    goal = (3.9, 3.9, 3.9)
    
    path = astar.plan(start, goal)
    
    print(f"地图分辨率: {map3d.resolution} 米/格")
    print(f"起点: {start}")
    print(f"终点: {goal}")
    print(f"路径点数: {len(path)}")
    
    if path:
        print("路径规划成功!")
        print(f"路径: {path}")
    else:
        print("未找到路径!")
    
    return map3d, path

def test_edge_cases():
    """
    测试边界情况
    """
    print("\n测试4: 边界情况")
    
    # 创建小地图
    map3d = Map3D(5, 5, 5, 1.0)
    
    # 几乎填满整个地图的障碍物
    for x in range(5):
        for y in range(5):
            for z in range(5):
                if (x, y, z) not in [(0, 0, 0), (4, 4, 4)]:
                    map3d.add_obstacle(float(x), float(y), float(z))
    
    # 创建A*算法实例
    astar = AStar3D(map3d)
    
    # 规划路径
    start = (0.0, 0.0, 0.0)
    goal = (4.0, 4.0, 4.0)
    
    path = astar.plan(start, goal)
    
    print(f"地图分辨率: {map3d.resolution} 米/格")
    print(f"起点: {start}")
    print(f"终点: {goal}")
    print(f"路径点数: {len(path)}")
    
    if path:
        print("路径规划成功!")
        print(f"路径: {path}")
    else:
        print("未找到路径!")
    
    return map3d, path

if __name__ == "__main__":
    print("开始3D A*算法测试")
    
    # 运行所有测试
    path1 = test_simple_path()
    map2, path2 = test_point_cloud_obstacles()
    map3, path3 = test_complex_obstacles()
    map4, path4 = test_edge_cases()
    
    print("\n测试完成")
    
    # 可视化最后一个测试结果
    print("\n可视化复杂障碍物环境的路径规划结果...")
    # map3.visualize(path3)