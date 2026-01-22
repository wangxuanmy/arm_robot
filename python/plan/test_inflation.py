import numpy as np
from map3d import Map3D
from astar3d import AStar3D

def test_inflation():
    """
    测试障碍物膨胀功能
    """
    print("测试障碍物膨胀功能")
    
    # 创建10x10x10的地图，分辨率为1.0米
    map3d = Map3D(10, 10, 10, 1.0)
    
    # 设置膨胀半径为1.5米
    map3d.set_inflation_radius(1.5)
    print(f"地图分辨率: {map3d.resolution} 米/格")
    print(f"膨胀半径: 1.5 米 ({int(np.ceil(1.5 / map3d.resolution))} 格)")
    
    # 添加一个障碍物在中心
    map3d.add_obstacle(5.0, 5.0, 5.0)
    print("在 (5.0, 5.0, 5.0) 添加了一个障碍物")
    
    # 检查膨胀效果
    print("\n膨胀后的障碍物位置:")
    obstacle_positions = np.where(map3d.grid == 1)
    for x, y, z in zip(obstacle_positions[0], obstacle_positions[1], obstacle_positions[2]):
        world_x, world_y, world_z = map3d._grid_to_world(x, y, z)
        print(f"  网格({x}, {y}, {z}) -> 世界坐标({world_x}, {world_y}, {world_z})")
    
    # 创建A*算法实例
    astar = AStar3D(map3d)
    
    # 规划路径，绕过膨胀的障碍物
    start = (0.0, 0.0, 0.0)
    goal = (9.0, 9.0, 9.0)
    
    print(f"\n路径规划:")
    print(f"起点: {start}")
    print(f"终点: {goal}")
    
    path = astar.plan(start, goal)
    
    if path:
        print(f"路径规划成功，路径长度: {len(path)}")
        print(f"路径: {path}")
    else:
        print("路径规划失败，未找到可行路径")
    
    # 可视化结果
    print("\n显示可视化结果...")
    map3d.visualize(path)

def test_point_cloud_with_inflation():
    """
    测试点云障碍物与膨胀功能
    """
    print("\n\n测试点云障碍物与膨胀功能")
    
    # 创建20x20x20的地图，分辨率为0.5米
    map3d = Map3D(20, 20, 20, 0.5)
    
    # 设置膨胀半径为0.75米
    map3d.set_inflation_radius(0.75)
    print(f"地图分辨率: {map3d.resolution} 米/格")
    print(f"膨胀半径: 0.75 米 ({int(np.ceil(0.75 / map3d.resolution))} 格)")
    
    # 生成点云数据（一条线上的点）
    point_cloud = []
    for i in range(8, 12):
        point_cloud.append((i * 0.5, 1.0, 1.0))  # x方向的一条线
    
    # 添加点云障碍物
    added_count = map3d.add_obstacles_from_point_cloud(point_cloud)
    print(f"从点云添加了 {added_count} 个障碍物")
    
    # 检查膨胀效果
    print("\n膨胀后的障碍物数量:")
    obstacle_count = np.sum(map3d.grid)
    print(f"  膨胀前: {len(point_cloud)}")
    print(f"  膨胀后: {obstacle_count}")
    
    # 创建A*算法实例
    astar = AStar3D(map3d)
    
    # 规划路径
    start = (0.0, 0.0, 0.0)
    goal = (9.5, 9.5, 9.5)
    
    print(f"\n路径规划:")
    print(f"起点: {start}")
    print(f"终点: {goal}")
    
    path = astar.plan(start, goal)
    
    if path:
        print(f"路径规划成功，路径长度: {len(path)}")
    else:
        print("路径规划失败，未找到可行路径")
    
    # 可视化结果
    print("\n显示可视化结果...")
    map3d.visualize(path)

if __name__ == "__main__":
    test_inflation()
    test_point_cloud_with_inflation()