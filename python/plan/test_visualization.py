import time
import numpy as np
from map3d import Map3D
from astar3d import AStar3D

def test_visualization_performance():
    """
    测试可视化性能
    """
    print("测试可视化性能")
    
    # 创建中等大小的地图
    map3d = Map3D(30, 30, 30, 0.5)
    
    # 添加大量障碍物
    print("添加障碍物...")
    obstacle_count = 0
    for x in range(5, 25):
        for y in range(5, 25):
            for z in range(5, 15):
                if (x + y + z) % 3 == 0:  # 稀疏一些
                    map3d.add_obstacle(x * 0.5, y * 0.5, z * 0.5)
                    obstacle_count += 1
    
    print(f"添加了 {obstacle_count} 个障碍物")
    
    # 测试简单可视化性能
    print("\n测试简单可视化性能...")
    start_time = time.time()
    # 不实际显示，只测试数据处理
    obstacle_grid_x, obstacle_grid_y, obstacle_grid_z = np.where(map3d.grid == 1)
    end_time = time.time()
    print(f"数据处理耗时: {end_time - start_time:.4f} 秒")
    
    # 创建A*算法实例
    astar = AStar3D(map3d)
    
    # 规划一条简单路径
    start = (0.0, 0.0, 0.0)
    goal = (14.5, 14.5, 14.5)
    
    print(f"\n路径规划:")
    print(f"起点: {start}")
    print(f"终点: {goal}")
    
    path = astar.plan(start, goal)
    
    if path:
        print(f"路径规划成功，路径长度: {len(path)}")
    else:
        print("路径规划失败，未找到可行路径")
    
    # 测试详细可视化（仅在障碍物较少时）
    print("\n测试可视化选项...")
    small_map = Map3D(10, 10, 10, 1.0)
    for x in range(3, 7):
        for y in range(3, 7):
            small_map.add_obstacle(float(x), float(y), 5.0)
    
    print("小地图障碍物数量:", np.sum(small_map.grid))
    
    # 可视化结果
    print("\n显示可视化结果...")
    small_map.visualize(None, detailed_obstacles=True)
    small_map.visualize(None, detailed_obstacles=False)

def test_large_map_visualization():
    """
    测试大地图可视化性能
    """
    print("\n\n测试大地图可视化性能")
    
    # 创建大地图
    map3d = Map3D(50, 50, 50, 0.2)
    print(f"创建了 {map3d.width}x{map3d.height}x{map3d.depth} 的地图")
    
    # 添加随机障碍物
    print("添加随机障碍物...")
    np.random.seed(42)  # 固定随机种子以便复现
    for _ in range(500):
        x = np.random.randint(5, 45)
        y = np.random.randint(5, 45)
        z = np.random.randint(5, 45)
        map3d.add_obstacle(x * 0.2, y * 0.2, z * 0.2)
    
    obstacle_count = np.sum(map3d.grid)
    print(f"总障碍物数量: {obstacle_count}")
    
    # 测试可视化性能
    print("测试可视化性能...")
    start_time = time.time()
    map3d.visualize()  # 这会显示简化版的障碍物
    end_time = time.time()
    print(f"可视化耗时: {end_time - start_time:.4f} 秒")

if __name__ == "__main__":
    test_visualization_performance()
    test_large_map_visualization()