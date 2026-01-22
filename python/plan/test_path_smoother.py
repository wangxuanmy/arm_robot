import numpy as np
from path_smoother import PathSmoother
from map3d import Map3D
from astar3d import AStar3D


def test_basic_smoothing():
    """
    测试基本路径平滑功能
    """
    print("测试基本路径平滑功能")
    
    # 创建一个锯齿状路径
    path = [
        (0.0, 0.0, 0.0),
        (1.0, 0.0, 0.0),
        (1.0, 1.0, 0.0),
        (2.0, 1.0, 0.0),
        (2.0, 2.0, 0.0),
        (3.0, 2.0, 0.0),
        (3.0, 3.0, 0.0),
        (4.0, 3.0, 0.0),
        (4.0, 4.0, 0.0)
    ]
    
    print(f"原始路径点数: {len(path)}")
    
    # 创建路径平滑器
    smoother = PathSmoother()
    
    # 平滑路径
    smoothed_path = smoother.smooth_path(path, smoothing_factor=0.5, num_points=50)
    
    print(f"平滑后路径点数: {len(smoothed_path)}")
    print(f"前5个平滑路径点: {smoothed_path[:5]}")
    print(f"后5个平滑路径点: {smoothed_path[-5:]}")
    
    return path, smoothed_path


def test_astar_path_smoothing():
    """
    测试A*路径的平滑处理
    """
    print("\n\n测试A*路径的平滑处理")
    
    # 创建地图
    map3d = Map3D(20, 20, 20, 0.5)
    
    # 添加一些障碍物
    for i in range(5, 15):
        map3d.add_obstacle(2.5, i * 0.5, 2.5)
        map3d.add_obstacle(5.0, i * 0.5, 2.5)
    
    # 创建A*算法实例
    astar = AStar3D(map3d)
    
    # 规划路径
    start = (0.0, 0.0, 0.0)
    goal = (9.5, 9.5, 9.5)
    
    print(f"路径规划: 从 {start} 到 {goal}")
    path = astar.plan(start, goal)
    
    if not path:
        print("未找到路径")
        return None, None
    
    print(f"原始路径点数: {len(path)}")
    
    # 平滑路径（带障碍物避让）
    smoother = PathSmoother()
    smoothed_path = smoother.smooth_path_with_obstacle_avoidance(path, map3d, smoothing_factor=1.0)
    
    print(f"平滑后路径点数: {len(smoothed_path)}")
    
    # 检查平滑路径是否与障碍物冲突
    conflict_count = 0
    for point in smoothed_path:
        if map3d.is_obstacle(point[0], point[1], point[2]):
            conflict_count += 1
    
    print(f"平滑路径与障碍物冲突点数: {conflict_count}")
    
    return path, smoothed_path


def compare_smoothing_factors():
    """
    比较不同平滑因子的效果
    """
    print("\n\n比较不同平滑因子的效果")
    
    # 创建一个弯曲路径
    path = []
    for i in range(20):
        t = i / 19.0 * 2 * np.pi
        x = t
        y = np.sin(t)
        z = np.cos(t)
        path.append((x, y, z))
    
    smoother = PathSmoother()
    
    # 测试不同的平滑因子
    factors = [0.1, 0.5, 1.0, 2.0]
    for factor in factors:
        smoothed_path = smoother.smooth_path(path, smoothing_factor=factor, num_points=100)
        print(f"平滑因子 {factor}: 生成 {len(smoothed_path)} 个点")


def test_obstacle_avoidance():
    """
    测试障碍物避让功能
    """
    print("\n\n测试障碍物避让功能")
    
    # 创建地图
    map3d = Map3D(20, 20, 20, 0.5)
    
    # 添加一些障碍物形成一个通道
    for i in range(8, 12):
        for j in range(8, 12):
            map3d.add_obstacle(i * 0.5, j * 0.5, 2.5)
    
    # 创建一个穿过障碍物区域的路径
    path = [
        (0.0, 0.0, 0.0),
        (1.0, 1.0, 1.0),
        (2.0, 2.0, 2.0),
        (3.0, 3.0, 3.0),
        (4.0, 4.0, 4.0)
    ]
    
    print(f"原始路径点数: {len(path)}")
    
    # 检查原始路径是否与障碍物冲突
    original_conflicts = 0
    for point in path:
        if map3d.is_obstacle(point[0], point[1], point[2]):
            original_conflicts += 1
    
    print(f"原始路径与障碍物冲突点数: {original_conflicts}")
    
    # 平滑路径（带障碍物避让）
    smoother = PathSmoother()
    smoothed_path = smoother.smooth_path_with_obstacle_avoidance(path, map3d, smoothing_factor=0.5, num_points=30)
    
    print(f"平滑后路径点数: {len(smoothed_path)}")
    
    # 检查平滑路径是否与障碍物冲突
    smoothed_conflicts = 0
    for point in smoothed_path:
        if map3d.is_obstacle(point[0], point[1], point[2]):
            smoothed_conflicts += 1
    
    print(f"平滑路径与障碍物冲突点数: {smoothed_conflicts}")
    
    return path, smoothed_path


def test_improved_features():
    """
    测试改进的功能：启发式搜索和局部再平滑
    """
    print("\n\n测试改进的功能：启发式搜索和局部再平滑")
    
    # 创建地图
    map3d = Map3D(20, 20, 20, 0.5)
    
    # 添加一个障碍物块
    for x in range(8, 12):
        for y in range(8, 12):
            for z in range(8, 12):
                map3d.add_obstacle(x * 0.5, y * 0.5, z * 0.5)
    
    # 创建一条穿过障碍物中心的路径
    path = [
        (1.0, 1.0, 1.0),
        (2.0, 2.0, 2.0),
        (3.0, 3.0, 3.0),
        (4.0, 4.0, 4.0),
        (5.0, 5.0, 5.0)
    ]
    
    print(f"原始路径点数: {len(path)}")
    
    # 检查原始路径是否与障碍物冲突
    original_conflicts = 0
    for point in path:
        if map3d.is_obstacle(point[0], point[1], point[2]):
            original_conflicts += 1
    
    print(f"原始路径与障碍物冲突点数: {original_conflicts}")
    
    # 使用改进的平滑算法
    smoother = PathSmoother()
    smoothed_path = smoother.smooth_path_with_obstacle_avoidance(
        path, map3d, smoothing_factor=0.8, num_points=40
    )
    
    print(f"改进算法平滑后路径点数: {len(smoothed_path)}")
    
    # 检查平滑路径是否与障碍物冲突
    smoothed_conflicts = 0
    for point in smoothed_path:
        if map3d.is_obstacle(point[0], point[1], point[2]):
            smoothed_conflicts += 1
    
    print(f"改进算法平滑路径与障碍物冲突点数: {smoothed_conflicts}")
    
    return path, smoothed_path


if __name__ == "__main__":
    # 运行测试
    test_basic_smoothing()
    test_astar_path_smoothing()
    compare_smoothing_factors()
    test_obstacle_avoidance()
    test_improved_features()
    
    print("\n所有测试完成")