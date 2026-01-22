import numpy as np
from path_smoother import PathSmoother
from map3d import Map3D


def test_heuristic_improvement():
    """
    测试启发式改进功能
    """
    print("测试启发式改进功能")
    
    # 创建地图
    map3d = Map3D(20, 20, 20, 0.5)
    
    # 添加一个障碍物块
    for x in range(8, 12):
        for y in range(8, 12):
            for z in range(8, 12):
                map3d.add_obstacle(x * 0.5, y * 0.5, z * 0.5)
    
    # 创建一条穿过障碍物的路径
    path = [
        (1.0, 1.0, 1.0),
        (2.0, 2.0, 2.0),
        (3.0, 3.0, 3.0),
        (4.0, 4.0, 4.0),
        (5.0, 5.0, 5.0)
    ]
    
    print(f"原始路径: {path}")
    
    # 使用改进的平滑器
    smoother = PathSmoother()
    smoothed_path = smoother.smooth_path_with_obstacle_avoidance(
        path, map3d, smoothing_factor=0.5, num_points=30
    )
    
    print(f"平滑后路径点数: {len(smoothed_path)}")
    print(f"前5个平滑点: {smoothed_path[:5]}")
    
    # 检查冲突点数
    conflict_count = 0
    for point in smoothed_path:
        if map3d.is_obstacle(point[0], point[1], point[2]):
            conflict_count += 1
    
    print(f"冲突点数: {conflict_count}")
    
    return path, smoothed_path


def test_local_resmoothing():
    """
    测试局部再平滑功能
    """
    print("\n\n测试局部再平滑功能")
    
    # 创建地图
    map3d = Map3D(20, 20, 20, 0.5)
    
    # 添加一些障碍物
    for i in range(5, 15):
        map3d.add_obstacle(2.5, i * 0.5, 2.5)
        map3d.add_obstacle(5.0, i * 0.5, 2.5)
    
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
    
    # 使用改进的平滑器
    smoother = PathSmoother()
    smoothed_path = smoother.smooth_path_with_obstacle_avoidance(
        path, map3d, smoothing_factor=0.5, num_points=50
    )
    
    print(f"平滑后路径点数: {len(smoothed_path)}")
    
    return path, smoothed_path


if __name__ == "__main__":
    test_heuristic_improvement()
    test_local_resmoothing()
    print("\n改进功能测试完成")