# arm_robot

**arm_robot** 是一个机器人控制系统库，旨在提供先进的路径规划和逆运动学解决方案。该项目采用C++后端和Python前端的混合架构，实现了高性能的3D路径规划和机器人控制功能。

## 功能特点

### 1. 3D路径规划
- 实现了基于A*算法的3D路径规划器
- 支持任意形状和位置的障碍物
- 具备路径平滑功能，生成更加自然的运动轨迹
- 支持负坐标系环境下的路径规划
- 可处理复杂的3D迷宫环境

### 2. 逆运动学
- 基于DH参数的机器人模型
- 支持多关节机器人的逆运动学求解
- 提供多种控制模式（control_part 和 control_hand）
- 支持实时关节角度更新

### 3. 混合架构设计
- **C++后端**：负责核心算法实现（路径规划、机器人模型计算等）
- **Python前端**：负责测试驱动、数据展示和可视化呈现
- 通过pybind11实现C++与Python的双向交互
- 支持Eigen与numpy数组互操作

## 目录结构

```
arm_robot/
├── cpp/                    # C++源码
│   ├── arm_utils/          # 机器人基础工具
│   │   ├── body_base.cpp   # 机器人基类实现
│   │   ├── body_base.h     # 机器人基类定义
│   │   ├── coord.cpp       # 坐标系处理
│   │   ├── coord.h         # 坐标系定义
│   │   ├── darwin.cpp      # Darwin机器人模型
│   │   ├── darwin.h        # Darwin机器人定义
│   │   ├── dh.cpp          # DH参数实现
│   │   ├── dh.h            # DH参数定义
│   │   ├── marvin_base.hpp # Marvin机器人基类
│   │   ├── quaternion_utils.cpp # 四元数工具实现
│   │   └── quaternion_utils.h   # 四元数工具定义
│   ├── plan/               # 规划算法
│   │   ├── AStar3D.cpp     # 3D A*算法实现
│   │   ├── AStar3D.h       # 3D A*算法定义
│   │   ├── Map3D.cpp       # 3D地图实现
│   │   ├── Map3D.h         # 3D地图定义
│   │   ├── PathSmoother.cpp # 路径平滑实现
│   │   └── PathSmoother.h   # 路径平滑定义
│   ├── PathPlanner3D.cpp   # 3D路径规划器实现
│   ├── PathPlanner3D.h     # 3D路径规划器定义
│   ├── pybind_interface.cpp # PyBind接口实现
│   ├── Makefile            # 编译配置
│   ├── test_astar3d.cpp    # A*算法测试
│   ├── test_darwin.cpp     # Darwin机器人测试
│   └── test_path_smoother.cpp # 路径平滑测试
├── python/                 # Python接口和测试
│   ├── arm_utils/          # 机器人基础工具Python版
│   │   ├── body_base.py    # 机器人基类Python实现
│   │   ├── coord.py        # 坐标系处理Python版
│   │   ├── darwin.py       # Darwin机器人Python实现
│   │   ├── dh.py           # DH参数Python实现
│   │   └── quaternion_utils.py # 四元数工具Python实现
│   ├── plan/               # 规划算法Python版
│   │   ├── astar3d.py      # 3D A*算法Python实现
│   │   ├── demo_smooth_path.py # 路径平滑演示
│   │   ├── main.py         # 主程序
│   │   ├── map3d.py        # 3D地图Python实现
│   │   ├── path_smoother.py # 路径平滑Python实现
│   │   ├── test_astar3d.py # A*算法测试Python版
│   │   ├── test_improved_smoothing.py # 改进平滑测试
│   │   ├── test_inflation.py # 膨胀测试
│   │   ├── test_path_smoother.py # 路径平滑测试
│   │   └── test_visualization.py # 可视化测试
│   ├── cpp_path_planning_demo.py # C++路径规划演示
│   └── visualize_cpp_backend.py # C++后端可视化
└── README.md               # 项目文档
```

## 编译和安装

### 依赖项
- C++17兼容编译器
- Python 3.x
- NumPy
- Matplotlib
- Eigen3
- PyBind11

### 编译步骤

1. 克隆项目到本地：
   ```bash
   git clone <repository-url>
   cd arm_robot/cpp
   ```

2. 编译项目：
   ```bash
   make pybind
   ```

这将生成Python可以导入的`arm_robot_cpp.so`模块。

## 使用示例

### 3D路径规划示例

```python
import arm_robot_cpp as arm_robot
import numpy as np

# 创建3D路径规划器
planner = arm_robot.PathPlanner3D(20, 20, 20, 0.2)  # 20x20x20地图，分辨率为0.2m

# 定义障碍物
obstacles = [(2.0, i * 0.2, 2.0) for i in range(10)]

# 定义起点和终点
start = (0.0, 0.0, 0.0)
goal = (3.8, 3.8, 3.8)

# 执行路径规划
raw_path, smoothed_path = planner.planWithObstacles(start, goal, obstacles, useSmoothing=True)

if raw_path:
    print(f"原始路径长度: {len(raw_path)}")
    print(f"平滑后路径长度: {len(smoothed_path)}")
else:
    print("路径规划失败")
```

### 机器人逆运动学示例

```python
import arm_robot_cpp as arm_robot
import numpy as np

# 创建人体模型
human = arm_robot.Human()

# 设置DH参数
body_dh = [[-1.5707, 0, 0, -1.5707, 0], 
           [-3.1415, 0.41, 0.0, 0, 0],
           [0, 0.48, 0, 1.5707, 0],
           [1.5707, 0, 0.35, 3.1415, 0]]

left_hand_dh = [
    [-1.5707, 0.0, 0.253, 1.5707, 0],
    [1.5707, 0.0, 0.0, -1.5707, 0],
    [-1.5707, 0.0, 0.3579, 3.1415, 0],
    [-1.5707, -0.0537, 0.0, 3.1415, 0],
    [-1.5707, -0.036, 0.2041, 1.5707, 0],
    [-1.5707, 0.0, 0.0, -1.5707, 0],
    [1.5707, 0.08, 0.0, 1.5707, 0]]

human.setBodyDH(body_dh)
human.setHandDH(left_hand_dh, "left")

# 定义目标矩阵
target_matrix = np.eye(4)
target_matrix[0, 3] = 0.5  # x
target_matrix[1, 3] = 0.2  # y
target_matrix[2, 3] = 0.8  # z

# 执行逆运动学求解
result_theta = human.control_hand(target_matrix, "left")
```

## 演示程序

项目包含多个演示程序：

1. **cpp_path_planning_demo.py**：展示了多种3D路径规划场景，包括：
   - 基础路径规划
   - 点云数据处理
   - 随机障碍物环境
   - 负坐标系支持
   - 3D迷宫环境

2. **visualize_cpp_backend.py**：提供了机器人模型的可视化界面，支持：
   - 人体模型3D可视化
   - 关节角度控制
   - 逆运动学测试

## 许可证

本项目采用LICENSE许可证，请参见LICENSE文件获取详细信息。

## 作者

本项目由arm_robot团队开发。