# 机械臂动力学标定软件

这个软件包提供了使用NumPy、SciPy、Matplotlib和Pinocchio进行机械臂动力学参数辨识和摩擦建模的完整解决方案。

## 概述

动力学标定软件帮助识别机器人操纵器的动力学参数，包括：
- 每个连杆的质量、质心和惯性参数
- 摩擦参数（库伦摩擦、粘滞摩擦和静摩擦）
- 重力补偿项

## 组件

### 1. 动态标定器 ([dynamic_calibrator.py](file:///home/std/arm_robot/python/dynamic/dynamic_calibrator.py))
主类，实现动态参数估计算法，使用：
- 动力学参数线性的逆动力学模型
- 最小二乘参数辨识
- 包含Striebeck效应的摩擦建模

### 2. 标定工作流 ([calibration_workflow.py](file:///home/std/arm_robot/python/dynamic/calibration_workflow.py))
动态标定的完整工作流，包括：
- 从实验中收集数据
- 信号预处理和滤波
- 参数辨识
- 验证和报告

### 3. 实用工具 ([utils.py](file:///home/std/arm_robot/python/dynamic/utils.py))
辅助函数，包括：
- 激励轨迹生成
- 信号滤波
- 频域分析
- 数据验证

## 依赖

该软件需要以下Python包：
- NumPy
- SciPy
- Matplotlib
- Pinocchio（用于机器人动力学计算）

使用以下命令安装Pinocchio：
```bash
pip install pinocchio
```

更多信息请参见：https://github.com/stack-of-tasks/pinocchio

## 用法

### 基本用法示例

```python
from dynamic_calibrator import DynamicCalibrator

# 使用机器人模型初始化标定器
calibrator = DynamicCalibrator(urdf_path="path/to/robot.urdf")

# 加载实验数据
calibrator.load_data(positions, velocities, accelerations, torques)

# 估计动态参数
dyn_params = calibrator.estimate_dynamic_parameters()

# 估计摩擦参数
friction_params = calibrator.estimate_friction_parameters()

# 评估模型
calibrator.evaluate_fit()

# 绘制结果
calibrator.plot_results()
```

### 完整工作流示例

```python
from calibration_workflow import CalibrationWorkflow

# 初始化工作流
workflow = CalibrationWorkflow(robot_urdf_path="path/to/robot.urdf")

# 收集实验数据
workflow.collect_experiment_data("exp1", positions, velocities, accelerations, torques)

# 预处理数据
workflow.preprocess_data("exp1")

# 执行标定
dyn_params, friction_params = workflow.perform_calibration(["exp1"])

# 验证模型
rmse, mae, var_exp = workflow.validate_calibration(["exp1"])

# 生成报告
workflow.generate_report("calibration_report.pdf")
```

## 理论基础

机器人操纵器的动力学模型表示为：

```
τ = Y(θ, θ̇, θ̈) × θ̇ + τ_friction
```

其中：
- `τ` 是关节力矩向量
- `Y` 是包含运动学信息的回归矩阵
- `θ, θ̇, θ̈` 是关节位置、速度和加速度
- `θ` 是待辨识的动态参数向量
- `τ_friction` 表示摩擦力矩

动态参数包括：
- 每个连杆的质量
- 质心坐标
- 惯性张量分量

摩擦力建模为以下组合：
- 静摩擦
- 库伦摩擦
- 粘滞摩擦
- Striebeck效应（可选）

## 激励轨迹设计

为了准确的参数辨识，激励轨迹应该：
1. 激发所有运动模式
2. 在感兴趣的频率范围内具有足够的能量
3. 遵守关节限制和执行器约束
4. 提供持续激励

使用实用函数生成最佳轨迹：

```python
from utils import generate_excitation_trajectory

# 定义关节限制为 [(min1, max1), (min2, max2), ...]
joint_limits = [(-3.14, 3.14)] * 6  # 示例：6自由度机器人

trajectory = generate_excitation_trajectory(
    joint_limits=joint_limits,
    duration=10.0,  # 秒
    freq_range=(0.1, 2.0),  # Hz
    num_joints=6,
    sampling_freq=100
)
```

## 验证指标

识别模型的质量使用以下指标评估：
- 均方根误差（RMSE）
- 平均绝对误差（MAE）
- 方差解释率（VAF）

## 故障排除

### 常见问题

1. **识别质量差**：
   - 检查激励轨迹是否足够丰富
   - 验证数据是否经过适当滤波
   - 确保遵守关节限制

2. **数值不稳定**：
   - 增加正则化因子
   - 减少轨迹的频率内容
   - 检查传感器噪声

3. **Pinocchio安装**：
   - 在Ubuntu上：`sudo apt install python3-pinocchio`
   - 使用pip：`pip install pinocchio`

## 参考文献

1. Gautier, M. (1988). "Direct calculation of minimum set of inertial parameters of serial robots." IEEE Transactions on Robotics.
2. Swevers, J. et al. (2007). "An improved method for identifying the parameters of the friction model of industrial manipulators." Proceedings of the IEEE International Conference on Robotics and Automation.
3. Janot, A. et al. (2014). "A variant of the direct kinematic model-based control for robots with elastic joints." Robotica.