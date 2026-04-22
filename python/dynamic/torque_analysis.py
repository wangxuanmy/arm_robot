"""
力矩分析程序
详细分析各关节的测量力矩和计算力矩对比
"""

import pandas as pd
import numpy as np
import pinocchio as pin
import matplotlib.pyplot as plt
from scipy.signal import savgol_filter

# 设置matplotlib支持中文字体（但图表标签使用英文以避免字体问题）
plt.rcParams['font.sans-serif'] = ['DejaVu Sans', 'SimHei', 'WenQuanYi Micro Hei', 'Arial Unicode MS', 'Liberation Sans']
plt.rcParams['axes.unicode_minus'] = False  # 正确显示负号


def load_arm_data_from_excel(file_path):
    """
    从Excel文件加载左手机械臂数据
    """
    df = pd.read_excel(file_path)
    
    # 提取左手关节数据（7个关节）
    joints = ["left_shoulder_pan_joint", "left_shoulder_lift_joint", "left_elbow_1_joint", 
              "left_elbow_2_joint", "left_wrist_1_joint", "left_wrist_2_joint", "left_wrist_3_joint"]
    
    # 提取位置、速度和力矩数据
    positions = np.column_stack([
        df[f'{joint}_pos'].values for joint in joints
    ])
    
    velocities = np.column_stack([
        df[f'{joint}_vel'].values for joint in joints
    ])
    
    torques = np.column_stack([
        df[f'{joint}_eff'].values for joint in joints
    ])
    
    timestamps = df['timestamp'].values
    
    print(f"数据加载成功:")
    print(f"- 采样点数: {len(timestamps)}")
    print(f"- 关节数: {positions.shape[1]}")
    print(f"- 时间范围: {timestamps[0]:.2f} - {timestamps[-1]:.2f}")
    
    return positions, velocities, torques, timestamps


def convert_units_to_radians(positions, velocities):
    """
    将位置和速度从度转换为弧度
    """
    print("\n正在将数据从度单位转换为弧度单位...")
    
    # 度到弧度的转换
    positions_rad = positions
    velocities_rad = np.radians(velocities)
    
    print("转换完成！")
    
    return positions_rad, velocities_rad


def compute_accelerations_from_velocity(velocities, dt):
    """
    通过对速度求导计算加速度
    使用Savitzky-Golay滤波器计算加速度，以减少噪声
    """
    n_samples, n_joints = velocities.shape
    
    window_length = min(51, n_samples - (n_samples % 2 == 0))  # 确保窗口长度为奇数
    polyorder = 3
    
    accelerations = np.zeros_like(velocities)
    
    for i in range(n_joints):
        accelerations[:, i] = savgol_filter(velocities[:, i], window_length, polyorder, deriv=1, delta=dt)
    
    print(f"加速度计算完成")
    
    return accelerations


def forward_dynamics_validation(urdf_path, positions, velocities, accelerations, measured_torques):
    """
    使用Pinocchio计算正向动力学并与测量数据对比
    """
    # 加载机器人模型
    model = pin.buildModelFromUrdf(urdf_path)
    data = model.createData()
    
    print(f"模型信息:")
    print(f"- 关节数: {model.njoints}")
    print(f"- 自由度数: {model.nv}")
    print(f"- 关节名称: {[model.names[i] for i in range(1, len(model.names))]}")  # 跳过第一个固定坐标系
    
    n_samples, n_data_joints = positions.shape
    
    # 检查数据中的关节数量是否与模型匹配
    if n_data_joints != model.nv:
        print(f"警告: 数据中的关节数({n_data_joints})与模型中的自由度数({model.nv})不匹配")
        print(f"将使用两者中的较小值进行计算")
        n_calc_joints = min(n_data_joints, model.nv)
    else:
        n_calc_joints = n_data_joints
    
    print(f"将使用 {n_calc_joints} 个关节进行计算")
    
    # 计算每个采样点的理论力矩
    computed_torques = np.zeros((n_samples, n_data_joints))
    
    print(f"开始计算正向动力学...")
    
    for i in range(n_samples):
        # 创建与模型匹配的向量
        q = np.zeros(model.nv)
        v = np.zeros(model.nv)
        a = np.zeros(model.nv)
        
        # 将数据复制到对应的维度
        q[:n_calc_joints] = positions[i, :n_calc_joints]
        v[:n_calc_joints] = velocities[i, :n_calc_joints]
        a[:n_calc_joints] = accelerations[i, :n_calc_joints]
        
        # 使用逆动力学计算理论力矩 (RNEA - Recursive Newton-Euler Algorithm)
        tau_computed = pin.rnea(model, data, q, v, a)
        
        # 将计算出的力矩复制回结果数组，只复制与数据相同的维度
        computed_torques[i, :n_calc_joints] = tau_computed[:n_calc_joints]
    
    print(f"正向动力学计算完成")
    
    return computed_torques


def analyze_torque_statistics(measured_torques, computed_torques, joint_names):
    """
    分析测量力矩和计算力矩的详细统计数据
    """
    print("\n" + "="*80)
    print("力矩统计分析")
    print("="*80)
    
    n_joints = min(len(joint_names), measured_torques.shape[1], computed_torques.shape[1])
    
    print(f"{'Joint':<20} {'Meas Min':<10} {'Meas Max':<10} {'Comp Min':<10} {'Comp Max':<10} {'Diff Range':<12} {'Ratio':<10}")
    print("-"*80)
    
    for i in range(n_joints):
        meas_min = np.min(measured_torques[:, i])
        meas_max = np.max(measured_torques[:, i])
        comp_min = np.min(computed_torques[:, i])
        comp_max = np.max(computed_torques[:, i])
        
        meas_range = meas_max - meas_min
        comp_range = comp_max - comp_min
        
        # 计算比例，避免除以0
        ratio = comp_range / meas_range if meas_range != 0 else float('inf')
        
        print(f"{joint_names[i]:<20} {meas_min:<10.3f} {meas_max:<10.3f} {comp_min:<10.3f} {comp_max:<10.3f} {(meas_range-comp_range):<12.3f} {ratio:<10.3f}")
    
    print("\n各关节力矩范围对比:")
    for i in range(n_joints):
        meas_range = np.max(measured_torques[:, i]) - np.min(measured_torques[:, i])
        comp_range = np.max(computed_torques[:, i]) - np.min(computed_torques[:, i])
        
        print(f"  {joint_names[i]}:")
        print(f"    测量力矩范围: {meas_range:.3f} Nm")
        print(f"    计算力矩范围: {comp_range:.3f} Nm")
        print(f"    差值: {meas_range - comp_range:.3f} Nm")
        print(f"    比例: {comp_range/meas_range:.3f}" if meas_range != 0 else f"    比例: 无穷大")
        print()


def main():
    """
    主函数：执行详细力矩分析
    """
    print("="*60)
    print("详细力矩分析程序")
    print("比较测量力矩和计算力矩的统计数据")
    print("="*60)
    
    # 加载数据
    print("\n1. 加载实验数据...")
    excel_file = "/home/std/arm_robot/python/dynamic/traj_test_data.xlsx"
    urdf_file = "/home/std/arm_robot/python/dynamic/Marvin.urdf"  # 使用Marvin模型
    
    try:
        positions, velocities, torques, timestamps = load_arm_data_from_excel(excel_file)
    except Exception as e:
        print(f"加载Excel数据失败: {e}")
        return
    
    # 转换数据单位
    print("\n2. 转换数据单位（度 -> 弧度）...")
    positions_rad, velocities_rad = convert_units_to_radians(positions, velocities)
    
    # 计算采样时间间隔（假设均匀采样）
    dt = (timestamps[-1] - timestamps[0]) / len(timestamps)
    print(f"采样时间间隔: {dt:.4f}s, 采样频率: {1/dt:.2f}Hz")
    
    # 计算加速度（使用转换后的弧度数据）
    print("\n3. 计算加速度...")
    accelerations = compute_accelerations_from_velocity(velocities_rad, dt)
    
    # 执行正向动力学验证
    print("\n4. 执行正向动力学验证...")
    computed_torques = forward_dynamics_validation(urdf_file, positions_rad, velocities_rad, accelerations, torques)
    
    # 定义关节名称
    joint_names = ["left_shoulder_pan_joint", "left_shoulder_lift_joint", "left_elbow_1_joint", 
                   "left_elbow_2_joint", "left_wrist_1_joint", "left_wrist_2_joint", "left_wrist_3_joint"]
    
    # 分析力矩统计数据
    print("\n5. 分析力矩统计数据...")
    analyze_torque_statistics(torques, computed_torques, joint_names)
    
    print(f"\n力矩分析完成!")


if __name__ == "__main__":
    main()