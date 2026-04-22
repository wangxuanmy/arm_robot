"""
数据验证检查程序
使用Pinocchio计算正向动力学并与测量数据对比
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
    
    return accelerations


def forward_dynamics_validation(urdf_path, positions, velocities, accelerations, measured_torques):
    """
    使用Pinocchio计算正向动力学并与测量数据对比
    """
    # 加载机器人模型
    model = pin.buildModelFromUrdf(urdf_path)
    data = model.createData()
    
    n_samples, n_joints = positions.shape
    
    # 计算每个采样点的理论力矩
    computed_torques = np.zeros_like(measured_torques)
    
    print(f"开始计算正向动力学...")
    for i in range(n_samples):
        q = positions[i, :]  # 关节位置
        v = velocities[i, :]  # 关节速度
        a = accelerations[i, :]  # 关节加速度
        
        # 使用逆动力学计算理论力矩 (RNEA - Recursive Newton-Euler Algorithm)
        tau_computed = pin.rnea(model, data, q, v, a)
        computed_torques[i, :] = tau_computed
    
    print(f"正向动力学计算完成")
    
    return computed_torques


def plot_comparison(timestamps, measured_torques, computed_torques, joint_names):
    """
    绘制测量力矩与计算力矩的对比图
    """
    n_joints = len(joint_names)
    
    # 创建子图
    fig, axes = plt.subplots(n_joints, 2, figsize=(15, 5*n_joints))
    if n_joints == 1:
        axes = axes.reshape(1, -1)
    
    for j in range(n_joints):
        # 左侧子图：对比图
        axes[j, 0].plot(timestamps, measured_torques[:, j], label='Measured Torque', alpha=0.7)
        axes[j, 0].plot(timestamps, computed_torques[:, j], label='Computed Torque', alpha=0.7)
        axes[j, 0].set_title(f'Joint {j+1} ({joint_names[j]}) - Torque Comparison')
        axes[j, 0].set_xlabel('Time (s)')
        axes[j, 0].set_ylabel('Torque (Nm)')
        axes[j, 0].legend()
        axes[j, 0].grid(True, alpha=0.3)
        
        # 右侧子图：误差图
        error = measured_torques[:, j] - computed_torques[:, j]
        axes[j, 1].plot(timestamps, error, label='Error (measured-computed)', color='red', alpha=0.7)
        axes[j, 1].set_title(f'Joint {j+1} ({joint_names[j]}) - Torque Error')
        axes[j, 1].set_xlabel('Time (s)')
        axes[j, 1].set_ylabel('Error (Nm)')
        axes[j, 1].legend()
        axes[j, 1].grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig('/home/std/arm_robot/python/dynamic/validation_comparison.png', dpi=300, bbox_inches='tight')
    plt.show()


def calculate_statistics(measured_torques, computed_torques):
    """
    计算误差统计信息
    """
    # 计算每个关节的误差
    errors = measured_torques - computed_torques
    
    # RMSE (均方根误差)
    rmse = np.sqrt(np.mean(errors**2, axis=0))
    
    # MAE (平均绝对误差)
    mae = np.mean(np.abs(errors), axis=0)
    
    # 最大误差
    max_error = np.max(np.abs(errors), axis=0)
    
    # 平均误差
    mean_error = np.mean(errors, axis=0)
    
    return {
        'rmse': rmse,
        'mae': mae,
        'max_error': max_error,
        'mean_error': mean_error
    }


def main():
    """
    主函数：执行数据验证
    """
    print("="*60)
    print("机械臂数据验证检查程序")
    print("使用Pinocchio计算正向动力学并与测量数据对比")
    print("="*60)
    
    # 加载数据
    print("\n1. 加载实验数据...")
    excel_file = "/home/std/arm_robot/python/dynamic/traj_test_data.xlsx"
    urdf_file = "/home/std/arm_robot/python/dynamic/Marvin.urdf"
    
    try:
        positions, velocities, torques, timestamps = load_arm_data_from_excel(excel_file)
    except Exception as e:
        print(f"加载Excel数据失败: {e}")
        print("尝试使用Marvin.urdf")
        urdf_file = "/home/std/arm_robot/python/dynamic/Marvin.urdf"
        positions, velocities, torques, timestamps = load_arm_data_from_excel(excel_file)
    
    # 计算采样时间间隔（假设均匀采样）
    dt = (timestamps[-1] - timestamps[0]) / len(timestamps)
    print(f"采样时间间隔: {dt:.4f}s, 采样频率: {1/dt:.2f}Hz")
    
    # 计算加速度
    print("\n2. 计算加速度...")
    accelerations = compute_accelerations_from_velocity(velocities, dt)
    
    # 执行正向动力学验证
    print("\n3. 执行正向动力学验证...")
    computed_torques = forward_dynamics_validation(urdf_file, positions, velocities, accelerations, torques)
    
    # 定义关节名称
    joint_names = ["left_shoulder_pan_joint", "left_shoulder_lift_joint", "left_elbow_1_joint", 
                   "left_elbow_2_joint", "left_wrist_1_joint", "left_wrist_2_joint", "left_wrist_3_joint"]
    
    # 绘制对比图
    print("\n4. 绘制对比图...")
    plot_comparison(timestamps, torques, computed_torques, joint_names)
    
    # 计算统计信息
    print("\n5. 计算误差统计信息...")
    stats = calculate_statistics(torques, computed_torques)
    
    # 输出结果摘要
    print("\n" + "="*60)
    print("数据验证结果摘要:")
    print("="*60)
    
    print(f"\n各关节误差统计:")
    print(f"{'Joint':<15} {'RMSE (Nm)':<12} {'MAE (Nm)':<12} {'Max Error':<12} {'Mean Error':<12}")
    print("-"*60)
    
    for i, name in enumerate(joint_names):
        print(f"{name:<15} {stats['rmse'][i]:<12.4f} {stats['mae'][i]:<12.4f} "
              f"{stats['max_error'][i]:<12.4f} {stats['mean_error'][i]:<12.4f}")
    
    print(f"\n总体统计:")
    print(f"平均RMSE: {np.mean(stats['rmse']):.4f} Nm")
    print(f"平均MAE: {np.mean(stats['mae']):.4f} Nm")
    print(f"最大误差: {np.max(stats['max_error']):.4f} Nm")
    
    # 数据质量评估
    print(f"\n数据质量评估:")
    avg_rmse = np.mean(stats['rmse'])
    if avg_rmse < 0.5:
        quality = "优秀 - 数据质量很高，误差很小"
    elif avg_rmse < 1.0:
        quality = "良好 - 数据质量较好，误差较小"
    elif avg_rmse < 2.0:
        quality = "一般 - 数据存在一定误差，可能需要进一步检查"
    else:
        quality = "较差 - 数据误差较大，可能存在严重问题"
    
    print(f"综合评估: {quality}")
    
    if avg_rmse > 1.0:
        print(f"\n建议检查以下方面:")
        print("- 传感器校准是否准确")
        print("- 数据采集过程中是否存在干扰")
        print("- 是否需要重新进行滤波处理")
        print("- 模型参数是否与实际机器人一致")
    
    print(f"\n数据验证检查完成!")
    print(f"对比图已保存至: /home/std/arm_robot/python/dynamic/validation_comparison.png")


if __name__ == "__main__":
    main()