"""
数据单位转换验证程序
检查数据是否以度为单位，需要转换为弧度进行计算
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
    
    # 打印原始测量力矩的统计信息
    print(f"\n原始测量力矩统计信息:")
    for i, joint in enumerate(joints):
        joint_torques = torques[:, i]
        print(f"  {joint}: min={np.min(joint_torques):.4f}, max={np.max(joint_torques):.4f}, mean={np.mean(joint_torques):.4f}, std={np.std(joint_torques):.4f}")
    
    # 打印原始位置和速度的统计信息
    print(f"\n原始位置数据统计信息:")
    for i, joint in enumerate(joints):
        joint_pos = positions[:, i]
        print(f"  {joint}: min={np.min(joint_pos):.4f}, max={np.max(joint_pos):.4f}, mean={np.mean(joint_pos):.4f}, std={np.std(joint_pos):.4f}")
    
    print(f"\n原始速度数据统计信息:")
    for i, joint in enumerate(joints):
        joint_vel = velocities[:, i]
        print(f"  {joint}: min={np.min(joint_vel):.4f}, max={np.max(joint_vel):.4f}, mean={np.mean(joint_vel):.4f}, std={np.std(joint_vel):.4f}")
    
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
    
    # 打印转换后的统计信息
    joint_names = ["left_shoulder_pan_joint", "left_shoulder_lift_joint", "left_elbow_1_joint", 
                   "left_elbow_2_joint", "left_wrist_1_joint", "left_wrist_2_joint", "left_wrist_3_joint"]
    
    print(f"\n转换后的速度数据统计信息 (弧度/秒):")
    for i, joint in enumerate(joint_names):
        joint_vel = velocities_rad[:, i]
        print(f"  {joint}: min={np.min(joint_vel):.4f}, max={np.max(joint_vel):.4f}, mean={np.mean(joint_vel):.4f}, std={np.std(joint_vel):.4f}")
    
    max_vel_rad = np.max(np.abs(velocities_rad))
    print(f"最大速度绝对值 (转换后): {max_vel_rad:.4f} rad/s ({np.degrees(max_vel_rad):.2f} °/s)")
    
    return positions_rad, velocities_rad


def compute_accelerations_from_velocity(velocities, dt):
    """
    通过对速度求导计算加速度
    使用Savitzky-Golay滤波器计算加速度，以减少噪声
    """
    n_samples, n_joints = velocities.shape
    
    # 首先检查速度数据是否有异常
    vel_max = np.max(np.abs(velocities))
    if vel_max > 10:  # 如果速度超过10 rad/s，给出提醒
        print(f"注意: 检测到较大的速度值: {vel_max:.2f} rad/s")
    
    window_length = min(51, n_samples - (n_samples % 2 == 0))  # 确保窗口长度为奇数
    polyorder = 3
    
    accelerations = np.zeros_like(velocities)
    
    for i in range(n_joints):
        accelerations[:, i] = savgol_filter(velocities[:, i], window_length, polyorder, deriv=1, delta=dt)
    
    # 检查加速度是否有异常
    acc_max = np.max(np.abs(accelerations))
    if acc_max > 100:  # 如果加速度超过100 rad/s²，给出提醒
        print(f"注意: 检测到较大的加速度值: {acc_max:.2f} rad/s²")
    
    print(f"加速度计算完成，最大值: {acc_max:.4f} rad/s²")
    
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
    
    # 检查计算出的力矩是否异常
    max_computed_torque = np.max(np.abs(computed_torques))
    if max_computed_torque > 100:  # 如果最大力矩超过100Nm，发出警告
        print(f"警告: 计算出的力矩值过大 (最大值: {max_computed_torque:.2f}Nm)")
        print("这可能表示:")
        print("1. URDF模型的物理参数不准确")
        print("2. 数据与模型来自不同的机器人系统")
        print("3. 加速度计算过程中引入了噪声")
    else:
        print(f"计算出的力矩值在合理范围内 (最大值: {max_computed_torque:.2f}Nm)")
    
    return computed_torques


def plot_comparison(timestamps, measured_torques, computed_torques, joint_names):
    """
    绘制测量力矩与计算力矩的对比图
    """
    n_joints = min(len(joint_names), measured_torques.shape[1], computed_torques.shape[1])
    
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
    plt.savefig('/home/std/arm_robot/python/dynamic/unit_conversion_comparison.png', dpi=300, bbox_inches='tight')
    plt.show()


def calculate_statistics(measured_torques, computed_torques):
    """
    计算误差统计信息
    """
    # 取两者中较小的关节数量
    n_joints = min(measured_torques.shape[1], computed_torques.shape[1])
    measured_subset = measured_torques[:, :n_joints]
    computed_subset = computed_torques[:, :n_joints]
    
    # 检查是否有异常大的计算力矩值
    large_values = np.where(np.abs(computed_subset) > 1000)
    if len(large_values[0]) > 0:
        print(f"警告: 发现 {len(large_values[0])} 个大于1000Nm的计算力矩值")
    
    # 计算每个关节的误差
    errors = measured_subset - computed_subset
    
    # RMSE (均方根误差)
    rmse = np.sqrt(np.mean(errors**2, axis=0))
    
    # MAE (平均绝对误差)
    mae = np.mean(np.abs(errors), axis=0)
    
    # 最大误差
    max_error = np.max(np.abs(errors), axis=0)
    
    # 平均误差
    mean_error = np.mean(errors, axis=0)
    
    # VAF (Variance Accounted For)
    # VAF = (1 - var(error)/var(measured))*100%
    var_measured = np.var(measured_subset, axis=0)
    var_error = np.var(errors, axis=0)
    vaf = (1 - var_error/var_measured)*100
    
    return {
        'rmse': rmse,
        'mae': mae,
        'max_error': max_error,
        'mean_error': mean_error,
        'vaf': vaf
    }


def main():
    """
    主函数：执行单位转换验证
    """
    print("="*60)
    print("数据单位转换验证程序")
    print("检查数据是否以度为单位，需要转换为弧度进行计算")
    print("="*60)
    
    # 加载数据
    print("\n1. 加载实验数据...")
    excel_file = "/home/std/arm_robot/python/dynamic/traj_test_data.xlsx"
    urdf_file = "/home/std/arm_robot/python/dynamic/Marvin.urdf"  # 默认使用Marvin
    
    try:
        positions, velocities, torques, timestamps = load_arm_data_from_excel(excel_file)
    except Exception as e:
        print(f"加载Excel数据失败: {e}")
        print("尝试使用Marvin.urdf")
        urdf_file = "/home/std/arm_robot/python/dynamic/Marvin.urdf"
        positions, velocities, torques, timestamps = load_arm_data_from_excel(excel_file)
    
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
    
    # 绘制对比图
    print("\n5. 绘制对比图...")
    plot_comparison(timestamps, torques, computed_torques, joint_names)
    
    # 计算统计信息
    print("\n6. 计算误差统计信息...")
    stats = calculate_statistics(torques, computed_torques)
    
    # 输出结果摘要
    print("\n" + "="*60)
    print("数据验证结果摘要:")
    print("="*60)
    
    n_display_joints = min(len(joint_names), len(stats['rmse']))
    
    # 按照要求的格式输出详细指标
    print(f"\n验证指标:")
    print(f"{'关节':<8} {'RMSE (Nm)':<15} {'MAE (Nm)':<15} {'VAF (%)':<15}")
    print("-" + "-"*55)
    
    for i in range(n_display_joints):
        print(f"{i+1:<8} {stats['rmse'][i]:<15.4f} {stats['mae'][i]:<15.4f} {stats['vaf'][i]:<15.2f}%")
    
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
    
    print(f"\n单位转换验证完成!")
    print(f"对比图已保存至: /home/std/arm_robot/python/dynamic/unit_conversion_comparison.png")


if __name__ == "__main__":
    main()