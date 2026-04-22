"""
左手7自由度机械臂动力学标定脚本
使用提供的URDF文件和Excel数据进行动力学参数标定
"""

import pandas as pd
import numpy as np
from dynamic_calibrator import DynamicCalibrator
from calibration_workflow import CalibrationWorkflow
import matplotlib.pyplot as plt


def load_arm_data_from_excel(file_path):
    """
    从Excel文件加载左手机械臂数据
    """
    df = pd.read_excel(file_path)
    
    # 提取左手关节数据（7个关节）
    # joints = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'joint7']
    joints = ["left_shoulder_pan_joint", "left_shoulder_lift_joint", "left_elbow_1_joint", "left_elbow_2_joint", "left_wrist_1_joint", "left_wrist_2_joint", "left_wrist_3_joint"]
    
    # 提取位置、速度和力矩数据
    positions = np.column_stack([
        df[f'{joint}_pos'].values for joint in joints
    ])
    
    velocities = np.column_stack([
        df[f'{joint}_vel'].values for joint in joints
    ]) / 57.3
    
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
    """
    # 使用Savitzky-Golay滤波器计算加速度，以减少噪声
    from scipy.signal import savgol_filter
    n_samples, n_joints = velocities.shape
    window_length = min(51, n_samples - (n_samples % 2 == 0))  # 确保窗口长度为奇数
    polyorder = 3
    
    accelerations = np.zeros_like(velocities)
    
    for i in range(n_joints):
        accelerations[:, i] = savgol_filter(velocities[:, i], window_length, polyorder, deriv=1, delta=dt)
    
    return accelerations


def main():
    """
    主函数：执行左手机械臂的动力学标定
    """
    print("="*60)
    print("左手机械臂7自由度动力学标定")
    print("="*60)
    
    # 1. 加载数据
    print("\n1. 加载实验数据...")
    excel_file = "/home/std/arm_robot/python/dynamic/traj_test_data_filtered.xlsx"
    urdf_file = "/home/std/arm_robot/python/dynamic/Marvin.urdf"
    
    positions, velocities, torques, timestamps = load_arm_data_from_excel(excel_file)
    
    # 计算采样时间间隔（假设均匀采样）
    dt = (timestamps[-1] - timestamps[0]) / len(timestamps)
    print(f"采样时间间隔: {dt:.4f}s, 采样频率: {1/dt:.2f}Hz")

    
    
    # 计算加速度
    print("\n2. 计算加速度...")
    accelerations = compute_accelerations_from_velocity(velocities, dt)
    
    # 3. 指定要标定的关节
    # joints_to_calibrate = ['openarm_joint1', 'openarm_joint2', 'openarm_joint3', 
    #                       'openarm_joint4', 'openarm_joint5', 'openarm_joint6', 'openarm_joint7']
    joints_to_calibrate = ['Joint1_L', 'Joint2_L', 'Joint3_L', 'Joint4_L', 'Joint5_L', 'Joint6_L', 'Joint7_L']
    
    # 4. 初始化标定工作流（使用指定的关节）
    print("\n3. 初始化标定工作流...")
    workflow = CalibrationWorkflow(
        robot_urdf_path=urdf_file,
        joints_to_calibrate=joints_to_calibrate
    )
    
    # 5. 手动填充实验数据
    print("\n4. 收集实验数据...")
    n_samples = len(timestamps)
    n_joints = len(joints_to_calibrate)  # 实际要标定的关节数
    
    # 创建实验数据字典
    workflow.experiment_data["左手7自由度轨迹测试数据"] = {
        'timestamps': timestamps,
        'positions': positions,
        'velocities': velocities,
        'accelerations': accelerations,
        'torques': torques,
        'n_samples': n_samples,
        'n_joints': n_joints
    }
    
    print(f"已收集实验数据: 左手7自由度轨迹测试数据")
    print(f"数据详情:")
    print(f"  - 采样点数: {n_samples}")
    print(f"  - 关节数: {n_joints}")
    print(f"  - 时间范围: {timestamps[0]:.2f} - {timestamps[-1]:.2f}")
    print(f"  - 采样时间间隔: {dt:.3f}s, 采样频率: {1/dt:.2f}Hz")
    
    # 6. 预处理数据
    print("\n5. 预处理数据...")
    # workflow.preprocess_data("左手7自由度轨迹测试数据")  # 启用预处理
    # velocities = workflow.experiment_data["左手7自由度轨迹测试数据"]['velocities']
    # accelerations = compute_accelerations_from_velocity(velocities, dt)
    # workflow.experiment_data["左手7自由度轨迹测试数据"]['accelerations'] = accelerations
    
    # 7. 执行标定
    print("\n6. 执行动态标定...")
    # 使用合适的阈值进行分步标定
    calibration_results = workflow.perform_calibration(
        "左手7自由度轨迹测试数据",
        velocity_threshold_high=0.08,  # 增加高速阈值，更好地区分动态和摩擦参数
        velocity_threshold_low=0.05
    )
    dyn_params = calibration_results['dynamic_params']
    friction_params = calibration_results['friction_params']
    print(f"\n动态参数估计:")
    print(dyn_params)
    print(f"\n摩擦参数估计:")
    print(friction_params)
    
    # 8. 验证标定结果
    print("\n7. 验证标定结果...")
    validation_results = workflow.validate_calibration("左手7自由度轨迹测试数据")
    rmse = np.array(validation_results['rmse'])
    mae = np.array(validation_results['mae'])
    var_exp = np.array(validation_results['vaf'])
    
    # 8.1 绘制测量力矩和估计力矩对比图
    print("\n绘制测量力矩和估计力矩对比图...")
    n_joints = len(joints_to_calibrate)
    time_axis = np.arange(len(timestamps))
    
    # 获取预测力矩
    predicted_torques = workflow.calibrator.simulate_model(
        positions, 
        velocities, 
        accelerations
    )
    
    # 绘制对比图
    fig, axes = plt.subplots(n_joints, 1, figsize=(12, 3*n_joints))
    if n_joints == 1:
        axes = [axes]
    
    joint_names = ["j1", "j2", "j3", "j4", "j5", "j6", "j7"]
    
    for j in range(n_joints):
        axes[j].plot(time_axis, torques[:, j], label='mT', alpha=0.7, linewidth=1.2)
        axes[j].plot(time_axis, predicted_torques[:, j], label='calT', alpha=0.7, linewidth=1.2, linestyle='--')
        axes[j].set_title(f'joint {j+1} ({joint_names[j]}) - Torque')
        axes[j].set_xlabel('sample')
        axes[j].set_ylabel('T(Nm)')
        axes[j].grid(True, alpha=0.3)
        axes[j].legend()
    
    plt.tight_layout()
    plot_filename = "torque_comparison_validation.png"
    plt.savefig(plot_filename, dpi=150, bbox_inches='tight')
    print(f"力矩对比图已保存为 {plot_filename}")
    plt.show()

    # 9. 生成报告
    print("\n8. 生成标定报告...")
    workflow.generate_calibration_report(
        "左手7自由度轨迹测试数据", 
        calibration_results, 
        validation_results,
        "left_arm_7dof_calibration_report.json"
    )
    
    # 10. 显示结果摘要
    print("\n" + "="*60)
    print("标定结果摘要:")
    print("="*60)
    
    print(f"\n动态参数估计:")
    print(f"参数向量长度: {len(dyn_params) if dyn_params is not None else 0}")
    
    print(f"\n参数向量π的详细信息:")
    if dyn_params is not None:
        n_params_per_joint = 10  # 每个连杆10个惯性参数
        print("参数向量π的含义:")
        print("- 每个连杆j有10个连续的参数: [π[j*10], π[j*10+1], ..., π[j*10+9]]")
        print("分别对应: 质量, 质心x, 质心y, 质心z, Ixx, Ixy, Ixz, Iyy, Iyz, Izz")
        print("\n参数向量详细值:")
        for i in range(len(dyn_params)):
            joint_idx = i // n_params_per_joint  # 每个关节10个参数
            param_type = i % n_params_per_joint
            param_types = ["质量", "质心x", "质心y", "质心z", "Ixx", "Ixy", "Ixz", "Iyy", "Iyz", "Izz"]
            print(f"  π[{i}] = {dyn_params[i]:.6f} ({param_types[param_type]}, 连杆{joint_idx+1})")
    
    print(f"\n摩擦参数估计:")
    for i, params in enumerate(friction_params):
        if isinstance(params, dict):
            print(f"  关节 {i+1}: 库伦={params.get('coulomb', 0):.3f}, "
                  f"粘滞={params.get('viscous', 0):.3f}, "
                  f"静态={params.get('static', 0):.3f}")
        else:
            print(f"  关节 {i+1}: {params}")
    
    print(f"\n验证指标:")
    print(f"关节\tRMSE (Nm)\tMAE (Nm)\tVAF (%)")
    for i in range(len(rmse)):
        print(f"{i+1}\t{rmse[i]:.4f}\t\t{mae[i]:.4f}\t\t{var_exp[i]:.2f}%")
    
    print(f"\n平均性能:")
    print(f"平均RMSE: {np.mean(rmse):.4f} Nm")
    print(f"平均MAE: {np.mean(mae):.4f} Nm")
    print(f"平均VAF: {np.mean(var_exp):.2f}%")
    
    # 11. 输出回归矩阵和惯性矩阵
    print(f"\n回归矩阵信息:")
    reg_matrix = workflow.calibrator.get_regression_matrix()
    if reg_matrix is not None:
        print(f"回归矩阵形状: {reg_matrix.shape}")
        print(f"回归矩阵前几行前几列:\n{reg_matrix[:5, :5]}")  # 显示前5行5列
        print(f"\n回归矩阵Y的构造方式:")
        print(f"- Y矩阵的每一行对应一个时刻的一个关节的动力学方程")
        print(f"- Y[i*n_joints+j, k] 表示第i个样本时刻第j个关节的第k个动力学参数的系数")
        print(f"- Y矩阵的列按连杆分组，每组包含10个参数: [质量, 质心x, 质心y, 质心z, Ixx, Ixy, Ixz, Iyy, Iyz, Izz]")
        print(f"- 摩擦项独立处理，不包含在回归矩阵中")
    else:
        print("回归矩阵未计算")
    
    print(f"\n参数向量π的数值:")
    params = workflow.calibrator.get_dynamic_params()
    if params is not None:
        print(f"参数向量长度: {len(params)}")
        print(f"参数向量全部值: {params}")
    
    print(f"\n示例惯性矩阵 (在初始位置):")
    initial_pos = positions[0, :7]  # 取前7个关节位置
    inertia_matrix = workflow.calibrator.get_inertia_matrix(initial_pos)
    print(inertia_matrix)
    
    print(f"\n动力学方程:")
    print("τ = Y(θ, θ̇, θ̈) × π + τ_friction")
    print("其中:")
    print("τ - 关节扭矩向量")
    print("Y - 回归矩阵，元素为关节位置、速度、加速度的函数")
    print("π - 动态参数向量（物理参数）")
    print("τ_friction - 摩擦力矩向量")
    
    print("\n左手7自由度动力学标定完成!")
    print("详细报告已保存到 'left_arm_7dof_calibration_report.txt'")


if __name__ == "__main__":
    main()