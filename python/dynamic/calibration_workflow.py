"""
标定工作流模块
该模块实现了完整的动态标定工作流，包括数据预处理、参数估计和结果验证
"""

import numpy as np
import os
import json
from datetime import datetime
from dynamic_calibrator import DynamicCalibrator


class CalibrationWorkflow:
    """
    动态标定工作流类
    实现了从数据加载到参数估计再到结果输出的完整流程
    """
    
    def __init__(self, robot_urdf_path, joints_to_calibrate=None):
        """
        初始化标定工作流
        
        参数:
            robot_urdf_path: 机器人URDF模型路径
            joints_to_calibrate: 要标定的关节名称列表
        """
        print("初始化标定工作流...")
        self.calibrator = DynamicCalibrator(
            urdf_path=robot_urdf_path,
            joints_to_calibrate=joints_to_calibrate
        )
        self.experiment_data = {}
        self.preprocessing_options = {
            'filter_data': True,
            'outlier_removal': True,
            'normalize_signals': False
        }
    
    def collect_experiment_data(self, experiment_name):
        """
        收集实验数据
        
        参数:
            experiment_name: 实验名称
        """
        print(f"正在收集实验数据: {experiment_name}")
        
        # 模拟加载实验数据
        # 在实际应用中，这里会从文件或数据库加载真实实验数据
        n_samples = 2000
        n_joints = len(self.calibrator.joint_indices)  # 实际要标定的关节数
        
        # 创建模拟轨迹数据
        t = np.linspace(0, 20, n_samples)  # 20秒数据
        positions = np.zeros((n_samples, n_joints))
        velocities = np.zeros((n_samples, n_joints))
        accelerations = np.zeros((n_samples, n_joints))
        
        for j in range(n_joints):
            # 每个关节使用不同的频率和幅度
            freq = 0.1 + j * 0.02  # 基频递增
            amp = 0.5 + j * 0.1    # 幅度递增
            
            # 创建复合正弦轨迹
            positions[:, j] = (
                amp * 0.7 * np.sin(freq * t) + 
                amp * 0.3 * np.sin(3 * freq * t)
            )
            velocities[:, j] = (
                amp * 0.7 * freq * np.cos(freq * t) + 
                amp * 0.3 * 3 * freq * np.cos(3 * freq * t)
            )
            accelerations[:, j] = (
                -amp * 0.7 * freq**2 * np.sin(freq * t) - 
                amp * 0.3 * 9 * freq**2 * np.sin(3 * freq * t)
            )
        
        # 添加适量噪声模拟真实数据
        noise_level_pos = 1e-3
        noise_level_vel = 1e-2
        noise_level_acc = 1e-1
        
        positions += np.random.normal(0, noise_level_pos, positions.shape)
        velocities += np.random.normal(0, noise_level_vel, velocities.shape)
        accelerations += np.random.normal(0, noise_level_acc, accelerations.shape)
        
        # 创建模拟的测量扭矩数据
        torques = np.zeros((n_samples, n_joints))
        for i in range(n_samples):
            for j in range(n_joints):
                # 简化的动力学模型: tau = M(q)*q_ddot + C(q,q_dot)*q_dot + G(q)
                # 这里使用简化的近似模型
                inertia_effect = 0.8 * accelerations[i, j]
                coriolis_effect = 0.1 * velocities[i, j] * abs(velocities[i, j])
                gravity_effect = 0.5 * np.sin(positions[i, j])
                
                torques[i, j] = inertia_effect + coriolis_effect + gravity_effect
        
        # 添加扭矩噪声
        torques += np.random.normal(0, 0.05, torques.shape)
        
        # 存储实验数据
        self.experiment_data[experiment_name] = {
            'timestamps': t,
            'positions': positions,
            'velocities': velocities,
            'accelerations': accelerations,
            'torques': torques,
            'n_samples': n_samples,
            'n_joints': n_joints
        }
        
        print(f"已收集实验数据: {experiment_name}")
        print(f"数据详情:")
        print(f"  - 采样点数: {n_samples}")
        print(f"  - 关节数: {n_joints}")
        print(f"  - 时间范围: {t[0]:.2f} - {t[-1]:.2f}")
        print(f"  - 采样时间间隔: {t[1]-t[0]:.3f}s, 采样频率: {1/(t[1]-t[0]):.2f}Hz")
    
    def preprocess_data(self, experiment_name):
        """
        预处理实验数据
        
        参数:
            experiment_name: 实验名称
        """
        print(f"正在预处理实验数据: {experiment_name}")
        
        data = self.experiment_data[experiment_name]
        
        if self.preprocessing_options['filter_data']:
            # 计算采样频率
            dt = (data['timestamps'][-1] - data['timestamps'][0]) / len(data['timestamps'])
            sampling_freq = 1.0 / dt
            nyquist_freq = sampling_freq / 2.0
            
            # 应用低通滤波器减少高频噪声
            from scipy.signal import butter, filtfilt
            # 设计巴特沃斯低通滤波器
            cutoff_freq = 0.3 * nyquist_freq  # 截止频率
            b, a = butter(4, cutoff_freq / nyquist_freq, btype='low')
            
            for i in range(data['n_joints']):
                data['positions'][:, i] = filtfilt(b, a, data['positions'][:, i])
                data['velocities'][:, i] = filtfilt(b, a, data['velocities'][:, i])
                data['accelerations'][:, i] = filtfilt(b, a, data['accelerations'][:, i])
        
        # if self.preprocessing_options['outlier_removal']:
        #     # 移除异常值
        #     for i in range(data['n_joints']):
        #         # 使用IQR方法检测异常值
        #         Q1 = np.percentile(data['torques'][:, i], 25)
        #         Q3 = np.percentile(data['torques'][:, i], 75)
        #         IQR = Q3 - Q1
        #         lower_bound = Q1 - 1.5 * IQR
        #         upper_bound = Q3 + 1.5 * IQR
                
        #         # 将异常值替换为边界值
        #         data['torques'][:, i] = np.clip(data['torques'][:, i], lower_bound, upper_bound)
        
        print(f"已预处理实验数据: {experiment_name}")
    
    def perform_calibration(self, experiment_name, velocity_threshold_high=0.5, velocity_threshold_low=0.05):
        """
        执行动态标定
        
        参数:
            experiment_name: 实验名称
            velocity_threshold_high: 高速数据阈值，用于动态参数估计（单位：rad/s）
            velocity_threshold_low: 低速数据阈值，用于摩擦参数估计（单位：rad/s）
        """
        print("正在执行动态标定...")
        
        data = self.experiment_data[experiment_name]
        
        # 加载数据到校准器
        self.calibrator.load_data(
            data['positions'],
            data['velocities'], 
            data['accelerations'],
            data['torques']
        )
        
        # 估计动态参数（使用高速数据）
        print("正在估计动态参数（使用高速数据）...")
        dynamic_params = self.calibrator.estimate_dynamic_parameters_with_velocity_filter(
            velocity_threshold=velocity_threshold_high,
            exclude_low_speed=True
        )
        
        # 估计摩擦参数（使用低速数据和残差）
        print("正在估计摩擦参数（使用低速数据和残差）...")
        friction_params = self.calibrator.estimate_friction_parameters_using_residuals(
            velocity_threshold=velocity_threshold_low
        )
        
        print("动态标定完成!")
        
        return {
            'dynamic_params': dynamic_params,
            'friction_params': friction_params
        }
    
    def validate_calibration(self, experiment_name):
        """
        验证标定结果
        
        参数:
            experiment_name: 实验名称
            
        返回:
            验证结果字典
        """
        print("正在验证标定结果...")
        
        # 评估拟合质量
        rmse, mae, var_exp = self.calibrator.evaluate_fit()
        
        validation_results = {
            'rmse': rmse.tolist(),
            'mae': mae.tolist(),
            'vaf': var_exp.tolist()
        }
        
        print("验证结果:")
        print(f"关节\tRMSE (Nm)\tMAE (Nm)\tVAF (%)")
        for i in range(len(rmse)):
            print(f"{i+1}\t{rmse[i]:.4f}\t\t{mae[i]:.4f}\t\t{var_exp[i]:.2f}%")
        
        return validation_results
    
    def generate_calibration_report(self, experiment_name, results, validation_results, output_path):
        """
        生成标定报告
        
        参数:
            experiment_name: 实验名称
            results: 标定结果
            validation_results: 验证结果
            output_path: 输出路径
        """
        print(f"正在生成标定报告...")
        
        report = {
            'timestamp': datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            'experiment_name': experiment_name,
            'robot_model': '7-DOF Left Arm',
            'n_samples': self.experiment_data[experiment_name]['n_samples'],
            'n_joints': self.experiment_data[experiment_name]['n_joints'],
            'dynamic_params': results['dynamic_params'].tolist(),
            'friction_params': [
                {
                    'coulomb': p['coulomb'],
                    'viscous': p['viscous'],
                    'static': p['static']
                } for p in results['friction_params']
            ],
            'validation_metrics': {
                'rmse': validation_results['rmse'],
                'mae': validation_results['mae'],
                'vaf': validation_results['vaf']
            },
            'regression_matrix_shape': self.calibrator.get_regression_matrix().shape if self.calibrator.get_regression_matrix() is not None else None,
            'inertia_matrix_example': self.calibrator.get_inertia_matrix(
                self.experiment_data[experiment_name]['positions'][0, :]
            ).tolist()
        }
        
        # 写入JSON格式报告
        with open(output_path, 'w', encoding='utf-8') as f:
            json.dump(report, f, indent=2, ensure_ascii=False)
        
        # 写入人类可读的文本格式报告
        with open(output_path.replace('.json', '.txt'), 'w', encoding='utf-8') as f:
            f.write("="*60 + "\n")
            f.write("左手7自由度动力学标定报告\n")
            f.write("="*60 + "\n\n")
            
            f.write(f"标定时间: {report['timestamp']}\n")
            f.write(f"实验名称: {experiment_name}\n")
            f.write(f"机器人模型: {report['robot_model']}\n")
            f.write(f"样本数量: {report['n_samples']}\n")
            f.write(f"关节数量: {report['n_joints']}\n\n")
            
            f.write("动态参数估计:\n")
            f.write(f"参数向量长度: {len(results['dynamic_params'])}\n\n")
            
            f.write("参数向量π的详细说明:\n")
            n_params_per_joint = 10  # 每个连杆10个惯性参数
            for i in range(len(results['dynamic_params'])):
                joint_idx = i // n_params_per_joint  # 每个关节10个参数
                param_type = i % n_params_per_joint
                param_types = ["质量", "质心x", "质心y", "质心z", "Ixx", "Ixy", "Ixz", "Iyy", "Iyz", "Izz"]
                f.write(f"  π[{i}] = {results['dynamic_params'][i]:.6f} ({param_types[param_type]}, 连杆{joint_idx+1})\n")
            f.write("\n")
            
            f.write("摩擦参数估计:\n")
            for i, p in enumerate(results['friction_params'], 1):
                f.write(f"  关节 {i}: 库伦={p['coulomb']:.3f}, 粘滞={p['viscous']:.3f}, 静态={p['static']:.3f}\n")
            f.write("\n")
            
            f.write("验证指标:\n")
            f.write("关节\tRMSE (Nm)\tMAE (Nm)\tVAF (%)\n")
            for i in range(len(validation_results['rmse'])):
                f.write(f"{i+1}\t{validation_results['rmse'][i]:.4f}\t\t{validation_results['mae'][i]:.4f}\t\t{validation_results['vaf'][i]:.2f}%\n")
            f.write("\n")
            
            f.write("回归矩阵信息:\n")
            if report['regression_matrix_shape']:
                f.write(f"回归矩阵形状: {report['regression_matrix_shape']}\n")
                f.write("回归矩阵Y的构造方式:\n")
                f.write("- Y矩阵的每一行对应一个时刻的一个关节的动力学方程\n")
                f.write("- Y[i*n_joints+j, k] 表示第i个样本时刻第j个关节的第k个动力学参数的系数\n")
                f.write("- Y矩阵的列按连杆分组，每组包含10个参数: [质量, 质心x, 质心y, 质心z, Ixx, Ixy, Ixz, Iyy, Iyz, Izz]\n")
                f.write("- 摩擦项独立处理，不包含在回归矩阵中\n\n")
            else:
                f.write("回归矩阵未计算\n")
            f.write("\n")
            
            f.write("示例惯性矩阵 (在初始位置):\n")
            inertia_matrix = np.array(report['inertia_matrix_example'])
            for row in inertia_matrix:
                f.write(f"  [{', '.join([f'{val:8.4f}' for val in row])}]\n")
            f.write("\n")
            
            avg_rmse = np.mean(validation_results['rmse'])
            avg_mae = np.mean(validation_results['mae'])
            avg_vaf = np.mean(validation_results['vaf'])
            
            f.write("平均性能:\n")
            f.write(f"平均RMSE: {avg_rmse:.4f} Nm\n")
            f.write(f"平均MAE: {avg_mae:.4f} Nm\n")
            f.write(f"平均VAF: {avg_vaf:.2f}%\n\n")
            
            f.write("动力学方程:\n")
            f.write("τ = Y(θ, θ̇, θ̈) × π + τ_friction\n")
            f.write("其中:\n")
            f.write("τ - 关节扭矩向量\n")
            f.write("Y - 回归矩阵，元素为关节位置、速度、加速度的函数\n")
            f.write("π - 动态参数向量（物理参数）\n")
            f.write("τ_friction - 摩擦力矩向量\n\n")
            
            f.write("="*60 + "\n")
            f.write("报告结束\n")
            f.write("="*60 + "\n")
        
        print(f"报告已保存至: {output_path}")
        print(f"文本报告已保存至: {output_path.replace('.json', '.txt')}")


def run_calibration(experiment_name, robot_urdf_path, joints_to_calibrate, output_path="calibration_report.json"):
    """
    运行完整的标定流程
    
    参数:
        experiment_name: 实验名称
        robot_urdf_path: 机器人URDF路径
        joints_to_calibrate: 要标定的关节名称列表
        output_path: 输出路径
    
    返回:
        标定结果字典
    """
    print("开始运行标定流程...")
    
    # 创建标定工作流
    workflow = CalibrationWorkflow(robot_urdf_path, joints_to_calibrate)
    
    # 收集实验数据
    workflow.collect_experiment_data(experiment_name)
    
    # 预处理数据
    workflow.preprocess_data(experiment_name)
    
    # 执行标定
    calibration_results = workflow.perform_calibration(experiment_name)
    
    # 验证结果
    validation_results = workflow.validate_calibration(experiment_name)
    
    # 生成报告
    workflow.generate_calibration_report(experiment_name, calibration_results, validation_results, output_path)
    
    # 输出回归矩阵和惯性矩阵信息
    print("\n回归矩阵信息:")
    reg_matrix = workflow.calibrator.get_regression_matrix()
    if reg_matrix is not None:
        print(f"回归矩阵形状: {reg_matrix.shape}")
        print(f"回归矩阵前几行前几列:\n{reg_matrix[:5, :5]}")  # 显示前5行5列
    
    print("\n参数向量π的详细信息:")
    params = workflow.calibrator.get_dynamic_params()
    if params is not None:
        print(f"参数向量长度: {len(params)}")
        print("参数向量π的含义:")
        print("- 每个连杆j有10个连续的参数: [π[j*10], π[j*10+1], ..., π[j*10+9]]")
        print("分别对应: 质量, 质心x, 质心y, 质心z, Ixx, Ixy, Ixz, Iyy, Iyz, Izz")
        print(f"参数向量前10个值: {params[:10]}")
    
    print("\n示例惯性矩阵 (在初始位置):")
    initial_pos = workflow.experiment_data[experiment_name]['positions'][0, :]
    inertia_matrix = workflow.calibrator.get_inertia_matrix(initial_pos)
    print(inertia_matrix)
    
    print("\n动力学方程:")
    print("τ = Y(θ, θ̇, θ̈) × π + τ_friction")
    print("其中:")
    print("τ - 关节扭矩向量")
    print("Y - 回归矩阵，元素为关节位置、速度、加速度的函数")
    print("π - 动态参数向量（物理参数）")
    print("τ_friction - 摩擦力矩向量")
    
    return {
        'workflow': workflow,
        'calibration_results': calibration_results,
        'validation_results': validation_results
    }


def main():
    """
    主函数 - 演示标定工作流
    """
    print("左手7自由度机械臂动力学标定")
    print("="*60)
    
    # 设置实验参数
    experiment_name = "左手7自由度轨迹测试数据"
    urdf_file = "v10.urdf"  # 机械臂模型文件
    output_file = "left_arm_7dof_calibration_report.txt"
    
    # 指定要标定的关节
    joints_to_calibrate = ['openarm_joint1', 'openarm_joint2', 'openarm_joint3', 
                          'openarm_joint4', 'openarm_joint5', 'openarm_joint6', 'openarm_joint7']
    
    print("\n1. 加载实验数据...")
    
    # 检查URDF文件是否存在
    if not os.path.exists(urdf_file):
        print(f"错误: URDF文件 '{urdf_file}' 不存在")
        return
    
    print("数据加载成功")
    
    print("\n2. 初始化标定工作流...")
    
    print("\n3. 收集实验数据...")
    
    print("\n4. 预处理数据...")
    
    print("\n5. 执行动态标定...")
    
    print("\n6. 验证标定结果...")
    
    # 运行完整的标定流程
    results = run_calibration(experiment_name, urdf_file, joints_to_calibrate, output_file.replace('.txt', '.json'))
    
    print("\n7. 生成标定报告...")
    
    print("\n" + "="*60)
    print("标定结果摘要:")
    print("="*60)
    
    # 输出标定参数
    dyn_params = results['calibration_results']['dynamic_params']
    friction_params = results['calibration_results']['friction_params']
    validation_results = results['validation_results']
    
    print(f"\n动态参数估计:")
    print(f"参数向量长度: {len(dyn_params)}")
    
    print(f"\n参数向量π的详细信息:")
    n_params_per_joint = 10  # 每个连杆10个惯性参数
    for i in range(min(20, len(dyn_params))):  # 只显示前20个参数
        joint_idx = i // n_params_per_joint  # 每个关节10个参数
        param_type = i % n_params_per_joint
        param_types = ["质量", "质心x", "质心y", "质心z", "Ixx", "Ixy", "Ixz", "Iyy", "Iyz", "Izz"]
        print(f"  π[{i}] = {dyn_params[i]:.6f} ({param_types[param_type]}, 连杆{joint_idx+1})")
    if len(dyn_params) > 20:
        print(f"  ... 还有{len(dyn_params)-20}个参数")
    
    print(f"\n摩擦参数估计:")
    for i, p in enumerate(friction_params, 1):
        print(f"  关节 {i}: 库伦={p['coulomb']:.3f}, 粘滞={p['viscous']:.3f}, 静态={p['static']:.3f}")
    
    print(f"\n验证指标:")
    print("关节\tRMSE (Nm)\tMAE (Nm)\tVAF (%)")
    for i in range(len(validation_results['rmse'])):
        print(f"{i+1}\t{validation_results['rmse'][i]:.4f}\t\t{validation_results['mae'][i]:.4f}\t\t{validation_results['vaf'][i]:.2f}%")
    
    avg_rmse = np.mean(validation_results['rmse'])
    avg_mae = np.mean(validation_results['mae'])
    avg_vaf = np.mean(validation_results['vaf'])
    
    print(f"\n平均性能:")
    print(f"平均RMSE: {avg_rmse:.4f} Nm")
    print(f"平均MAE: {avg_mae:.4f} Nm")
    print(f"平均VAF: {avg_vaf:.2f}%")
    
    print(f"\n左手7自由度动力学标定完成!")
    print(f"详细报告已保存到 '{output_file}'")


if __name__ == "__main__":
    main()