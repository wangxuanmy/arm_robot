import numpy as np
import pinocchio as pin
import pandas as pd
from pathlib import Path


def generate_exciting_trajectory(n_samples, n_joints, t_max, method='random_combined'):
    """
    生成激励性更强的轨迹
    
    参数:
    - n_samples: 采样点数
    - n_joints: 关节数
    - t_max: 最大时间
    - method: 生成方法 ('sine_sweep', 'random_combined', 'prbs', 'multi_frequency')
    """
    t = np.linspace(0, t_max, n_samples)
    
    if method == 'sine_sweep':
        # 扫频正弦信号，覆盖多个频率范围
        q = np.zeros((n_samples, n_joints))
        for i in range(n_joints):
            # 每个关节使用不同的频率范围
            min_freq = 0.2
            max_freq = 1.0 + 0.2 * i
            freqs = np.linspace(min_freq, max_freq, num=n_samples)
            
            for j, freq in enumerate(freqs):
                # 累积多个频率成分
                q[j, i] = 0.5 * np.sin(2 * np.pi * freq * t[j] * (j/n_samples))
                
    elif method == 'random_combined':
        # 随机信号与正弦信号结合
        q = np.zeros((n_samples, n_joints))
        for i in range(n_joints):
            # 基础正弦信号
            base_freq = 5.5 + 0.7 * i
            sine_signal = 0.6 * np.sin(base_freq * t)
            
            # 随机低频信号
            random_freq = 1.2 + 0.5 * i
            random_signal = 0.4 * np.sin(random_freq * 2 * t + np.random.uniform(0, 2*np.pi))
            
            # 组合信号
            q[:, i] = sine_signal + random_signal
    
    elif method == 'prbs':
        # 伪随机二进制序列（近似）
        q = np.zeros((n_samples, n_joints))
        for i in range(n_joints):
            # PRBS-like signal for each joint with different switching times
            switch_prob = 0.05  # Probability of switching at each time step
            current_val = np.random.choice([-1, 1]) * 0.8
            prbs_signal = np.zeros(n_samples)
            
            for j in range(n_samples):
                if np.random.rand() < switch_prob:
                    current_val = np.random.choice([-1, 1]) * 0.8
                prbs_signal[j] = current_val
                
            # Apply smoothing to reduce discontinuities
            from scipy import signal
            b, a = signal.butter(3, 0.1)
            prbs_signal = signal.filtfilt(b, a, prbs_signal)
            
            q[:, i] = prbs_signal * (0.5 + 0.1 * i)  # Scale by joint index
    
    elif method == 'multi_frequency':
        # 多频率叠加信号
        q = np.zeros((n_samples, n_joints))
        for i in range(n_joints):
            # 每个关节使用不同的频率组合
            for harmonic in range(1, 3):  # 1st to 5th harmonics
                freq = (0.5 + 0.2 * harmonic + 0.2 * i * harmonic)
                amp = 0.6 / harmonic  # Decreasing amplitude with frequency
                phase = np.random.uniform(0, 2*np.pi)  # Random phase
                q[:, i] += amp * np.sin(2 * np.pi * freq * t + phase)
    
    else:
        # 默认使用原始的正弦波合成
        q = np.zeros((n_samples, n_joints))
        for i in range(n_joints):
            # 不同关节使用不同的频率和幅度
            freq_factor = 3.0 + 0.2 * i  # 随着关节编号增加频率增加
            amp_factor = 0.8 - 0.05 * i  # 随着关节编号增加幅度减小
            q[:, i] = amp_factor * np.sin(freq_factor * 0.5 * t) + \
                      0.3 * amp_factor * np.sin(0.3 * freq_factor * t + np.pi/4 * i)
    
    # 确保关节限制在合理范围内
    q = np.clip(q, -np.pi, np.pi)
    
    return q, t


def generate_dynamics_data_for_calibration(robot_model_path=None, n_samples=2000, trajectory_method='random_combined'):
    """
    使用Pinocchio生成用于动力学标定的数据
    使用项目中的v10.urdf模型生成数据
    
    Parameters:
    - robot_model_path: URDF文件路径，如果为None则使用项目中的v10.urdf
    - n_samples: 生成样本的数量
    - trajectory_method: 轨迹生成方法 ('sine_sweep', 'random_combined', 'prbs', 'multi_frequency')
    """
    
    if robot_model_path is None:
        # 使用项目中的v10.urdf文件
        robot_model_path = Path(__file__).parent / "v10.urdf"
    
    # 从URDF文件加载模型
    model = pin.buildModelFromUrdf(str(robot_model_path))
    
    # 创建数据结构
    data = model.createData()
    nv = model.nv  # 关节自由度数
    
    print(f"URDF模型的自由度是: {nv}")
    
    n_joints = nv  # 使用实际的关节数量
    
    # 生成时间序列
    t_max = 10.0  # 总时间
    t = np.linspace(0, t_max, n_samples)
    
    # 生成激励性更强的关节位置序列
    q, t = generate_exciting_trajectory(n_samples, n_joints, t_max, method=trajectory_method)
    
    # 计算速度和加速度使用更平滑的方法
    dt = t[1] - t[0]
    
    # 使用Savitzky-Golay滤波器计算导数，以减少噪声
    from scipy.signal import savgol_filter
    window_length = min(51, n_samples - (n_samples % 2 == 0))  # 确保窗口长度为奇数
    polyorder = 3
    
    dq = np.zeros_like(q)
    ddq = np.zeros_like(q)
    
    for i in range(n_joints):
        # 平滑位置数据
        q_smooth = savgol_filter(q[:, i], window_length, polyorder)
        # 计算速度
        dq[:, i] = savgol_filter(q[:, i], window_length, polyorder, deriv=1, delta=dt)
        # 计算加速度
        ddq[:, i] = savgol_filter(q[:, i], window_length, polyorder, deriv=2, delta=dt)
    
    # 使用Pinocchio计算逆动力学（RNEA - Recursive Newton-Euler Algorithm）
    tau = np.zeros((n_samples, n_joints))
    for i in range(n_samples):
        tau[i, :] = pin.rnea(model, data, q[i], dq[i], ddq[i])
    
    # 创建时间戳（从当前时间开始）
    base_timestamp = 1776066000.0  # 示例基准时间戳
    timestamps = base_timestamp + t
    
    # 构建DataFrame，按照与7自由度相同的方式命名列
    df_dict = {'timestamp': timestamps}
    
    for i in range(n_joints):
        df_dict[f'openarm_left_joint{i+1}_pos'] = q[:, i]
        df_dict[f'openarm_left_joint{i+1}_vel'] = dq[:, i]
        df_dict[f'openarm_left_joint{i+1}_eff'] = tau[:, i]
    
    # 如果少于7个关节，用0填充多余的列
    for i in range(n_joints, 7):
        df_dict[f'openarm_left_joint{i+1}_pos'] = np.zeros(n_samples)
        df_dict[f'openarm_left_joint{i+1}_vel'] = np.zeros(n_samples)
        df_dict[f'openarm_left_joint{i+1}_eff'] = np.zeros(n_samples)
    
    df = pd.DataFrame(df_dict)
    
    print(f"生成的数据形状: {df.shape}")
    print(f"时间范围: {t[0]:.2f}s - {t[-1]:.2f}s")
    print(f"采样点数: {n_samples}")
    print(f"实际关节数量: {n_joints}")
    print(f"轨迹生成方法: {trajectory_method}")
    
    # 显示前几行数据
    print("\n前5行数据:")
    print(df.head())
    
    return df, model, data


def save_generated_data(df, output_path='generated_dynamics_data.xlsx'):
    """保存生成的数据到Excel文件"""
    df.to_excel(output_path, index=False)
    print(f"\n数据已保存到: {output_path}")


if __name__ == "__main__":
    # 生成数据
    df, model, data = generate_dynamics_data_for_calibration(
        robot_model_path=None,
        n_samples=4000,
        trajectory_method=''  # 可以改为 'multi_frequency', 'prbs' 或 'sine_sweep'
    )
    
    # 保存数据
    output_path = Path(__file__).parent / "generated_dynamics_data.xlsx"
    save_generated_data(df, output_path)
    
    # 打印一些统计信息
    print("\n数据统计:")
    print(f"位置范围: {df.filter(regex='_pos$').min().min():.3f} ~ {df.filter(regex='_pos$').max().max():.3f}")
    print(f"速度范围: {df.filter(regex='_vel$').min().min():.3f} ~ {df.filter(regex='_vel$').max().max():.3f}")
    print(f"力矩范围: {df.filter(regex='_eff$').min().min():.3f} ~ {df.filter(regex='_eff$').max().max():.3f}")
    
    # 输出数据激励特性
    print(f"\n数据激励特性:")
    for i in range(min(7, model.nv)):  # 最多显示7个关节
        pos_data = df[f'openarm_left_joint{i+1}_pos']
        vel_data = df[f'openarm_left_joint{i+1}_vel']
        eff_data = df[f'openarm_left_joint{i+1}_eff']
        
        pos_var = np.var(pos_data)
        vel_var = np.var(vel_data)
        eff_var = np.var(eff_data)
        
        print(f"关节{i+1}: 位置方差={pos_var:.3f}, 速度方差={vel_var:.3f}, 力矩方差={eff_var:.3f}")