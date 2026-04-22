"""
机械臂动态标定的实用函数
"""

import numpy as np
from scipy import signal
import matplotlib.pyplot as plt


def generate_excitation_trajectory(joint_limits, duration, freq_range, num_joints, 
                                  sampling_freq=100, trajectory_type='multi_sine'):
    """
    为动态标定生成激励轨迹
    
    参数:
        joint_limits: 每个关节的元组列表 [(min, max), ...]
        duration: 轨迹持续时间（秒）
        freq_range: 激励频率的元组 (min_freq, max_freq)
        num_joints: 关节数量
        sampling_freq: 采样频率（Hz）
        trajectory_type: 轨迹类型 ('multi_sine', 'random', 'step')
    
    返回:
        包含位置、速度、加速度曲线的字典
    """
    n_points = int(duration * sampling_freq)
    time = np.linspace(0, duration, n_points)
    
    if trajectory_type == 'multi_sine':
        # 多正弦轨迹，频率在指定范围内
        positions = np.zeros((n_points, num_joints))
        velocities = np.zeros((n_points, num_joints))
        accelerations = np.zeros((n_points, num_joints))
        
        for j in range(num_joints):
            min_freq, max_freq = freq_range
            # 生成多个具有不同频率的正弦分量
            n_components = 5  # 正弦分量数量
            freqs = np.linspace(min_freq, max_freq, n_components)
            
            # 每个分量的随机相位和幅度
            phases = np.random.uniform(0, 2*np.pi, n_components)
            amps = np.random.uniform(0.3, 0.7, n_components)
            
            # 按关节限制缩放幅度
            joint_range = joint_limits[j][1] - joint_limits[j][0]
            scale_factor = 0.8 * joint_range / 2.0  # 使用80%的可用范围
            amps = amps * scale_factor / n_components  # 按分量数量归一化
            
            for i, (freq, amp, phase) in enumerate(zip(freqs, amps, phases)):
                # 位置
                pos_component = amp * np.sin(2*np.pi*freq*time + phase)
                positions[:, j] += pos_component
                
                # 速度（位置的导数）
                vel_component = amp * 2*np.pi*freq * np.cos(2*np.pi*freq*time + phase)
                velocities[:, j] += vel_component
                
                # 加速度（位置的二阶导数）
                acc_component = -amp * (2*np.pi*freq)**2 * np.sin(2*np.pi*freq*time + phase)
                accelerations[:, j] += acc_component
    
    elif trajectory_type == 'random':
        # 限制内的随机轨迹
        positions = np.zeros((n_points, num_joints))
        velocities = np.zeros((n_points, num_joints))
        accelerations = np.zeros((n_points, num_joints))
        
        for j in range(num_joints):
            # 生成随机平滑轨迹
            coeffs = np.random.randn(10)  # 随机多项式系数
            poly = np.poly1d(coeffs)
            
            # 将时间归一化到[0, 1]并缩放到持续时间
            t_norm = time / duration
            positions[:, j] = poly(t_norm)
            
            # 缩放到关节限制
            pos_min, pos_max = joint_limits[j]
            pos_range = pos_max - pos_min
            positions[:, j] = pos_min + (positions[:, j] - positions[:, j].min()) / \
                             (positions[:, j].max() - positions[:, j].min()) * pos_range
    
    else:
        raise ValueError(f"未知轨迹类型: {trajectory_type}")
    
    # 确保遵守关节限制
    for j in range(num_joints):
        pos_min, pos_max = joint_limits[j]
        positions[:, j] = np.clip(positions[:, j], pos_min, pos_max)
    
    return {
        'time': time,
        'positions': positions,
        'velocities': velocities,
        'accelerations': accelerations
    }


def filter_signals(signals, cutoff_freq, sampling_freq, filter_order=4, filter_type='low'):
    """
    对信号应用数字滤波器
    
    参数:
        signals: 信号数组 [n_samples, n_signals]
        cutoff_freq: 截止频率（Hz）
        sampling_freq: 采样频率（Hz）
        filter_order: 滤波器阶数
        filter_type: 滤波器类型 ('low', 'high', 'band')
    
    返回:
        滤波后的信号
    """
    nyquist_freq = sampling_freq / 2.0
    normalized_cutoff = cutoff_freq / nyquist_freq
    
    if normalized_cutoff >= 1.0:
        print(f"警告: 归一化截止频率 ({normalized_cutoff}) >= 1.0, 减少到0.95")
        normalized_cutoff = 0.95
    
    sos = signal.butter(filter_order, normalized_cutoff, filter_type, output='sos')
    
    filtered_signals = np.zeros_like(signals)
    for j in range(signals.shape[1]):
        filtered_signals[:, j] = signal.sosfilt(sos, signals[:, j])
    
    return filtered_signals


def compute_fd_criteria(regressor_matrix):
    """
    计算输入信号评估的频域(FD)准则
    
    参数:
        regressor_matrix: 来自动力学模型的回归矩阵Y
    
    返回:
        包含FD准则值的字典
    """
    # Y^T*Y的条件数
    YT_Y = regressor_matrix.T @ regressor_matrix
    cond_num = np.linalg.cond(YT_Y)
    
    # 奇异值
    singular_vals = np.linalg.svd(YT_Y, compute_uv=False)
    
    # 最小奇异值
    min_sv = np.min(singular_vals)
    
    # 最大与最小奇异值的比率
    sv_ratio = np.max(singular_vals) / (np.min(singular_vals) + 1e-12)
    
    return {
        'condition_number': cond_num,
        'min_singular_value': min_sv,
        'singular_value_ratio': sv_ratio,
        'rank': np.linalg.matrix_rank(YT_Y)
    }


def plot_trajectory_profile(trajectory_data, joint_indices=None):
    """
    绘制轨迹轮廓
    
    参数:
        trajectory_data: 包含时间、位置、速度、加速度的字典
        joint_indices: 要绘制的关节索引列表（None表示全部）
    """
    if joint_indices is None:
        joint_indices = range(trajectory_data['positions'].shape[1])
    
    n_joints = len(joint_indices)
    fig, axes = plt.subplots(3, n_joints, figsize=(15, 10))
    
    if n_joints == 1:
        axes = axes.reshape(-1, 1)
    elif n_joints == 0:
        return
    
    time = trajectory_data['time']
    
    for idx, j in enumerate(joint_indices):
        # 位置
        axes[0, idx].plot(time, trajectory_data['positions'][:, j])
        axes[0, idx].set_title(f'关节 {j} 位置')
        axes[0, idx].set_ylabel('位置 (rad)')
        axes[0, idx].grid(True)
        
        # 速度
        axes[1, idx].plot(time, trajectory_data['velocities'][:, j])
        axes[1, idx].set_title(f'关节 {j} 速度')
        axes[1, idx].set_ylabel('速度 (rad/s)')
        axes[1, idx].grid(True)
        
        # 加速度
        axes[2, idx].plot(time, trajectory_data['accelerations'][:, j])
        axes[2, idx].set_title(f'关节 {j} 加速度')
        axes[2, idx].set_ylabel('加速度 (rad/s²)')
        axes[2, idx].set_xlabel('时间 (s)')
        axes[2, idx].grid(True)
    
    plt.tight_layout()
    plt.show()


def plot_excitation_frequencies(trajectory_data, joint_indices=None, sampling_freq=100):
    """
    绘制轨迹的频率内容
    
    参数:
        trajectory_data: 包含时间、位置、速度、加速度的字典
        joint_indices: 要绘制的关节索引列表（None表示全部）
        sampling_freq: 采样频率（Hz）
    """
    if joint_indices is None:
        joint_indices = range(trajectory_data['positions'].shape[1])
    
    n_joints = len(joint_indices)
    fig, axes = plt.subplots(3, n_joints, figsize=(15, 10))
    
    if n_joints == 1:
        axes = axes.reshape(-1, 1)
    elif n_joints == 0:
        return
    
    time_interval = trajectory_data['time'][1] - trajectory_data['time'][0]
    sampling_freq = 1 / time_interval
    
    for idx, j in enumerate(joint_indices):
        # 计算每个信号的FFT
        N = len(trajectory_data['positions'][:, j])
        freqs = np.fft.fftfreq(N, d=time_interval)[:N//2]
        
        # 位置FFT
        pos_fft = np.fft.fft(trajectory_data['positions'][:, j])[:N//2]
        axes[0, idx].plot(freqs, np.abs(pos_fft))
        axes[0, idx].set_title(f'关节 {j} 位置频谱')
        axes[0, idx].set_ylabel('|X(f)|')
        axes[0, idx].grid(True)
        axes[0, idx].set_xlim([0, min(20, sampling_freq/2)])  # 限制为20Hz或奈奎斯特频率
        
        # 速度FFT
        vel_fft = np.fft.fft(trajectory_data['velocities'][:, j])[:N//2]
        axes[1, idx].plot(freqs, np.abs(vel_fft))
        axes[1, idx].set_title(f'关节 {j} 速度频谱')
        axes[1, idx].set_ylabel('|X(f)|')
        axes[1, idx].grid(True)
        axes[1, idx].set_xlim([0, min(20, sampling_freq/2)])
        
        # 加速度FFT
        acc_fft = np.fft.fft(trajectory_data['accelerations'][:, j])[:N//2]
        axes[2, idx].plot(freqs, np.abs(acc_fft))
        axes[2, idx].set_title(f'关节 {j} 加速度频谱')
        axes[2, idx].set_ylabel('|X(f)|')
        axes[2, idx].set_xlabel('频率 (Hz)')
        axes[2, idx].grid(True)
        axes[2, idx].set_xlim([0, min(20, sampling_freq/2)])
    
    plt.tight_layout()
    plt.show()


def validate_joint_limits(positions, joint_limits):
    """
    验证位置是否在关节限制内
    
    参数:
        positions: 关节位置 [n_samples, n_joints]
        joint_limits: 每个关节的元组列表 [(min, max), ...]
    
    返回:
        表示有效样本的布尔数组
    """
    n_joints = positions.shape[1]
    valid_samples = np.ones(positions.shape[0], dtype=bool)
    
    for j in range(n_joints):
        min_limit, max_limit = joint_limits[j]
        valid_for_joint = (positions[:, j] >= min_limit) & (positions[:, j] <= max_limit)
        valid_samples = valid_samples & valid_for_joint
    
    return valid_samples


def estimate_noise_ratio(signals, window_size=50):
    """
    使用局部方差估计信号中的噪声水平
    
    参数:
        signals: 信号数组 [n_samples, n_signals]
        window_size: 局部方差估计的滑动窗口大小
    
    返回:
        每个信号的噪声比估计值
    """
    n_signals = signals.shape[1]
    noise_ratios = np.zeros(n_signals)
    
    for j in range(n_signals):
        signal_data = signals[:, j]
        local_means = []
        local_vars = []
        
        for i in range(len(signal_data) - window_size):
            window_data = signal_data[i:i+window_size]
            local_mean = np.mean(window_data)
            local_var = np.var(window_data)
            
            local_means.append(local_mean)
            local_vars.append(local_var)
        
        # 平均局部方差作为噪声估计
        avg_noise_var = np.mean(local_vars)
        signal_power = np.var(signal_data)
        
        # 噪声比（信噪比，dB）
        if signal_power > 0:
            noise_ratios[j] = 10 * np.log10(avg_noise_var / signal_power)
        else:
            noise_ratios[j] = float('inf')
    
    return noise_ratios