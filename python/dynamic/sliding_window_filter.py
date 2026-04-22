import pandas as pd
import numpy as np
import os
from pathlib import Path


def sliding_window_filter(data, window_size=5):
    """
    使用滑动窗口滤波处理数据
    
    参数:
    data: 输入的一维数组
    window_size: 窗口大小，默认为5
    
    返回:
    filtered_data: 滤波后的数据，开头和结尾的数据被截掉
    """
    if len(data) < window_size:
        raise ValueError(f"数据长度({len(data)})小于窗口大小({window_size})，无法进行滤波")
    
    half_window = window_size // 2
    filtered_data = np.zeros(len(data) - 2 * half_window)
    
    for i in range(half_window, len(data) - half_window):
        # 取当前位置及前后各half_window个数据点
        window_data = data[i - half_window:i + half_window + 1]
        filtered_data[i - half_window] = np.mean(window_data)
    
    return filtered_data


def process_motor_data_with_sliding_window(input_file_path, output_file_path=None):
    """
    使用滑动窗口滤波处理电机数据文件
    
    参数:
    input_file_path: 输入Excel文件的路径
    output_file_path: 输出文件路径，如果为None则自动生成
    """
    # 检查输入文件是否存在
    if not os.path.exists(input_file_path):
        print(f"错误：找不到输入文件 {input_file_path}")
        return
    
    try:
        # 读取Excel文件
        df = pd.read_excel(input_file_path)
        
        print("原始数据文件的表头信息:")
        print(df.columns.tolist())
        print(f"原始数据形状: {df.shape}")
        
        # 获取列名
        columns = df.columns.tolist()
        
        # 识别角度、速度和电流的列
        angle_cols = [col for col in columns if 'pos' in col.lower()]
        velocity_cols = [col for col in columns if 'vel' in col.lower()]
        current_cols = [col for col in columns if 'eff' in col.lower()]  # eff表示effort，即力矩/电流
        
        # 计算滤波后剩余的数据长度
        window_size = 91
        half_window = window_size // 2
        filtered_length = len(df) - 2 * half_window
        
        if filtered_length <= 0:
            print(f"错误：原始数据长度({len(df)})小于窗口大小({window_size})，无法进行滤波")
            return
        
        # 创建新的DataFrame存储滤波后的数据
        filtered_df = pd.DataFrame()
        
        # 复制时间戳或其他非滤波列
        for col in columns:
            if 'pos' not in col.lower() and 'vel' not in col.lower() and 'eff' not in col.lower():
                # 对于非滤波列，截取中间部分
                filtered_df[col] = df[col][half_window:len(df)-half_window].reset_index(drop=True)
        
        # 处理角度数据
        for col in angle_cols:
            if col in df.columns:
                filtered_data = sliding_window_filter(df[col].values, window_size)
                filtered_df[col] = filtered_data
        
        # 处理速度数据
        for col in velocity_cols:
            if col in df.columns:
                filtered_data = sliding_window_filter(df[col].values, window_size)
                filtered_df[col] = filtered_data
        
        # 处理力矩/电流数据
        for col in current_cols:
            if col in df.columns:
                filtered_data = sliding_window_filter(df[col].values, window_size)
                filtered_df[col] = filtered_data
        
        # 如果没有指定输出文件路径，则生成默认路径
        if output_file_path is None:
            input_path = Path(input_file_path)
            output_file_path = str(input_path.parent / f"{input_path.stem}_filtered{input_path.suffix}")
        
        # 保存滤波后的数据
        filtered_df.to_excel(output_file_path, index=False)
        
        print(f"滤波完成！")
        print(f"原始数据长度: {len(df)}, 滤波后数据长度: {len(filtered_df)}")
        print(f"滤波后的数据已保存至: {output_file_path}")
        
        # 显示滤波后数据的统计信息
        print("\n滤波后数据统计信息:")
        numeric_df = filtered_df.select_dtypes(include=[np.number])
        if not numeric_df.empty:
            print(numeric_df.describe())
        else:
            print("没有数值型数据可统计")
        
        return filtered_df
        
    except Exception as e:
        print(f"读取或处理文件时发生错误: {str(e)}")
        import traceback
        traceback.print_exc()


def sliding_window_multiple_signals(signals, window_size=5):
    """
    对多个信号同时进行滑动窗口滤波
    
    参数:
    signals: 输入的二维数组 [n_samples, n_signals]
    window_size: 窗口大小，默认为5
    
    返回:
    filtered_signals: 滤波后的信号，开头和结尾的数据被截掉
    """
    if len(signals) < window_size:
        raise ValueError(f"数据长度({len(signals)})小于窗口大小({window_size})，无法进行滤波")
    
    half_window = window_size // 2
    n_signals = signals.shape[1] if len(signals.shape) > 1 else 1
    
    if n_signals == 1:
        # 如果是一维信号
        filtered_data = np.zeros(len(signals) - 2 * half_window)
        for i in range(half_window, len(signals) - half_window):
            window_data = signals[i - half_window:i + half_window + 1]
            filtered_data[i - half_window] = np.mean(window_data)
        return filtered_data
    else:
        # 如果是多维信号
        filtered_data = np.zeros((len(signals) - 2 * half_window, n_signals))
        for j in range(n_signals):
            for i in range(half_window, len(signals) - half_window):
                window_data = signals[i - half_window:i + half_window + 1, j]
                filtered_data[i - half_window, j] = np.mean(window_data)
        return filtered_data


if __name__ == "__main__":
    # 默认输入路径
    input_file_path = "/home/std/arm_robot/python/dynamic/traj_test_data.xlsx"
    
    # 如果用户提供了路径参数，则使用用户提供的路径
    import sys
    if len(sys.argv) > 1:
        input_file_path = sys.argv[1]
    
    # 处理数据
    process_motor_data_with_sliding_window(input_file_path)