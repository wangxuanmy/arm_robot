import subprocess
import sys
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import os


def install_openpyxl():
    """安装openpyxl库以支持Excel文件读取"""
    try:
        import openpyxl
        return True
    except ImportError:
        print("正在安装openpyxl库...")
        try:
            subprocess.check_call([sys.executable, "-m", "pip", "install", "openpyxl"])
            print("openpyxl库安装成功!")
            return True
        except subprocess.CalledProcessError:
            print("openpyxl库安装失败，请手动安装: pip install openpyxl")
            return False


def load_and_visualize_motor_data(file_path):
    """
    读取Excel文件中的电机数据并进行可视化
    
    参数:
    file_path: Excel文件的路径
    """
    # 尝试安装openpyxl
    if not install_openpyxl():
        return
    
    # 检查文件是否存在
    if not os.path.exists(file_path):
        print(f"错误：找不到文件 {file_path}")
        return
    
    try:
        # 读取Excel文件
        df = pd.read_excel(file_path)
        
        print("数据文件的表头信息:")
        print(df.columns.tolist())
        print("\n数据预览 (前5行):")
        print(df.head())
        print(f"\n数据形状: {df.shape}")
        
        # 获取列名
        columns = df.columns.tolist()
        
        # 识别角度、速度和电流的列
        angle_cols = [col for col in columns if 'pos' in col.lower()]
        velocity_cols = [col for col in columns if 'vel' in col.lower()]
        current_cols = [col for col in columns if 'eff' in col.lower()]  # eff表示effort，即力矩/电流
        
        # 使用数据索引作为X轴
        x_axis = np.arange(len(df))
        
        # 计算有数据的子图数量
        plot_count = sum([bool(angle_cols), bool(velocity_cols), bool(current_cols)])
        
        
        if plot_count > 0:
            # 如果没有同时找到位置和速度列，使用原来的逻辑
            # 创建图形
            fig, axes = plt.subplots(plot_count, 1, figsize=(14, 5*plot_count))
            
            # 确保axes始终是列表格式
            if plot_count == 1:
                axes = [axes]
            
            plot_idx = 0
            
            # 绘制角度数据
            if angle_cols:
                for col in angle_cols:
                    axes[plot_idx].plot(x_axis, df[col].values, label=col, alpha=0.7)
                axes[plot_idx].set_title('Motor Position (Angle)')
                axes[plot_idx].set_ylabel('Position (rad)')
                axes[plot_idx].set_xlabel('Sample Index')
                axes[plot_idx].legend(bbox_to_anchor=(1.05, 1), loc='upper left')
                axes[plot_idx].grid(True)
                plot_idx += 1
            
            # 绘制速度数据
            if velocity_cols:
                for col in velocity_cols:
                    axes[plot_idx].plot(x_axis, df[col].values, label=col, alpha=0.7)
                axes[plot_idx].set_title('Motor Velocity')
                axes[plot_idx].set_ylabel('Velocity (rad/s)')
                axes[plot_idx].set_xlabel('Sample Index')
                axes[plot_idx].legend(bbox_to_anchor=(1.05, 1), loc='upper left')
                axes[plot_idx].grid(True)
                plot_idx += 1
            
            # 绘制电流/力矩数据
            if current_cols:
                for col in current_cols:
                    axes[plot_idx].plot(x_axis, df[col].values, label=col, alpha=0.7)
                axes[plot_idx].set_title('Motor Torque/Current')
                axes[plot_idx].set_xlabel('Sample Index')
                axes[plot_idx].set_ylabel('Torque/Current (Nm/A)')
                axes[plot_idx].legend(bbox_to_anchor=(1.05, 1), loc='upper left')
                axes[plot_idx].grid(True)
            
            plt.tight_layout()
            plt.show()
        else:
            print("未能识别出任何电机数据列（位置、速度或力矩/电流）")
        
        # 输出一些统计信息
        print("\n数据统计信息:")
        numeric_df = df.select_dtypes(include=[np.number])
        if not numeric_df.empty:
            print(numeric_df.describe())
        else:
            print("没有数值型数据可统计")
    
    except Exception as e:
        print(f"读取或处理文件时发生错误: {str(e)}")
        import traceback
        traceback.print_exc()


def load_and_visualize_motor_data_with_derivative_speed(file_path):
    """
    读取Excel文件中的电机数据并进行可视化，包括位置、原始速度和通过对位置求导得到的速度
    
    参数:
    file_path: Excel文件的路径
    """
    # 尝试安装openpyxl
    if not install_openpyxl():
        return
    
    # 检查文件是否存在
    if not os.path.exists(file_path):
        print(f"错误：找不到文件 {file_path}")
        return
    
    try:
        # 读取Excel文件
        df = pd.read_excel(file_path)
        
        print("数据文件的表头信息:")
        print(df.columns.tolist())
        print("\n数据预览 (前5行):")
        print(df.head())
        print(f"\n数据形状: {df.shape}")
        
        # 获取列名
        columns = df.columns.tolist()
        
        # 识别角度、速度和电流的列
        angle_cols = [col for col in columns if 'pos' in col.lower()]
        velocity_cols = [col for col in columns if 'vel' in col.lower()]
        
        if not angle_cols:
            print("未找到位置数据列")
            return
            
        # 使用数据索引作为X轴
        x_axis = np.arange(len(df))
        
        # 创建三个子图：位置、原始速度、导出速度
        fig, axes = plt.subplots(3, 1, figsize=(14, 15))
        
        # 绘制角度数据
        for col in angle_cols:
            axes[0].plot(x_axis, df[col].values, label=col, alpha=0.7)
        axes[0].set_title('Motor Position (Angle)')
        axes[0].set_ylabel('Position (rad)')
        axes[0].set_xlabel('Sample Index')
        axes[0].legend(bbox_to_anchor=(1.05, 1), loc='upper left')
        axes[0].grid(True)
        
        # 绘制原始速度数据（如果存在）
        if velocity_cols:
            for col in velocity_cols:
                axes[1].plot(x_axis, df[col].values, label=f'Original {col}', alpha=0.7)
            axes[1].set_title('Original Motor Velocity')
            axes[1].set_ylabel('Velocity (rad/s)')
            axes[1].set_xlabel('Sample Index')
            axes[1].legend(bbox_to_anchor=(1.05, 1), loc='upper left')
            axes[1].grid(True)
        else:
            axes[1].text(0.5, 0.5, 'No velocity data found', horizontalalignment='center', verticalalignment='center', transform=axes[1].transAxes)
            axes[1].set_title('Original Motor Velocity (Not Available)')
        
        # 通过对位置求导来计算速度，并绘制出来
        for pos_col in angle_cols:
            # 计算位置的导数（速度）
            if len(df[pos_col]) > 1:
                # 使用numpy的diff函数计算差分，这相当于对位置求导
                derived_velocity = np.diff(df[pos_col].values)
                # 为了保持相同的长度，在数组末尾添加最后一个值
                derived_velocity = np.append(derived_velocity, derived_velocity[-1])
                
                axes[2].plot(x_axis, derived_velocity, label=f'Derived velocity from {pos_col}', alpha=0.7)
        
        axes[2].set_title('Derived Motor Velocity (from position differentiation)')
        axes[2].set_ylabel('Velocity (rad/s)')
        axes[2].set_xlabel('Sample Index')
        axes[2].legend(bbox_to_anchor=(1.05, 1), loc='upper left')
        axes[2].grid(True)
        
        plt.tight_layout()
        plt.show()
        
        # 输出一些统计信息
        print("\n数据统计信息:")
        numeric_df = df.select_dtypes(include=[np.number])
        if not numeric_df.empty:
            print(numeric_df.describe())
        else:
            print("没有数值型数据可统计")
    
    except Exception as e:
        print(f"读取或处理文件时发生错误: {str(e)}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    # 默认路径
    file_path = "/home/std/arm_robot/python/dynamic/traj_test_data.xlsx"
    
    # 如果用户提供了路径参数，则使用用户提供的路径
    if len(sys.argv) > 1:
        file_path = sys.argv[1]

    load_and_visualize_motor_data(file_path)
    
    # 调用新的函数，包括位置、原始速度和导出速度的可视化
    # load_and_visualize_motor_data_with_derivative_speed(file_path)