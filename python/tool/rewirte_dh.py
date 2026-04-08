import os
import sys
# 添加arm_utils目录到路径
sys.path.append(os.path.join(os.path.dirname(__file__), '../arm_utils'))

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
import coord


def tf2tf(aim_base, now_tf):
    """
    将当前tf坐标系下的变换转换到目标坐标系下的变换
    
    Parameters:
    aim_base: 目标坐标系（相对于全局坐标系的变换）
    now_tf: 当前坐标系（相对于全局坐标系的变换）
    
    Returns:
    从目标坐标系观察的当前坐标系的变换
    """
    # 计算now_tf相对于aim_base的变换
    # aim_base^(-1) * now_tf 就是从aim_base坐标系看now_tf的变换
    aim_base_inv = np.linalg.inv(aim_base)
    out_tf = np.dot(aim_base_inv, now_tf)
    return out_tf


def plot_coordinate_systems(tf_list, labels, colors, ax, title="Coordinate Systems", base_idx=None):
    """
    在3D图中绘制多个坐标系
    """
    ax.set_title(title)
    
    origins = []
    for i, (tf, label, color) in enumerate(zip(tf_list, labels, colors)):
        # 绘制坐标轴
        coord.plot_3d_axes(tf, axis_length=0.2, ax=ax)
        
        # 记录坐标原点
        origin = tf[:3, 3]
        origins.append(origin)
        ax.text(origin[0], origin[1], origin[2], f'{label}', color=color, fontsize=10, weight='bold')
    
    # 如果指定了基准索引，则从该基准连接到其他坐标系
    if base_idx is not None and base_idx < len(origins):
        base_origin = origins[base_idx]
        for i, origin in enumerate(origins):
            if i != base_idx:  # 不画自己到自己的线
                ax.plot([base_origin[0], origin[0]], 
                        [base_origin[1], origin[1]], 
                        [base_origin[2], origin[2]], 
                        color='gray', linestyle='--', alpha=0.5)


def visualize_relative_transformations(base_tf, tf_list, labels):
    """
    可视化相对于不同基准坐标系的变换
    """
    fig = plt.figure(figsize=(15, 10))
    
    # 定义颜色
    colors = ['black', 'red', 'blue', 'green']  # 为base, tf1, tf2, tf3分配颜色
    
    # 第一个子图：所有坐标系相对于全局坐标系(base)
    ax1 = fig.add_subplot(221, projection='3d')
    all_tfs = [base_tf] + tf_list
    all_labels = ['Base'] + labels
    all_colors = ['black', 'red', 'blue', 'green']
    plot_coordinate_systems(all_tfs, all_labels, all_colors, ax1, "Original Coordinate Systems", base_idx=0)
    
    # 第二个子图：以tf1为基准
    ax2 = fig.add_subplot(222, projection='3d')
    tf1_as_base = tf_list[0]  # tf1作为新的基准
    # 计算所有坐标系相对于tf1的变换
    rel_base = tf2tf(tf1_as_base, base_tf)  # base相对于tf1
    rel_tf1 = np.eye(4)  # tf1相对于自身
    rel_tf2 = tf2tf(tf1_as_base, tf_list[1])  # tf2相对于tf1
    rel_tf3 = tf2tf(tf1_as_base, tf_list[2])  # tf3相对于tf1
    
    rel_tfs = [rel_base, rel_tf1, rel_tf2, rel_tf3]
    rel_labels = ['Rel Base', 'TF1(Base)', 'Rel TF2', 'Rel TF3']
    rel_colors = ['black', 'red', 'blue', 'green']
    plot_coordinate_systems(rel_tfs, rel_labels, rel_colors, ax2, f"Relative to {labels[0]}", base_idx=1)  # tf1为基准
    
    # 第三个子图：以tf2为基准
    ax3 = fig.add_subplot(223, projection='3d')
    tf2_as_base = tf_list[1]  # tf2作为新的基准
    # 计算所有坐标系相对于tf2的变换
    rel_base_2 = tf2tf(tf2_as_base, base_tf)  # base相对于tf2
    rel_tf1_2 = tf2tf(tf2_as_base, tf_list[0])  # tf1相对于tf2
    rel_tf2_2 = np.eye(4)  # tf2相对于自身
    rel_tf3_2 = tf2tf(tf2_as_base, tf_list[2])  # tf3相对于tf2
    
    rel_tfs_2 = [rel_base_2, rel_tf1_2, rel_tf2_2, rel_tf3_2]
    rel_labels_2 = ['Rel Base', 'Rel TF1', 'TF2(Base)', 'Rel TF3']
    rel_colors_2 = ['black', 'red', 'blue', 'green']
    plot_coordinate_systems(rel_tfs_2, rel_labels_2, rel_colors_2, ax3, f"Relative to {labels[1]}", base_idx=2)  # tf2为基准
    
    # 第四个子图：以tf3为基准
    ax4 = fig.add_subplot(224, projection='3d')
    tf3_as_base = tf_list[2]  # tf3作为新的基准
    # 计算所有坐标系相对于tf3的变换
    rel_base_3 = tf2tf(tf3_as_base, base_tf)  # base相对于tf3
    rel_tf1_3 = tf2tf(tf3_as_base, tf_list[0])  # tf1相对于tf3
    rel_tf2_3 = tf2tf(tf3_as_base, tf_list[1])  # tf2相对于tf3
    rel_tf3_3 = np.eye(4)  # tf3相对于自身
    
    rel_tfs_3 = [rel_base_3, rel_tf1_3, rel_tf2_3, rel_tf3_3]
    rel_labels_3 = ['Rel Base', 'Rel TF1', 'Rel TF2', 'TF3(Base)']
    rel_colors_3 = ['black', 'red', 'blue', 'green']
    plot_coordinate_systems(rel_tfs_3, rel_labels_3, rel_colors_3, ax4, f"Relative to {labels[2]}", base_idx=3)  # tf3为基准
    
    plt.tight_layout()
    plt.show()


def test_tf_tree():
    # 定义基础坐标系和几个其他的坐标系
    base = coord.euler_rpy_to_homogeneous_matrix(0, 0, 0, 0.0, 0.0, 0.0)
    tf1 = coord.euler_rpy_to_homogeneous_matrix(1.57, 0, 0, 1.0, 0.2, 0.1)
    tf2 = coord.euler_rpy_to_homogeneous_matrix(0, 1.57, 0, 0.1, 0.2, 0.0)
    tf3 = coord.euler_rpy_to_homogeneous_matrix(0, 0, 0, 0.0, 0.0, 1.0)
    
    # 测试tf2tf函数
    print("Original transformations:")
    print(f"Base: \n{base}")
    print(f"TF1: \n{tf1}")
    print(f"TF2: \n{tf2}")
    print(f"TF3: \n{tf3}")
    
    # 计算相对于不同基准的坐标
    tf1_rel_to_base = tf2tf(base, tf1)
    print("\nTF1 relative to Base:")
    print(tf1_rel_to_base)
    
    tf2_rel_to_tf1 = tf2tf(tf1, tf2)
    print("\nTF2 relative to TF1:")
    print(tf2_rel_to_tf1)
    
    tf3_rel_to_tf2 = tf2tf(tf2, tf3)
    print("\nTF3 relative to TF2:")
    print(tf3_rel_to_tf2)
    
    # 执行可视化
    tf_list = [tf1, tf2, tf3]
    labels = ['TF1', 'TF2', 'TF3']
    visualize_relative_transformations(base, tf_list, labels)

if __name__ == '__main__':
    test_tf_tree()