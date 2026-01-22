import sys
import os
import traceback

# ========== 核心修改：用脚本自身路径计算cpp目录的绝对路径 ==========
# 获取当前脚本（visualize_cpp_backend.py）的绝对路径
script_dir = os.path.abspath(os.path.dirname(__file__))
# 计算cpp目录的绝对路径（无论在哪运行，路径都正确）
cpp_dir = os.path.abspath(os.path.join(script_dir, "../cpp"))
# 添加到Python路径（优先添加绝对路径）
sys.path.insert(0, cpp_dir)

# ========== 增强版导入：带详细错误日志 ==========
try:
    import arm_robot_cpp as arm_robot
except ImportError as e:
    print("="*50)
    print("C++模块导入失败，错误详情：")
    print(f"1. 脚本所在目录：{script_dir}")
    print(f"2. 期望的C++模块目录：{cpp_dir}")
    print(f"3. 该目录是否存在：{os.path.exists(cpp_dir)}")
    # 列出cpp目录下的文件（排查.so是否存在）
    if os.path.exists(cpp_dir):
        print(f"4. C++目录下的文件：{os.listdir(cpp_dir)}")
    else:
        print("4. C++目录不存在！")
    print(f"5. ImportError原始信息：{str(e)}")
    print("="*50)
    print("解决步骤：")
    print("1. 检查cpp目录是否存在：ls", cpp_dir)
    print("2. 重新编译模块：cd", cpp_dir, "&& make clean && make pybind")
    print("3. 确认模块名是否为arm_robot_cpp.so（不是arm_robot.so）")
    print("="*50)
    exit(1)
except Exception as e:
    # 捕获其他异常（比如.so加载失败、依赖库缺失）
    print("C++模块加载异常（非导入找不到）：")
    traceback.print_exc()
    exit(1)

# 验证导入成功（可选）
print(f"✅ C++模块导入成功！模块路径：{arm_robot.__file__}")

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.widgets import Slider
from mpl_toolkits.mplot3d import Axes3D
import math


class HumanVisualizer:
    def __init__(self, body_dh, left_hand_dh, right_hand_dh, head_dh, theta_limit=None):
        # 创建C++ Human对象
        if theta_limit is None:
            self.human_cpp = arm_robot.Human(body_dh, left_hand_dh, right_hand_dh, head_dh)
        else:
            self.human_cpp = arm_robot.Human(body_dh, left_hand_dh, right_hand_dh, head_dh, theta_limit)


        # 创建3D图形
        self.fig = plt.figure('Arm Robot Visualization with C++ Backend')
        self.ax = self.fig.add_subplot(111, projection="3d")

        self.show_len = 2.6
        self.axis_len = 0.05

        self.body_num_joints = len(body_dh)
        self.left_hand_num_joints = len(left_hand_dh)
        self.right_hand_num_joints = len(right_hand_dh)
        self.head_num_joints = len(head_dh)
        
        # 当前关节角度
        self.current_theta = [0.0] * len(self.human_cpp.get_all_theta())

    def draw_coord(self, mat,ax, ls='--'):
        """绘制坐标系"""
        # 提取旋转矩阵（3x3）
        rotation_matrix = mat[:3, :3]

        # 提取平移向量（3x1）
        translation_vector = mat[:3, 3]

        axis_length = 0.03

        # 定义原始坐标轴的起点
        origin = np.array([0, 0, 0])

        # 定义原始坐标轴的方向向量
        x_axis_original = np.array([axis_length, 0, 0])
        y_axis_original = np.array([0, axis_length, 0])
        z_axis_original = np.array([0, 0, axis_length])

        # 应用旋转矩阵到原始坐标轴
        x_axis_transformed = rotation_matrix @ x_axis_original
        y_axis_transformed = rotation_matrix @ y_axis_original
        z_axis_transformed = rotation_matrix @ z_axis_original

        # 绘制平移后的坐标轴
        ax.quiver(
            translation_vector[0],
            translation_vector[1],
            translation_vector[2],
            x_axis_transformed[0],
            x_axis_transformed[1],
            x_axis_transformed[2],
            color="blue",
            linestyle=ls,
            label="X-axis",
        )

        ax.quiver(
            translation_vector[0],
            translation_vector[1],
            translation_vector[2],
            y_axis_transformed[0],
            y_axis_transformed[1],
            y_axis_transformed[2],
            color="green",
            linestyle=ls,
            label="Y-axis",
        )

        ax.quiver(
            translation_vector[0],
            translation_vector[1],
            translation_vector[2],
            z_axis_transformed[0],
            z_axis_transformed[1],
            z_axis_transformed[2],
            color="red",
            linestyle=ls,
            label="Z-axis",
        )

        # 设置坐标轴范围
        ax.set_xlim([-axis_length, axis_length])
        ax.set_ylim([-axis_length, axis_length])
        ax.set_zlim([-axis_length, axis_length])

        # 设置坐标轴标签
        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.set_zlabel("Z")

    def draw(self, base, tf_tree):
        """绘制机械臂连杆结构"""
        base_origin = []
        tf = np.array(base)  # Ensure it's a 4x4 matrix
        base_origin.append(tf[:3, 3].copy())

        self.draw_coord(tf, self.ax)
        for i in tf_tree:
            tf = i
            base_origin.append(tf[:3, 3].copy())
            self.draw_coord(tf, self.ax)
        # 绘制连杆之间的连接线
        for i in range(len(base_origin) - 1):
            x = [base_origin[i][0], base_origin[i + 1][0]]
            y = [base_origin[i][1], base_origin[i + 1][1]]
            z = [base_origin[i][2], base_origin[i + 1][2]]
            self.ax.plot3D(x, y, z, 'k-', alpha=0.5)

        return tf

    def show_body(self):
        """显示身体结构及控制滑动条"""
        # 设置滑动条的位置
        ax_theta = []
        num_joints = len(self.human_cpp.get_all_theta())

        for i in range(num_joints):
            ax_theta.append(
                plt.axes(
                    [
                        0.1,
                        0.95 - 0.03 * i,
                        0.3,
                        0.04,
                    ]
                )
            )

        # 创建滑动条
        self.s_theta = []
        all_theta_values = self.human_cpp.get_all_theta()
        
        # 获取theta限制，如果C++端没有提供此方法则使用默认值
        try:
            theta_limits = self.human_cpp.get_theta_limit()
            has_theta_limits = True
        except AttributeError:
            has_theta_limits = False

        for i, ax_the in enumerate(ax_theta):
            if has_theta_limits and i < len(theta_limits):
                theta_min = float(theta_limits[i][0])
                theta_max = float(theta_limits[i][1])
            else:
                theta_min = -np.pi
                theta_max = np.pi
                
            init_theta = all_theta_values[i] if i < len(all_theta_values) else 0.0
            
            self.s_theta.append(
                Slider(
                    ax_the,
                    f"Theta{i+1}",
                    theta_min,
                    theta_max,
                    valinit=init_theta,
                )
            )

        # 连接滑动条的更新函数
        for i in self.s_theta:
            i.on_changed(self.update)

        # 重新绘制坐标轴和连杆
        self.update(None)

    def show_control(self):
        """显示控制界面，允许用户控制末端执行器的目标位置"""
        # 设置滑动条的位置 (12个参数: 6个用于左臂，6个用于右臂)
        ax_theta = []

        for i in range(12):
            ax_theta.append(
                plt.axes(
                    [
                        0.1,
                        0.95 - 0.03 * i,
                        0.3,
                        0.04,
                    ]
                )
            )

        # 创建滑动条
        self.s_theta = []
        # 默认值: 前6个参数用于左臂，后6个用于右臂
        for i, ax_the in enumerate(ax_theta):
            self.s_theta.append(
                Slider(
                    ax_the,
                    f"Param{i+1}",
                    -3.14,  # 角度范围 -π 到 π
                    3.14,
                    valinit=0.0,
                )
            )

        # 连接滑动条的更新函数
        for i in self.s_theta:
            i.on_changed(self.update_control)

        # 初始化一次绘制
        self.update_control(None)

    def update_control(self, val):
        """根据滑动条更新目标位置和机器人姿态"""
        mat_value = [0] * 12

        # 获取滑动条的当前值
        for i in range(12):
            mat_value[i] = self.s_theta[i].val

        #------------------------------------------------------
        # 左臂目标矩阵 (前6个参数)
        # 使用与Python版本类似的转换方式
        aim_mat = self.create_homogeneous_matrix(
            mat_value[0], 
            1.57 + mat_value[1], 
            1.57 + mat_value[2], 
            mat_value[3], 
            0.179 + mat_value[4], 
            0.547 + mat_value[5]
        )
        
        q0_body = [0.0] * self.body_num_joints
        all_theta = self.human_cpp.get_all_theta()
        q0_hand = all_theta[self.body_num_joints:self.body_num_joints+self.left_hand_num_joints]
        # 调用C++端的controlHand方法控制左臂
        result = self.human_cpp.control_hand(aim_mat, "left", q0 = q0_body + q0_hand)
        print(result[0])
        if result[0]:  # 如果求解成功
            theta_end = result[1][-1]  # 获取最终角度
            all_theta = self.human_cpp.get_all_theta()
            all_theta[:len(theta_end)] = theta_end
            # 更新身体和左手角度
            self.human_cpp.flash_theta(all_theta)
            print("left hand:", theta_end)

        #------------------------------------------------------
        # 右臂目标矩阵 (后6个参数)
        aim_mat_right = self.create_homogeneous_matrix(
            mat_value[6], 
            mat_value[7], 
            mat_value[8], 
            mat_value[9], 
            mat_value[10], 
            mat_value[11]
        )
        
        # 调用C++端的controlPart方法控制右臂
        result = self.human_cpp.control_part(aim_mat_right, "right")
        if result[0]:  # 如果求解成功
            theta_end = result[1][-1]  # 获取最终角度
            # 更新右臂角度
            current_theta = self.human_cpp.get_all_theta()
            # 只更新右臂部分的角度
            right_start_idx = self.body_num_joints + self.left_hand_num_joints
            right_end_idx = right_start_idx + self.right_hand_num_joints
            current_theta[right_start_idx:right_end_idx] = theta_end
            self.human_cpp.flash_theta(current_theta)

        #----------------------------------------------------


        def draw_other():
            # 绘制目标位置
            self.draw_coord(aim_mat, self.ax)
            # 计算并绘制右臂目标位置在世界坐标系中的实际位置
            self.draw_coord(aim_mat_right, self.ax)

        self.show(draw_other)
    
    def show(self, others = None):
        # 保存当前视图限制和视角
        try:
            xlim = self.ax.get_xlim()
            ylim = self.ax.get_ylim()
            zlim = self.ax.get_zlim()
            elev, azim = self.ax.elev, self.ax.azim  # 保存仰角和方位角
        except:
            # 如果无法获取当前视图参数，使用默认值
            xlim = [-self.show_len/2, self.show_len/2]
            ylim = [-self.show_len/2, self.show_len/2]
            zlim = [0, self.show_len]
            elev, azim = 30, 45

        # 清除之前的绘图
        self.ax.clear()
        
        # 获取所有变换
        all_tf = self.human_cpp.get_all_tf()
        
        # 使用预先计算的关节数
        body_num_joints = self.body_num_joints
        left_hand_num_joints = self.left_hand_num_joints
        right_hand_num_joints = self.right_hand_num_joints
        head_num_joint = self.head_num_joints
        
        # 分别获取各部分的变换
        body_tf = all_tf[:body_num_joints+1]
        left_hand_tf = all_tf[body_num_joints+1:body_num_joints+left_hand_num_joints+2]
        right_hand_tf = all_tf[body_num_joints+left_hand_num_joints+2:body_num_joints+left_hand_num_joints+right_hand_num_joints+3]
        head_tf = all_tf[-head_num_joint-1:]

        # 绘制身体
        body_base = self.human_cpp.get_part_base("body")
        self.draw(body_base, body_tf[1:])
        
        # 绘制左手
        left_base = self.human_cpp.get_part_base("left")
        self.draw(left_base, left_hand_tf)
        
        # 绘制右手
        right_base = self.human_cpp.get_part_base("right")
        self.draw(right_base, right_hand_tf)

        # 绘制头
        head_base = self.human_cpp.get_part_base("head")
        self.draw(head_base, head_tf)

        if others != None:
            others()

        # 设置坐标轴范围
        self.ax.set_xlim([-self.show_len/2, self.show_len/2])
        self.ax.set_ylim([-self.show_len/2, self.show_len/2])
        self.ax.set_zlim([0, self.show_len])
        
        # 设置标签
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('Z')

        # 恢复视图限制和视角
        try:
            self.ax.set_xlim(xlim)
            self.ax.set_ylim(ylim)
            self.ax.set_zlim(zlim)
            self.ax.view_init(elev=elev, azim=azim)  # 恢复视角
        except:
            pass

        # 更新图形
        plt.draw()


    def create_homogeneous_matrix(self, rx, ry, rz, x, y, z):
        """从欧拉角和平移向量创建齐次变换矩阵"""
        # 先创建旋转矩阵 (假设使用ZYX顺序)
        cx, cy, cz = np.cos(rx), np.cos(ry), np.cos(rz)
        sx, sy, sz = np.sin(rx), np.sin(ry), np.sin(rz)
        
        # ZYX旋转矩阵
        rotation_matrix = np.array([
            [cy * cz, -cy * sz, sy],
            [sx * sy * cz + cx * sz, -sx * sy * sz + cx * cz, -sx * cy],
            [-cx * sy * cz + sx * sz, cx * sy * sz + sx * cz, cx * cy]
        ])
        
        # 构建齐次变换矩阵
        homog_matrix = np.eye(4)
        homog_matrix[:3, :3] = rotation_matrix
        homog_matrix[:3, 3] = [x, y, z]
        
        return homog_matrix

    def update(self, val):
        """更新关节角度和可视化"""
        # 获取滑动条的当前值
        theta_values = []
        for i in range(len(self.s_theta)):
            theta_values.append(self.s_theta[i].val)

        # 更新C++ Human对象中的theta值
        self.human_cpp.flash_theta(theta_values)

        self.show()


    def pad_or_truncate_theta(self, theta_list, target_size):
        """将theta列表填充或截断到目标大小"""
        result = theta_list[:]
        if len(result) < target_size:
            result.extend([0.0] * (target_size - len(result)))
        elif len(result) > target_size:
            result = result[:target_size]
        return result

    def update_visualization(self):
        """更新可视化显示"""
        self.ax.clear()
        self.show_body()
        plt.draw()


def main():
    # 创建人体模型参数
    body_dh = [[-1.5707, 0, 0, -1.5707, 0], 
               [-3.1415, 0.41, 0.0, 0, 0],
               [0, 0.48, 0, 1.5707, 0],
               [1.5707, 0, 0.35, 3.1415, 0]]

    left_hand_dh = [
        [-1.5707, 0.0, 0.253, 1.5707, 0],
        [1.5707, 0.0, 0.0, -1.5707, 0],
        [-1.5707, 0.0, 0.3579, 3.1415, 0],
        [-1.5707, -0.0537, 0.0, 3.1415, 0],
        [-1.5707, -0.036, 0.2041, 1.5707, 0],
        [-1.5707, 0.0, 0.0, -1.5707, 0],
        [1.5707, 0.08, 0.0, 1.5707, 0]]

    right_hand_dh = [
        [1.5707, 0.0, 0.253, -1.5707, 0],
        [-1.5707, 0.0, 0.0, -1.5707, 0],
        [-1.5707, 0.0, 0.3579, 3.1415, 0],
        [-1.5707, 0.0537, 0.0, 3.1415, 0],
        [-1.5707, 0.036, 0.2041, 1.5707, 0],
        [1.5707, 0.0, 0.0, 1.5707, 0],
        [-1.5707, 0.08, 0.0, 1.5707, 0]]
    
    head_dh = [
        [0.0, 0.0, 0.2, 0.0, 0],
        [1.57, 0.0, 0.0, 0.0, 0],
    ]
    
    # 创建可视化器
    visualizer = HumanVisualizer(body_dh, left_hand_dh, right_hand_dh, head_dh)
    
    # 选择要显示的模式
    import sys
    if len(sys.argv) > 1 and sys.argv[1] == "control":
        visualizer.show_control()
    else:
        visualizer.show_body()
    
    plt.show()


if __name__ == '__main__':
    main()