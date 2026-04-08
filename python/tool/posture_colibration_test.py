import numpy as np
from scipy.optimize import least_squares
import matplotlib.pyplot as plt
import read
from quaternion_utils import matrix_to_quaternion, quaternion_error
import coord 

N_JOINTS = 7
V_JOINTS_F = 2 #虚拟关节,用于统一坐标系
V_JOINTS_E = 2

def plot_3d_axes(matrix, axis_length, ax, ls="-"):
    # 提取旋转矩阵（3x3）
    rotation_matrix = matrix[:3, :3]

    # 提取平移向量（3x1）
    translation_vector = matrix[:3, 3]

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
        label=None,
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
        label=None,
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
        label=None,
    )

    # 设置坐标轴范围
    # ax.set_xlim([-axis_length, axis_length])
    # ax.set_ylim([-axis_length, axis_length])
    # ax.set_zlim([-axis_length, axis_length])

    # 设置坐标轴标签
    # ax.set_xlabel("X")
    # ax.set_ylabel("Y")
    # ax.set_zlabel("Z")

    # 显示图例
    # ax.legend()

    # 显示图形
    # plt.show()
def homogeneous_matrix(theta, params):
    """
    生成单个齐次变换矩阵
    theta: 输入角度(7维向量中的一个元素)
    params: 4个矩阵参数 [alpha, a, d, theta_offset]
    """
    alpha, a, d, theta_offset = params
    # 计算实际使用的角度（包含偏移量）
    actual_theta = theta + theta_offset

    matrix = np.array(
        [
            [
                np.cos(actual_theta),
                -np.sin(actual_theta),
                0,
                a,
            ],
            [
                np.cos(alpha) * np.sin(actual_theta),
                np.cos(actual_theta) * np.cos(alpha),
                -np.sin(alpha),
                -d * np.sin(alpha),
            ],
            [
                np.sin(alpha) * np.sin(actual_theta),
                np.sin(alpha) * np.cos(actual_theta),
                np.cos(alpha),
                d * np.cos(alpha),
            ],
            [0, 0, 0, 1],
        ])
    
    return matrix

def forward_kinematics(thetas, params_vector):
    """
    前向运动学计算：7个齐次矩阵相乘
    thetas: 7维输入向量
    params_vector: 28个参数的一维向量 (7个矩阵×4个参数)
    """
    # 重塑参数向量为7x4的矩阵
    params = params_vector.reshape(N_JOINTS + V_JOINTS_F + V_JOINTS_E, 4)
    
    # 初始化齐次矩阵为单位矩阵
    transform = np.eye(4)


    # 依次乘以7个齐次矩阵
    for i in range(N_JOINTS + V_JOINTS_F + V_JOINTS_E):
        transform =  homogeneous_matrix(thetas[-(i+1)], params[-(i+1)]) @ transform
    
    return transform

def objective_function(params_vector, thetas_list, target_positions):
    """
    目标函数：计算预测位置与目标位置的误差
    """
    errors = []
    for thetas, target_pos in zip(thetas_list, target_positions):
        pred_pose = forward_kinematics(thetas, params_vector)

        err_pos = np.linalg.norm(pred_pose[:3,3] - target_pos[:3,3])

        # aim_quaternion = matrix_to_quaternion(target_pos[:3,:3])
        # now_quaternion = matrix_to_quaternion(pred_pose[:3,:3])
        # d_rotation = quaternion_error(now_quaternion, aim_quaternion)
        # d_rotation = 0.5 * d_rotation
        # err_theta = np.linalg.norm(d_rotation)
        err_theta = 0

        errors.append(err_theta + err_pos)
    
    # 展平误差数组
    return np.array(errors).flatten()

def solve_parameters(thetas_list, target_positions, initial_guess=None):
    """
    求解28个矩阵参数
    thetas_list: N组7维输入向量
    target_positions: N组目标位置(x,y,z)
    initial_guess: 参数的初始猜测值，若未提供则使用默认值
    """
    # 转换为numpy数组
    thetas_array = np.array(thetas_list)
    target_positions_array = np.array(target_positions)
    
    # 确保输入数据格式正确
    assert thetas_array.shape[1] == N_JOINTS + V_JOINTS_F + V_JOINTS_E, "输入应为7维向量"
    assert target_positions_array.shape[1] == 4, "目标位置应为齐次矩阵"
    
    # 设置初始猜测值
    if initial_guess is None:
        # 初始化为小的随机值或零
        initial_guess = np.zeros((N_JOINTS + V_JOINTS_F + V_JOINTS_E)*4)

    # 为28个参数设置约束
    # lower_bounds = np.array([
    #     # 每个关节的4个参数：[alpha, a, d, theta_offset]
    #     -np.inf, -np.inf, -np.inf, -np.inf,  # 关节0的约束
    #     -np.inf, -np.inf, -np.inf, -np.inf,  # 关节0的约束


    #     -np.pi, -0.0001, -0.0001, -0.0001,  # 关节1的约束
    #     -np.pi, -0.5, -0.5, -np.pi,  # 关节2的约束
    #     -np.pi, -0.5, -0.5, -np.pi,  # 关节3的约束
    #     -np.pi, -0.5, -0.5, -np.pi,  # 关节4的约束
    #     -np.pi, -0.5, -0.5, -np.pi,  # 关节5的约束
    #     -np.pi, -0.5, -0.5, -np.pi,  # 关节6的约束
    #     -np.pi, -0.5, -0.5, -np.pi   # 关节7的约束
    # ])

    # lower_bounds = initial_guess - 0.0001
    # upper_bounds = initial_guess

    lower_bounds = initial_guess -0.1
    upper_bounds = initial_guess +0.1

    lower_bounds[:2] = -3.14159
    upper_bounds[:2] = 3.14159

    lower_bounds[-2:] = -3.14159
    upper_bounds[-2:] = 3.14159

    # print(lower_bounds)

    # upper_bounds = np.array([
    #     np.inf, np.inf, np.inf, np.inf,  # 关节0的约束
    #     np.inf, np.inf, np.inf, np.inf,  # 关节0的约束

    #     np.pi, 0.0001, 0.0001, 0.0001,    # 关节1的约束
    #     np.pi, 0.5, 0.5, np.pi,    # 关节2的约束
    #     np.pi, 0.5, 0.5, np.pi,    # 关节3的约束
    #     np.pi, 0.5, 0.5, np.pi,    # 关节4的约束
    #     np.pi, 0.5, 0.5, np.pi,    # 关节5的约束
    #     np.pi, 0.5, 0.5, np.pi,    # 关节6的约束
    #     np.pi, 0.5, 0.5, np.pi,    # 关节7的约束
    # ])

    print("开始求解")
    
    # 使用最小二乘法求解
    result = least_squares(
        objective_function, 
        initial_guess, 
        args=(thetas_list, target_positions),
        method='lm',  # Trust Region Reflective算法，适合约束优化
        # bounds=(lower_bounds, upper_bounds),  # 添加参数约束
        ftol=0.5e-15,     # 函数收敛容差
        xtol=0.5e-15,     # 参数收敛容差
        gtol=0.5e-15,
        max_nfev=5000  # 最大函数评估次数
    )
    
    return result.x, result

def visualize_results(params, thetas_list, target_positions, num_samples=20):
    """可视化求解结果"""
    # 随机选择一些样本进行可视化
    indices = np.random.choice(len(thetas_list), num_samples, replace=False)
    
    # 3D位置误差
    fig = plt.figure(figsize=(12, 10))
    ax = fig.add_subplot(111, projection='3d')

    # 存储所有点的信息 (类型, 索引, 坐标)
    points_info = []
    
    # 分别存储目标点和预测点的艺术家对象
    target_artists = []
    pred_artists = []

    

    
    for i in indices:
        pred_pos = forward_kinematics(thetas_list[i], params)
        plot_3d_axes(pred_pos, axis_length=0.02, ax=ax, ls="-")
        pred_pos = pred_pos[:3,3]
        target_pos = target_positions[i][:3,3]
        plot_3d_axes(target_positions[i], axis_length=0.02, ax=ax, ls="--")

        
        # 存储点信息
        points_info.append(('target', i, target_pos))
        points_info.append(('pred', i, pred_pos))
        
        # 绘制目标点和预测点，并保存艺术家对象
        target_artist = ax.scatter(target_pos[0], target_pos[1], target_pos[2], 
                                  color='blue', label='Target' if i==indices[0] else None, picker=5)
        pred_artist = ax.scatter(pred_pos[0], pred_pos[1], pred_pos[2], 
                                color='red', label='Prediction' if i==indices[0] else None, picker=5)
        ax.plot([target_pos[0], pred_pos[0]], [target_pos[1], pred_pos[1]], [target_pos[2], pred_pos[2]], 'k--', alpha=0.3)
        
        target_artists.append(target_artist)
        pred_artists.append(pred_artist)

    plot_3d_axes(np.array([[1.0,0.0,0.0,0.0],
                           [0.0,1.0,0.0,0.0],
                           [0.0,0.0,1.0,0.0],
                           [0.0,0.0,0.0,1.0]]), axis_length=0.5, ax=ax, ls="-")
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('Target vs Predicted Positions (Click on points for details)')
    ax.legend()
    
    # 位置误差分布
    plt.figure(figsize=(12, 5))
    errors = []
    for thetas, target_pos in zip(thetas_list, target_positions):
        pred_pos = forward_kinematics(thetas, params)
        pred_pos = pred_pos[:3,3]
        errors.append(np.linalg.norm(pred_pos - target_pos[:3,3]))
    print(f'mean:{np.mean(errors)} std:{np.std(errors)}')

    plt.hist(errors, bins=20, alpha=0.7)
    plt.xlabel('Position Error (m)')
    plt.ylabel('Frequency')
    plt.title('Distribution of Position Errors')
    
    # 点选交互函数
    def on_pick(event):
        # 获取点击的艺术家对象
        artist = event.artist
        
        # 判断是目标点还是预测点
        if artist in target_artists:
            point_type = 'target'
        elif artist in pred_artists:
            point_type = 'pred'
        else:
            return
        
        # 获取该艺术家对象在列表中的索引
        artist_idx = target_artists.index(artist) if point_type == 'target' else pred_artists.index(artist)
        
        # 获取被点击的点在艺术家对象中的索引 (通常为0，因为每个scatter只绘制一个点)
        ind = event.ind[0]
        
        # 计算在points_info中的实际索引
        base_idx = artist_idx * 2  # 每个艺术家对应两个点 (target和pred)
        actual_idx = base_idx if point_type == 'target' else base_idx + 1
        
        # 获取点的信息
        point_info = points_info[actual_idx]
        point_index = point_info[1]
        point_coord = point_info[2]
        
        # 显示信息
        ax.set_title(f'Point #{point_index} - {point_type.capitalize()}: '
                    f'({point_coord[0]:.4f}, {point_coord[1]:.4f}, {point_coord[2]:.4f})')
        fig.canvas.draw_idle()
    
    # 连接点击事件
    fig.canvas.mpl_connect('pick_event', on_pick)
    
    plt.show()

# 示例用法
if __name__ == "__main__":
    # 生成模拟数据
    np.random.seed(66)
    
    # 真实参数 (28个未知数)
    # true_params = np.random.uniform(-0.1, 0.1, size=N_JOINTS*4)

    true_params = np.array([
        0,0,0,0,
        0,0,0,0,
            0.0, 0.0, 0.1746, 1.5707,
            1.5707, 0.0, 0.0, 0,
            -1.5707, 0.0, 0.287, 0,
            1.5707, 0.018, 0.0, 3.14159,
            1.5707, 0.018, 0.314, 3.14159,
            1.5707, 0.0, 0.0, 1.57,
            1.5707, 0.0, 0.0, 1.57,
                        0,0,0,0,
                        0,0,0,0,
                        ])
    
    # 生成N组输入数据
    # N = 50  # 数据样本数
    # thetas_list = np.random.uniform(-np.pi, np.pi, size=(N, N_JOINTS))
    # # 添加一列虚拟关节角度 0 
    # if  V_JOINTS_F > 0:
    #     thetas_list = np.hstack((np.zeros((N, V_JOINTS_F)), thetas_list))
    # if V_JOINTS_E > 0:
    #     thetas_list = np.hstack((thetas_list, np.zeros((N, V_JOINTS_E))))
    
    # 计算目标位置（添加一些噪声）
    target_positions = []
    target_positions_with_noise = []
    # for thetas in thetas_list:
    #     true_pos = forward_kinematics(thetas, true_params)
    #     # 添加随机噪声
    #     noise = np.random.normal(0, 0.0003, 3)
    #     target_positions.append(true_pos.copy())

    #     true_pos[:3,3] = true_pos[:3,3] + noise
    #     target_positions_with_noise.append(true_pos.copy())

    

    df = read.read_excel("test.xls" )
    for i in np.array(df.values[:,-7:]):
        pos = np.eye(4)
        pos[:3, :3] = coord.quaternion_to_rotation_matrix([i[6], i[3], i[4], i[5]])
        pos[:3,3] = i[:3]
        target_positions_with_noise.append(pos)
    target_positions = target_positions_with_noise

    thetas_list = np.array(df.values[:,:-7])
    N = len(thetas_list)

    # for i, thetas in enumerate(thetas_list):
    #     true_pos = forward_kinematics(thetas, true_params)
        
    #     print(true_pos, target_positions[i], np.linalg.norm(true_pos - target_positions[i]))
    
    
    # 求解参数
    initial_guess = np.zeros((N_JOINTS + V_JOINTS_F + V_JOINTS_E)*4)  # 初始猜测为零

    # initial_guess = np.array([0.0, 0.0, 0.0, 0.0,
    #                         0.0, 0.0, 0.0, 0.0,
    #                         1.5707, 0.0, 0.0, 0.0,
    #                         -1.5707, 0.0, 0.0, 0.0,
    #                         1.5707, 0.024, 0.3, 0.0,
    #                         -1.5707, -0.024, 0.0, 0.0,
    #                         1.5707, 0.0, 0.246, 0.0,
    #                         -1.5707, 0.054, 0.0, 1.5707,
    #                         0.0, 0.0, 0.0, 0.0,
    #                         0.0, 0.0, 0.0, 0.0,
    #                         ])

    initial_guess = np.array([
        -1.5707, -0.2, 0.2, 2.35605,
            -1.5707, 0.0, -0.15, 1.5707,
                        0.0, 0.0, 0.1746, 1.5707,
                        1.5707, 0.0, 0.0, 0,
                        -1.5707, 0.0, 0.287, 0,
                        1.5707, 0.018, 0.0, 3.14159,
                        1.5707, 0.018, 0.314, 3.14159,
                        1.5707, 0.0, 0.0, 1.57,
                        1.5707, 0.0, 0.0, 1.57,
        0, 0.04, 0.18, 0,                
        -1.5707, 0.0,-0.12, -0.768,                 
        ])
    
    
    solved_params, result = solve_parameters(thetas_list, target_positions_with_noise, initial_guess)
    
    # 输出结果
    print(f"求解状态: {result.message}")
    print(f"迭代次数: {result.nfev}")
    print(f"最终误差: {result.cost}")
    
    # 计算真实参数与求解参数的差异
    param_diff = np.abs(true_params - solved_params)
    print(true_params.reshape(N_JOINTS + V_JOINTS_F + V_JOINTS_E, 4))

    clib_para = solved_params.reshape(N_JOINTS + V_JOINTS_F + V_JOINTS_E, 4)
    print(solved_params.reshape(N_JOINTS + V_JOINTS_F + V_JOINTS_E, 4))
    print(f"参数平均差异: {np.mean(param_diff):.6f}")


    for p in clib_para:
        print('[', end = '')
        for i in p:
            print(f'{i:.4f}', end = ', ')
        print('0]')
    
    # 可视化结果
    visualize_results(solved_params, thetas_list, target_positions, int(N * 0.3))