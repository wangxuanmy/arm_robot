import numpy as np
from quaternion_utils import matrix_to_quaternion, quaternion_error

def modified_dh_to_homogeneous_matrix(d, theta, r, alpha):
    """
    根据改进 DH 参数生成齐次变换矩阵。

    参数:
    - d: 沿当前关节 z 轴的距离
    - theta: 绕当前关节 z 轴的旋转角度（弧度）
    - r: 沿前一关节 x 轴的距离（连杆长度）
    - alpha: 绕前一关节 x 轴的旋转角度（连杆扭转角，弧度）

    顺序  alpha, r, d ,theta

    返回:
    - 4x4 齐次变换矩阵
    """
    # 计算旋转矩阵和位移矩阵
    T = np.array(
        [
            [
                np.cos(theta),
                -np.sin(theta),
                0,
                r,
            ],
            [
                np.cos(alpha) * np.sin(theta),
                np.cos(theta) * np.cos(alpha),
                -np.sin(alpha),
                -d * np.sin(alpha),
            ],
            [
                np.sin(alpha) * np.sin(theta),
                np.sin(alpha) * np.cos(theta),
                np.cos(alpha),
                d * np.cos(alpha),
            ],
            [0, 0, 0, 1],
        ]
    )

    return T


def modified_old_dh_to_homogeneous_matrix(d, theta, r, alpha):
    """
    根据 DH 参数生成齐次变换矩阵。

    参数:
    - d: 沿前一关节 z 轴的距离
    - theta: 绕前一关节 z 轴的旋转角度（弧度）
    - r: 沿当前关节 x 轴的距离（连杆长度）
    - alpha: 绕当前关节 x 轴的旋转角度（连杆扭转角，弧度）

    返回:
    - 4x4 齐次变换矩阵
    """
    # 计算旋转矩阵和位移矩阵
    T = np.array(
        [
            [
                np.cos(theta),
                -np.sin(theta) * np.cos(alpha),
                np.sin(theta) * np.sin(alpha),
                r * np.cos(theta),
            ],
            [
                np.sin(theta),
                np.cos(theta) * np.cos(alpha),
                -np.cos(theta) * np.sin(alpha),
                r * np.sin(theta),
            ],
            [0, np.sin(alpha), np.cos(alpha), d],
            [0, 0, 0, 1],
        ]
    )

    return T


def angle_normal(theta):
    return np.arctan2(np.sin(theta), np.cos(theta))

def angle_limit(theta: np.ndarray, joint_min, joint_max) -> np.ndarray:
    '''
    处理旋转关节的角度限制（支持numpy数组输入）
    当角度超过最大值时绕回至最小值附近，避免因直接截断导致局部最优问题
    
    参数:
        theta: 关节角度数组（弧度或角度，需与关节限制单位一致）
        joint_min: 关节最小角度限制（可单个值或与theta同长度的数组）
        joint_max: 关节最大角度限制（可单个值或与theta同长度的数组）
    
    返回:
        处理后的角度数组，确保在[joint_min, joint_max]范围内或通过周期性绕回
    '''
    # 确保关节限制与输入数组形状一致
    joint_min = np.asarray(joint_min)
    joint_max = np.asarray(joint_max)
    
    # 计算关节角度范围
    joint_range = joint_max - joint_min
    
    # 创建掩码区分不同情况
    # in_range = (theta >= joint_min) & (theta <= joint_max)
    above_max = theta > joint_max
    below_min = theta < joint_min
    
    # 初始化结果数组
    result = np.copy(theta)
    
    # 对超过最大值的角度：绕回至最小值方向
    excess = theta[above_max] - joint_max[above_max]
    result[above_max] = joint_min[above_max] + (excess % joint_range[above_max])
    
    # 对低于最小值的角度：绕回至最大值方向
    deficit = joint_min[below_min] - theta[below_min]
    result[below_min] = joint_max[below_min] - (deficit % joint_range[below_min])
    
    return result

def damped_least_squares(J, lambda_d=0.3):
    """带阻尼的雅克比伪逆，避免奇异点"""
    # 阻尼伪逆公式：J^T (J J^T + λ² I)^(-1)
    J_T = J.T
    damping = lambda_d ** 2 * np.eye(J.shape[0])  # 阻尼项
    return J_T @ np.linalg.pinv(J @ J_T + damping)

def cal_qpos(start_theta, aim_matrix, dh_type, get_dh_mat, get_theta_limit, ignore_angle=False, random_flag = False, q0 = None):
    now_theta = start_theta

    theta_can = []
    track = []

    # 将目标矩阵转换为四元数
    aim_rot_matrix = aim_matrix[:3, :3]
    aim_quaternion = matrix_to_quaternion(aim_rot_matrix)
    aim_position = aim_matrix[:3, 3]

    get_aim = False

    for cal_times in range(1, 400):
        b = get_dh_mat(now_theta)
        b_r = b.copy()
        b_r.reverse()
        T = []
        tf = b_r[0]
        T.append(tf)
        for i in range(1, len(b)):
            tf = b_r[i] @ tf
            T.append(tf)

        T.reverse()

        # 位置误差计算
        d_pose = aim_position - T[0][:3, 3]

        # 使用四元数计算旋转误差，避免万向节锁
        now_rot_matrix = T[0][:3, :3]
        now_quaternion = matrix_to_quaternion(now_rot_matrix)
        
        # 计算四元数误差
        d_rotation = quaternion_error(now_quaternion, aim_quaternion)
        d_rotation = 0.5 * d_rotation

        err_s = np.linalg.norm(d_pose)
        err_theta = 0

        if ignore_angle == False:
            err_theta = 0.3 * np.linalg.norm(d_rotation)

        track.append(T[0][0:3, 3])
        
        # print(err, coord.homogeneous_matrix_to_euler_and_translation(T[0]))
        # print(err, now_theta)
        theta_can.append(now_theta.copy())


        if err_s < 0.0005 and err_theta < 0.001:
            get_aim = True
            break

        j = []
        for index, i in enumerate(T[1:]):
            if(dh_type[index] == 0):
                j.append(
                    [
                        i[0, 3] * i[1, 0] - i[1, 3] * i[0, 0],
                        i[0, 3] * i[1, 1] - i[1, 3] * i[0, 1],
                        i[0, 3] * i[1, 2] - i[1, 3] * i[0, 2],
                        i[2, 0],
                        i[2, 1],
                        i[2, 2],
                    ]
                )
            elif(dh_type[index] == 1):
                j.append(
                    [
                        i[1, 3] * i[2, 0] - i[2, 3] * i[1, 0],
                        i[1, 3] * i[2, 1] - i[2, 3] * i[1, 1],
                        i[1, 3] * i[2, 2] - i[2, 3] * i[1, 2],
                        i[0, 0],
                        i[0, 1],
                        i[0, 2],
                    ]
                )
            elif(dh_type[index] == 2):
                j.append(
                    [
                        i[2, 0],
                        i[2, 1],
                        i[2, 2],
                        0,
                        0,
                        0,
                    ]
                )
        #默认末端是旋转z
        j.append([0.0, 0.0, 0.0, 0.0, 0.0, 1.0])
        j = np.array(j, dtype=np.float64)
        j = j.T
        temp = np.zeros([6, 6])
        temp[0:3, 0:3] = T[0][0:3, 0:3]
        temp[3:6, 3:6] = T[0][0:3, 0:3]
        j0 = temp @ j

        if ignore_angle:
            ds = [d_pose[0], d_pose[1], d_pose[2],0,0,0]
        else:
            ds = [d_pose[0], d_pose[1], d_pose[2], d_rotation[0], d_rotation[1], d_rotation[2]]

        v = (
            # 0.1 * np.abs(np.sin(cal_times * 0.015707))
            0.9*
            np.array(ds).T
        )
    
        # dq = np.linalg.pinv(j0) @ (v)

        kn = 0.9
        zeros_dq = np.zeros_like(now_theta)
        if(q0 is not None):
            dq0 = q0 - now_theta
            zeros_dq = (np.eye(len(now_theta)) - np.linalg.pinv(j0) @ j0) @ (kn * dq0)

        dq = damped_least_squares(j0) @ v

        new_theta = np.array(now_theta) + dq + zeros_dq

        if cal_times % 150 == 0 and random_flag:
            new_theta = np.random.uniform(
                -np.pi, np.pi, size=len(new_theta)
            )
        
        new_theta = angle_normal(new_theta)
        
        new_theta_limit = get_theta_limit(new_theta)
        # new_theta = angle_limit(new_theta, new_theta_limit[:, 0], new_theta_limit[:, 1])
        new_theta = np.clip(new_theta, new_theta_limit[:, 0], new_theta_limit[:, 1])

        now_theta = list(new_theta)

    return now_theta, theta_can, get_aim

def cal_qpos_v(start_theta, aim_v, dh_type, get_dh_mat, get_theta_limit):
    now_theta = start_theta

    b = get_dh_mat(now_theta)
    b_r = b.copy()
    b_r.reverse()
    T = []
    tf = b_r[0]
    T.append(tf)
    for i in range(1, len(b)):
        tf = b_r[i] @ tf
        T.append(tf)

    T.reverse()

        
    j = []
    for index, i in enumerate(T[1:]):
        if(dh_type[index] == 0):
            j.append(
                [
                    i[0, 3] * i[1, 0] - i[1, 3] * i[0, 0],
                    i[0, 3] * i[1, 1] - i[1, 3] * i[0, 1],
                    i[0, 3] * i[1, 2] - i[1, 3] * i[0, 2],
                    i[2, 0],
                    i[2, 1],
                    i[2, 2],
                ]
            )
        elif(dh_type[index] == 1):
            j.append(
                [
                    i[1, 3] * i[2, 0] - i[2, 3] * i[1, 0],
                    i[1, 3] * i[2, 1] - i[2, 3] * i[1, 1],
                    i[1, 3] * i[2, 2] - i[2, 3] * i[1, 2],
                    i[0, 0],
                    i[0, 1],
                    i[0, 2],
                ]
            )
        elif(dh_type[index] == 2):
            j.append(
                [
                    i[2, 0],
                    i[2, 1],
                    i[2, 2],
                    0,
                    0,
                    0,
                ]
            )
    #默认末端是旋转z
    j.append([0.0, 0.0, 0.0, 0.0, 0.0, 1.0])
    j = np.array(j, dtype=np.float64)
    j = j.T
    temp = np.zeros([6, 6])
    temp[0:3, 0:3] = T[0][0:3, 0:3]
    temp[3:6, 3:6] = T[0][0:3, 0:3]
    j0 = temp @ j

    v = aim_v

    dq = np.linalg.pinv(j0) @ (v)
    # dq = self.damped_least_squares(j0) @ v

    new_theta = np.array(now_theta) + dq

    new_theta = angle_normal(new_theta)
    
    new_theta_limit = get_theta_limit(new_theta)
    new_theta = np.clip(new_theta, new_theta_limit[:, 0], new_theta_limit[:, 1])

    now_theta = list(new_theta)

    return now_theta



if __name__ == "__main__":
    # 示例输入
    d = 1.0  # 沿前一关节 z 轴的距离
    theta = 30 / 180.0 * np.pi  # 绕前一关节 z 轴的旋转角度（45 度）
    r = 0  # 沿当前关节 x 轴的距离（连杆长度）
    alpha = 0  # 绕当前关节 x 轴的旋转角度（30 度）

    # 调用函数
    homogeneous_matrix = modified_dh_to_homogeneous_matrix(d, theta, r, alpha)
    print("改进 DH 参数的齐次变换矩阵:")
    print(homogeneous_matrix)
