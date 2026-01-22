import numpy as np


def euler_to_homogeneous_matrix(alpha, beta, gamma, x, y, z):
    """使用欧拉角和位移创建齐次变换矩阵"""
    # 计算旋转矩阵
    cx, sx = np.cos(alpha), np.sin(alpha)
    cy, sy = np.cos(beta), np.sin(beta)
    cz, sz = np.cos(gamma), np.sin(gamma)

    # 构建旋转矩阵 (ZYX欧拉角顺序)
    R = np.array([[cy * cz, sx * sy * cz - cx * sz, cx * sy * cz + sx * sz],
                  [cy * sz, sx * sy * sz + cx * cz, cx * sy * sz - sx * cz],
                  [-sy,     sx * cy,                cx * cy]])

    # 构建齐次变换矩阵
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [x, y, z]
    
    return T


def euler_rpy_to_homogeneous_matrix(roll, pitch, yaw, x, y, z):
    """使用RPY欧拉角和位移创建齐次变换矩阵"""
    # 计算旋转矩阵
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)

    # 构建旋转矩阵 (RPY顺序)
    R = np.array([[cp * cy, -cr * sy + sr * sp * cy, sr * sy + cr * sp * cy],
                  [cp * sy, cr * cy + sr * sp * sy, -sr * cy + cr * sp * sy],
                  [-sp,     sr * cp,                cr * cp]])

    # 构建齐次变换矩阵
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [x, y, z]
    
    return T


def quaternion_to_homogeneous_matrix(w, x, y, z, tx, ty, tz):
    """使用四元数和位移创建齐次变换矩阵"""
    # 使用transforms3d库将四元数转换为旋转矩阵
    q = [w, x, y, z]
    R = quaternion_to_rotation_matrix(q)
    
    # 构建齐次变换矩阵
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [tx, ty, tz]
    
    return T


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

    # 显示图例
    # ax.legend()

    # 显示图形
    # plt.show()


def euler_to_homogeneous_matrix(pitch, roll, yaw, x, y, z):
    """
    将欧拉角（pitch, roll, yaw）和平移坐标（x, y, z）转换为齐次变换矩阵。

    参数:
    - pitch: 绕 y 轴的旋转角度（弧度）
    - roll: 绕 x 轴的旋转角度（弧度）
    - yaw: 绕 z 轴的旋转角度（弧度）
    - x, y, z: 平移坐标

    返回:
    - 4x4 齐次变换矩阵
    """
    # 计算旋转矩阵
    # 绕 z 轴（yaw）旋转
    R_z = np.array(
        [[np.cos(yaw), -np.sin(yaw), 0], [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]]
    )

    # 绕 y 轴（pitch）旋转
    R_y = np.array(
        [
            [np.cos(pitch), 0, np.sin(pitch)],
            [0, 1, 0],
            [-np.sin(pitch), 0, np.cos(pitch)],
        ]
    )

    # 绕 x 轴（roll）旋转
    R_x = np.array(
        [[1, 0, 0], [0, np.cos(roll), -np.sin(roll)], [0, np.sin(roll), np.cos(roll)]]
    )

    # 组合旋转矩阵（R_z * R_y * R_x）
    R = R_z @ R_y @ R_x

    # 构建齐次变换矩阵
    homogeneous_matrix = np.eye(4)  # 初始化为单位矩阵
    homogeneous_matrix[:3, :3] = R  # 旋转矩阵
    homogeneous_matrix[:3, 3] = [x, y, z]  # 平移向量

    return homogeneous_matrix


def euler_rpy_to_homogeneous_matrix(pitch, roll, yaw, x, y, z):
    """
    将欧拉角（pitch, roll, yaw）和平移坐标（x, y, z）转换为齐次变换矩阵。

    参数:
    - pitch: 绕 y 轴的旋转角度（弧度）
    - roll: 绕 x 轴的旋转角度（弧度）
    - yaw: 绕 z 轴的旋转角度（弧度）
    - x, y, z: 平移坐标

    返回:
    - 4x4 齐次变换矩阵
    """
    # 计算旋转矩阵
    # 绕 z 轴（yaw）旋转
    R_z = np.array(
        [[np.cos(yaw), -np.sin(yaw), 0], [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]]
    )

    # 绕 y 轴（pitch）旋转
    R_y = np.array(
        [
            [np.cos(pitch), 0, np.sin(pitch)],
            [0, 1, 0],
            [-np.sin(pitch), 0, np.cos(pitch)],
        ]
    )

    # 绕 x 轴（roll）旋转
    R_x = np.array(
        [[1, 0, 0], [0, np.cos(roll), -np.sin(roll)], [0, np.sin(roll), np.cos(roll)]]
    )

    # 组合旋转矩阵（R_z * R_y * R_x）
    # R = R_z @ R_y @ R_x
    R = R_x @ R_y @ R_z
    # R = R_y @ R_x @ R_z

    # 构建齐次变换矩阵
    homogeneous_matrix = np.eye(4)  # 初始化为单位矩阵
    homogeneous_matrix[:3, :3] = R  # 旋转矩阵
    homogeneous_matrix[:3, 3] = [x, y, z]  # 平移向量

    return homogeneous_matrix

def matrix_to_euler(R):
    yaw = np.arctan2(R[1, 0], R[0, 0])

    # 计算 pitch（绕 y 轴）
    pitch = np.arctan2(-R[2, 0], np.sqrt(R[2, 1] ** 2 + R[2, 2] ** 2))

    # 计算 roll（绕 x 轴）
    roll = np.arctan2(R[2, 1], R[2, 2])

    return [roll, pitch, yaw]

def homogeneous_matrix_to_euler_and_translation(homogeneous_matrix):
    """
    将齐次变换矩阵转换为欧拉角（pitch, roll, yaw）和平移坐标（x, y, z）。

    参数:
    - homogeneous_matrix: 4x4 齐次变换矩阵

    返回:
    - pitch: 绕 y 轴的旋转角度（弧度）
    - roll: 绕 x 轴的旋转角度（弧度）
    - yaw: 绕 z 轴的旋转角度（弧度）
    - x: 沿 x 轴的平移
    - y: 沿 y 轴的平移
    - z: 沿 z 轴的平移
    """
    # 提取旋转矩阵（3x3）
    R = homogeneous_matrix[:3, :3]

    # 提取平移向量（3x1）
    translation = homogeneous_matrix[:3, 3]
    x, y, z = translation

    # 计算欧拉角（pitch, roll, yaw）
    # 假设旋转顺序为 z-y-x（yaw -> pitch -> roll）
    # 计算 yaw（绕 z 轴）
    yaw = np.arctan2(R[1, 0], R[0, 0])

    # 计算 pitch（绕 y 轴）
    pitch = np.arctan2(-R[2, 0], np.sqrt(R[2, 1] ** 2 + R[2, 2] ** 2))

    # 计算 roll（绕 x 轴）
    roll = np.arctan2(R[2, 1], R[2, 2])

    return pitch, roll, yaw, x, y, z


def euler_rpy_to_quaternion_outer(pitch, roll, yaw):
    # 计算 cos 和 sin 值
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    # 计算四元数的分量
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return np.array([w, x, y, z])


def euler_rpy_to_quaternion_in(pitch, roll, yaw):
    # 计算 cos 和 sin 值
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    # 计算四元数的分量
    w = cy * cp * cr - sy * sp * sr
    x = cy * cp * sr + sy * sp * cr
    y = cy * sp * cr - sy * cp * sr
    z = sy * cp * cr + cy * sp * sr
    return np.array([w, x, y, z])


def euler_pry_to_quaternion(pitch, roll, yaw):
    # 计算 cos 和 sin 值
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    # 计算四元数的分量
    w = cp * cr * cy - sp * sr * sy
    x = cp * cr * sy + sp * sr * cy
    y = sp * cr * sy - cp * sr * cy
    z = sp * cr * cy + cp * sr * sy
    return np.array([w, x, y, z])


def euler_to_quaternion(roll, pitch, yaw):
    """
    将欧拉角（按照z->y->x顺序）转换为四元数。

    参数:
    roll (float): 绕x轴旋转的角度（弧度）
    pitch (float): 绕y轴旋转的角度（弧度）
    yaw (float): 绕z轴旋转的角度（弧度）

    返回:
    np.array: 表示四元数的数组，顺序为 [w, x, y, z]
    """
    cy = np.cos(yaw * 0.5)
    sy = np.sin(yaw * 0.5)
    cp = np.cos(pitch * 0.5)
    sp = np.sin(pitch * 0.5)
    cr = np.cos(roll * 0.5)
    sr = np.sin(roll * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy

    return np.array([w, x, y, z])


def quaternion_to_euler(q):
    """
    将四元数转换为欧拉角（按照z->y->x顺序）。

    参数:
    q (np.array): 表示四元数的数组，顺序为 [w, x, y, z]

    返回:
    roll (float): 绕x轴旋转的角度（弧度）
    pitch (float): 绕y轴旋转的角度（弧度）
    yaw (float): 绕z轴旋转的角度（弧度）
    """
    w, x, y, z = q

    roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x**2 + y**2))
    pitch = np.arcsin(2 * (w * y - z * x))
    yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y**2 + z**2))

    return roll, pitch, yaw


def rotation_matrix_to_quaternion(R):
    """
    将3x3旋转矩阵转换为四元数。

    参数:
    R (np.array): 3x3的旋转矩阵

    返回:
    np.array: 表示四元数的数组，顺序为 [w, x, y, z]
    """
    tr = np.trace(R)
    if tr > 0:
        S = np.sqrt(tr + 1.0) * 2
        qw = 0.25 * S
        qx = (R[2, 1] - R[1, 2]) / S
        qy = (R[0, 2] - R[2, 0]) / S
        qz = (R[1, 0] - R[0, 1]) / S
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        S = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        qw = (R[2, 1] - R[1, 2]) / S
        qx = 0.25 * S
        qy = (R[0, 1] + R[1, 0]) / S
        qz = (R[0, 2] + R[2, 0]) / S
    elif R[1, 1] > R[2, 2]:
        S = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        qw = (R[0, 2] - R[2, 0]) / S
        qx = (R[0, 1] + R[1, 0]) / S
        qy = 0.25 * S
        qz = (R[1, 2] + R[2, 1]) / S
    else:
        S = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        qw = (R[1, 0] - R[0, 1]) / S
        qx = (R[0, 2] + R[2, 0]) / S
        qy = (R[1, 2] + R[2, 1]) / S
        qz = 0.25 * S

    return np.array([qw, qx, qy, qz])


def quaternion_to_rotation_matrix(q):
    """
    将四元数转换为3x3旋转矩阵。

    参数:
    q (np.array): 表示四元数的数组，顺序为 [w, x, y, z]

    返回:
    np.array: 3x3的旋转矩阵
    """
    w, x, y, z = q
    xx = x * x
    xy = x * y
    xz = x * z
    yy = y * y
    yz = y * z
    zz = z * z
    wx = w * x
    wy = w * y
    wz = w * z

    R = np.array(
        [
            [1 - 2 * (yy + zz), 2 * (xy - wz), 2 * (xz + wy)],
            [2 * (xy + wz), 1 - 2 * (xx + zz), 2 * (yz - wx)],
            [2 * (xz - wy), 2 * (yz + wx), 1 - 2 * (xx + yy)],
        ]
    )

    return R


class ShowTest:
    def __init__(self):
        # 创建一个3D图形
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection="3d")

        # 设置滑动条的位置
        ax_theta1 = plt.axes([0.1, 0.95, 0.8, 0.03])
        ax_theta2 = plt.axes([0.1, 0.92, 0.8, 0.03])
        ax_theta3 = plt.axes([0.1, 0.89, 0.8, 0.03])

        ax_theta4 = plt.axes([0.1, 0.86, 0.8, 0.03])
        ax_theta5 = plt.axes([0.1, 0.83, 0.8, 0.03])
        ax_theta6 = plt.axes([0.1, 0.80, 0.8, 0.03])

        arm_len = 1.3
        self.show_len = 0.5

        self.theta = [
            -2.383,
            0.016,
            -1.621,
            0.303,
            0.182,
            0.272,
        ]
        # 创建滑动条
        self.s_theta1 = Slider(
            ax_theta1,
            r"$\theta_1$",
            -np.pi,
            np.pi,
            valinit=self.theta[0],
        )
        self.s_theta2 = Slider(
            ax_theta2,
            r"$\theta_2$",
            -np.pi,
            np.pi,
            valinit=self.theta[1],
        )
        self.s_theta3 = Slider(
            ax_theta3,
            r"$\theta_3$",
            -np.pi,
            np.pi,
            valinit=self.theta[2],
        )

        self.s_theta4 = Slider(
            ax_theta4,
            r"$\theta_4$",
            -arm_len,
            arm_len,
            valinit=self.theta[3],
        )
        self.s_theta5 = Slider(
            ax_theta5,
            r"$\theta_5$",
            -arm_len,
            arm_len,
            valinit=self.theta[4],
        )
        self.s_theta6 = Slider(
            ax_theta6,
            r"$\theta_6$",
            -arm_len,
            arm_len,
            valinit=self.theta[5],
        )

        # 连接滑动条的更新函数
        self.s_theta1.on_changed(self.update)
        self.s_theta2.on_changed(self.update)
        self.s_theta3.on_changed(self.update)
        self.s_theta4.on_changed(self.update)
        self.s_theta5.on_changed(self.update)
        self.s_theta6.on_changed(self.update)

        self.update([])
        plt.show()

    def update(self, val):
        """
        更新函数，当滑动条值改变时调用。
        """
        # 获取滑动条的当前值
        self.theta[0] = self.s_theta1.val
        self.theta[1] = self.s_theta2.val
        self.theta[2] = self.s_theta3.val
        self.theta[3] = self.s_theta4.val
        self.theta[4] = self.s_theta5.val
        self.theta[5] = self.s_theta6.val

        # 清除之前的绘图
        self.ax.clear()

        matrix_origin = euler_rpy_to_homogeneous_matrix(-1.57, 0, -3.14, 0.0, 0.0, 0.0)
        # matrix_origin = euler_rpy_to_homogeneous_matrix(0.0, 0, 0, 0.0, 0.0, 0.0)
        plot_3d_axes(matrix_origin, 0.1, self.ax)
        matrix = euler_rpy_to_homogeneous_matrix(
            self.theta[1],
            self.theta[0],
            self.theta[2],
            self.theta[3],
            self.theta[4],
            self.theta[5],
        )
        # matrix = matrix @ matrix_origin
        print(
            [
                self.theta[3],
                self.theta[4],
                self.theta[5],
                self.theta[0],
                self.theta[1],
                self.theta[2],
            ]
        )
        print("source:")
        print(matrix[:3, :3])
        q = rotation_matrix_to_quaternion(matrix[:3, :3])

        print("m->q:")
        print(q)
        print(quaternion_to_rotation_matrix(q))

        plot_3d_axes(matrix_origin @ matrix, 0.1, self.ax, "--")

        print("euler:")
        print(euler_rpy_to_quaternion_in(self.theta[1], self.theta[0], self.theta[2]))

        m = quaternion_to_rotation_matrix(
            euler_rpy_to_quaternion_in(self.theta[1], self.theta[0], self.theta[2])
        )

        print(m)

        matrix[:3, :3] = m

        plot_3d_axes(matrix_origin @ matrix, 0.1, self.ax, "-.")

        self.ax.set_xlim([-self.show_len, self.show_len])
        self.ax.set_ylim([-self.show_len, self.show_len])
        self.ax.set_zlim([-self.show_len, self.show_len])
        # 更新图形
        self.fig.canvas.draw_idle()


def test():
    # 测试欧拉角转四元数

    roll = 3.14159
    pitch = -4.91094e-06
    yaw = -5.5386e-06
    q = euler_to_quaternion(roll, pitch, yaw)
    print("欧拉角转四元数结果:", q)

    q = [0, 0, 0, 1]
    # 测试四元数转欧拉角
    roll_result, pitch_result, yaw_result = quaternion_to_euler(q)
    print("四元数转欧拉角结果:", roll_result, pitch_result, yaw_result)

    r = euler_to_homogeneous_matrix(0.2, 0.1, 0.3, 0, 0, 0)
    print(r)
    # 测试旋转矩阵转四元数，这里构造一个简单的旋转矩阵示例
    R = r[0:3, 0:3]
    q_from_R = rotation_matrix_to_quaternion(R)
    print("旋转矩阵转四元数结果:", q_from_R)

    # 测试四元数转旋转矩阵
    R_result = quaternion_to_rotation_matrix(q_from_R)
    print("四元数转旋转矩阵结果:\n", R_result)


def change():
    pose = [254.5, 100.00075696, 280.499, 3.14159, -4.91094e-06, -5.5386e-06]
    q = euler_rpy_to_homogeneous_matrix(
        pose[4], pose[3], pose[5], pose[0], pose[1], pose[2]
    )

    print([pose[0], pose[1], pose[2]], rotation_matrix_to_quaternion(q[0:3, 0:3]))


if __name__ == "__main__":
    # # 示例输入
    # pitch = np.pi
    # roll = 0
    # yaw = 0
    # x, y, z = 1, 2, 3  # 平移坐标

    # # 创建一个3D图形
    # fig = plt.figure()
    # ax = fig.add_subplot(111, projection="3d")

    # # 示例齐次变换矩阵（4x4）
    # matrix = euler_to_homogeneous_matrix(pitch, roll, yaw, x, y, z)
    # matrix = euler_to_homogeneous_matrix(
    #     -0.6999898430987282, -0.12504635986499557, 0.19270484450276465, 0.5, 0.0, 0.5
    # )

    # print(homogeneous_matrix_to_euler_and_translation(matrix))

    # # 坐标轴长度
    # axis_length = 5

    # # 调用函数绘制3D坐标轴
    # plot_3d_axes(matrix, axis_length, ax)
    # plt.show()

    ShowTest()
    # test()
    # change()
