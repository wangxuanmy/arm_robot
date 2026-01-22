import numpy as np
import coord
import dh
from quaternion_utils import matrix_to_quaternion, quaternion_error, quaternion_to_rotation_vector


class Body:
    def __init__(self, dh_table: list, theta_limit=None):
        """
        dh_table: nx4 矩阵，每一行表示一个连杆的 DH 参数
        ----------------------------------------
        |  alpha | a | d  | theta | fun_type |
        | 1
        | 2
        | 3
        | ... 末端
        ----------------------------------------
        """
        # 初始化 self.theta 值
        self.num_joints = len(dh_table)

        self.theta = [0] * self.num_joints

        self.dh_table = dh_table

        if theta_limit == None:
            self.theta_limit = np.array([[-360, 360] * self.num_joints]) / 57.2958
        else:
            self.theta_limit = self.theta_limit

    def is_singular(self, J, threshold=1000):
        """判断雅克比矩阵是否接近奇异点"""
        # 计算奇异值
        singular_values = np.linalg.svd(J, compute_uv=False)
        # 最大奇异值 / 最小奇异值 = 条件数
        cond = singular_values[0] / singular_values[-1]
        return cond > threshold  # 超过阈值则视为接近奇异点
    
    def get_dh_matrix(self, theta) -> list:
        b = []
        # 初始化 DH 矩阵
        for i, value in enumerate(self.dh_table):
            if value[-1] == 0:
                b.append(
                    dh.modified_dh_to_homogeneous_matrix(
                        value[2], theta[i] + value[3], value[1], value[0]
                    )
                )
            elif value[-1] == 1:
                b.append(
                    dh.modified_old_dh_to_homogeneous_matrix(
                        value[2], value[2], value[1], theta[i] + value[0]
                    )
                )
            elif value[-1] == 2:
                b.append(
                    dh.modified_dh_to_homogeneous_matrix(
                        value[2] + theta[i], value[3], value[1], value[0]
                    )
                )
        return b
    
    def get_dh_type(self) -> list:
        b = []
        # 获取 DH 参数类型
        for i, value in enumerate(self.dh_table):
            if value[-1] == 0:
                b.append(0)
            elif value[-1] == 1:
                b.append(1)
            elif value[-1] == 2:
                b.append(2)
        return b

    def get_joint_limit(self, joint_value):
        return self.theta_limit

    def cal_qpos(self, start_theta, aim_matrix, ignore_angle=False,random_flag = False, q0 = None):
        def dh_mat_fun(theta):
            return self.get_dh_matrix(theta)
        
        def get_theta_limit(theta):
            return self.get_joint_limit(theta)

        return dh.cal_qpos(start_theta, aim_matrix, self.get_dh_type(), dh_mat_fun,get_theta_limit,ignore_angle,random_flag,q0)

    def cal_qpos_v(self, start_theta, aim_v, theta_limit = []):
        def dh_mat_fun(theta):
            return self.get_dh_matrix(theta)
        
        def get_theta_limit(theta):
            return self.get_joint_limit(theta)

        return dh.cal_qpos_v(start_theta, aim_v, self.get_dh_type(), dh_mat_fun, get_theta_limit)

    def get_eef(self):
        base = coord.euler_to_homogeneous_matrix(0, 0, 0, 0, 0, 0)
        mat = self.get_dh_matrix(self.theta)
        tf = base
        for i in mat:
            tf = tf @ i
        return tf


class Marvin_body(Body):
    def __init__(self, dh_table: list, theta_limit=None):
        super().__init__(dh_table, theta_limit)

    def marvin_cross_joint_limit(self, j6, j7):
        j7_limit = 0
        j6_limit = 0
        if (abs(j6) <= 0.349):
            j7_limit = 1.5707
        else:
            j7_limit = -1.024 * abs(j6) + 1.9275

        if (abs(j7) <= 0.855):
            j6_limit = 1.047
        else:
            j6_limit = -0.9765 * abs(j7) + 1.8823

        return [j6_limit, j7_limit]

    def get_joint_limit(self, joint_value):
        joint_limit = self.theta_limit

        joint6_7 = self.marvin_cross_joint_limit(joint_value[-2],joint_value[-1])

        joint_limit[-2, 0] = -joint6_7[-2]
        joint_limit[-1, 0] = -joint6_7[-1]

        joint_limit[-2, 1] = joint6_7[-2]
        joint_limit[-1, 1] = joint6_7[-1]

        return joint_limit


if __name__ == "__main__":
    body = Body(
        [
            [0.0, 0.0, 0.097, 0.0, 0],
            [1.5707, 0.0, 0.0, -1.5707, 0],
            [-1.5707, 0.0, 0.3579, 0.0, 0],
            [1.5707, -0.0537, 0.0, 0.0, 0],
            [-1.5707, 0.036, 0.2041, 0.0, 0],
            [1.5707, 0.0, 0.0, -1.5707, 0],
            [1.5707, 0.08, 0.0, 1.5707, 0],


                
        ]
    )


    q_can = body.cal_qpos(np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
        coord.euler_to_homogeneous_matrix(0.0, 0.0, 0.3, 0.4, 0.0, 0.353)
    )
    q_can = np.array(q_can)
    print(q_can)

