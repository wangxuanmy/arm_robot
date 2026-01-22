import body_base
import matplotlib.pyplot as plt
import coord
import dh
from matplotlib.widgets import Slider
import numpy as np
from quaternion_utils import matrix_to_quaternion, quaternion_error, quaternion_to_rotation_vector
from mpl_toolkits.mplot3d import Axes3D  # 关键：导入3D投影模块



class Human:
    
    def __init__(self, body_dh, left_hand_dh, right_hand_dh, theta_limit=[]):
        self.body = body_base.Body(body_dh)
        self.left_hand = body_base.Marvin_body(left_hand_dh)
        self.right_hand = body_base.Marvin_body(right_hand_dh)

        self.hand_id = {"body": 0, "left": 1, "right": 2}

        self.num_joints = self.body.num_joints + self.left_hand.num_joints + self.right_hand.num_joints

        self.theta = [0] * self.num_joints


        if len(theta_limit) == 0:
            self.theta_limit = np.array([[-360, 360]] * self.num_joints) / 57.2958
        else:
            self.theta_limit = theta_limit

        self.body.theta_limit = self.theta_limit[:self.body.num_joints, :]
        self.left_hand.theta_limit = self.theta_limit[self.body.num_joints:self.body.num_joints + self.left_hand.num_joints, :]
        self.right_hand.theta_limit = self.theta_limit[self.body.num_joints + self.left_hand.num_joints:, :]

        self.base_tf = {"body":coord.euler_rpy_to_homogeneous_matrix(0,0,0,0,0,0),
                        'left':coord.euler_rpy_to_homogeneous_matrix(0,0,0,0.1,0.0,0.1),
                        'right': coord.euler_rpy_to_homogeneous_matrix(0,0,0,0.1,0.0,-0.1)}

        # 创建一个3D图形
        self.fig = plt.figure('Body')
        self.ax = self.fig.add_subplot(111, projection="3d")

        self.show_len = 2.6
        self.axis_len = 0.05

    def flash_theta(self, theta):
        '''设定角度到所有机械臂关节'''
        self.body.theta = theta[:self.body.num_joints]
        self.left_hand.theta = theta[self.body.num_joints:self.body.num_joints + self.left_hand.num_joints]
        self.right_hand.theta = theta[self.body.num_joints + self.left_hand.num_joints:] 

    def get_aix_tf(self, base, tf_tree):
        '''获取每个关节的坐标变换矩阵'''
        tf_can = []

        tf = base
        tf_can.append(tf)
        for i in tf_tree:
            tf = tf @ i
            tf_can.append(tf)

        return tf_can
    
    def show_body(self):
        # 设置滑动条的位置
        ax_theta = []

        for i in range(self.num_joints):
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
        print(self.theta_limit)
        self.s_theta = []
        for i, ax_the in enumerate(ax_theta):
            self.s_theta.append(
                Slider(
                    ax_the,
                    f"Theta{i+1}",
                    self.theta_limit[i, 0],
                    self.theta_limit[i, 1],
                    valinit=self.theta[i],
                )
            )


        # 连接滑动条的更新函数
        for i in self.s_theta:
            i.on_changed(self.update)


        # 重新绘制坐标轴和连杆
        body_mat = self.body.get_dh_matrix(self.body.theta)

        body_end = self.draw(self.base_tf['body'], body_mat)

        self.draw(body_end @ self.base_tf['left'], self.left_hand.get_dh_matrix(self.left_hand.theta))

        self.draw(body_end @ self.base_tf['right'], self.right_hand.get_dh_matrix(self.right_hand.theta))

        self.ax.set_xlim([-self.show_len, self.show_len])
        self.ax.set_ylim([-self.show_len, self.show_len])
        self.ax.set_zlim([-self.show_len, self.show_len])
        
    def update(self, val):
        # 获取滑动条的当前值
        for i in range(self.num_joints):
            self.theta[i] = self.s_theta[i].val

        end_joint_index = self.body.num_joints
        self.body.theta = self.theta[:end_joint_index]

        self.left_hand.theta = self.theta[end_joint_index: end_joint_index + self.left_hand.num_joints]
        end_joint_index += self.left_hand.num_joints

        self.right_hand.theta = self.theta[end_joint_index: end_joint_index + self.right_hand.num_joints]

        
        # 保存当前视图限制和视角
        xlim = self.ax.get_xlim()
        ylim = self.ax.get_ylim()
        zlim = self.ax.get_zlim()
        elev, azim = self.ax.elev, self.ax.azim  # 保存仰角和方位角

        # 清除之前的绘图
        self.ax.clear()

        # 重新绘制坐标轴和连杆
        body_mat = self.body.get_dh_matrix(self.body.theta)

        body_end = self.draw(self.base_tf['body'], body_mat)

        self.draw(body_end @ self.base_tf['left'], self.left_hand.get_dh_matrix(self.left_hand.theta))

        self.draw(body_end @ self.base_tf['right'], self.right_hand.get_dh_matrix(self.right_hand.theta))

        # 恢复视图限制和视角
        self.ax.set_xlim(xlim)
        self.ax.set_ylim(ylim)
        self.ax.set_zlim(zlim)
        self.ax.view_init(elev=elev, azim=azim)  # 恢复视角

        # 更新图形
        self.fig.canvas.draw_idle()

    def draw_coord(self, mat, ls = '--'):
        coord.plot_3d_axes(
            mat, self.axis_len, self.ax, ls
        )

    def draw(self, base, tf_tree):
        base_origin = []

        tf = base
        base_origin.append(tf[:3, 3])
        coord.plot_3d_axes(tf, self.axis_len, self.ax)

        for i in tf_tree:
            tf = tf @ i
            base_origin.append(tf[:3, 3])
            coord.plot_3d_axes(tf, self.axis_len, self.ax)

        # 绘制连杆之间的连接线
        for i in range(len(base_origin) - 1):
            x = [base_origin[i][0], base_origin[i + 1][0]]
            y = [base_origin[i][1], base_origin[i + 1][1]]
            z = [base_origin[i][2], base_origin[i + 1][2]]
            self.ax.plot3D(x, y, z)

        return tf
    
    def draw_out(self,base, tf_tree, ax):
        base_origin = []

        tf = base
        base_origin.append(tf[:3, 3])
        coord.plot_3d_axes(tf, self.axis_len, ax)
        for i in tf_tree:
            tf = tf @ i
            base_origin.append(tf[:3, 3])
            coord.plot_3d_axes(tf, self.axis_len, ax)

        # 绘制连杆之间的连接线
        for i in range(len(base_origin) - 1):
            x = [base_origin[i][0], base_origin[i + 1][0]]
            y = [base_origin[i][1], base_origin[i + 1][1]]
            z = [base_origin[i][2], base_origin[i + 1][2]]
            ax.plot3D(x, y, z)

        return tf
    
    def show_contorl(self):
        # 设置滑动条的位置
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
        for i, ax_the in enumerate(ax_theta):
            self.s_theta.append(
                Slider(
                    ax_the,
                    f"Theta{i+1}",
                    self.theta_limit[i,0],
                    self.theta_limit[i,1],
                    valinit=self.theta[i],
                )
            )
            print(self.theta_limit[i])
        print(self.theta_limit)

        # 连接滑动条的更新函数
        for i in self.s_theta:
            i.on_changed(self.update_ctrl)


        # 重新绘制坐标轴和连杆
        body_mat = self.body.get_dh_matrix(self.body.theta)

        body_end = self.draw(self.base_tf['body'], body_mat)

        left_end = self.draw(body_end @ self.base_tf['left'], self.left_hand.get_dh_matrix(self.left_hand.theta))

        print(coord.homogeneous_matrix_to_euler_and_translation(left_end))

        self.draw(body_end @ self.base_tf['right'], self.right_hand.get_dh_matrix(self.right_hand.theta))

        self.ax.set_xlim([-self.show_len, self.show_len])
        self.ax.set_ylim([-self.show_len, self.show_len])
        self.ax.set_zlim([-self.show_len, self.show_len])

    def update_ctrl(self, val):
        mat_value = [0] * 12

        # 获取滑动条的当前值
        for i in range(12):
            mat_value[i] = self.s_theta[i].val

        #------------------------------------------------------
        
        aim_mat = coord.euler_rpy_to_homogeneous_matrix(mat_value[0], 1.57 + mat_value[1], 1.57 + mat_value[2], mat_value[3], 0.179 + mat_value[4], 0.547+mat_value[5])
        
        tehta_end, _, _ = self.control_hand(aim_mat, 'left')

        self.body.theta = tehta_end[:self.body.num_joints]
        self.left_hand.theta = tehta_end[self.body.num_joints:]

        #------------------------------------------------------
        aim_mat_right = coord.euler_rpy_to_homogeneous_matrix(mat_value[6], mat_value[7], mat_value[8],mat_value[9], mat_value[10], mat_value[11])
        
        tehta_end, _, _ = self.control_part(aim_mat_right, 'right')

        self.right_hand.theta = tehta_end

        #----------------------------------------------------
        # 保存当前视图限制和视角
        xlim = self.ax.get_xlim()
        ylim = self.ax.get_ylim()
        zlim = self.ax.get_zlim()
        elev, azim = self.ax.elev, self.ax.azim  # 保存仰角和方位角

        # 清除之前的绘图
        self.ax.clear()

        # 重新绘制坐标轴和连杆
        body_mat = self.body.get_dh_matrix(self.body.theta)

        body_end = self.draw(self.base_tf['body'], body_mat)

        self.draw(body_end @ self.base_tf['left'], self.left_hand.get_dh_matrix(self.left_hand.theta))

        self.draw(body_end @ self.base_tf['right'], self.right_hand.get_dh_matrix(self.right_hand.theta))


        self.draw_coord(aim_mat)
        aim_mat_right_show = body_end @ aim_mat_right 
        self.draw_coord(aim_mat_right)


        # 恢复视图限制和视角
        self.ax.set_xlim(xlim)
        self.ax.set_ylim(ylim)
        self.ax.set_zlim(zlim)
        self.ax.view_init(elev=elev, azim=azim)  # 恢复视角

        # 更新图形
        self.fig.canvas.draw_idle()
        # self.fig.canvas.draw()

    def control_part_v(self, aim_v, part_name='body'):
        ''' 单独控制某个部分速度,输出目标角度'''
        part = None
        tf2base = self.get_part_base(part_name)[:3, :3]
        vel = np.linalg.inv(tf2base) @ aim_v[:3]
        rota = np.linalg.inv(tf2base) @ aim_v[3:]
        base_aim_v = np.array([vel[0], vel[1], vel[2], rota[0], rota[1], rota[2]])
        if part_name == "right":
            part = self.right_hand
        elif part_name == "left":
            part = self.left_hand
        elif part_name == "body":
            part = self.body

        start_theta = part.theta

        theta_limit = part.theta_limit
        
        theta = part.cal_qpos_v(start_theta, base_aim_v, theta_limit)

        return theta

    def control_part(self, aim_mat, part_name='body', ignore_angle = False):
        ''' 单独控制某个部分,输出到目标的位姿的关节角度theta 以及过程角度序列theta_can'''
        part = None
        if part_name == "right":
            part = self.right_hand
        elif part_name == "left":
            part = self.left_hand
        elif part_name == "body":
            part = self.body

        base = self.get_part_base(part_name)
        arm_aim = np.linalg.inv(base) @ aim_mat

        start_theta = part.theta
        
        theta, theta_can, get_aim = part.cal_qpos(start_theta, arm_aim, ignore_angle)

        return theta, theta_can, get_aim

    def control_hand(self,  aim_mat, hand_name='left', ignore_angle = False):
        '''身体和手作为一个整体,输出到目标的位姿的关节角度theta 以及过程角度序列theta_can'''
        hand = None
        if hand_name == "right":
            hand = self.right_hand
        elif hand_name == "left":
            hand = self.left_hand

        dh_type = self.body.get_dh_type() + hand.get_dh_type()

        base = self.get_part_base('body')
        arm_aim = np.linalg.inv(base) @ aim_mat

        def get_matrix(theta):
            theta_body = theta[:self.body.num_joints]
            theta_hand = theta[self.body.num_joints:]

            dh_boyd = self.body.get_dh_matrix(theta_body)
            dh_hand = self.left_hand.get_dh_matrix(theta_hand)
            hand_base_tf = self.base_tf[hand_name]
            dh_hand[0] = hand_base_tf @ dh_hand[0]

            dh_boyd.extend(dh_hand)

            return dh_boyd
        
        start_theta = list(self.body.theta) + list(hand.theta)

        def get_theta_limit(theta):
            theta_limit = np.concatenate((self.body.get_joint_limit(theta[:self.body.num_joints]), hand.get_joint_limit(theta[self.body.num_joints:])), axis=0)
            return theta_limit
        
        theta, theta_can, get_aim = dh.cal_qpos(start_theta, arm_aim,dh_type, get_matrix, get_theta_limit, ignore_angle)

        return theta, theta_can, get_aim


    def show(self, ax):
        # 重新绘制坐标轴和连杆
        body_mat = self.body.get_dh_matrix(self.body.theta)

        body_end = self.draw_out(self.base_tf['body'], body_mat, ax)

        left_end = self.draw_out(body_end, self.left_hand.get_dh_matrix(self.left_hand.theta), ax)

        right_end = self.draw_out(body_end, self.right_hand.get_dh_matrix(self.right_hand.theta), ax)

        return body_end, left_end, right_end


    def get_all_tf(self):
        '''获取所有关节的坐标齐次矩阵'''
        
        # 重新绘制坐标轴和连杆
        body_mat = self.body.get_dh_matrix(self.body.theta)

        body_end = self.get_aix_tf(self.base_tf['body'], body_mat)

        left_end = self.get_aix_tf(body_end[-1] @ self.base_tf['left'], self.left_hand.get_dh_matrix(self.left_hand.theta))

        right_end = self.get_aix_tf(body_end[-1] @ self.base_tf['right'], self.right_hand.get_dh_matrix(self.right_hand.theta))

        body_end.extend(left_end)

        body_end.extend(right_end)

        return body_end

    def get_eef(self):
        '''获取末端的齐次矩阵'''
        base = self.base_tf['body']

        body_mat = self.body.get_dh_matrix(self.body.theta)

        tf = base

        for i in body_mat:
            tf = tf @ i

        body_end = tf.copy()

        base = tf.copy()

        tf = base @ self.base_tf['left']
        body_mat = self.left_hand.get_dh_matrix(self.left_hand.theta)
        for i in body_mat:
            tf = tf @ i
        body_left = tf.copy()

        tf = base @ self.base_tf['right']
        body_mat = self.right_hand.get_dh_matrix(self.right_hand.theta)
        for i in body_mat:
            tf = tf @ i
        body_right = tf.copy()

        return [body_end, body_left, body_right]
    
    def get_part_base(self, part_name):
        '''获取在机身原点下的某个部件的基座矩阵'''
        if part_name == "body":
            return self.base_tf['body']
        elif part_name == "left":
            return self.get_eef()[0] @ self.base_tf['left']
        elif part_name == "right":
            return self.get_eef()[0] @ self.base_tf['right']

    def get_eef_by_name(self, part_name):
        return self.get_eef()[self.hand_id[part_name]]
    
    def get_part(self, part_name):
        hand_part = {
            "body": self.body,
            "left": self.left_hand,
            "right": self.right_hand,
        }
        return hand_part[part_name]

    def get_all_theta(self):
        return list(self.body.theta) + list(self.left_hand.theta) + list(self.right_hand.theta)



if __name__ == '__main__':
    body = Human([[-1.5707, 0, 0, -1.5707, 0], 
                  [-3.1415, 0.41, 0.0, 0, 0],
                  [0, 0.48, 0, 1.5707, 0],
                  [1.5707, 0, 0.35, 3.1415, 0]],
                 
                 [
                  [-1.5707, 0.0, 0.253, 1.5707, 0],
                  [1.5707, 0.0, 0.0, -1.5707, 0],
                  [-1.5707, 0.0, 0.3579, 3.1415, 0],
                  [-1.5707, -0.0537, 0.0, 3.1415, 0],
                  [-1.5707, -0.036, 0.2041, 1.5707, 0],
                  [-1.5707, 0.0, 0.0, -1.5707, 0],
                  [1.5707, 0.08, 0.0, 1.5707, 0]],

                 [[1.5707, 0.0, 0.253, -1.5707, 0],
                  [-1.5707, 0.0, 0.0, -1.5707, 0],
                  [-1.5707, 0.0, 0.3579, 3.1415, 0],
                  [-1.5707, 0.0537, 0.0, 3.1415, 0],
                  [-1.5707, 0.036, 0.2041, 1.5707, 0],
                  [1.5707, 0.0, 0.0, 1.5707, 0],
                  [-1.5707, 0.08, 0.0, 1.5707, 0]])
    # body = Human([
    #         [0, 0, 0.8, 0, 2],
    #         [-1.5707, 0, 0, -1.57, 0]
    #     ],
    #     [
    #         [0.0, 0.0, 0.1746, 1.5707, 0],
    #         [1.5707, 0.0, 0.0, 0, 0],
    #         [-1.5707, 0.0, 0.287, 0, 0],
    #         [1.5707, 0.018, 0.0, 3.14159, 0],
    #         [1.5707, 0.018, 0.314, 3.14159, 0],
    #         [1.5707, 0.0, 0.0, 1.57, 0],
    #         [1.5707, 0.0, 0.0, 1.57, 0]
    #     ],
    #     [
    #         [3.14159, 0.0, 0.1746, -1.5707, 0],
    #         [1.5707, 0.0, 0.0, 0, 0],
    #         [-1.5707, 0.0, 0.287, 0, 0],
    #         [1.5707, 0.018, 0.0, 3.14159, 0],
    #         [1.5707, 0.018, 0.314, 3.14159, 0],
    #         [1.5707, 0.0, 0.0, 1.57, 0],
    #         [1.5707, 0.0, 0.0, 1.57, 0]
    #     ])

    # print(body.get_all_tf())
    
    # body.show_body()

    # plt.show()

    body.show_contorl()

    plt.show()



