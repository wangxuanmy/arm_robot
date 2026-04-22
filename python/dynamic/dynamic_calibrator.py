"""
机械臂动态标定器模块
该模块实现了用于机器人动力学参数估计的标定算法。
它使用逆动力学模型和测量数据来估计机器人的动态参数。
"""

import numpy as np
import pinocchio as pin
import scipy.optimize as opt
import matplotlib.pyplot as plt
from matplotlib import rcParams
import os
import sys
import scipy


rcParams['font.size'] = 12
rcParams['axes.labelsize'] = 14
rcParams['xtick.labelsize'] = 12
rcParams['ytick.labelsize'] = 12
rcParams['legend.fontsize'] = 12

def extract_base_parameters(Y, eps=1e-10):
    """
    使用带列主元的 QR 分解提取真实的基参数（机器人动力学标定标准做法）
    返回：
        Yb: 基参数回归矩阵 (N, rank)
        idx_base: 真实的独立参数索引 (rank,)
        rank: 基参数数量
    """
    # 1. 带列主元的 QR 分解
    Q, R, P = scipy.linalg.qr(Y, mode='economic', pivoting=True)
    
    # 2. 根据 R 的对角线元素计算秩
    diag_R = np.abs(np.diag(R))
    rank = np.sum(diag_R > eps * diag_R[0])
    
    # 3. P 的前 rank 个元素，就是原 70 个参数中真正独立的列索引！
    idx_base = P[:rank]
    
    # 4. 构建基回归矩阵（直接从 Y 中抽出这些独立列）
    Yb = Y[:, idx_base]      # 形状: (N, rank)
    
    return Yb, idx_base, rank


class DynamicCalibrator:
    """
    机械臂动态标定器类，用于基于关节扭矩测量的动态标定
    
    该类实现了基于以下内容的动态参数估计：
    - 机器人动力学模型
    - 摩擦力建模（静态、粘滞、库伦）
    - 数据预处理和滤波
    - 使用最小二乘法进行参数识别
    """
    
    def __init__(self, urdf_path=None, srdf_path=None, joints_to_calibrate=None):
        """
        初始化动态标定器
        
        参数:
            urdf_path: 机器人URDF模型文件路径
            srdf_path: SRDF配置文件路径（可选）
            joints_to_calibrate: 要标定的关节名称列表，默认为None（使用所有关节）
        """
        # 模型存储
        self.model = None
        self.data = None
        self.collision_model = None
        self.collision_data = None
        self.visual_model = None
        self.visual_data = None
        self.urdf_path = urdf_path
        self.idx_base = None
        
        # 要标定的关节索引
        self.joints_to_calibrate = joints_to_calibrate if joints_to_calibrate else None
        self.joint_indices = []  # 在完整模型中的索引
        
        # 数据存储
        self.measured_torques = None
        self.joint_positions = None
        self.joint_velocities = None
        self.joint_accelerations = None
        self.gravity_vector = np.array([0, 0, -9.81])
        
        # 估计参数
        self.dynamic_parameters = None
        self.friction_parameters = None
        
        # 存储回归矩阵
        self.regression_matrix = None
        
        # 如果提供了URDF路径，则加载模型
        if urdf_path is not None:
            self.load_model_from_urdf(urdf_path, srdf_path)
    
    def load_model_from_urdf(self, urdf_path, srdf_path=None):
        """
        从URDF文件加载机器人模型
        
        参数:
            urdf_path: URDF文件路径
            srdf_path: SRDF配置文件路径（可选）
        """
        if not os.path.exists(urdf_path):
            raise FileNotFoundError(f"URDF文件不存在: {urdf_path}")
        
        # 使用Pinocchio从URDF构建模型，只加载动力学部分
        self.model = pin.buildModelFromUrdf(urdf_path)
        
        # 创建数据结构
        self.data = self.model.createData()
        
        print(f"机器人模型已从URDF加载: {urdf_path}")
        print(f"模型信息:")
        print(f"- 关节数: {self.model.njoints}")
        print(f"- 自由度数: {self.model.nv}")
        print(f"- 关节名称: {[self.model.names[i] for i in range(len(self.model.names))]}")
        
        # 确定要标定的关节索引
        if self.joints_to_calibrate is None:
            # 默认使用除了最后一个手指关节外的所有关节
            # 注意：这取决于具体的机器人结构，这里保留原逻辑但建议检查
            self.joint_indices = list(range(self.model.nv - 2))  # 排除两个手指关节
        else:
            # 根据指定的关节名称查找索引
            self.joint_indices = []
            for joint_name in self.joints_to_calibrate:
                found = False
                for idx in range(1, self.model.njoints):  # 跳过根关节 (idx=0)
                    if self.model.names[idx] == joint_name:
                        # 获取该关节在速度空间 (tangent space) 的起始索引
                        joint_id = idx
                        v_id = self.model.joints[joint_id].idx_v
                        
                        # 确保索引有效且未重复添加
                        if 0 <= v_id < self.model.nv and v_id not in self.joint_indices:
                            self.joint_indices.append(v_id)
                            found = True
                            break
                
                if not found:
                    print(f"警告: 未找到关节 '{joint_name}' 或无法映射到有效索引")
        
        # 排序以确保顺序一致
        self.joint_indices.sort()
        
        print(f"要标定的关节索引 (速度空间): {self.joint_indices}")
        print(f"要标定的关节数: {len(self.joint_indices)}")
    
    def load_data(self, positions, velocities, accelerations, torques):
        """
        为动态标定加载实验数据
        
        参数:
            positions: 关节位置 [n_samples x n_joints]
            velocities: 关节速度 [n_samples x n_joints]
            accelerations: 关节加速度 [n_samples x n_joints]
            torques: 测量的关节扭矩 [n_samples x n_joints]
        """
        self.joint_positions = np.array(positions)
        self.joint_velocities = np.array(velocities)
        self.joint_accelerations = np.array(accelerations)
        self.measured_torques = np.array(torques)
        
        # 验证所有数组长度相同
        n_samples = self.joint_positions.shape[0]
        assert self.joint_velocities.shape[0] == n_samples
        assert self.joint_accelerations.shape[0] == n_samples
        assert self.measured_torques.shape[0] == n_samples
        
        # 验证数据维度与要标定的关节数匹配
        assert self.joint_positions.shape[1] == len(self.joint_indices), \
               f"数据关节数({self.joint_positions.shape[1]})与要标定的关节数({len(self.joint_indices)})不匹配"
        
        print(f"已加载{n_samples}个样本的实验数据，包含{len(self.joint_indices)}个关节")
        
    def compute_regression_matrix(self):
        """
        计算逆动力学回归矩阵Y，使得：
        tau = Y(θ, θ̇, θ̈) * π
        
        其中π是标准的物理参数向量，包含连杆质量和惯性张量
        这里采用独立摩擦项的方法，不将摩擦项包含在回归矩阵中
        
        使用 Pinocchio 的 computeJointTorqueRegressor 来精确计算。
        """
        if self.model is None:
            raise ValueError("必须先加载机器人模型")
        
        n_samples = self.joint_positions.shape[0]
        n_joints = len(self.joint_indices)  # 实际要标定的关节数
        
        # 初始化回归矩阵
        # 行: n_samples * n_joints
        # 列: total_params
        Y_list = []

        for i in range(n_samples):
            # 直接取当前样本的状态（不需要自己填充全局向量！）
            q = self.joint_positions[i]
            dq = self.joint_velocities[i]
            ddq = self.joint_accelerations[i]

            # 一步得到【已经是最小参数集】的回归矩阵
            Yi = pin.computeJointTorqueRegressor(self.model, self.data, q, dq, ddq)
            
            Y_list.append(Yi)

        # 自动堆叠，自动保持正确列数（基参数数量）
        Y = np.vstack(Y_list)
        Yb, self.idx_base, nb = extract_base_parameters(Y)
        print(f"原始参数: 70")
        print(f"基参数数量: {nb}")  # 通常 60~66
        print(f"基回归矩阵形状: {Yb.shape}")  # (70000, nb)
        print(f"基参数索引: {self.idx_base}")

        print(Yb[0, :])
        print(f"回归矩阵维度: {Yb.shape}")
        return Yb

    def estimate_dynamic_parameters(self, regularization_factor=1e-4):
        """
        使用最小二乘法估计动态参数
        
        参数:
            regularization_factor: 正则化因子，用于数值稳定性
        """
        # 计算回归矩阵
        Y = self.compute_regression_matrix()
        
        # 存储回归矩阵
        self.regression_matrix = Y
        
        # 获取测量的扭矩数据
        n_samples, n_joints = self.joint_positions.shape
        n_joints = len(self.joint_indices)  # 实际要标定的关节数
        
        # 使用所有测量扭矩数据进行动态参数估计（包含摩擦）
        tau_all = self.measured_torques.flatten()
        
        # 求解参数: π = (Y^T*Y + lambda*I)^(-1)*Y^T*tau
        YTY = Y.T @ Y
        reg_matrix = regularization_factor * np.eye(YTY.shape[0])
        params = np.linalg.solve(YTY + reg_matrix, Y.T @ tau_all)
        
        self.dynamic_parameters = params
        print("动态参数估计成功")
        
        # 输出回归矩阵Y的构造方式
        print(f"\n回归矩阵Y构造说明:")
        print(f"- 形状: {Y.shape}")
        print(f"- 行: 每个样本的每个关节 (共{n_samples * n_joints}行)")
        print(f"- 列: 每个连杆的10个惯性参数 (共{n_joints * 10}列)")
        print(f"- 每个连杆的参数依次为: 质量, 质心x, 质心y, 质心z, Ixx, Ixy, Ixz, Iyy, Iyz, Izz")
        print(f"- 摩擦项将在后续步骤中处理")
        
        return params
    
    def estimate_dynamic_parameters_with_velocity_filter(self, velocity_threshold=0.5, exclude_low_speed=True, regularization_factor=1e-3):
        """
        使用速度筛选估计动态参数，优先使用高速数据排除摩擦影响
        
        参数:
            velocity_threshold: 速度阈值，用于筛选数据
            exclude_low_speed: 是否排除低速数据（True表示只使用高于阈值的数据）
            regularization_factor: 正则化因子，用于数值稳定性
        """
        # 计算回归矩阵
        Y = self.compute_regression_matrix()
        
        # 存储回归矩阵
        self.regression_matrix = Y
        
        # 获取测量的扭矩数据
        n_samples, n_joints = self.joint_positions.shape
        n_joints = len(self.joint_indices)  # 实际要标定的关节数
        
        # 根据速度筛选数据
        if exclude_low_speed:
            # 只使用高速数据（速度大于阈值）
            mask = np.all(np.abs(self.joint_velocities) > velocity_threshold, axis=1)
        else:
            # 只使用低速数据（速度小于等于阈值）
            mask = np.all(np.abs(self.joint_velocities) <= velocity_threshold, axis=1)
        
        # 回归矩阵的行数是 n_samples * n_joints，所以需要扩展mask以适应回归矩阵的维度
        # 对于每个样本，有n_joints个行，所以我们要将mask扩展成每n_joints个重复一次
        expanded_mask = np.repeat(mask, n_joints)
        
        # 应用掩码筛选数据
        Y_filtered = Y[expanded_mask, :]
        tau_filtered = self.measured_torques[mask, :].flatten()
        
        # 检查是否有足够的数据
        if Y_filtered.shape[0] == 0:
            print("警告: 速度筛选后没有可用数据，将使用全部数据")
            Y_filtered = Y
            tau_filtered = self.measured_torques.flatten()
        
        print(f"使用 {Y_filtered.shape[0]} 个样本进行动态参数估计（筛选条件: 速度 {'>' if exclude_low_speed else '<='} {velocity_threshold} rad/s）")
        
        # 求解参数: π = (Y^T*Y + lambda*I)^(-1)*Y^T*tau
        YTY = Y_filtered.T @ Y_filtered
        reg_matrix = regularization_factor * np.eye(YTY.shape[0])
        params = np.linalg.solve(YTY + reg_matrix, Y_filtered.T @ tau_filtered)


        self.dynamic_parameters = params
        print("动态参数估计成功")
        
        # 输出回归矩阵Y的构造方式
        print(f"\n回归矩阵Y构造说明:")
        print(f"- 形状: {Y_filtered.shape}")
        print(f"- 行: 每个样本的每个关节 (共{Y_filtered.shape[0]}行)")
        print(f"- 列: 每个连杆的10个惯性参数 (共{n_joints * 10}列)")
        print(f"- 每个连杆的参数依次为: 质量, 质心x, 质心y, 质心z, Ixx, Ixy, Ixz, Iyy, Iyz, Izz")
        print(f"- 摩擦项将在后续步骤中处理")
        
        return params
    
    def estimate_friction_parameters(self):
        """
        估计每个关节的摩擦参数（库伦摩擦和粘滞摩擦）
        这个函数现在在动态参数估计完成后调用
        """
        n_joints = len(self.joint_indices)  # 实际要标定的关节数
        friction_params = []
        
        # 使用标定好的动态参数预测理论扭矩
        predicted_torques = self.simulate_dynamics_only(
            self.joint_positions, 
            self.joint_velocities, 
            self.joint_accelerations
        )
        
        # 计算残差（测量值 - 理论值），这主要是摩擦力
        residual_torques = self.measured_torques - predicted_torques
        
        for j in range(n_joints):
            # 提取关节j的速度和残差扭矩
            vel = self.joint_velocities[:, j]
            residual_tau = residual_torques[:, j]
            
            # 使用简单方法估计摩擦
            # 假设: tau_friction = Fc*sign(w) + Fv*w
            # 其中 Fc=库伦摩擦, Fv=粘滞摩擦系数
            
            # 从残差数据中估计库伦摩擦
            # 在接近零速度时，残差主要由库伦摩擦决定
            low_speed_threshold = 0.05  # rad/s
            low_speed_mask = np.abs(vel) < low_speed_threshold
            
            # 从低速数据估计库伦摩擦
            positive_velocities = vel[(vel > 0) & low_speed_mask]
            negative_velocities = vel[(vel < 0) & low_speed_mask]
            
            if len(positive_velocities) > 0:
                pos_residuals = residual_tau[(vel > 0) & low_speed_mask]
                coulomb_pos = np.median(pos_residuals) if len(pos_residuals) > 0 else 0
            else:
                coulomb_pos = 0
                
            if len(negative_velocities) > 0:
                neg_residuals = residual_tau[(vel < 0) & low_speed_mask]
                coulomb_neg = np.median(neg_residuals) if len(neg_residuals) > 0 else 0
            else:
                coulomb_neg = 0
            
            # 计算库伦摩擦系数
            coulomb_friction = abs(coulomb_pos - coulomb_neg) / 2 if coulomb_pos != 0 or coulomb_neg != 0 else 0
            
            # 估计粘滞摩擦系数 - 使用线性回归
            high_speed_mask = np.abs(vel) > 0.2  # rad/s
            if np.any(high_speed_mask):
                A = np.vstack([vel[high_speed_mask], np.ones(np.sum(high_speed_mask))]).T
                try:
                    coeffs, residuals, rank, s = np.linalg.lstsq(A, residual_tau[high_speed_mask], rcond=None)
                    viscous_coeff = abs(coeffs[0])  # 确保为正值
                except:
                    viscous_coeff = 0
            else:
                viscous_coeff = 0
            
            # 静摩擦通常略大于库伦摩擦
            static_friction = coulomb_friction * 1.2
            
            friction_params.append({
                'coulomb': abs(coulomb_friction),
                'viscous': viscous_coeff,
                'static': abs(static_friction),
                'offset': 0  # 重力或其他偏移已由动态参数处理
            })
        
        self.friction_parameters = friction_params
        print("摩擦参数估计成功")
        return friction_params

    def estimate_friction_parameters_using_residuals(self, velocity_threshold=0.05):
        """
        使用残差估计摩擦参数，基于已估计的动态参数计算残差
        这个函数现在在动态参数估计完成后调用
        
        参数:
            velocity_threshold: 低速阈值，用于摩擦参数估计
        """
        n_joints = len(self.joint_indices)  # 实际要标定的关节数
        friction_params = []
        
        # 使用标定好的动态参数预测理论扭矩
        predicted_torques = self.simulate_dynamics_only(
            self.joint_positions, 
            self.joint_velocities, 
            self.joint_accelerations
        )
        
        # 计算残差（测量值 - 理论值），这主要是摩擦力和其他未建模动态
        residual_torques = self.measured_torques - predicted_torques

        
        for j in range(n_joints):
            # 提取关节j的速度和残差扭矩
            vel = self.joint_velocities[:, j]
            residual_tau = residual_torques[:, j]
            
            # 使用低速数据估计摩擦参数
            low_speed_mask = np.abs(vel) <= velocity_threshold
            
            # 只使用低速数据来估计摩擦参数
            vel_low_speed = vel[low_speed_mask]
            residual_low_speed = residual_tau[low_speed_mask]
            
            if len(vel_low_speed) == 0:
                print(f"警告: 关节 {j} 没有足够的低速数据用于摩擦参数估计")
                friction_params.append({
                    'coulomb': 0,
                    'viscous': 0,
                    'static': 0,
                    'offset': 0
                })
                continue
            
            # 估计库伦摩擦：使用低速数据中的最大绝对值
            positive_vel_mask = vel_low_speed > 0
            negative_vel_mask = vel_low_speed < 0
            
            if np.any(positive_vel_mask):
                coulomb_pos = np.median(residual_low_speed[positive_vel_mask])
            else:
                coulomb_pos = 0
                
            if np.any(negative_vel_mask):
                coulomb_neg = np.median(residual_low_speed[negative_vel_mask])
            else:
                coulomb_neg = 0
            
            # 计算库伦摩擦系数
            coulomb_friction = abs(coulomb_pos - coulomb_neg) / 2 if coulomb_pos != 0 or coulomb_neg != 0 else 0
            
            # 估计粘滞摩擦系数 - 使用整个数据集的速度相关性
            if len(vel) > 0:
                # 线性回归拟合速度与残差之间的关系
                A = np.vstack([vel, np.ones(len(vel))]).T
                coeffs, residuals, rank, s = np.linalg.lstsq(A, residual_tau, rcond=None)
                viscous_coeff = abs(coeffs[0])  # 确保为正值
            else:
                viscous_coeff = 0
            
            # 静摩擦通常略大于库伦摩擦
            static_friction = coulomb_friction * 1.2
            
            friction_params.append({
                'coulomb': abs(coulomb_friction),
                'viscous': viscous_coeff,
                'static': abs(static_friction),
                'offset': 0  # 重力或其他偏移已由动态参数处理
            })
        
        self.friction_parameters = friction_params
        print("摩擦参数估计成功")
        return friction_params

    def simulate_dynamics_only(self, positions, velocities, accelerations):
        """
        仅模拟动态部分（惯性、科里奥利、重力等），不包括摩擦
        
        参数:
            positions: 关节位置
            velocities: 关节速度  
            accelerations: 关节加速度
            
        返回:
            predicted_torques: 模型预测的动态扭矩（不含摩擦）
        """
        if self.dynamic_parameters is None:
            raise ValueError("动态参数尚未估计。先运行estimate_dynamic_parameters。")
        
        n_samples = positions.shape[0]
        n_joints = len(self.joint_indices)  # 实际要标定的关节数
        predicted_torques = np.zeros((n_samples, n_joints))
        
        for i in range(n_samples):
            # 直接获取当前时刻状态
            q = positions[i]
            dq = velocities[i]
            ddq = accelerations[i]

            # --------------------------
            # ✅ 关键：Pinocchio 直接计算回归矩阵（最小参数集版本）
            # --------------------------
            Y_i = pin.computeJointTorqueRegressor(self.model, self.data, q, dq, ddq)

            Y_i = Y_i[:, self.idx_base]
            # --------------------------
            # ✅ 直接用最小参数集计算力矩：τ = Y · π
            # --------------------------
            dyn_torques = Y_i @ self.dynamic_parameters

            # 填入结果
            predicted_torques[i, :] = dyn_torques

        return predicted_torques
    
    
    
    def get_inertia_matrix(self, joint_positions):
        """
        获取在给定关节位置下的惯性矩阵
        
        参数:
            joint_positions: 关节位置向量 (仅包含要标定的关节)
        
        返回:
            惯性矩阵 (仅包含要标定关节的子矩阵)
        """
        if self.model is None:
            raise ValueError("必须先加载机器人模型")
        
        # 构建完整模型的q向量
        q_full = np.zeros(self.model.nq)
        for idx, model_idx in enumerate(self.joint_indices):
            if idx < len(joint_positions):
                q_full[model_idx] = joint_positions[idx]
        
        # 计算完整惯性矩阵
        M_full = pin.crba(self.model, self.data, q_full)
        
        # 提取要标定关节的子矩阵
        n_joints = len(self.joint_indices)
        M_sub = np.zeros((n_joints, n_joints))
        
        for i, idx_i in enumerate(self.joint_indices):
            for j, idx_j in enumerate(self.joint_indices):
                if idx_i < M_full.shape[0] and idx_j < M_full.shape[1]:
                    M_sub[i, j] = M_full[idx_i, idx_j]
        
        return M_sub
    
    def simulate_model(self, positions, velocities, accelerations):
        """
        模拟标定后的模型以预测扭矩
        
        参数:
            positions: 关节位置
            velocities: 关节速度  
            accelerations: 关节加速度
            
        返回:
            predicted_torques: 模型预测的关节扭矩
        """
        if self.dynamic_parameters is None or self.friction_parameters is None:
            raise ValueError("参数尚未估计。先运行estimate_dynamic_parameters和estimate_friction_parameters。")
        
        n_samples = positions.shape[0]
        n_joints = len(self.joint_indices)  # 实际要标定的关节数
        predicted_torques = np.zeros((n_samples, n_joints))
        
        n_params_per_body = 10
        
        # 预分配
        q_full = pin.neutral(self.model)
        v_full = np.zeros(self.model.nv)
        a_full = np.zeros(self.model.nv)
        
        for i in range(n_samples):
           # 直接获取当前时刻状态
            q = positions[i]
            dq = velocities[i]
            ddq = accelerations[i]

            # --------------------------
            # ✅ 关键：Pinocchio 直接计算回归矩阵（最小参数集版本）
            # --------------------------
            Y_i = pin.computeJointTorqueRegressor(self.model, self.data, q, dq, ddq)

            Y_i = Y_i[:, self.idx_base]
            # --------------------------
            # ✅ 直接用最小参数集计算力矩：τ = Y · π
            # --------------------------
            dyn_torques = Y_i @ self.dynamic_parameters
            
            
            # 摩擦补偿
            friction_torques = np.zeros(n_joints)
            for j_idx in range(n_joints):
                friction_params = self.friction_parameters[j_idx]
                vel = velocities[i, j_idx]
                
                if abs(vel) > 1e-3:
                    friction_torques[j_idx] = friction_params['coulomb'] * np.sign(vel) + \
                                             friction_params['viscous'] * vel
                else:
                    # 静止摩擦处理
                    net_force = dyn_torques[j_idx]
                    if abs(net_force) <= friction_params['static']:
                        friction_torques[j_idx] = -net_force
                    else:
                        sign = 1 if net_force > 0 else -1
                        friction_torques[j_idx] = sign * friction_params['coulomb']
                        
            predicted_torques[i, :] = dyn_torques + friction_torques
            
        return predicted_torques
    
    def evaluate_fit(self):
        """
        评估识别模型的质量
        """
        if self.dynamic_parameters is None or self.friction_parameters is None:
            raise ValueError("参数尚未估计。")
        
        n_joints = len(self.joint_indices)  # 实际要标定的关节数
        predicted_torques = self.simulate_model(
            self.joint_positions, 
            self.joint_velocities, 
            self.joint_accelerations
        )
        
        measured_torques = self.measured_torques
        
        # 计算拟合指标
        rmse = np.sqrt(np.mean((measured_torques - predicted_torques)**2, axis=0))
        mae = np.mean(np.abs(measured_torques - predicted_torques), axis=0)
        
        # 方差解释率 (VAF)
        var_exp = 100 * (1 - np.var(measured_torques - predicted_torques, axis=0) / 
                         np.var(measured_torques, axis=0))
        
        print("模型评估结果:")
        print(f"关节\tRMSE (Nm)\tMAE (Nm)\tVAF (%)")
        for i in range(len(rmse)):
            print(f"{i+1}\t{rmse[i]:.4f}\t\t{mae[i]:.4f}\t\t{var_exp[i]:.2f}%")
        
        return rmse, mae, var_exp
    
    def plot_results(self):
        """
        绘制测量扭矩与预测扭矩的比较图
        """
        if self.dynamic_parameters is None or self.friction_parameters is None:
            raise ValueError("参数尚未估计。")
        
        n_joints = len(self.joint_indices)  # 实际要标定的关节数
        predicted_torques = self.simulate_model(
            self.joint_positions, 
            self.joint_velocities, 
            self.joint_accelerations
        )
        
        time_axis = np.arange(self.measured_torques.shape[0])
        
        fig, axes = plt.subplots(n_joints, 1, figsize=(12, 4*n_joints))
        if n_joints == 1:
            axes = [axes]
        
        for j in range(n_joints):
            axes[j].plot(time_axis, self.measured_torques[:, j], label='测量值', linewidth=1.5)
            axes[j].plot(time_axis, predicted_torques[:, j], label='预测值', linestyle='--', linewidth=1.5)
            axes[j].set_title(f'关节 {j+1}: 扭矩比较 (对应模型关节{self.joint_indices[j]})')
            axes[j].set_xlabel('样本')
            axes[j].set_ylabel('扭矩 (Nm)')
            axes[j].grid(True, alpha=0.3)
            axes[j].legend()
        
        plt.tight_layout()
        plt.savefig('torque_comparison.png', dpi=150, bbox_inches='tight')
        print("扭矩比较图已保存为 torque_comparison.png")
        
        # 绘制摩擦特性
        if self.friction_parameters is not None:
            fig, axes = plt.subplots(2, n_joints, figsize=(14, 8))
            if n_joints == 1:
                axes = axes.reshape(-1, 1)
            
            for j in range(n_joints):
                vel = self.joint_velocities[:, j]
                tau = self.measured_torques[:, j]
                
                # 绘制摩擦与速度的关系
                axes[0, j].scatter(vel, tau, alpha=0.5, s=10)
                axes[0, j].set_title(f'关节 {j+1}: 扭矩 vs 速度 (测量值)')
                axes[0, j].set_xlabel('速度 (rad/s)')
                axes[0, j].set_ylabel('扭矩 (Nm)')
                axes[0, j].grid(True, alpha=0.3)
                
                # 绘制残差
                residual = self.measured_torques[:, j] - predicted_torques[:, j]
                axes[1, j].plot(time_axis, residual)
                axes[1, j].set_title(f'关节 {j+1}: 残余扭矩')
                axes[1, j].set_xlabel('样本')
                axes[1, j].set_ylabel('残差 (Nm)')
                axes[1, j].grid(True, alpha=0.3)
        
            plt.tight_layout()
            plt.savefig('friction_analysis.png', dpi=150, bbox_inches='tight')
            print("摩擦分析图已保存为 friction_analysis.png")

    def get_regression_matrix(self):
        """
        获取回归矩阵
        """
        if self.regression_matrix is None:
            print("回归矩阵尚未计算，请先运行estimate_dynamic_parameters")
            return None
        return self.regression_matrix
        
    def get_dynamic_params(self):
        """
        获取动态参数向量
        """
        if self.dynamic_parameters is None:
            print("动态参数尚未估计，请先运行estimate_dynamic_parameters")
            return None
        return self.dynamic_parameters
    
    def _compute_idim_row(self, q, v, a, joint_id, col_start):
        """
        使用IDIM方法计算回归矩阵的一行
        这是更精确的方法，基于每个连杆的物理参数对关节力矩的贡献
        """
        # 初始化这一行的值
        row = np.zeros(70)  # 7个关节 * 10个参数
        
        # 对每个连杆计算其参数对该关节的贡献
        for body_id in range(len(self.joint_indices)):  # 对每个连杆
            # 计算该连杆的物理参数对指定关节的贡献
            col_offset = col_start + body_id * 10
            
            # 获取当前连杆的雅可比矩阵
            link_id = body_id + 1  # 跳过根节点
            if link_id < len(self.model.names):
                # 创建临时数据结构
                temp_data = self.model.createData()
                
                # 计算正向运动学
                pin.forwardKinematics(self.model, temp_data, q, v)
                pin.computeJointJacobians(self.model, temp_data, q)
                
                # 获取关节的雅可比矩阵
                try:
                    J = pin.getJointJacobian(self.model, temp_data, link_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
                    
                    # 提取平移部分（前3行）和旋转部分（后3行）
                    J_trans = J[:3, :]
                    J_rot = J[3:, :]
                    
                    # 计算每个参数的贡献
                    # 这里使用简化的IDIM公式
                    if col_offset < len(row) and link_id-1 < len(a):
                        # 质量项贡献
                        row[col_offset] = a[link_id-1] if link_id-1 < len(a) else 0
                        
                        # 质心项贡献
                        if link_id-1 < len(v):
                            row[col_offset+1] = v[link_id-1]*v[link_id-1]  # 质心x项
                            row[col_offset+2] = v[link_id-1]*a[link_id-1]  # 质心y项
                            row[col_offset+3] = np.sin(q[link_id-1])  # 质心z项（重力项）
                            
                            # 惯性张量项贡献
                            row[col_offset+4] = a[link_id-1]*a[link_id-1]  # Ixx
                            row[col_offset+5] = v[link_id-1]*a[link_id-1]  # Ixy
                            row[col_offset+6] = q[link_id-1]*v[link_id-1]  # Ixz
                            row[col_offset+7] = a[link_id-1]*v[link_id-1]  # Iyy
                            row[col_offset+8] = np.cos(q[link_id-1])  # Iyz
                            row[col_offset+9] = q[link_id-1]*q[link_id-1]  # Izz
                except:
                    # 如果获取雅可比失败，使用默认值
                    if col_offset < len(row):
                        row[col_offset] = a[joint_id] if joint_id < len(a) else 0
                        if joint_id < len(v):
                            row[col_offset+1] = v[joint_id]*v[joint_id]
                            row[col_offset+2] = v[joint_id]*a[joint_id]
                            row[col_offset+3] = np.sin(q[joint_id])
                            row[col_offset+4] = a[joint_id]*a[joint_id]
                            row[col_offset+5] = v[joint_id]*a[joint_id]
                            row[col_offset+6] = q[joint_id]*v[joint_id]
                            row[col_offset+7] = a[joint_id]*v[joint_id]
                            row[col_offset+8] = np.cos(q[joint_id])
                            row[col_offset+9] = q[joint_id]*q[joint_id]
        
        return row


def main():
    """
    演示动态标定过程的主函数
    """
    print("机械臂动态标定器演示")
    print("="*50)
    
    # 检查所需库
    required_libs = ['numpy', 'pinocchio', 'scipy', 'matplotlib']
    missing_libs = []
    
    for lib in required_libs:
        try:
            __import__(lib)
        except ImportError:
            missing_libs.append(lib)
    
    if missing_libs:
        print(f"缺少所需库: {missing_libs}")
        return
    
    print("\n1. 初始化标定器...")
    # 指定要标定的关节
    joints_to_calibrate = ['openarm_joint1', 'openarm_joint2', 'openarm_joint3', 
                          'openarm_joint4', 'openarm_joint5', 'openarm_joint6', 'openarm_joint7']
    calibrator = DynamicCalibrator(joints_to_calibrate=joints_to_calibrate)
    
    print("\n2. 生成示例数据...")
    # 为演示生成合成数据
    n_samples = 1000
    n_joints = len(joints_to_calibrate)  # 7个关节
    
    # 创建合成轨迹数据
    t = np.linspace(0, 10, n_samples)
    positions = np.zeros((n_samples, n_joints))
    velocities = np.zeros((n_samples, n_joints))
    accelerations = np.zeros((n_samples, n_joints))
    
    for j in range(n_joints):
        # 每个关节的复合正弦轨迹
        freq1 = 0.2 + j*0.05  # 基频
        freq2 = 0.5 + j*0.1   # 高频成分
        amp1 = 0.5 + j*0.05   # 低频幅度
        amp2 = 0.2 + j*0.02   # 高频幅度
        
        positions[:, j] = (
            amp1 * np.sin(freq1 * t) + 
            amp2 * np.sin(freq2 * t) * 0.5
        )
        velocities[:, j] = (
            amp1 * freq1 * np.cos(freq1 * t) + 
            amp2 * freq2 * np.cos(freq2 * t) * 0.5
        )
        accelerations[:, j] = (
            -amp1 * freq1**2 * np.sin(freq1 * t) - 
            amp2 * freq2**2 * np.sin(freq2 * t) * 0.5
        )
    
    # 添加一些噪声使数据更真实
    noise_level = 0.01
    velocities += np.random.normal(0, noise_level, velocities.shape)
    accelerations += np.random.normal(0, noise_level*2, accelerations.shape)
    
    # 基于简单模型创建合成测量扭矩
    measured_torques = np.zeros((n_samples, n_joints))
    for i in range(n_samples):
        for j in range(n_joints):
            # 简单动力学模型: tau = m*a + b*v*|v| + g*sin(q) + friction
            inertia_part = (0.8 + j*0.1) * accelerations[i, j]  # 不同关节不同惯性
            cor_centr_part = 0.1 * velocities[i, j] * abs(velocities[i, j])  # 科里奥利/离心
            gravity_part = (0.3 + j*0.05) * np.sin(positions[i, j])  # 重力效应
            
            # 摩擦部分
            if velocities[i, j] > 0:
                friction_part = 0.2 + 0.05 * velocities[i, j]  # 库伦+粘滞摩擦
            elif velocities[i, j] < 0:
                friction_part = -0.2 + 0.05 * velocities[i, j]  # 库伦+粘滞摩擦
            else:
                friction_part = 0  # 静止状态
                
            measured_torques[i, j] = inertia_part + cor_centr_part + gravity_part + friction_part
    
    # 添加测量噪声
    measured_torques += np.random.normal(0, 0.05, measured_torques.shape)
    
    calibrator.load_data(positions, velocities, accelerations, measured_torques)
    
    print("\n3. 估计动态参数...")
    dyn_params = calibrator.estimate_dynamic_parameters()
    
    print("\n4. 估计摩擦参数...")
    friction_params = calibrator.estimate_friction_parameters()
    
    print("\n5. 评估模型拟合...")
    calibrator.evaluate_fit()
    
    print("\n6. 输出回归矩阵信息...")
    reg_matrix = calibrator.get_regression_matrix()
    if reg_matrix is not None:
        print(f"回归矩阵形状: {reg_matrix.shape}")
        print(f"回归矩阵前几行前几列:\n{reg_matrix[:5, :5]}")  # 显示前5行5列
    
    print("\n7. 输出参数向量...")
    params = calibrator.get_dynamic_params()
    if params is not None:
        print(f"参数向量长度: {len(params)}")
        print(f"参数向量前10个值: {params[:10]}")
    
    print("\n8. 输出惯性矩阵示例...")
    # 计算初始位置下的惯性矩阵
    initial_pos = positions[0, :]  # 取所有关节的初始位置
    inertia_matrix = calibrator.get_inertia_matrix(initial_pos)
    print(f"初始位置下的惯性矩阵:\n{inertia_matrix}")
    
    print("\n8. 输出回归矩阵Y的构造方式和参数向量详情...")
    print(f"回归矩阵Y的构造方式:")
    print(f"- Y矩阵的每一行对应一个时刻的一个关节的动力学方程")
    print(f"- Y[i*n_joints+j, k] 表示第i个样本时刻第j个关节的第k个动力学参数的系数")
    print(f"- Y矩阵的列按连杆分组，每组包含10个参数: [质量, 质心x, 质心y, 质心z, Ixx, Ixy, Ixz, Iyy, Iyz, Izz]")
    print(f"- 摩擦项独立处理，不包含在回归矩阵中")
    
    print(f"\n参数向量π的含义:")
    print(f"- π[k] 对应Y矩阵中第k列的系数，代表特定的动力学参数")
    print(f"- 每个连杆j有10个连续的参数: [π[j*10], π[j*10+1], ..., π[j*10+9]]")
    print(f"- 分别对应: 质量, 质心x, 质心y, 质心z, Ixx, Ixy, Ixz, Iyy, Iyz, Izz")
    
    print("\n动力学方程:")
    print("τ = Y(θ, θ̇, θ̈) × π + τ_friction")
    print("其中:")
    print("τ - 关节扭矩向量")
    print("Y - 回归矩阵，元素为关节位置、速度、加速度的函数")
    print("π - 动态参数向量（物理参数）")
    print("τ_friction - 摩擦力矩向量 (通过独立模型计算)")
    
    print("\n8. 绘制结果...")
    # 注意：在无GUI环境中，绘图可能会失败
    try:
        calibrator.plot_results()
    except Exception as e:
        print(f"绘图失败（可能是GUI后端问题）: {e}")
        print("结果计算成功，但跳过可视化。")
    
    print("\n动态标定演示完成!")


if __name__ == "__main__":
    main()