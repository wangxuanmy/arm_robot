import numpy as np


def matrix_to_quaternion(R):
    """
    将旋转矩阵转换为四元数
    
    参数:
    R: 3x3 旋转矩阵
    
    返回:
    q: [w, x, y, z] 四元数
    """
    trace = np.trace(R)
    if trace > 0:
        S = np.sqrt(trace + 1.0) * 2
        w = 0.25 * S
        x = (R[2, 1] - R[1, 2]) / S
        y = (R[0, 2] - R[2, 0]) / S
        z = (R[1, 0] - R[0, 1]) / S
    elif (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
        S = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        w = (R[2, 1] - R[1, 2]) / S
        x = 0.25 * S
        y = (R[0, 1] + R[1, 0]) / S
        z = (R[0, 2] + R[2, 0]) / S
    elif R[1, 1] > R[2, 2]:
        S = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        w = (R[0, 2] - R[2, 0]) / S
        x = (R[0, 1] + R[1, 0]) / S
        y = 0.25 * S
        z = (R[1, 2] + R[2, 1]) / S
    else:
        S = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        w = (R[1, 0] - R[0, 1]) / S
        x = (R[0, 2] + R[2, 0]) / S
        y = (R[1, 2] + R[2, 1]) / S
        z = 0.25 * S
    return np.array([w, x, y, z])


def quaternion_to_matrix(q):
    """
    将四元数转换为旋转矩阵
    
    参数:
    q: [w, x, y, z] 四元数
    
    返回:
    R: 3x3 旋转矩阵
    """
    w, x, y, z = q
    return np.array([
        [1 - 2*y*y - 2*z*z, 2*x*y - 2*w*z, 2*x*z + 2*w*y],
        [2*x*y + 2*w*z, 1 - 2*x*x - 2*z*z, 2*y*z - 2*w*x],
        [2*x*z - 2*w*y, 2*y*z + 2*w*x, 1 - 2*x*x - 2*y*y]
    ])


def quaternion_multiply(q1, q2):
    """
    四元数乘法
    
    参数:
    q1, q2: [w, x, y, z] 四元数
    
    返回:
    q: [w, x, y, z] 四元数乘积
    """
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    w = w1*w2 - x1*x2 - y1*y2 - z1*z2
    x = w1*x2 + x1*w2 + y1*z2 - z1*y2
    y = w1*y2 - x1*z2 + y1*w2 + z1*x2
    z = w1*z2 + x1*y2 - y1*x2 + z1*w2
    return np.array([w, x, y, z])


def quaternion_conjugate(q):
    """
    四元数共轭
    
    参数:
    q: [w, x, y, z] 四元数
    
    返回:
    q*: [w, -x, -y, -z] 四元数共轭
    """
    return np.array([q[0], -q[1], -q[2], -q[3]])


def quaternion_error(q1, q2):
    """
    计算两个四元数之间的误差
    
    参数:
    q1, q2: [w, x, y, z] 四元数
    
    返回:
    误差向量
    """
    # 计算 q_err = q2 * conjugate(q1)
    q_conj = quaternion_conjugate(q1)
    q_err = quaternion_multiply(q2, q_conj)
    
    # 将四元数误差转换为轴角表示，只取向量部分
    # 这给出了旋转轴和角度的一半正弦值
    if q_err[0] >= 0:  # w分量
        return 2 * q_err[1:]  # 返回[x, y, z]部分的2倍
    else:
        return -2 * q_err[1:]  # 返回负值


def quaternion_to_rotation_vector(q):
    """
    将四元数转换为旋转矢量
    
    参数:
    q: 单位四元数 [w, x, y, z]
    
    返回:
    旋转矢量 [rx, ry, rz]
    """
    w, x, y, z = q
    # 确保四元数是单位四元数
    norm = np.sqrt(w*w + x*x + y*y + z*z)
    if norm != 0:
        w, x, y, z = w/norm, x/norm, y/norm, z/norm
    
    # 计算旋转角度
    angle = 2 * np.arccos(w)
    
    # 避免在角度接近0时的数值问题
    sin_half_angle = np.sqrt(1 - w*w)
    
    if sin_half_angle < 1e-10:
        return np.array([0, 0, 0])
    
    # 计算旋转轴
    axis = np.array([x, y, z]) / sin_half_angle
    
    # 返回旋转矢量
    return axis * angle