#include "quaternion_utils.h"
#include <Eigen/Dense>
#include <cmath>

namespace arm_robot {


/**
 * 将旋转矩阵转换为四元数
 * 
 * 参数:
 * R: 3x3 旋转矩阵
 * 
 * 返回:
 * q: [w, x, y, z] 四元数
 */
Eigen::Vector4d matrix_to_quaternion(const Eigen::Matrix3d& R) {
    double trace = R(0, 0) + R(1, 1) + R(2, 2);
    double w, x, y, z;

    if (trace > 0) {
        double S = sqrt(trace + 1.0) * 2;
        w = 0.25 * S;
        x = (R(2, 1) - R(1, 2)) / S;
        y = (R(0, 2) - R(2, 0)) / S;
        z = (R(1, 0) - R(0, 1)) / S;
    } else if ((R(0, 0) > R(1, 1)) && (R(0, 0) > R(2, 2))) {
        double S = sqrt(1.0 + R(0, 0) - R(1, 1) - R(2, 2)) * 2;
        w = (R(2, 1) - R(1, 2)) / S;
        x = 0.25 * S;
        y = (R(0, 1) + R(1, 0)) / S;
        z = (R(0, 2) + R(2, 0)) / S;
    } else if (R(1, 1) > R(2, 2)) {
        double S = sqrt(1.0 + R(1, 1) - R(0, 0) - R(2, 2)) * 2;
        w = (R(0, 2) - R(2, 0)) / S;
        x = (R(0, 1) + R(1, 0)) / S;
        y = 0.25 * S;
        z = (R(1, 2) + R(2, 1)) / S;
    } else {
        double S = sqrt(1.0 + R(2, 2) - R(0, 0) - R(1, 1)) * 2;
        w = (R(1, 0) - R(0, 1)) / S;
        x = (R(0, 2) + R(2, 0)) / S;
        y = (R(1, 2) + R(2, 1)) / S;
        z = 0.25 * S;
    }

    return Eigen::Vector4d(w, x, y, z);
}

/**
 * 将四元数转换为旋转矩阵
 * 
 * 参数:
 * q: [w, x, y, z] 四元数
 * 
 * 返回:
 * R: 3x3 旋转矩阵
 */
Eigen::Matrix3d quaternion_to_matrix(const Eigen::Vector4d& q) {
    double w = q(0), x = q(1), y = q(2), z = q(3);
    
    Eigen::Matrix3d R;
    R << 1 - 2*y*y - 2*z*z, 2*x*y - 2*w*z, 2*x*z + 2*w*y,
            2*x*y + 2*w*z, 1 - 2*x*x - 2*z*z, 2*y*z - 2*w*x,
            2*x*z - 2*w*y, 2*y*z + 2*w*x, 1 - 2*x*x - 2*y*y;
    
    return R;
}

/**
 * 四元数乘法
 * 
 * 参数:
 * q1, q2: [w, x, y, z] 四元数
 * 
 * 返回:
 * q: [w, x, y, z] 四元数乘积
 */
Eigen::Vector4d quaternion_multiply(const Eigen::Vector4d& q1, const Eigen::Vector4d& q2) {
    double w1 = q1(0), x1 = q1(1), y1 = q1(2), z1 = q1(3);
    double w2 = q2(0), x2 = q2(1), y2 = q2(2), z2 = q2(3);
    
    double w = w1*w2 - x1*x2 - y1*y2 - z1*z2;
    double x = w1*x2 + x1*w2 + y1*z2 - z1*y2;
    double y = w1*y2 - x1*z2 + y1*w2 + z1*x2;
    double z = w1*z2 + x1*y2 - y1*x2 + z1*w2;
    
    return Eigen::Vector4d(w, x, y, z);
}

/**
 * 四元数共轭
 * 
 * 参数:
 * q: [w, x, y, z] 四元数
 * 
 * 返回:
 * q*: [w, -x, -y, -z] 四元数共轭
 */
Eigen::Vector4d quaternion_conjugate(const Eigen::Vector4d& q) {
    return Eigen::Vector4d(q(0), -q(1), -q(2), -q(3));
}

/**
 * 计算两个四元数之间的误差
 * 
 * 参数:
 * q1, q2: [w, x, y, z] 四元数
 * 
 * 返回:
 * 误差向量
 */
Eigen::Vector4d quaternion_error(const Eigen::Vector4d& q1, const Eigen::Vector4d& q2) {
    // 计算 q_err = q2 * conjugate(q1)
    Eigen::Vector4d q_conj = quaternion_conjugate(q1);
    Eigen::Vector4d q_err = quaternion_multiply(q2, q_conj);
    
    // 将四元数误差转换为轴角表示，只取向量部分
    // 这给出了旋转轴和角度的一半正弦值
    Eigen::Vector4d result = Eigen::Vector4d::Zero(); // 初始化为零向量
    if (q_err(0) >= 0) {  // w分量
        result.segment(1, 3) = 2 * q_err.segment(1, 3);  // 返回[x, y, z]部分的2倍
    } else {
        result.segment(1, 3) = -2 * q_err.segment(1, 3);  // 返回负值
    }
    return result;
}

/**
 * 将四元数转换为旋转矢量
 * 
 * 参数:
 * q: 单位四元数 [w, x, y, z]
 * 
 * 返回:
 * 旋转矢量 [rx, ry, rz]
 */
Eigen::Vector3d quaternion_to_rotation_vector(const Eigen::Vector4d& q) {
    double w = q(0), x = q(1), y = q(2), z = q(3);
    
    // 确保四元数是单位四元数
    double norm = sqrt(w*w + x*x + y*y + z*z);
    if (norm != 0) {
        w /= norm;
        x /= norm;
        y /= norm;
        z /= norm;
    }
    
    // 计算旋转角度
    double angle = 2 * acos(w);
    
    // 避免在角度接近0时的数值问题
    double sin_half_angle = sqrt(1 - w*w);
    
    if (sin_half_angle < 1e-10) {
        return Eigen::Vector3d(0, 0, 0);
    }
    
    // 计算旋转轴
    Eigen::Vector3d axis(x, y, z);
    if (sin_half_angle != 0) {
        axis /= sin_half_angle;
    }
    
    // 返回旋转矢量
    return axis * angle;
}

} // namespace arm_robot