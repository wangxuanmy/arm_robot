#ifndef QUATERNION_UTILS_HPP
#define QUATERNION_UTILS_HPP

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
Eigen::Vector4d matrix_to_quaternion(const Eigen::Matrix3d& R);

/**
 * 将四元数转换为旋转矩阵
 * 
 * 参数:
 * q: [w, x, y, z] 四元数
 * 
 * 返回:
 * R: 3x3 旋转矩阵
 */
Eigen::Matrix3d quaternion_to_matrix(const Eigen::Vector4d& q);

/**
 * 四元数乘法
 * 
 * 参数:
 * q1, q2: [w, x, y, z] 四元数
 * 
 * 返回:
 * q: [w, x, y, z] 四元数乘积
 */
Eigen::Vector4d quaternion_multiply(const Eigen::Vector4d& q1, const Eigen::Vector4d& q2);

/**
 * 四元数共轭
 * 
 * 参数:
 * q: [w, x, y, z] 四元数
 * 
 * 返回:
 * q*: [w, -x, -y, -z] 四元数共轭
 */
Eigen::Vector4d quaternion_conjugate(const Eigen::Vector4d& q);

/**
 * 计算两个四元数之间的误差
 * 
 * 参数:
 * q1, q2: [w, x, y, z] 四元数
 * 
 * 返回:
 * 误差向量
 */
Eigen::Vector4d quaternion_error(const Eigen::Vector4d& q1, const Eigen::Vector4d& q2);

/**
 * 将四元数转换为旋转矢量
 * 
 * 参数:
 * q: 单位四元数 [w, x, y, z]
 * 
 * 返回:
 * 旋转矢量 [rx, ry, rz]
 */
Eigen::Vector3d quaternion_to_rotation_vector(const Eigen::Vector4d& q);

} // namespace arm_robot

#endif // QUATERNION_UTILS_HPP