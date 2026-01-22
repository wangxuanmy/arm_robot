#ifndef DH_HPP
#define DH_HPP

#include <Eigen/Dense>
#include <cmath>
#include <vector>  // 添加缺失的vector头文件
#include <functional>  // 添加function头文件
#include <algorithm>  // 添加algorithm头文件
#include <random>  // 添加random头文件
#include <memory>
#include "quaternion_utils.h"


namespace arm_robot {

/**
 * 根据改进 DH 参数生成齐次变换矩阵。
 * 
 * 参数:
 * - d: 沿当前关节 z 轴的距离
 * - theta: 绕当前关节 z 轴的旋转角度（弧度）
 * - r: 沿前一关节 x 轴的距离（连杆长度）
 * - alpha: 绕前一关节 x 轴的旋转角度（连杆扭转角，弧度）
 * 
 * 顺序  alpha, r, d ,theta
 * 
 * 返回:
 * - 4x4 齐次变换矩阵
 */
Eigen::Matrix4d modified_dh_to_homogeneous_matrix(double d, double theta, double r, double alpha);

/**
 * 根据 DH 参数生成齐次变换矩阵。
 * 
 * 参数:
 * - d: 沿前一关节 z 轴的距离
 * - theta: 绕前一关节 z 轴的旋转角度（弧度）
 * - r: 沿当前关节 x 轴的距离（连杆长度）
 * - alpha: 绕当前关节 x 轴的旋转角度（连杆扭转角，弧度）
 * 
 * 返回:
 * - 4x4 齐次变换矩阵
 */
Eigen::Matrix4d modified_old_dh_to_homogeneous_matrix(double d, double theta, double r, double alpha);

double angle_normal(double theta);

std::vector<double> angle_normal(const std::vector<double>& theta);

std::vector<double> angle_limit(const std::vector<double>& theta, 
    const std::vector<double>& joint_min, 
    const std::vector<double>& joint_max);

Eigen::MatrixXd pseudoInverse(const Eigen::MatrixXd& mat, double epsilon = 1e-6);

Eigen::MatrixXd damped_least_squares(const Eigen::MatrixXd& J, double lambda_d = 0.3);

bool is_singular(const Eigen::MatrixXd& J, double threshold = 1000);



std::pair<bool, std::vector<std::vector<double>>> cal_qpos(
    const std::vector<double>& start_theta,
    const Eigen::Matrix4d& aim_matrix,
    const std::vector<int>& dh_type,
    std::function<std::vector<Eigen::Matrix4d>(const std::vector<double>&)> get_dh_mat,
    std::function<Eigen::MatrixXd(const std::vector<double>&)> get_theta_limit,
    bool ignore_angle = false,
    bool random_flag = false,
    const std::shared_ptr<std::vector<double>> q0 = nullptr);

std::vector<double> cal_qpos_v(
    const std::vector<double>& start_theta,
    const Eigen::VectorXd& aim_v,
    const std::vector<int>& dh_type,
    std::function<std::vector<Eigen::Matrix4d>(const std::vector<double>&)> get_dh_mat,
    std::function<Eigen::MatrixXd(const std::vector<double>&)> get_theta_limit);


} // namespace arm_robot

#endif // DH_HPP