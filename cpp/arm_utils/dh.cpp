#include "dh.h"
#include <iostream>

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
Eigen::Matrix4d modified_dh_to_homogeneous_matrix(double d, double theta, double r, double alpha) {
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();

    T(0, 0) = cos(theta);
    T(0, 1) = -sin(theta);
    T(0, 2) = 0;
    T(0, 3) = r;

    T(1, 0) = cos(alpha) * sin(theta);
    T(1, 1) = cos(theta) * cos(alpha);
    T(1, 2) = -sin(alpha);
    T(1, 3) = -d * sin(alpha);

    T(2, 0) = sin(alpha) * sin(theta);
    T(2, 1) = sin(alpha) * cos(theta);
    T(2, 2) = cos(alpha);
    T(2, 3) = d * cos(alpha);

    return T;
}

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
Eigen::Matrix4d modified_old_dh_to_homogeneous_matrix(double d, double theta, double r, double alpha) {
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();

    T(0, 0) = cos(theta);
    T(0, 1) = -sin(theta) * cos(alpha);
    T(0, 2) = sin(theta) * sin(alpha);
    T(0, 3) = r * cos(theta);

    T(1, 0) = sin(theta);
    T(1, 1) = cos(theta) * cos(alpha);
    T(1, 2) = -cos(theta) * sin(alpha);
    T(1, 3) = r * sin(theta);

    T(2, 0) = 0;
    T(2, 1) = sin(alpha);
    T(2, 2) = cos(alpha);
    T(2, 3) = d;

    return T;
}

double angle_normal(double theta) {
    return atan2(sin(theta), cos(theta));
}

std::vector<double> angle_normal(const std::vector<double>& theta) {
    std::vector<double> result;
    for (double t : theta) {
        result.push_back(angle_normal(t));
    }
    return result;
}

std::vector<double> angle_limit(const std::vector<double>& theta, 
    const std::vector<double>& joint_min, 
    const std::vector<double>& joint_max) {
    std::vector<double> result = theta;
    size_t n = theta.size();

    for (size_t i = 0; i < n; ++i) {
        double range = joint_max[i] - joint_min[i];
        bool in_range = (theta[i] >= joint_min[i]) && (theta[i] <= joint_max[i]);
        bool above_max = theta[i] > joint_max[i];
        bool below_min = theta[i] < joint_min[i];

        if (above_max) {
            double excess = theta[i] - joint_max[i];
            result[i] = joint_min[i] + fmod(excess, range);
        } else if (below_min) {
            double deficit = joint_min[i] - theta[i];
            result[i] = joint_max[i] - fmod(deficit, range);
        }
    }

    return result;
}

Eigen::MatrixXd pseudoInverse(const Eigen::MatrixXd& mat, double epsilon) {
    // 步骤1：对矩阵做奇异值分解（SVD）
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(mat, Eigen::ComputeThinU | Eigen::ComputeThinV);
    
    // 步骤2：获取奇异值，并构造奇异值的伪逆矩阵
    Eigen::VectorXd singularValues_inv = svd.singularValues();
    for (int i = 0; i < singularValues_inv.size(); ++i) {
        // 过滤小奇异值（避免除以0，提升稳定性）
        if (singularValues_inv(i) > epsilon) {
            singularValues_inv(i) = 1.0 / singularValues_inv(i);
        } else {
            singularValues_inv(i) = 0.0;
        }
    }
    
    // 步骤3：计算伪逆 = V * Σ⁻¹ * Uᵀ
    Eigen::MatrixXd singularValues_inv_mat = singularValues_inv.asDiagonal();
    Eigen::MatrixXd pinv = svd.matrixV() * singularValues_inv_mat * svd.matrixU().transpose();
    
    return pinv;
}

Eigen::MatrixXd damped_least_squares(const Eigen::MatrixXd& J, double lambda_d) {
    Eigen::MatrixXd J_T = J.transpose();
    Eigen::MatrixXd damping = lambda_d * lambda_d * Eigen::MatrixXd::Identity(J.rows(), J.rows());
    return J_T * pseudoInverse(J * J_T + damping);
}

bool is_singular(const Eigen::MatrixXd& J, double threshold) {
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(J);
    const Eigen::VectorXd& singular_values = svd.singularValues();
    double cond = singular_values(0) / singular_values(singular_values.size() - 1);
    return cond > threshold;
}


std::pair<bool, std::vector<std::vector<double>>> cal_qpos(
    const std::vector<double>& start_theta,
    const Eigen::Matrix4d& aim_matrix,
    const std::vector<int>& dh_type,
    std::function<std::vector<Eigen::Matrix4d>(const std::vector<double>&)> get_dh_mat,
    std::function<Eigen::MatrixXd(const std::vector<double>&)> get_theta_limit,
    bool ignore_angle,
    bool random_flag,
    const std::shared_ptr<std::vector<double>> q0) {
    
    std::vector<double> now_theta = start_theta;
    std::vector<std::vector<double>> theta_can;
    std::vector<Eigen::Vector3d> track;

    // Extract target position and orientation
    Eigen::Matrix3d aim_rot_matrix = aim_matrix.block<3, 3>(0, 0);
    Eigen::Vector4d aim_quaternion = matrix_to_quaternion(aim_rot_matrix);
    Eigen::Vector3d aim_position(aim_matrix(0, 3), aim_matrix(1, 3), aim_matrix(2, 3));

    bool get_aim = false;

    for (int cal_times = 1; cal_times < 400; ++cal_times) {
        std::vector<Eigen::Matrix4d> b = get_dh_mat(now_theta);
        std::reverse(b.begin(), b.end()); // Reverse the vector

        std::vector<Eigen::Matrix4d> T;
        Eigen::Matrix4d tf = b[0];
        T.push_back(tf);
        for (size_t i = 1; i < b.size(); ++i) {
            tf = b[i] * tf;
            T.push_back(tf);
        }
        std::reverse(T.begin(), T.end());

        // Position error calculation
        Eigen::Vector3d d_pose = aim_position - T[0].block<3, 1>(0, 3);

        // Rotation error calculation using quaternions
        Eigen::Matrix3d now_rot_matrix = T[0].block<3, 3>(0, 0);
        Eigen::Vector4d now_quaternion = matrix_to_quaternion(now_rot_matrix);
        
        // Calculate rotation error
        Eigen::Vector4d d_rotation_vec = quaternion_error(now_quaternion, aim_quaternion);
        Eigen::Vector3d d_rotation(d_rotation_vec[1], d_rotation_vec[2], d_rotation_vec[3]);
        d_rotation *= 0.5;

        double err_s = d_pose.norm();
        double err_theta = 0;

        if (!ignore_angle) {
            err_theta = 0.3 * d_rotation.norm();
        }

        track.push_back(T[0].block<3, 1>(0, 3));
        theta_can.push_back(now_theta);

        std::cout << "Iteration: " << cal_times << "  Error: " << err_s << " " << err_theta << std::endl;

        if (err_s < 0.0005 && err_theta < 0.001) {
            get_aim = true;
            break;
        }

        std::vector<std::vector<double>> j;
        for (size_t index = 0; index < T.size() - 1; ++index) {
            const auto& i = T[index + 1];
            if (dh_type[index] == 0) {
                std::vector<double> row = {
                    i(0, 3) * i(1, 0) - i(1, 3) * i(0, 0),
                    i(0, 3) * i(1, 1) - i(1, 3) * i(0, 1),
                    i(0, 3) * i(1, 2) - i(1, 3) * i(0, 2),
                    i(2, 0),
                    i(2, 1),
                    i(2, 2)
                };
                j.push_back(row);
            } else if (dh_type[index] == 1) {
                std::vector<double> row = {
                    i(1, 3) * i(2, 0) - i(2, 3) * i(1, 0),
                    i(1, 3) * i(2, 1) - i(2, 3) * i(1, 1),
                    i(1, 3) * i(2, 2) - i(2, 3) * i(1, 2),
                    i(0, 0),
                    i(0, 1),
                    i(0, 2)
                };
                j.push_back(row);
            } else if (dh_type[index] == 2) {
                std::vector<double> row = {
                    i(2, 0),
                    i(2, 1),
                    i(2, 2),
                    0,
                    0,
                    0
                };
                j.push_back(row);
            }
        }
        // 默认末端是旋转z
        j.push_back({0.0, 0.0, 0.0, 0.0, 0.0, 1.0});

        Eigen::MatrixXd j_mat(j.size(), 6);
        for (size_t r = 0; r < j.size(); ++r) {
            for (size_t c = 0; c < 6; ++c) {
                j_mat(r, c) = j[r][c];
            }
        }
        j_mat.transposeInPlace();

        Eigen::Matrix<double, 6, 6> temp = Eigen::Matrix<double, 6, 6>::Zero();
        temp.block<3, 3>(0, 0) = T[0].block<3, 3>(0, 0);
        temp.block<3, 3>(3, 3) = T[0].block<3, 3>(0, 0);
        Eigen::MatrixXd j0 = temp * j_mat;

        Eigen::VectorXd ds(6);
        if (ignore_angle) {
            ds << d_pose[0], d_pose[1], d_pose[2], 0, 0, 0;
        } else {
            ds << d_pose[0], d_pose[1], d_pose[2], 
                   d_rotation[0], d_rotation[1], d_rotation[2];
        }

        Eigen::VectorXd v = 0.9 * ds;

        Eigen::VectorXd dq = damped_least_squares(j0) * v;

        Eigen::VectorXd zeros_dq = Eigen::VectorXd::Zero(now_theta.size());
        if (q0 != nullptr) {
            Eigen::VectorXd dq0(now_theta.size());
            for (size_t i = 0; i < now_theta.size(); ++i) {
                dq0[i] = (*q0)[i] - now_theta[i];
            }
            zeros_dq = (Eigen::MatrixXd::Identity(now_theta.size(), now_theta.size()) - 
            pseudoInverse(j0) * j0) * (0.9 * dq0);
        }

        Eigen::VectorXd new_theta_vec = Eigen::Map<const Eigen::VectorXd>(now_theta.data(), now_theta.size()) + 
                                       dq + zeros_dq;

        // Convert back to std::vector
        std::vector<double> new_theta(new_theta_vec.data(), 
                                    new_theta_vec.data() + new_theta_vec.size());

        if (cal_times % 150 == 0 && random_flag) {
            std::random_device rd;
            std::mt19937 gen(rd());
            std::uniform_real_distribution<> dis(-M_PI, M_PI);
            for (auto& val : new_theta) {
                val = dis(gen);
            }
        }

        new_theta = angle_normal(new_theta);

        Eigen::MatrixXd limited_theta = get_theta_limit(new_theta);

        std::vector<double> joint_min(limited_theta.col(0).data(), 
            limited_theta.col(0).data() + limited_theta.rows());
        std::vector<double> joint_max(limited_theta.col(1).data(), 
            limited_theta.col(1).data() + limited_theta.rows());

        new_theta = angle_limit(new_theta, joint_min, joint_max);
        // for (size_t i = 0; i < new_theta.size(); ++i) {
            // new_theta[i] = std::max(limited_theta(i, 0), std::min(limited_theta(i, 1), new_theta[i]));
        // }


        now_theta = new_theta;
    }

    return std::make_pair(get_aim, theta_can);
}

std::vector<double> cal_qpos_v(
    const std::vector<double>& start_theta,
    const Eigen::VectorXd& aim_v,
    const std::vector<int>& dh_type,
    std::function<std::vector<Eigen::Matrix4d>(const std::vector<double>&)> get_dh_mat,
    std::function<Eigen::MatrixXd(const std::vector<double>&)> get_theta_limit) {
    
    std::vector<double> now_theta = start_theta;

    std::vector<Eigen::Matrix4d> b = get_dh_mat(now_theta);
    std::reverse(b.begin(), b.end());

    std::vector<Eigen::Matrix4d> T;
    Eigen::Matrix4d tf = b[0];
    T.push_back(tf);
    for (size_t i = 1; i < b.size(); ++i) {
        tf = b[i] * tf;
        T.push_back(tf);
    }
    std::reverse(T.begin(), T.end());

    std::vector<std::vector<double>> j;
    for (size_t index = 0; index < T.size() - 1; ++index) {
        const auto& i = T[index + 1];
        if (dh_type[index] == 0) {
            std::vector<double> row = {
                i(0, 3) * i(1, 0) - i(1, 3) * i(0, 0),
                i(0, 3) * i(1, 1) - i(1, 3) * i(0, 1),
                i(0, 3) * i(1, 2) - i(1, 3) * i(0, 2),
                i(2, 0),
                i(2, 1),
                i(2, 2)
            };
            j.push_back(row);
        } else if (dh_type[index] == 1) {
            std::vector<double> row = {
                i(1, 3) * i(2, 0) - i(2, 3) * i(1, 0),
                i(1, 3) * i(2, 1) - i(2, 3) * i(1, 1),
                i(1, 3) * i(2, 2) - i(2, 3) * i(1, 2),
                i(0, 0),
                i(0, 1),
                i(0, 2)
            };
            j.push_back(row);
        } else if (dh_type[index] == 2) {
            std::vector<double> row = {
                i(2, 0),
                i(2, 1),
                i(2, 2),
                0,
                0,
                0
            };
            j.push_back(row);
        }
    }
    // 默认末端是旋转z
    j.push_back({0.0, 0.0, 0.0, 0.0, 0.0, 1.0});

    Eigen::MatrixXd j_mat(j.size(), 6);
    for (size_t r = 0; r < j.size(); ++r) {
        for (size_t c = 0; c < 6; ++c) {
            j_mat(r, c) = j[r][c];
        }
    }
    j_mat.transposeInPlace();

    Eigen::Matrix<double, 6, 6> temp = Eigen::Matrix<double, 6, 6>::Zero();
    temp.block<3, 3>(0, 0) = T[0].block<3, 3>(0, 0);
    temp.block<3, 3>(3, 3) = T[0].block<3, 3>(0, 0);
    Eigen::MatrixXd j0 = temp * j_mat;

    Eigen::VectorXd v = aim_v;

    Eigen::VectorXd dq = j0.completeOrthogonalDecomposition().pseudoInverse() * v;

    Eigen::VectorXd new_theta_vec = Eigen::Map<const Eigen::VectorXd>(now_theta.data(), now_theta.size()) + dq;

    std::vector<double> new_theta(new_theta_vec.data(), 
                                 new_theta_vec.data() + new_theta_vec.size());

    new_theta = angle_normal(new_theta);

    Eigen::MatrixXd limited_theta = get_theta_limit(new_theta);
    for (size_t i = 0; i < new_theta.size(); ++i) {
        new_theta[i] = std::max(limited_theta(i, 0), std::min(limited_theta(i, 1), new_theta[i]));
    }

    return new_theta;
}

} // namespace arm_robot