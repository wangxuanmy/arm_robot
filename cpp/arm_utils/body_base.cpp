#include "body_base.h"
#include "dh.h"
#include "quaternion_utils.h"
#include "coord.h"
#include <vector>
#include <Eigen/Dense>
#include <cmath>
#include <limits>
#include <algorithm>

namespace arm_robot {

    Body::Body(const std::vector<std::vector<double>>& dh_table_input) {
        num_joints_ = dh_table_input.size();
        theta_ = std::vector<double>(num_joints_, 0.0);
        dh_table_ = dh_table_input;

        theta_limit_.resize(num_joints_, 2);
        theta_limit_.col(0).setConstant(-360.0 / 57.2958);
        theta_limit_.col(1).setConstant(360.0 / 57.2958);
    }

    Body::Body(const std::vector<std::vector<double>>& dh_table_input, 
         const Eigen::MatrixXd& limit_input) {
        num_joints_ = dh_table_input.size();
        theta_ = std::vector<double>(num_joints_, 0.0);
        dh_table_ = dh_table_input;
        theta_limit_ = limit_input;
    }


    std::vector<Eigen::Matrix4d> Body::getDhMatrix(const std::vector<double>& theta_input) {
        std::vector<Eigen::Matrix4d> b;
        for (size_t i = 0; i < dh_table_.size(); ++i) {
            const auto& value = dh_table_[i];
            if (static_cast<int>(value[4]) == 0) {
                // 旋转关节
                b.push_back(modified_dh_to_homogeneous_matrix(value[2], 
                                                              theta_input[i] + value[3], 
                                                              value[1], 
                                                              value[0]));
            } else if (static_cast<int>(value[4]) == 1) {
                // 旋转关节
                b.push_back(modified_old_dh_to_homogeneous_matrix(value[2], 
                                                                  value[2], 
                                                                  value[1], 
                                                                  theta_input[i] + value[0]));
            } else if (static_cast<int>(value[4]) == 2) {
                // 移动关节
                b.push_back(modified_dh_to_homogeneous_matrix(value[2] + theta_input[i], 
                                                              value[3], 
                                                              value[1], 
                                                              value[0]));
            }
        }
        return b;
    }

    std::vector<int> Body::getDhType() {
        std::vector<int> b;
        for (const auto& value : dh_table_) {
            if (static_cast<int>(value[4]) == 0) {
                b.push_back(0);
            } else if (static_cast<int>(value[4]) == 1) {
                b.push_back(1);
            } else if (static_cast<int>(value[4]) == 2) {
                b.push_back(2);
            }
        }
        return b;
    }

    std::pair<bool, std::vector<std::vector<double>>> Body::calQpos(
        const std::vector<double>& start_theta,
        const Eigen::Matrix4d& aim_matrix,
        bool ignore_angle,
        bool random_flag,
        const std::shared_ptr<std::vector<double>> q0) {
            
        auto dh_mat_fun = [&](const std::vector<double>& theta) {
            return getDhMatrix(theta);
        };

        auto get_theta_limit = [this](const std::vector<double>& joint_value) { 
            return this->getThetaLimit(joint_value); 
        };

        return arm_robot::cal_qpos(start_theta, aim_matrix, getDhType(), dh_mat_fun, 
                        get_theta_limit, ignore_angle, random_flag, q0);
    }

    std::vector<double> Body::calQposV(
        const std::vector<double>& start_theta,
        const Eigen::VectorXd& aim_v) {
            
        auto dh_mat_fun = [&](const std::vector<double>& theta) {
            return getDhMatrix(theta);
        };

        auto get_theta_limit = [this](const std::vector<double>& joint_value) { 
            return this->getThetaLimit(joint_value); 
        };

        return arm_robot::cal_qpos_v(start_theta, aim_v, getDhType(), dh_mat_fun, 
        get_theta_limit);
        
    }

    Eigen::Matrix4d Body::getEef() {
        Eigen::Matrix4d base = coord::euler_and_coord_to_matrix(0, 0, 0, 0, 0, 0);
        std::vector<Eigen::Matrix4d> mat = getDhMatrix(theta_);
        Eigen::Matrix4d tf = base;
        for (const auto& m : mat) {
            tf = tf * m;
        }
        return tf;
    }

    // Getter and setter methods
    std::vector<double> Body::getTheta() { return theta_; }
    void Body::setTheta(const std::vector<double>& new_theta) { theta_ = new_theta; }
    int Body::getNumJoints() { return num_joints_; }
    Eigen::MatrixXd Body::getThetaLimit(const std::vector<double>& joint_value) { return theta_limit_; }
    void Body::setThetaLimit(const Eigen::MatrixXd& new_limit) { theta_limit_ = new_limit; }

} // namespace arm_robot