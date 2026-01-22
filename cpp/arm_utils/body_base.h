#ifndef BODY_BASE_HPP
#define BODY_BASE_HPP

#include "coord.h"
#include "dh.h"
#include "quaternion_utils.h"
#include <vector>
#include <Eigen/Dense>
#include <cmath>
#include <algorithm>
#include <numeric>
#include <random>

namespace arm_robot {


class Body {
protected:
    std::vector<std::vector<double>> dh_table_;
    std::vector<double> theta_;
    int num_joints_;
    Eigen::MatrixXd theta_limit_;

public:
    Body(const std::vector<std::vector<double>>& dh_table_input);

    Body(const std::vector<std::vector<double>>& dh_table_input, 
         const Eigen::MatrixXd& limit_input);


    std::vector<Eigen::Matrix4d> getDhMatrix(const std::vector<double>& theta_input);

    std::vector<int> getDhType();

    std::pair<bool, std::vector<std::vector<double>>> calQpos(
        const std::vector<double>& start_theta,
        const Eigen::Matrix4d& aim_matrix,
        bool ignore_angle = false,
        bool random_flag = false,
        const std::shared_ptr<std::vector<double>> q0 = nullptr);

    std::vector<double> calQposV(
        const std::vector<double>& start_theta,
        const Eigen::VectorXd& aim_v);

    Eigen::Matrix4d getEef();

    // Getter and setter methods
    std::vector<double> getTheta();
    void setTheta(const std::vector<double>& new_theta);
    int getNumJoints();
    virtual Eigen::MatrixXd getThetaLimit(const std::vector<double>& joint_value);
    void setThetaLimit(const Eigen::MatrixXd& new_limit);
};

} // namespace arm_robot

#endif // BODY_BASE_HPP