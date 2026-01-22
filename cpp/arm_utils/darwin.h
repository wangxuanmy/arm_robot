#ifndef DARWIN_H
#define DARWIN_H

#include "body_base.h"
#include "coord.h"
#include "quaternion_utils.h"
#include <vector>
#include <Eigen/Dense>
#include <map>
#include <string>
#include <memory>

namespace arm_robot {

class Human {
private:
    Body body_;
    Body left_hand_;
    Body right_hand_;
    Body head_;
    std::map<std::string, int> hand_id_;
    int num_joints_;
    std::vector<double> theta_;
    Eigen::MatrixXd theta_limit_;
    std::map<std::string, Eigen::Matrix4d> base_tf_;

public:
    Human(const std::vector<std::vector<double>>& body_dh, 
          const std::vector<std::vector<double>>& left_hand_dh,
          const std::vector<std::vector<double>>& right_hand_dh,
          const std::vector<std::vector<double>>& head_dh,
          const Eigen::MatrixXd& theta_limit_input = Eigen::MatrixXd(),
          const Eigen::MatrixXd& base_offset = Eigen::MatrixXd());

    void flashTheta(const std::vector<double>& theta_input);

    std::vector<Eigen::Matrix4d> getAxisTf(const Eigen::Matrix4d& base, 
                                           const std::vector<Eigen::Matrix4d>& tf_tree);

    std::vector<Eigen::Matrix4d> getAllTf();

    std::vector<Eigen::Matrix4d> getEef();

    Eigen::Matrix4d getPartBase(const std::string& part_name);

    Eigen::Matrix4d getEefByName(const std::string& part_name);

    std::shared_ptr<Body> getPart(const std::string& part_name);

    std::vector<double> getAllTheta();

    std::pair<bool, std::vector<std::vector<double>>> controlPart(const Eigen::Matrix4d& aim_mat, 
                                   const std::string& part_name = "body", 
                                   bool ignore_angle = false,
                                   bool random_flag = false,
                                   std::vector<double> q0 = std::vector<double>());

    std::pair<bool, std::vector<std::vector<double>>> controlHand(const Eigen::Matrix4d& aim_mat, 
                                   const std::string& hand_name = "left", 
                                   bool ignore_angle = false,
                                   bool random_flag = false,
                                   std::vector<double> q0 = std::vector<double>());

    std::vector<double> controlPartV(const Eigen::VectorXd& aim_v, 
                                     const std::string& part_name = "body");

    int getNumJoints();
};

} // namespace arm_robot

#endif // DARWIN_H