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
    std::map<std::string, int> hand_id_;
    int num_joints_;
    std::vector<double> theta_;
    Eigen::MatrixXd theta_limit_;
    std::map<std::string, Eigen::Matrix4d> base_tf_;

public:
    Human(const std::vector<std::vector<double>>& bodyDh, 
          const std::vector<std::vector<double>>& leftHandDh,
          const std::vector<std::vector<double>>& rightHandDh,
          const Eigen::MatrixXd& thetaLimitInput = Eigen::MatrixXd(),
          const Eigen::MatrixXd& base_offset = Eigen::MatrixXd());

    void flashTheta(const std::vector<double>& thetaInput);

    std::vector<Eigen::Matrix4d> getAxisTf(const Eigen::Matrix4d& base, 
                                           const std::vector<Eigen::Matrix4d>& tfTree);

    std::vector<Eigen::Matrix4d> getAllTf();

    std::vector<Eigen::Matrix4d> getEef();

    Eigen::Matrix4d getPartBase(const std::string& partName);

    Eigen::Matrix4d getEefByName(const std::string& partName);

    std::shared_ptr<Body> getPart(const std::string& partName);

    std::vector<double> getAllTheta();

    std::pair<bool, std::vector<std::vector<double>>> controlPart(const Eigen::Matrix4d& aimMat, 
                                   const std::string& partName = "body", 
                                   bool ignoreAngle = false,
                                   bool random_flag = false,
                                   std::vector<double> q0 = std::vector<double>());

    std::pair<bool, std::vector<std::vector<double>>> controlHand(const Eigen::Matrix4d& aimMat, 
                                   const std::string& handName = "left", 
                                   bool ignoreAngle = false,
                                   bool random_flag = false,
                                   std::vector<double> q0 = std::vector<double>());

    std::vector<double> controlPartV(const Eigen::VectorXd& aimV, 
                                     const std::string& partName = "body");

    int getNumJoints();
};

} // namespace arm_robot

#endif // DARWIN_H