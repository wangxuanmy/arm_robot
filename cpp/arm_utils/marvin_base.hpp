#ifndef MARVIN_BASE_HPP
#define MARVIN_BASE_HPP

#include "body_base.h"
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

class MarvinArm : public Body {
private:
    std::vector<double> marvinCrossJointLimit(double j6, double j7) {
        double j7Limit = 0;
        double j6Limit = 0;
        if (fabs(j6) <= 0.349) {
            j7Limit = 1.5707;
        } else {
            j7Limit = -1.024 * fabs(j6) + 1.9275;
        }
    
        if (fabs(j7) <= 0.855) {
            j6Limit = 1.047;
        } else {
            j6Limit = -0.9765 * fabs(j7) + 1.8823;
        }
    
        return {j6Limit, j7Limit};
    }

    Eigen::MatrixXd getJointLimit(const std::vector<double>& jointValue, 
        Eigen::MatrixXd jointLimit) {
        // Note: Only considering cross joint limits for last two joints
        if (jointValue.size() >= 2) {
        std::vector<double> joint6_7 = marvinCrossJointLimit(jointValue[jointValue.size()-2], 
                                            jointValue[jointValue.size()-1]);

        jointLimit(jointLimit.rows()-2, 0) = -joint6_7[0];
        jointLimit(jointLimit.rows()-1, 0) = -joint6_7[1];

        jointLimit(jointLimit.rows()-2, 1) = joint6_7[0];
        jointLimit(jointLimit.rows()-1, 1) = joint6_7[1];
        }

        return jointLimit;
    }

public:
    MarvinArm(const std::vector<std::vector<double>>& dhTableInput): Body(dhTableInput) {
        
    }

    MarvinArm(const std::vector<std::vector<double>>& dhTableInput, 
         const Eigen::MatrixXd& limitInput): Body(dhTableInput, limitInput){
    }

    
    Eigen::MatrixXd getThetaLimit(const std::vector<double>& jointValue) override { return getJointLimit(jointValue, thetaLimit_); }
};



} // namespace arm_robot

#endif // MARVIN_BASE_HPP