#include "darwin.h"
#include "dh.h"  // Include the header that contains cal_qpos
#include <iostream>

namespace arm_robot {

Human::Human(const std::vector<std::vector<double>>& bodyDh, 
             const std::vector<std::vector<double>>& leftHandDh,
             const std::vector<std::vector<double>>& rightHandDh,
             const Eigen::MatrixXd& thetaLimitInput,
             const Eigen::MatrixXd& base_offset) :
    body_(bodyDh), left_hand_(leftHandDh), right_hand_(rightHandDh) {
    
    hand_id_ = {{"body", 0}, {"left", 1}, {"right", 2}};

    num_joints_ = body_.getNumJoints() + left_hand_.getNumJoints() + right_hand_.getNumJoints();
    theta_ = std::vector<double>(num_joints_, 0.0);

    if (thetaLimitInput.rows() == 0) {
        theta_limit_.resize(num_joints_, 2);
        theta_limit_.col(0).setConstant(-360.0 / 57.2958);
        theta_limit_.col(1).setConstant(360.0 / 57.2958);
    } else {
        theta_limit_ = thetaLimitInput;
    }

    // 设置各个部分的角度限制
    int bodyJoints = body_.getNumJoints();
    int leftJoints = left_hand_.getNumJoints();
    int rightJoints = right_hand_.getNumJoints();

    Eigen::MatrixXd bodyLimits = theta_limit_.block(0, 0, bodyJoints, 2);
    Eigen::MatrixXd leftLimits = theta_limit_.block(bodyJoints, 0, leftJoints, 2);
    Eigen::MatrixXd rightLimits = theta_limit_.block(bodyJoints + leftJoints, 0, rightJoints, 2);

    body_.setThetaLimit(bodyLimits);
    left_hand_.setThetaLimit(leftLimits);
    right_hand_.setThetaLimit(rightLimits);

    if(base_offset.rows() == 0) { 
        // 设置基础变换矩阵
        base_tf_["body"] = coord::euler_and_coord_to_matrix(0, 0, 0, 0, 0, 0);
        base_tf_["left"] = coord::euler_and_coord_to_matrix(0, 0, 0, 0, 0, 0);
        base_tf_["right"] = coord::euler_and_coord_to_matrix(0, 0, 0, 0, 0, 0);
    }else{
        base_tf_["body"] = coord::euler_and_coord_to_matrix(base_offset(0, 0), base_offset(0, 1), base_offset(0, 2), base_offset(0, 3), base_offset(0, 4), base_offset(0, 5));
        base_tf_["left"] = coord::euler_and_coord_to_matrix(base_offset(1, 0), base_offset(1, 1), base_offset(1, 2), base_offset(1, 3), base_offset(1, 4), base_offset(1, 5));
        base_tf_["right"] = coord::euler_and_coord_to_matrix(base_offset(2, 0), base_offset(2, 1), base_offset(2, 2), base_offset(2, 3), base_offset(2, 4), base_offset(2, 5));
    }
    
}

void Human::flashTheta(const std::vector<double>& thetaInput) {
    int bodyJoints = body_.getNumJoints();
    int leftJoints = left_hand_.getNumJoints();

    std::vector<double> bodyTheta(thetaInput.begin(), thetaInput.begin() + bodyJoints);
    std::vector<double> leftTheta(thetaInput.begin() + bodyJoints, 
                                  thetaInput.begin() + bodyJoints + leftJoints);
    std::vector<double> rightTheta(thetaInput.begin() + bodyJoints + leftJoints, 
                                   thetaInput.end());

    body_.setTheta(bodyTheta);
    left_hand_.setTheta(leftTheta);
    right_hand_.setTheta(rightTheta);
}

std::vector<Eigen::Matrix4d> Human::getAxisTf(const Eigen::Matrix4d& base, 
                                              const std::vector<Eigen::Matrix4d>& tfTree) {
    std::vector<Eigen::Matrix4d> tfCan;
    Eigen::Matrix4d tf = base;
    tfCan.push_back(tf);

    for (const auto& transform : tfTree) {
        tf = tf * transform;
        tfCan.push_back(tf);
    }

    return tfCan;
}

std::vector<Eigen::Matrix4d> Human::getAllTf() {
    // 获取所有关节的坐标齐次矩阵
    std::vector<Eigen::Matrix4d> bodyMat = body_.getDhMatrix(body_.getTheta());

    std::vector<Eigen::Matrix4d> bodyEnd = getAxisTf(base_tf_["body"], bodyMat);

    std::vector<Eigen::Matrix4d> leftEnd = getAxisTf(
        bodyEnd.back() * base_tf_["left"], 
        left_hand_.getDhMatrix(left_hand_.getTheta())
    );

    std::vector<Eigen::Matrix4d> rightEnd = getAxisTf(
        bodyEnd.back() * base_tf_["right"], 
        right_hand_.getDhMatrix(right_hand_.getTheta())
    );

    bodyEnd.insert(bodyEnd.end(), leftEnd.begin(), leftEnd.end());
    bodyEnd.insert(bodyEnd.end(), rightEnd.begin(), rightEnd.end());

    return bodyEnd;
}

std::vector<Eigen::Matrix4d> Human::getEef() {
    /* 获取末端的齐次矩阵 */
    Eigen::Matrix4d base = base_tf_["body"];

    std::vector<Eigen::Matrix4d> bodyMat = body_.getDhMatrix(body_.getTheta());

    Eigen::Matrix4d tf = base;

    for (const auto& i : bodyMat) {
        tf = tf * i;
    }

    Eigen::Matrix4d bodyEnd = tf;

    base = tf;

    tf = base * base_tf_["left"];
    bodyMat = left_hand_.getDhMatrix(left_hand_.getTheta());
    for (const auto& i : bodyMat) {
        tf = tf * i;
    }
    Eigen::Matrix4d bodyLeft = tf;

    tf = base * base_tf_["right"];
    bodyMat = right_hand_.getDhMatrix(right_hand_.getTheta());
    for (const auto& i : bodyMat) {
        tf = tf * i;
    }
    Eigen::Matrix4d bodyRight = tf;

    return {bodyEnd, bodyLeft, bodyRight};
}

Eigen::Matrix4d Human::getPartBase(const std::string& partName) {
    /* 获取某部分的基座矩阵 */
    if (partName == "body") {
        return base_tf_[partName];
    } else if (partName == "left") {
        Eigen::Matrix4d body_eef = body_.getEef();
        return body_eef * base_tf_[partName];
    } else if (partName == "right") {
        Eigen::Matrix4d body_eef = body_.getEef();
        return body_eef * base_tf_[partName];
    }
    return Eigen::Matrix4d::Identity();
}

Eigen::Matrix4d Human::getEefByName(const std::string& partName) {
    /* 获取特定部分的末端执行器 */
    if (partName == "body") {
        return base_tf_[partName] * body_.getEef();
    } else if (partName == "left") {
        Eigen::Matrix4d body_eef = body_.getEef();
        return base_tf_["body"] * body_eef * base_tf_["left"] * left_hand_.getEef();
    } else if (partName == "right") {
        Eigen::Matrix4d body_eef = body_.getEef();
        return base_tf_["body"] * body_eef * base_tf_["right"] * right_hand_.getEef();
    }
    return Eigen::Matrix4d::Identity();
}

std::shared_ptr<Body> Human::getPart(const std::string& partName) {
    // 返回部件的共享指针
    if (partName == "body") {
        return std::make_shared<Body>(body_);
    } else if (partName == "left") {
        return std::make_shared<Body>(left_hand_);
    } else if (partName == "right") {
        return std::make_shared<Body>(right_hand_);
    }
    return nullptr;
}

std::vector<double> Human::getAllTheta() {
    std::vector<double> allTheta = body_.getTheta();
    std::vector<double> leftTheta = left_hand_.getTheta();
    std::vector<double> rightTheta = right_hand_.getTheta();
    
    allTheta.insert(allTheta.end(), leftTheta.begin(), leftTheta.end());
    allTheta.insert(allTheta.end(), rightTheta.begin(), rightTheta.end());
    
    return allTheta;
}

std::pair<bool, std::vector<std::vector<double>>> Human::controlPart(const Eigen::Matrix4d& aimMat, 
                                      const std::string& partName, 
                                      bool ignoreAngle,
                                      bool random_flag,
                                      std::vector<double> q0) {
    /* 单独控制某个部分,输出到目标的位姿的关节角度theta */
    std::shared_ptr<Body> part = getPart(partName);

    if (!part) return {};

    Eigen::Matrix4d base = getPartBase(partName);
    Eigen::Matrix4d armAim = base.inverse() * aimMat;

    std::vector<double> startTheta = part->getTheta();

    std::shared_ptr<std::vector<double>> input_q0;

    if (!q0.empty()) {
        input_q0 = std::make_shared<std::vector<double>>(q0);  // 类型匹配
    } else {
        input_q0 = nullptr;  // 空值时赋值 nullptr（合法，shared_ptr 支持）
    }
    return part->calQpos(startTheta, armAim, ignoreAngle, random_flag, input_q0);
}

std::pair<bool, std::vector<std::vector<double>>> Human::controlHand(const Eigen::Matrix4d& aimMat, 
                                      const std::string& handName, 
                                      bool ignoreAngle,
                                      bool random_flag,
                                      std::vector<double> q0) {
    std::shared_ptr<Body> hand;
    if (handName == "left") {
        hand = std::make_shared<Body>(left_hand_);
    } else if (handName == "right") {
        hand = std::make_shared<Body>(right_hand_);
    }

    if (!hand) {
        return {};
    }

    // 合并身体和手的DH参数类型
    std::vector<int> dhType = body_.getDhType();
    std::vector<int> handDhType = hand->getDhType();
    dhType.insert(dhType.end(), handDhType.begin(), handDhType.end());

    Eigen::Matrix4d base = getPartBase("body");
    Eigen::Matrix4d armAim = base.inverse() * aimMat;

    auto get_matrix = [this, hand, handName](const std::vector<double>& theta) { 
        std::vector<double> theta_body(theta.begin(), theta.begin() + body_.getNumJoints());
        std::vector<double> theta_hand(theta.begin() + body_.getNumJoints(), theta.end());
        std::vector<Eigen::Matrix4d> dh_body = body_.getDhMatrix(theta_body);
        std::vector<Eigen::Matrix4d> dh_hand = hand->getDhMatrix(theta_hand);
        dh_hand[0] = this->base_tf_[handName] * dh_hand[0];
        dh_body.insert(dh_body.end(), dh_hand.begin(), dh_hand.end());
        return dh_body;
    };

    std::vector<double> start_theta = body_.getTheta();
    std::vector<double> hand_theta = hand->getTheta(); 
    start_theta.insert(start_theta.end(), hand_theta.begin(), hand_theta.end());

    std::shared_ptr<std::vector<double>> input_q0;

    if (!q0.empty()) {
        input_q0 = std::make_shared<std::vector<double>>(q0);  // 类型匹配
    } else {
        input_q0 = nullptr;  // 空值时赋值 nullptr（合法，shared_ptr 支持）
    }

    return cal_qpos(
        start_theta,
        armAim,
        dhType,
        get_matrix,
        [this, hand](const std::vector<double>& theta) {
            std::vector<double> theta_body(theta.begin(), theta.begin() + body_.getNumJoints());
            std::vector<double> theta_hand(theta.begin() + body_.getNumJoints(), theta.end());
            Eigen::MatrixXd body_limit = body_.getThetaLimit(theta_body);
            Eigen::MatrixXd hand_limit = hand->getThetaLimit(theta_hand);
            Eigen::MatrixXd total_limit(body_limit.rows() + hand_limit.rows(), 2);
            total_limit.block(0, 0, body_limit.rows(), 2) = body_limit;
            total_limit.block(body_limit.rows(), 0, hand_limit.rows(), 2) = hand_limit;
            return total_limit;
        },
        ignoreAngle,
        random_flag,
        input_q0
    );

    
}

std::vector<double> Human::controlPartV(const Eigen::VectorXd& aimV, 
                                        const std::string& partName) {
    /* 单独控制某个部分速度,输出目标角度 */
    std::shared_ptr<Body> part = getPart(partName);
    if (!part) return {};

    Eigen::Matrix4d tf2base = getPartBase(partName);
    Eigen::Matrix3d rotPart = tf2base.block<3, 3>(0, 0);
    
    Eigen::VectorXd vel = rotPart.inverse() * aimV.head(3);
    Eigen::VectorXd rota = rotPart.inverse() * aimV.tail(3);
    
    Eigen::VectorXd baseAimV(6);
    baseAimV << vel, rota;

    std::vector<double> startTheta = part->getTheta();

    return part->calQposV(startTheta, baseAimV);
}

int Human::getNumJoints() {
    return num_joints_;
}

} // namespace arm_robot