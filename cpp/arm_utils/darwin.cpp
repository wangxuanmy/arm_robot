#include "darwin.h"
#include "dh.h"  // Include the header that contains cal_qpos
#include <iostream>

namespace arm_robot {

Human::Human(const std::vector<std::vector<double>>& body_dh, 
             const std::vector<std::vector<double>>& left_hand_dh,
             const std::vector<std::vector<double>>& right_hand_dh,
             const std::vector<std::vector<double>>& head_dh,
             const Eigen::MatrixXd& theta_limit_input,
             const Eigen::MatrixXd& base_offset) :
    body_(body_dh), left_hand_(left_hand_dh), right_hand_(right_hand_dh), head_(head_dh) {
    
    hand_id_ = {{"body", 0}, {"left", 1}, {"right", 2}, {"head", 3}};

    num_joints_ = body_.getNumJoints() + left_hand_.getNumJoints() + right_hand_.getNumJoints() + head_.getNumJoints();
    theta_ = std::vector<double>(num_joints_, 0.0);

    if (theta_limit_input.rows() == 0) {
        theta_limit_.resize(num_joints_, 2);
        theta_limit_.col(0).setConstant(-360.0 / 57.2958);
        theta_limit_.col(1).setConstant(360.0 / 57.2958);
    } else {
        theta_limit_ = theta_limit_input;
    }

    // 设置各个部分的角度限制
    int body_joints = body_.getNumJoints();
    int left_joints = left_hand_.getNumJoints();
    int right_joints = right_hand_.getNumJoints();
    int head_joints = head_.getNumJoints();

    Eigen::MatrixXd body_limits = theta_limit_.block(0, 0, body_joints, 2);
    Eigen::MatrixXd left_limits = theta_limit_.block(body_joints, 0, left_joints, 2);
    Eigen::MatrixXd right_limits = theta_limit_.block(body_joints + left_joints, 0, right_joints, 2);
    Eigen::MatrixXd head_limits = theta_limit_.block(body_joints + left_joints + right_joints, 0, head_joints, 2);


    body_.setThetaLimit(body_limits);
    left_hand_.setThetaLimit(left_limits);
    right_hand_.setThetaLimit(right_limits);


    if(base_offset.rows() == 0) { 
        // 设置基础变换矩阵
        base_tf_["body"] = coord::euler_and_coord_to_matrix(0, 0, 0, 0, 0, 0);
        base_tf_["left"] = coord::euler_and_coord_to_matrix(0, 0, 0, 0, 0, 0);
        base_tf_["right"] = coord::euler_and_coord_to_matrix(0, 0, 0, 0, 0, 0);
        base_tf_["head"] = coord::euler_and_coord_to_matrix(0, 0, 0, 0, 0, 0);
    }else{
        base_tf_["body"] = coord::euler_and_coord_to_matrix(base_offset(0, 0), base_offset(0, 1), base_offset(0, 2), base_offset(0, 3), base_offset(0, 4), base_offset(0, 5));
        base_tf_["left"] = coord::euler_and_coord_to_matrix(base_offset(1, 0), base_offset(1, 1), base_offset(1, 2), base_offset(1, 3), base_offset(1, 4), base_offset(1, 5));
        base_tf_["right"] = coord::euler_and_coord_to_matrix(base_offset(2, 0), base_offset(2, 1), base_offset(2, 2), base_offset(2, 3), base_offset(2, 4), base_offset(2, 5));
        base_tf_["head"] = coord::euler_and_coord_to_matrix(base_offset(3, 0), base_offset(3, 1), base_offset(3, 2), base_offset(3, 3), base_offset(3, 4), base_offset(3, 5));
    }
    
}

void Human::flashTheta(const std::vector<double>& theta_input) {
    int body_joints = body_.getNumJoints();
    int left_joints = left_hand_.getNumJoints();
    int right_joints = right_hand_.getNumJoints();

    std::vector<double> body_theta(theta_input.begin(), theta_input.begin() + body_joints);
    std::vector<double> left_theta(theta_input.begin() + body_joints, 
                                  theta_input.begin() + body_joints + left_joints);
    std::vector<double> right_theta(theta_input.begin() + body_joints + left_joints, 
                                   theta_input.begin() + body_joints + left_joints + right_joints);
    std::vector<double> head_theta(theta_input.begin() + body_joints + left_joints + right_joints, 
                                   theta_input.end());

    body_.setTheta(body_theta);
    left_hand_.setTheta(left_theta);
    right_hand_.setTheta(right_theta);
    head_.setTheta(head_theta);
}

std::vector<Eigen::Matrix4d> Human::getAxisTf(const Eigen::Matrix4d& base, 
                                              const std::vector<Eigen::Matrix4d>& tf_tree) {
    std::vector<Eigen::Matrix4d> tf_can;
    Eigen::Matrix4d tf = base;
    tf_can.push_back(tf);

    for (const auto& transform : tf_tree) {
        tf = tf * transform;
        tf_can.push_back(tf);
    }

    return tf_can;
}

std::vector<Eigen::Matrix4d> Human::getAllTf() {
    // 获取所有关节的坐标齐次矩阵
    std::vector<Eigen::Matrix4d> body_mat = body_.getDhMatrix(body_.getTheta());

    std::vector<Eigen::Matrix4d> body_end = getAxisTf(base_tf_["body"], body_mat);

    std::vector<Eigen::Matrix4d> left_end = getAxisTf(
        body_end.back() * base_tf_["left"], 
        left_hand_.getDhMatrix(left_hand_.getTheta())
    );

    std::vector<Eigen::Matrix4d> right_end = getAxisTf(
        body_end.back() * base_tf_["right"], 
        right_hand_.getDhMatrix(right_hand_.getTheta())
    );

    std::vector<Eigen::Matrix4d> head_end = getAxisTf(
        body_end.back() * base_tf_["head"], 
        head_.getDhMatrix(head_.getTheta())
    );

    body_end.insert(body_end.end(), left_end.begin(), left_end.end());
    body_end.insert(body_end.end(), right_end.begin(), right_end.end());
    body_end.insert(body_end.end(), head_end.begin(), head_end.end());

    return body_end;
}

std::vector<Eigen::Matrix4d> Human::getEef() {
    /* 获取末端的齐次矩阵 */
    Eigen::Matrix4d base = base_tf_["body"];

    std::vector<Eigen::Matrix4d> body_mat = body_.getDhMatrix(body_.getTheta());

    Eigen::Matrix4d tf = base;

    for (const auto& i : body_mat) {
        tf = tf * i;
    }

    Eigen::Matrix4d body_end = tf;

    base = tf;

    tf = base * base_tf_["left"];
    body_mat = left_hand_.getDhMatrix(left_hand_.getTheta());
    for (const auto& i : body_mat) {
        tf = tf * i;
    }
    Eigen::Matrix4d body_left = tf;

    tf = base * base_tf_["right"];
    body_mat = right_hand_.getDhMatrix(right_hand_.getTheta());
    for (const auto& i : body_mat) {
        tf = tf * i;
    }
    Eigen::Matrix4d body_right = tf;

    tf = base * base_tf_["head"];
    body_mat = head_.getDhMatrix(head_.getTheta());
    for (const auto& i : body_mat) {
        tf = tf * i;
    }
    Eigen::Matrix4d body_head = tf;

    return {body_end, body_left, body_right, body_head};
}

Eigen::Matrix4d Human::getPartBase(const std::string& part_name) {
    /* 获取某部分的基座矩阵 */
    if (part_name == "body") {
        return base_tf_[part_name];
    } else if (part_name == "left") {
        Eigen::Matrix4d body_eef = body_.getEef();
        return body_eef * base_tf_[part_name];
    } else if (part_name == "right") {
        Eigen::Matrix4d body_eef = body_.getEef();
        return body_eef * base_tf_[part_name];
    } else if (part_name == "head") {
        Eigen::Matrix4d body_eef = body_.getEef();
        return body_eef * base_tf_[part_name];
    }
    return Eigen::Matrix4d::Identity();
}

Eigen::Matrix4d Human::getEefByName(const std::string& part_name) {
    /* 获取特定部分的末端执行器 */
    if (part_name == "body") {
        return base_tf_[part_name] * body_.getEef();
    } else if (part_name == "left") {
        Eigen::Matrix4d body_eef = body_.getEef();
        return base_tf_["body"] * body_eef * base_tf_["left"] * left_hand_.getEef();
    } else if (part_name == "right") {
        Eigen::Matrix4d body_eef = body_.getEef();
        return base_tf_["body"] * body_eef * base_tf_["right"] * right_hand_.getEef();
    } else if (part_name == "head") {
        Eigen::Matrix4d body_eef = body_.getEef();
        return base_tf_["body"] * body_eef * base_tf_["head"] * head_.getEef();
    }
    return Eigen::Matrix4d::Identity();
}

std::shared_ptr<Body> Human::getPart(const std::string& part_name) {
    // 返回部件的共享指针
    if (part_name == "body") {
        return std::make_shared<Body>(body_);
    } else if (part_name == "left") {
        return std::make_shared<Body>(left_hand_);
    } else if (part_name == "right") {
        return std::make_shared<Body>(right_hand_);
    } else if (part_name == "head") {
        return std::make_shared<Body>(head_);
    }
    return nullptr;
}

std::vector<double> Human::getAllTheta() {
    std::vector<double> all_theta = body_.getTheta();
    std::vector<double> left_theta = left_hand_.getTheta();
    std::vector<double> right_theta = right_hand_.getTheta();
    std::vector<double> head_theta = head_.getTheta();
    
    all_theta.insert(all_theta.end(), left_theta.begin(), left_theta.end());
    all_theta.insert(all_theta.end(), right_theta.begin(), right_theta.end());
    all_theta.insert(all_theta.end(), head_theta.begin(), head_theta.end());
    
    return all_theta;
}

std::pair<bool, std::vector<std::vector<double>>> Human::controlPart(const Eigen::Matrix4d& aim_mat, 
                                      const std::string& part_name, 
                                      bool ignore_angle,
                                      bool random_flag,
                                      std::vector<double> q0) {
    /* 单独控制某个部分,输出到目标的位姿的关节角度theta */
    std::shared_ptr<Body> part = getPart(part_name);

    if (!part) return {};

    Eigen::Matrix4d base = getPartBase(part_name);
    Eigen::Matrix4d arm_aim = base.inverse() * aim_mat;

    std::vector<double> start_theta = part->getTheta();

    std::shared_ptr<std::vector<double>> input_q0;

    if (!q0.empty()) {
        input_q0 = std::make_shared<std::vector<double>>(q0);  // 类型匹配
    } else {
        input_q0 = nullptr;  // 空值时赋值 nullptr（合法，shared_ptr 支持）
    }
    return part->calQpos(start_theta, arm_aim, ignore_angle, random_flag, input_q0);
}

std::pair<bool, std::vector<std::vector<double>>> Human::controlHand(const Eigen::Matrix4d& aim_mat, 
                                      const std::string& hand_name, 
                                      bool ignore_angle,
                                      bool random_flag,
                                      std::vector<double> q0) {
    std::shared_ptr<Body> hand;
    if (hand_name == "left") {
        hand = std::make_shared<Body>(left_hand_);
    } else if (hand_name == "right") {
        hand = std::make_shared<Body>(right_hand_);
    } else if (hand_name == "head") {
        hand = std::make_shared<Body>(head_);
    }
    if (!hand) {
        return {};
    }

    // 合并身体和手的DH参数类型
    std::vector<int> dh_type = body_.getDhType();
    std::vector<int> hand_dh_type = hand->getDhType();
    dh_type.insert(dh_type.end(), hand_dh_type.begin(), hand_dh_type.end());

    Eigen::Matrix4d base = getPartBase("body");
    Eigen::Matrix4d arm_aim = base.inverse() * aim_mat;

    auto get_matrix = [this, hand, hand_name](const std::vector<double>& theta) { 
        std::vector<double> theta_body(theta.begin(), theta.begin() + body_.getNumJoints());
        std::vector<double> theta_hand(theta.begin() + body_.getNumJoints(), theta.end());
        std::vector<Eigen::Matrix4d> dh_body = body_.getDhMatrix(theta_body);
        std::vector<Eigen::Matrix4d> dh_hand = hand->getDhMatrix(theta_hand);
        dh_hand[0] = this->base_tf_[hand_name] * dh_hand[0];
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
        arm_aim,
        dh_type,
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
        ignore_angle,
        random_flag,
        input_q0
    );

    
}

std::vector<double> Human::controlPartV(const Eigen::VectorXd& aim_v, 
                                        const std::string& part_name) {
    /* 单独控制某个部分速度,输出目标角度 */
    std::shared_ptr<Body> part = getPart(part_name);
    if (!part) return {};

    Eigen::Matrix4d tf2base = getPartBase(part_name);
    Eigen::Matrix3d rot_part = tf2base.block<3, 3>(0, 0);
    
    Eigen::VectorXd vel = rot_part.inverse() * aim_v.head(3);
    Eigen::VectorXd rota = rot_part.inverse() * aim_v.tail(3);
    
    Eigen::VectorXd baseaim_v(6);
    baseaim_v << vel, rota;

    std::vector<double> start_theta = part->getTheta();

    return part->calQposV(start_theta, baseaim_v);
}

int Human::getNumJoints() {
    return num_joints_;
}

} // namespace arm_robot