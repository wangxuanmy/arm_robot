#include "darwin.h"
#include "dh.h"  // Include the header that contains cal_qpos
#include <iostream>

namespace arm_robot {

Human::Human(const std::vector<std::vector<double>>& body_dh, 
             const std::vector<std::vector<double>>& left_hand_dh,
             const std::vector<std::vector<double>>& right_hand_dh,
             const std::vector<std::vector<double>>& head_dh,
             const Eigen::MatrixXd& theta_limit_input,
             const Eigen::MatrixXd& base_offset,
             const std::vector<std::string>& arm_types)
    {
    
    if (theta_limit_input.rows() == 0) {
        // 在创建实际对象之前，先使用DH参数的尺寸来初始化
        num_joints_ = body_dh.size() + left_hand_dh.size() + right_hand_dh.size() + head_dh.size();
        theta_limit_.resize(num_joints_, 2);
        theta_limit_.col(0).setConstant(-360.0 / 57.2958);
        theta_limit_.col(1).setConstant(360.0 / 57.2958);
    } else {
        theta_limit_ = theta_limit_input;
        num_joints_ = theta_limit_input.rows();
    }

    // 创建各部分实例，根据arm_types决定使用哪个类
    if (arm_types.size() > 0 && arm_types[0] == "marvin") {
        body_ptr_ = std::make_shared<MarvinArm>(body_dh, theta_limit_.block(0, 0, body_dh.size(), 2));
    } else {
        body_ptr_ = std::make_shared<Body>(body_dh, theta_limit_.block(0, 0, body_dh.size(), 2));
    }

    int body_joints = body_dh.size();
    if (arm_types.size() > 1 && arm_types[1] == "marvin") {
        left_hand_ptr_ = std::make_shared<MarvinArm>(left_hand_dh, theta_limit_.block(body_joints, 0, left_hand_dh.size(), 2));
    } else {
        left_hand_ptr_ = std::make_shared<Body>(left_hand_dh, theta_limit_.block(body_joints, 0, left_hand_dh.size(), 2));
    }

    
    int left_joints = left_hand_dh.size();
    if (arm_types.size() > 2 && arm_types[2] == "marvin") {
        right_hand_ptr_ = std::make_shared<MarvinArm>(right_hand_dh, theta_limit_.block(body_joints + left_joints, 0, right_hand_dh.size(), 2));
    } else {
        right_hand_ptr_ = std::make_shared<Body>(right_hand_dh, theta_limit_.block(body_joints + left_joints, 0, right_hand_dh.size(), 2));
    }

    if (!head_dh.empty()) {
        int right_joints = right_hand_dh.size();
        if (arm_types.size() > 3 && arm_types[3] == "marvin") {
            head_ptr_ = std::make_shared<MarvinArm>(head_dh, theta_limit_.block(body_joints + left_joints + right_joints, 0, head_dh.size(), 2));
        } else {
            head_ptr_ = std::make_shared<Body>(head_dh, theta_limit_.block(body_joints + left_joints + right_joints, 0, head_dh.size(), 2));
        }
    } else {
        head_ptr_ = nullptr;
    }

    
    hand_id_ = {{"body", 0}, {"left", 1}, {"right", 2}, {"head", 3}};

    // 重新设置num_joints_为实际对象的总关节数
    num_joints_ = body_ptr_->getNumJoints() + left_hand_ptr_->getNumJoints() + 
                  right_hand_ptr_->getNumJoints();
    if (head_ptr_) {
        num_joints_ += head_ptr_->getNumJoints();
    }

    // 设置各个部分的角度限制 - 使用实际对象的关节数
    int body_joints_actual = body_ptr_->getNumJoints();
    int left_joints_actual = left_hand_ptr_->getNumJoints();
    int right_joints_actual = right_hand_ptr_->getNumJoints();
    int head_joints_actual = head_ptr_ ? head_ptr_->getNumJoints() : 0;

    Eigen::MatrixXd body_limits = theta_limit_.block(0, 0, body_joints_actual, 2);
    Eigen::MatrixXd left_limits = theta_limit_.block(body_joints_actual, 0, left_joints_actual, 2);
    Eigen::MatrixXd right_limits = theta_limit_.block(body_joints_actual + left_joints_actual, 0, right_joints_actual, 2);
    Eigen::MatrixXd head_limits;
    
    if (head_ptr_) {
        head_limits = theta_limit_.block(body_joints_actual + left_joints_actual + right_joints_actual, 0, head_joints_actual, 2);
        head_ptr_->setThetaLimit(head_limits);
    }

    body_ptr_->setThetaLimit(body_limits);
    left_hand_ptr_->setThetaLimit(left_limits);
    right_hand_ptr_->setThetaLimit(right_limits);


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

        if(head_ptr_)
            base_tf_["head"] = coord::euler_and_coord_to_matrix(base_offset(3, 0), base_offset(3, 1), base_offset(3, 2), base_offset(3, 3), base_offset(3, 4), base_offset(3, 5));
    }
    
}

void Human::flashTheta(const std::vector<double>& theta_input) {
    int body_joints = body_ptr_->getNumJoints();
    int left_joints = left_hand_ptr_->getNumJoints();
    int right_joints = right_hand_ptr_->getNumJoints();

    std::vector<double> body_theta(theta_input.begin(), theta_input.begin() + body_joints);
    std::vector<double> left_theta(theta_input.begin() + body_joints, 
                                  theta_input.begin() + body_joints + left_joints);
    std::vector<double> right_theta(theta_input.begin() + body_joints + left_joints, 
                                   theta_input.begin() + body_joints + left_joints + right_joints);
    
    body_ptr_->setTheta(body_theta);
    left_hand_ptr_->setTheta(left_theta);
    right_hand_ptr_->setTheta(right_theta);
    
    if (head_ptr_) {
        std::vector<double> head_theta(theta_input.begin() + body_joints + left_joints + right_joints, 
                                       theta_input.end());
        head_ptr_->setTheta(head_theta);
    }
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
    std::vector<Eigen::Matrix4d> body_mat = body_ptr_->getDhMatrix(body_ptr_->getTheta());

    std::vector<Eigen::Matrix4d> body_end = getAxisTf(base_tf_["body"], body_mat);

    std::vector<Eigen::Matrix4d> left_end = getAxisTf(
        body_end.back() * base_tf_["left"], 
        left_hand_ptr_->getDhMatrix(left_hand_ptr_->getTheta())
    );

    std::vector<Eigen::Matrix4d> right_end = getAxisTf(
        body_end.back() * base_tf_["right"], 
        right_hand_ptr_->getDhMatrix(right_hand_ptr_->getTheta())
    );

    std::vector<Eigen::Matrix4d> head_end;
    if (head_ptr_) {
        head_end = getAxisTf(
            body_end.back() * base_tf_["head"], 
            head_ptr_->getDhMatrix(head_ptr_->getTheta())
        );
    }

    body_end.insert(body_end.end(), left_end.begin(), left_end.end());
    body_end.insert(body_end.end(), right_end.begin(), right_end.end());
    if (head_ptr_) {
        body_end.insert(body_end.end(), head_end.begin(), head_end.end());
    }

    return body_end;
}

std::vector<Eigen::Matrix4d> Human::getEef() {
    /* 获取末端的齐次矩阵 */
    Eigen::Matrix4d base = base_tf_["body"];

    std::vector<Eigen::Matrix4d> body_mat = body_ptr_->getDhMatrix(body_ptr_->getTheta());

    Eigen::Matrix4d tf = base;

    for (const auto& i : body_mat) {
        tf = tf * i;
    }

    Eigen::Matrix4d body_end = tf;

    base = tf;

    tf = base * base_tf_["left"];
    body_mat = left_hand_ptr_->getDhMatrix(left_hand_ptr_->getTheta());
    for (const auto& i : body_mat) {
        tf = tf * i;
    }
    Eigen::Matrix4d body_left = tf;

    tf = base * base_tf_["right"];
    body_mat = right_hand_ptr_->getDhMatrix(right_hand_ptr_->getTheta());
    for (const auto& i : body_mat) {
        tf = tf * i;
    }
    Eigen::Matrix4d body_right = tf;

    Eigen::Matrix4d body_head;
    if (head_ptr_) {
        tf = base * base_tf_["head"];
        body_mat = head_ptr_->getDhMatrix(head_ptr_->getTheta());
        for (const auto& i : body_mat) {
            tf = tf * i;
        }
        body_head = tf;
    } else {
        // 如果没有头部，返回单位矩阵
        body_head = Eigen::Matrix4d::Identity();
    }

    return {body_end, body_left, body_right, body_head};
}

Eigen::Matrix4d Human::getPartBase(const std::string& part_name) {
    /* 获取某部分的基座矩阵 */
    if (part_name == "body") {
        return base_tf_[part_name];
    } else if (part_name == "left") {
        Eigen::Matrix4d body_eef = getEef()[0];
        return body_eef * base_tf_[part_name];
    } else if (part_name == "right") {
        Eigen::Matrix4d body_eef = getEef()[0];
        return body_eef * base_tf_[part_name];
    } else if (part_name == "head") {
        Eigen::Matrix4d body_eef = getEef()[0];
        return body_eef * base_tf_[part_name];
    }
    return Eigen::Matrix4d::Identity();
}

Eigen::Matrix4d Human::getEefByName(const std::string& part_name) {
    /* 获取特定部分的末端执行器 */
    if (part_name == "body") {
        return base_tf_[part_name] * body_ptr_->getEef();
    } else if (part_name == "left") {
        Eigen::Matrix4d body_eef = body_ptr_->getEef();
        return base_tf_["body"] * body_eef * base_tf_["left"] * left_hand_ptr_->getEef();
    } else if (part_name == "right") {
        Eigen::Matrix4d body_eef = body_ptr_->getEef();
        return base_tf_["body"] * body_eef * base_tf_["right"] * right_hand_ptr_->getEef();
    } else if (part_name == "head" && head_ptr_) {
        Eigen::Matrix4d body_eef = body_ptr_->getEef();
        return base_tf_["body"] * body_eef * base_tf_["head"] * head_ptr_->getEef();
    } else if (part_name == "head" && !head_ptr_) {
        // 如果请求头部但没有头部，则返回单位矩阵
        return Eigen::Matrix4d::Identity();
    }
    return Eigen::Matrix4d::Identity();
}

std::shared_ptr<Body> Human::getPart(const std::string& part_name) {
    // 返回部件的共享指针
    if (part_name == "body") {
        return body_ptr_;
    } else if (part_name == "left") {
        return left_hand_ptr_;
    } else if (part_name == "right") {
        return right_hand_ptr_;
    } else if (part_name == "head") {
        return head_ptr_;
    }
    return nullptr;
}

std::vector<double> Human::getAllTheta() {
    std::vector<double> all_theta = body_ptr_->getTheta();
    std::vector<double> left_theta = left_hand_ptr_->getTheta();
    std::vector<double> right_theta = right_hand_ptr_->getTheta();
    
    all_theta.insert(all_theta.end(), left_theta.begin(), left_theta.end());
    all_theta.insert(all_theta.end(), right_theta.begin(), right_theta.end());
    
    // 只有当head_ptr_存在时才添加头部关节角度
    if (head_ptr_) {
        std::vector<double> head_theta = head_ptr_->getTheta();
        all_theta.insert(all_theta.end(), head_theta.begin(), head_theta.end());
    }
    
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
        hand = left_hand_ptr_;
    } else if (hand_name == "right") {
        hand = right_hand_ptr_;
    } else if (hand_name == "head") {
        hand = head_ptr_;
    }
    if (!hand) {
        return {};
    }

    // 合并身体和手的DH参数类型
    std::vector<int> dh_type = body_ptr_->getDhType();
    std::vector<int> hand_dh_type = hand->getDhType();
    dh_type.insert(dh_type.end(), hand_dh_type.begin(), hand_dh_type.end());

    Eigen::Matrix4d base = getPartBase("body");
    Eigen::Matrix4d arm_aim = base.inverse() * aim_mat;

    auto get_matrix = [this, hand, hand_name](const std::vector<double>& theta) { 
        std::vector<double> theta_body(theta.begin(), theta.begin() + body_ptr_->getNumJoints());
        std::vector<double> theta_hand(theta.begin() + body_ptr_->getNumJoints(), theta.end());
        std::vector<Eigen::Matrix4d> dh_body = body_ptr_->getDhMatrix(theta_body);
        std::vector<Eigen::Matrix4d> dh_hand = hand->getDhMatrix(theta_hand);
        dh_hand[0] = this->base_tf_[hand_name] * dh_hand[0];
        dh_body.insert(dh_body.end(), dh_hand.begin(), dh_hand.end());
        return dh_body;
    };

    std::vector<double> start_theta = body_ptr_->getTheta();
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
            std::vector<double> theta_body(theta.begin(), theta.begin() + body_ptr_->getNumJoints());
            std::vector<double> theta_hand(theta.begin() + body_ptr_->getNumJoints(), theta.end());
            Eigen::MatrixXd body_limit = body_ptr_->getThetaLimit(theta_body);
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
    int total_joints = body_ptr_->getNumJoints() + left_hand_ptr_->getNumJoints() + right_hand_ptr_->getNumJoints();
    if (head_ptr_) {
        total_joints += head_ptr_->getNumJoints();
    }
    return total_joints;
}

} // namespace arm_robot