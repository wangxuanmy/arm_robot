#include "robot_ctrl.h"
#include "urdf_processor.h"
#include "collision_detector.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <cmath>

namespace arm_robot {

// Forward declaration of the URDF processor and collision detector
static std::shared_ptr<UrdfProcessor> urdf_processor_ = nullptr;
static std::shared_ptr<CollisionDetector> collision_detector_ = nullptr;

RobotCtrl::RobotCtrl(const std::string& json_config_path) {
    if (!json_config_path.empty()) {
        loadConfig(json_config_path);
    }
    initRobot();
}

void RobotCtrl::loadConfig(const std::string& json_config_path) {
    config_path_ = json_config_path;
    
    // Read JSON file
    std::ifstream file(json_config_path);
    if (!file.is_open()) {
        throw std::runtime_error("Cannot open config file: " + json_config_path);
    }
    
    // Parse JSON content
    Json::Value root;
    Json::Reader reader;
    bool parsingSuccessful = reader.parse(file, root);
    file.close();
    
    if (!parsingSuccessful) {
        throw std::runtime_error("Failed to parse JSON config file");
    }
    
    // Extract joint limits
    const Json::Value& jointMin = root["joint_min"];
    const Json::Value& jointMax = root["joint_max"];
    
    for (int i = 0; i < jointMin.size(); ++i) {
        joint_min_.push_back(jointMin[i].asDouble());
    }
    
    for (int i = 0; i < jointMax.size(); ++i) {
        joint_max_.push_back(jointMax[i].asDouble());
    }
    
    // Extract enabled joints
    const Json::Value& enableJoint = root["enable_joint"];
    for (int i = 0; i < enableJoint.size(); ++i) {
        enable_joint_.push_back(enableJoint[i].asInt());
    }
    
    // Extract joint names
    const Json::Value& jointNames = root["joint_names"];
    for (int i = 0; i < jointNames.size(); ++i) {
        joint_names_.push_back(jointNames[i].asString());
    }
    
    // Extract DH parameters
    const Json::Value& dhParams = root["dh_params"];
    
    const Json::Value& bodyDh = dhParams["body"];
    for (int i = 0; i < bodyDh.size(); ++i) {
        std::vector<double> dh_row;
        for (int j = 0; j < bodyDh[i].size(); ++j) {
            dh_row.push_back(bodyDh[i][j].asDouble());
        }
        dh_params_body_.push_back(dh_row);
    }
    
    const Json::Value& leftHandDh = dhParams["left_hand"];
    for (int i = 0; i < leftHandDh.size(); ++i) {
        std::vector<double> dh_row;
        for (int j = 0; j < leftHandDh[i].size(); ++j) {
            dh_row.push_back(leftHandDh[i][j].asDouble());
        }
        dh_params_left_hand_.push_back(dh_row);
    }
    
    const Json::Value& rightHandDh = dhParams["right_hand"];
    for (int i = 0; i < rightHandDh.size(); ++i) {
        std::vector<double> dh_row;
        for (int j = 0; j < rightHandDh[i].size(); ++j) {
            dh_row.push_back(rightHandDh[i][j].asDouble());
        }
        dh_params_right_hand_.push_back(dh_row);
    }
    
    // Extract initial offsets
    const Json::Value& initOffset = root["init_offset"];
    for (int i = 0; i < initOffset.size(); ++i) {
        std::vector<double> offset_row;
        for (int j = 0; j < initOffset[i].size(); ++j) {
            offset_row.push_back(initOffset[i][j].asDouble());
        }
        init_offset_.push_back(offset_row);
    }
}

void RobotCtrl::initRobot() {
    // Create theta limits matrix
    Eigen::MatrixXd theta_limit_input(joint_min_.size(), 2);
    for (int i = 0; i < joint_min_.size(); ++i) {
        theta_limit_input(i, 0) = joint_min_[i];
        theta_limit_input(i, 1) = joint_max_[i];
    }
    
    // Create default head DH parameters (empty for now)
    std::vector<std::vector<double>> empty_head_dh;
    
    // Create robot instance
    robot_ = std::make_shared<Human>(
        dh_params_body_,
        dh_params_left_hand_,
        dh_params_right_hand_,
        empty_head_dh,
        theta_limit_input
    );
    
    // Create robot copy for planning
    robot_copy_ = std::make_shared<Human>(
        dh_params_body_,
        dh_params_left_hand_,
        dh_params_right_hand_,
        empty_head_dh,
        theta_limit_input
    );
    
    resetToInitialPosition();
}

void RobotCtrl::resetToInitialPosition() {
    if (robot_ && !init_offset_.empty()) {
        // We'll need to implement a method to set the initial positions based on init_offset_
        // For now, we'll just set the body to zeros as a placeholder
        std::vector<double> initial_theta(robot_->getNumJoints(), 0.0);
        
        // Apply initial offset if available
        if (!init_offset_.empty()) {
            // This is a simplified implementation - actual implementation would depend
            // on how the init_offset_ maps to joint angles
            for (int i = 0; i < init_offset_.size() && i < initial_theta.size(); ++i) {
                if (init_offset_[i].size() > 0) {
                    initial_theta[i] = init_offset_[i][0];
                }
            }
        }
        
        robot_->flashTheta(initial_theta);
        if (robot_copy_) {
            robot_copy_->flashTheta(initial_theta);
        }
    }
}

void RobotCtrl::initMap(int width, int height, int depth, double resolution,
                        double origin_x, double origin_y, double origin_z) {
    map3d_ = std::make_shared<Map3D>(width, height, depth, resolution, 
                                     origin_x, origin_y, origin_z);
    path_planner_ = std::make_shared<AStar3D>(map3d_);
    path_smoother_ = std::make_shared<PathSmoother>();
}

void RobotCtrl::setInflationRadius(double radius) {
    if (map3d_) {
        map3d_->setInflationRadius(radius);
    }
}

bool RobotCtrl::addObstacle(double x, double y, double z) {
    if (map3d_) {
        return map3d_->addObstacle(x, y, z);
    }
    return false;
}

int RobotCtrl::addObstaclesFromPointCloud(const std::vector<std::tuple<double, double, double>>& points) {
    if (map3d_) {
        return map3d_->addObstaclesFromPointCloud(points);
    }
    return 0;
}

bool RobotCtrl::removeObstacle(double x, double y, double z) {
    if (map3d_) {
        return map3d_->removeObstacle(x, y, z);
    }
    return false;
}

void RobotCtrl::clearMap() {
    if (map3d_) {
        map3d_->clearObstacles();
    }
}

std::vector<std::tuple<double, double, double>> RobotCtrl::planPath(
    std::tuple<double, double, double> start,
    std::tuple<double, double, double> goal) {
    
    if (path_planner_) {
        current_path_ = path_planner_->plan(start, goal);
        return current_path_;
    }
    
    return {};
}

std::vector<std::tuple<double, double, double>> RobotCtrl::smoothPath(
    const std::vector<std::tuple<double, double, double>>& path,
    double smoothing_factor,
    int num_points,
    bool with_obstacle_avoidance) {
    
    std::vector<std::tuple<double, double, double>> path_to_smooth = path;
    
    // If no path provided, use the most recent planned path
    if (path_to_smooth.empty()) {
        path_to_smooth = current_path_;
    }
    
    if (path_to_smooth.empty()) {
        return {};
    }
    
    if (with_obstacle_avoidance && map3d_ && path_smoother_) {
        smoothed_path_ = path_smoother_->smoothPathWithObstacleAvoidance(
            path_to_smooth, map3d_, smoothing_factor, num_points);
    } else if (path_smoother_) {
        smoothed_path_ = path_smoother_->smoothPath(path_to_smooth, smoothing_factor, num_points);
    } else {
        return {};
    }
    
    return smoothed_path_;
}


void RobotCtrl::updateRobotTheta(const std::vector<double>& theta) {
    if (robot_) {
        robot_->flashTheta(theta);
    }
}

std::vector<double> RobotCtrl::getRobotState() {
    if (robot_) {
        return robot_->getAllTheta();
    }
    return {};
}

bool RobotCtrl::isCollisionAtPose(const std::tuple<double, double, double>& pose, 
                                 double safety_margin) {
    if (map3d_) {
        double x, y, z;
        std::tie(x, y, z) = pose;
        
        // Check if current position has obstacles
        if (map3d_->isObstacle(x, y, z)) {
            return true;
        }
        
        // Check surrounding area for obstacles (considering safety margin)
        double resolution = map3d_->getResolution();
        for (double dx = -safety_margin; dx < safety_margin; dx += resolution) {
            for (double dy = -safety_margin; dy < safety_margin; dy += resolution) {
                for (double dz = -safety_margin; dz < safety_margin; dz += resolution) {
                    if (map3d_->isObstacle(x + dx, y + dy, z + dz)) {
                        return true;
                    }
                }
            }
        }
        return false;
    }
    return false;
}

std::vector<std::vector<double>> RobotCtrl::smoothJointAngles(
    const std::vector<std::vector<double>>& thetas,
    const Eigen::MatrixXd& theta_min,
    const Eigen::MatrixXd& theta_max,
    double max_threshold
    ){
    // 边界条件：输入序列长度<2时直接返回
    if (thetas.size() < 2) {
        return thetas;
    }

    // 校验输入参数维度一致性
    int num_joints = thetas[0].size();
    if (theta_min.cols() != 2 || theta_max.cols() != 2 || 
        theta_min.rows() != num_joints || theta_max.rows() != num_joints) {
        throw std::invalid_argument("theta_min/theta_max的行数必须和关节数一致");
    }

    std::vector<std::vector<double>> smoothed_thetas;
    smoothed_thetas.push_back(thetas[0]);  // 初始化平滑序列为第一个点

    // 辅助函数：检查单个关节角度是否在限位内
    auto is_joint_within_limits = [&](double angle, int joint_idx) {
        return (angle >= theta_min(joint_idx, 0)) && (angle <= theta_max(joint_idx, 1));
    };

    // 遍历所有时间步，逐点平滑
    for (int i = 1; i < thetas.size(); ++i) {
        const std::vector<double>& prev = smoothed_thetas.back();  // 上一个平滑点
        const std::vector<double>& curr = thetas[i];               // 当前原始点


        std::vector<double> adjusted_curr(num_joints);

        // 计算相邻点的最大角度差，判断是否需要插值
        double max_diff = 0.0;
        for (int j = 0; j < num_joints; ++j) {
            if(is_joint_within_limits(curr[j], j)){
                adjusted_curr[j] = curr[j];  
            }
            else{
                if(curr[j] > theta_max(j, 1)){
                    adjusted_curr[j] = theta_max(j, 1);
                }
                else{
                    adjusted_curr[j] = theta_min(j, 0);
                }
            }
            double diff = std::abs(adjusted_curr[j] - prev[j]);
            max_diff = std::max(max_diff, diff);
        }

        // 差值超过阈值时，插入插值点
        if (max_diff > max_threshold) {
            int num_insert = static_cast<int>(ceil(max_diff / max_threshold));
            for (int k = 1; k < num_insert; ++k) {
                double ratio = static_cast<double>(k) / num_insert;
                std::vector<double> interpolated(num_joints);
                for (int j = 0; j < num_joints; ++j) {
                    interpolated[j] = prev[j] + ratio * (adjusted_curr[j] - prev[j]);
                }
                smoothed_thetas.push_back(interpolated);
            }
        }

        // 添加调整后的当前点到平滑序列
        smoothed_thetas.push_back(adjusted_curr);
    }

    return smoothed_thetas;
}


void RobotCtrl::syncBodyAndCopy() {
    if (robot_ && robot_copy_) {
        robot_copy_->flashTheta(robot_->getAllTheta());
    }
}

bool RobotCtrl::isSameTheta(std::string part_name, double theta_threshold) { 
    if (!robot_ || !robot_copy_) {
        return false;
    }
    std::vector<double> body = robot_->getPart(part_name)->getTheta();
    std::vector<double> body_copy = robot_copy_->getPart(part_name)->getTheta();
    for (int i = 0; i < body.size(); i++) {
        if (std::abs(body[i] - body_copy[i]) > theta_threshold) {
            return false;
        }
    }
    return true;
}

std::pair<bool, std::vector<std::vector<double>>> RobotCtrl::findFollowPath(const Eigen::Matrix4d& aim_mat,
    const std::string& part_name,
    bool ignore_angle){

    if (!robot_copy_) {
        return std::make_pair(false, std::vector<std::vector<double>>());
    }
    
    std::shared_ptr<Body> part = robot_copy_->getPart(part_name);  
    Eigen::Matrix4d part_base = robot_copy_->getPartBase(part_name);  
    Eigen::Matrix4d start_mat = robot_copy_->getEefByName(part_name);  
    std::vector<double> start_theta = part->getTheta();
    std::vector<std::vector<double>> thetas;
    
    // 第一步检查目标是否可达
    std::pair<bool, std::vector<std::vector<double>>> back_pair = robot_copy_->controlPart(aim_mat, part_name, ignore_angle);  

    if(back_pair.first){
        for(int i = 0; i < 10; i++){
            // 第二步 尝试10次是否能找到路径
            std::tuple<double, double, double> start = std::make_tuple(start_mat(0,3), start_mat(1,3), start_mat(2,3));
            std::tuple<double, double, double> end = std::make_tuple(aim_mat(0,3), aim_mat(1,3), aim_mat(2,3));
            std::vector<std::tuple<double, double, double>> path = planPath(start, end);

            std::vector<std::vector<double>> thetas_can = std::vector<std::vector<double>>(); 

            part->setTheta(start_theta);

            if(path.size() > 0){

                thetas_can.push_back(start_theta);  

                Eigen::Matrix4d aim_tmp = aim_mat;

                for (auto point : path){
                    // 第三步检测路径的点是否可达/可执行
                    aim_tmp(0,3) = std::get<0>(point);
                    std::pair<bool, std::vector<std::vector<double>>> pair = robot_copy_->controlPart(aim_tmp, part_name, ignore_angle);   

                    if(pair.first){
                        thetas_can.push_back(pair.second.back());  
                    }
                }

                thetas = thetas_can;
                
            }
            else{
                return std::make_pair(false, std::vector<std::vector<double>>());
            }
        }

        thetas.push_back(back_pair.second.back());  
        
        return std::make_pair(true, thetas);
    }
    else{  
        return back_pair;
    }
}

std::pair<bool, std::vector<std::vector<double>>> RobotCtrl::goAim(const Eigen::Matrix4d& aim_mat,const std::string& part_name,
    bool ignore_angle){

    if (!robot_copy_) {
        return std::make_pair(false, std::vector<std::vector<double>>());
    }
    
    return robot_copy_->controlPart(aim_mat, part_name, ignore_angle);  
}

std::vector<double> RobotCtrl::moveViaVelocity(const std::string& part_name, Eigen::VectorXd vel){
    if(isSameTheta(part_name, MOVE_THETA_THRESHOLD_)){
        syncBodyAndCopy();
    }

    std::vector<double> theta = robot_copy_->controlPartV(vel, part_name);  

    robot_copy_->getPart(part_name)->setTheta(theta);  

    return theta;
}

std::pair<bool, std::vector<std::vector<double>>> RobotCtrl::cooperationJoint(const std::string& main_name, const std::string& sub_name,
     const std::vector<std::vector<double>>& theta_input){

        syncBodyAndCopy();

        std::shared_ptr<Body> main_part = robot_copy_->getPart(main_name);   
        std::shared_ptr<Body> sub_part = robot_copy_->getPart(sub_name);   
        Eigen::Matrix4d main_part_base = robot_copy_->getPartBase(main_name);   
        Eigen::Matrix4d sub_part_base = robot_copy_->getPartBase(sub_name);   

        std::vector<std::vector<double>> sub_theta_can = std::vector<std::vector<double>>();   

        std::vector<double> sub_theta = sub_part->getTheta();
        
        main_part->setTheta(theta_input[0]);

        //获得末端位置矩阵
        Eigen::Matrix4d main_mat = robot_copy_->getEefByName(main_name);   
        Eigen::Matrix4d sub_mat = robot_copy_->getEefByName(sub_name);   

        // 记录变换关系
        Eigen::Matrix4d main_to_sub =  main_mat.inverse() * sub_mat;

        bool find_path = true;

        for ( auto i : theta_input){
            main_part->setTheta(i);
            main_mat = robot_copy_->getEefByName(main_name);   
            sub_mat =  main_mat * main_to_sub;
            auto pair = robot_copy_->controlPart(sub_mat, sub_name);   

            if(pair.first){
                sub_theta_can.push_back(pair.second.back());   
            }
            else{
                find_path = false;
                break;
            }
        }

        return std::make_pair(find_path, sub_theta_can);
}



std::pair<bool, std::map<std::string, std::vector<std::vector<double>>>> RobotCtrl::cooperationBody(
     const std::vector<std::string>& sub_name, const std::vector<std::vector<double>>& theta_input){
        syncBodyAndCopy();
        std::string main_name = "body";
        std::shared_ptr<Body> main_part = robot_copy_->getPart(main_name);   
        
        std::map<std::string, std::shared_ptr<Body>> sub_parts;
        std::map<std::string, std::vector<std::vector<double>>> sub_theta_can;  // 修复错误：原来是 sub_theta_can
        std::map<std::string, std::vector<double>> sub_theta;
        std::map<std::string, Eigen::Matrix4d> sub_tf;
        for(auto name : sub_name){
            sub_parts[name] = robot_copy_->getPart(name);   
            sub_tf[name] = robot_copy_->getEefByName(name);   
        }

        bool find_path = true;
        for ( auto i : theta_input){
            main_part->setTheta(i);
            for(auto name : sub_name){
                auto pair = robot_copy_->controlPart(sub_tf[name], name);   

                if(pair.first){
                    sub_theta_can[name].push_back(pair.second.back());   
                }
                else{
                    sub_theta_can[name].push_back(sub_theta[name]);   
                    find_path = false;
                    break;
                }
            }
        }
        return std::make_pair(find_path, sub_theta_can);
}



std::vector<std::vector<double>> RobotCtrl::jointInterpolation(const std::vector<double>& thetas,
    const std::string& part_name,
    double speed){
        syncBodyAndCopy();

        std::shared_ptr<Body> part = robot_copy_->getPart(part_name);   
        std::vector<std::vector<double>> thetas_can = std::vector<std::vector<double>>();   
        thetas_can.push_back(part->getTheta());   
        // 这里检查一下角度序列长度是否一致
        if(thetas.size() != part->getNumJoints()){
            // 抛出异常
            throw std::invalid_argument("jointInterpolation thetas size is not equal to the number of joints");
        }

        thetas_can.push_back(thetas);   
        Eigen::MatrixXd theta_limit = part->getThetaLimit(thetas);

        std::vector<std::vector<double>> tehtas_can = smoothJointAngles(thetas_can, theta_limit.row(0), theta_limit.row(1), speed);

        // 检查在角度序列中，是否有角度超出限制
        for(auto theta : tehtas_can){
            Eigen::MatrixXd theta_limit = part->getThetaLimit(theta);

            // 如果角度序列中的角度超出上下限,则限制在角度限制内
            for (int i = 0; i < theta.size(); i++){
                if(theta[i] < theta_limit(i, 0)){
                    theta[i] = theta_limit(i, 0);
                }
                else if(theta[i] > theta_limit(i, 1)){
                    theta[i] = theta_limit(i, 1);
                }
            }
        }

        return tehtas_can;
}

// Implementation of URDF-based collision detection methods

bool RobotCtrl::loadUrdfModel(const std::string& urdf_file_path) {
    urdf_collision_checker = std::make_unique<UrdfCollisionChecker>();
    
    if (!urdf_collision_checker->loadModelFromFile(urdf_file_path)) {
        std::cerr << "Failed to load URDF model from: " << urdf_file_path << std::endl;
        return false;
    }
    
    if (!urdf_collision_checker->setupCollisionGeometries()) {
        std::cerr << "Failed to set up collision geometries from URDF" << std::endl;
        return false;
    }
    
    return true;
}

bool RobotCtrl::checkSelfCollision(const std::map<std::string, double>& joint_positions) {
    if (!urdf_collision_checker) {
        std::cerr << "URDF collision checker not initialized. Call loadUrdfModel first." << std::endl;
        return false;
    }
    
    return urdf_collision_checker->checkSelfCollision(joint_positions);
}

bool RobotCtrl::checkCollisionBetweenLinks(const std::string& link1_name, 
                                          const std::string& link2_name, 
                                          const std::map<std::string, double>& joint_positions) {
    if (!urdf_collision_checker) {
        std::cerr << "URDF collision checker not initialized. Call loadUrdfModel first." << std::endl;
        return false;
    }
    
    return urdf_collision_checker->checkCollisionBetweenLinks(link1_name, link2_name, joint_positions);
}

bool RobotCtrl::checkEnvironmentCollision(const std::map<std::string, double>& joint_positions,
                                         const std::vector<std::vector<double>>& obstacles) {
    if (!urdf_collision_checker) {
        std::cerr << "URDF collision checker not initialized. Call loadUrdfModel first." << std::endl;
        return false;
    }
    
    return urdf_collision_checker->checkEnvironmentCollision(joint_positions, obstacles);
}

bool RobotCtrl::checkTrajectoryCollision(const std::vector<std::vector<double>>& trajectory) {
    if (!urdf_collision_checker) {
        std::cerr << "URDF collision checker not initialized. Call loadUrdfModel first." << std::endl;
        return false; // Assume no collision if not initialized
    }
    
    // Convert trajectory to joint position maps and check each point
    for (const auto& waypoint : trajectory) {
        // This assumes we have a mapping from joint indices to names
        // In a real implementation, you'd need to map joint values to their names
        std::map<std::string, double> joint_positions;
        
        // Here we're assuming joint names follow a pattern like "joint0", "joint1", etc.
        // This would need to be adapted to your specific robot's joint naming
        for (size_t i = 0; i < waypoint.size(); ++i) {
            joint_positions["joint" + std::to_string(i)] = waypoint[i];
        }
        
        if (checkSelfCollision(joint_positions)) {
            return true; // Collision detected
        }
    }
    
    return false; // No collision detected
}

} // namespace arm_robot