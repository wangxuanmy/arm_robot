#include <iostream>
#include <fstream>
#include <sstream>
#include <map>
#include <vector>
#include "collision_detector.h"
#include "urdf_processor.h"

int main() {
    // 创建URDF处理器实例
    auto urdf_processor = std::make_shared<arm_robot::UrdfProcessor>();
    
    // 加载URDF文件
    std::string urdf_path = "/home/std/arm_robot/cpp/darwin02_description/urdf/darwin02_01.urdf";
    
    std::cout << "Loading URDF file: " << urdf_path << std::endl;
    
    if (!urdf_processor->loadModelFromFile(urdf_path)) {
        std::cerr << "Failed to load URDF model" << std::endl;
        return -1;
    }
    
    std::cout << "URDF model loaded successfully!" << std::endl;
    
    // 创建碰撞检测器实例
    arm_robot::CollisionDetector detector(urdf_processor);
    
    // 设置碰撞几何体
    if (!detector.setupCollisionGeometries()) {
        std::cerr << "Failed to setup collision geometries" << std::endl;
        return -1;
    }
    
    std::cout << "Collision geometries setup successfully!" << std::endl;
    
    // 定义一些关节位置进行测试
    std::map<std::string, double> joint_positions;
    
    // 为一些关键关节设置初始角度
    joint_positions["waist_joint"] = 0.0;
    joint_positions["neck_joint"] = 0.0;
    joint_positions["head_joint"] = 0.0;
    
    // 左臂关节
    joint_positions["left_shoulder_pan_joint"] = 0.0;
    joint_positions["left_shoulder_lift_joint"] = 0.0;
    joint_positions["left_elbow_1_joint"] = 0.0;
    joint_positions["left_elbow_2_joint"] = 0.0;
    joint_positions["left_wrist_1_joint"] = 0.0;
    joint_positions["left_wrist_2_joint"] = 0.0;
    joint_positions["left_wrist_3_joint"] = 0.0;
    
    // 右臂关节
    joint_positions["right_shoulder_pan_joint"] = 0.0;
    joint_positions["right_shoulder_lift_joint"] = 0.0;
    joint_positions["right_elbow_1_joint"] = 0.0;
    joint_positions["right_elbow_2_joint"] = 0.0;
    joint_positions["right_wrist_1_joint"] = 0.0;
    joint_positions["right_wrist_2_joint"] = 0.0;
    joint_positions["right_wrist_3_joint"] = 0.0;
    
    std::cout << "Testing forward kinematics..." << std::endl;
    
    // 测试正运动学计算
    if (!detector.updateTransforms(joint_positions)) {
        std::cerr << "Failed to update transforms" << std::endl;
        return -1;
    }
    
    std::cout << "Forward kinematics computed successfully!" << std::endl;
    
    // 获取所有链接的变换
    auto link_transforms = detector.getLinkTransforms(joint_positions);
    
    std::cout << "Retrieved transforms for " << link_transforms.size() << " links" << std::endl;
    
    // 输出一些链接的变换矩阵
    for (const auto& pair : link_transforms) {
        std::cout << "Link: " << pair.first << std::endl;
        std::cout << "Transform:\n" << pair.second << std::endl << std::endl;
        
        // 只输出前几个链接的变换
        static int count = 0;
        if (++count >= 5) break;
    }
    
    // 尝试一些关节角度变化，再次测试
    std::cout << "Testing with different joint angles..." << std::endl;
    
    joint_positions["waist_joint"] = 0.5;
    joint_positions["neck_joint"] = 0.2;
    joint_positions["left_shoulder_pan_joint"] = 0.3;
    joint_positions["right_shoulder_lift_joint"] = -0.3;
    
    if (!detector.updateTransforms(joint_positions)) {
        std::cerr << "Failed to update transforms with new joint angles" << std::endl;
        return -1;
    }
    
    std::cout << "Updated transforms with new joint angles successfully!" << std::endl;
    
    // 再次获取变换以验证变化
    auto new_link_transforms = detector.getLinkTransforms(joint_positions);
    
    std::cout << "Retrieved transforms for " << new_link_transforms.size() << " links with new angles" << std::endl;
    
    // 输出相同链接的新变换以比较
    for (const auto& pair : new_link_transforms) {
        if (pair.first == "base_link" || pair.first == "left_shoulder_pan_link" || 
            pair.first == "right_shoulder_pan_link" || pair.first == "head_link") {
            std::cout << "Link: " << pair.first << " (with new angles)" << std::endl;
            std::cout << "Transform:\n" << pair.second << std::endl << std::endl;
        }
    }
    
    // 测试特定链接之间的碰撞检测
    std::cout << "Testing collision detection between specific links..." << std::endl;
    
    bool collision = detector.checkCollisionBetweenLinks("base_link", "left_shoulder_pan_link", joint_positions);
    std::cout << "Collision between base_link and left_shoulder_pan_link: " << (collision ? "YES" : "NO") << std::endl;
    
    collision = detector.checkCollisionBetweenLinks("base_link", "right_shoulder_pan_link", joint_positions);
    std::cout << "Collision between base_link and right_shoulder_pan_link: " << (collision ? "YES" : "NO") << std::endl;
    
    // 测试环境碰撞检测
    std::vector<std::vector<double>> obstacles = {{0.5, 0.5, 1.0, 0.2}}; // x, y, z, radius
    collision = detector.checkEnvironmentCollision(joint_positions, obstacles);
    std::cout << "Environment collision check: " << (collision ? "YES" : "NO") << std::endl;
    
    // 测试自碰撞检测
    collision = detector.checkSelfCollision(joint_positions);
    std::cout << "Self collision check: " << (collision ? "YES" : "NO") << std::endl;
    
    std::cout << "\nComplete collision detection test completed!" << std::endl;
    
    return 0;
}