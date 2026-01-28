#include "robot_ctrl.h"
#include <iostream>
#include <tuple>
#include <vector>

int main() {
    try {
        // Initialize robot controller with the darwin_02.json configuration
        arm_robot::RobotCtrl robot_ctrl("../cpp/darwin_02.json");
        
        // Initialize a 3D map (10x10x5 meters with 0.1m resolution)
        robot_ctrl.initMap(10, 10, 5, 0.1, 0.0, 0.0, 0.0);
        
        // Set obstacle inflation radius
        robot_ctrl.setInflationRadius(0.2);
        
        // Add some obstacles
        robot_ctrl.addObstacle(5.0, 5.0, 1.0);
        robot_ctrl.addObstacle(5.0, 6.0, 1.0);
        robot_ctrl.addObstacle(5.0, 7.0, 1.0);
        
        // Plan a path
        auto start = std::make_tuple(1.0, 1.0, 1.0);
        auto goal = std::make_tuple(9.0, 9.0, 1.0);
        auto path = robot_ctrl.planPath(start, goal);
        
        std::cout << "Planned path with " << path.size() << " waypoints" << std::endl;
        
        // Smooth the path
        auto smoothed_path = robot_ctrl.smoothPath();
        std::cout << "Smoothed path with " << smoothed_path.size() << " waypoints" << std::endl;
        
        // Test collision detection
        auto test_pose = std::make_tuple(5.0, 5.0, 1.0);  // Position with obstacle
        bool is_collision = robot_ctrl.isCollisionAtPose(test_pose);
        std::cout << "Collision at obstacle position: " << (is_collision ? "Yes" : "No") << std::endl;
        
        // Test robot state
        auto robot_state = robot_ctrl.getRobotState();
        std::cout << "Robot has " << robot_state.size() << " joints" << std::endl;
        
        // Example of controlling robot parts (not executed in this example due to complexity)
        /*
        Eigen::Matrix4d target_pose = Eigen::Matrix4d::Identity();
        target_pose(0, 3) = 0.5;  // X translation
        target_pose(1, 3) = 0.3;  // Y translation
        target_pose(2, 3) = 0.8;  // Z translation
        
        auto result = robot_ctrl.controlPart(target_pose, "left");
        if (result.first) {
            std::cout << "Successfully moved left hand to target pose" << std::endl;
        } else {
            std::cout << "Failed to move left hand to target pose" << std::endl;
        }
        */
        
        std::cout << "RobotCtrl example completed successfully!" << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}