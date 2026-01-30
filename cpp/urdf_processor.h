#ifndef URDF_PROCESSOR_H
#define URDF_PROCESSOR_H

#include <urdf_model/model.h>
#include <urdf_parser/urdf_parser.h>
#include <kdl/tree.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/frames.hpp>
// 移除kdl_parser依赖
// #include <kdl_parser/kdl_parser.hpp>

#include <Eigen/Dense>
#include <vector>
#include <map>
#include <memory>
#include <string>

namespace arm_robot {

class UrdfProcessor {
private:
    std::shared_ptr<urdf::ModelInterface> robot_model_;
    // 移除KDL树，因为我们现在直接处理URDF模型
    // KDL::Tree kdl_tree_;

public:
    UrdfProcessor();
    
    // Load URDF model from file
    bool loadModelFromFile(const std::string& urdf_file_path);
    
    // Load URDF model from string
    bool loadModelFromString(const std::string& urdf_string);
    
    // Calculate forward kinematics and update transforms
    bool computeLinkTransforms(const std::map<std::string, double>& joint_positions, 
                               std::map<std::string, Eigen::Matrix4d>& link_transforms) const;
    
    // Get transforms of all links after FK calculation
    std::map<std::string, Eigen::Matrix4d> getLinkTransforms(const std::map<std::string, double>& joint_positions) const;

    // Get the robot model
    std::shared_ptr<urdf::ModelInterface> getModel() const { return robot_model_; }

private:
    // Helper function to recursively compute link transforms
    void computeLinkTransformsRecursive(
        const std::string& parent_link_name,
        std::map<std::string, Eigen::Matrix4d>& link_transforms,
        const std::map<std::string, double>& joint_positions) const;
};

} // namespace arm_robot

#endif // URDF_PROCESSOR_H