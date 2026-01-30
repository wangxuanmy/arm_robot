#include "urdf_processor.h"
#include <urdf_parser/urdf_parser.h>
#include <iostream>
#include <fstream>
#include <sstream>

namespace arm_robot {

UrdfProcessor::UrdfProcessor() {
    robot_model_ = nullptr;
}

bool UrdfProcessor::loadModelFromFile(const std::string& urdf_file_path) {
    std::ifstream file(urdf_file_path);
    if (!file) {
        std::cerr << "Could not open URDF file: " << urdf_file_path << std::endl;
        return false;
    }
    
    std::stringstream buffer;
    buffer << file.rdbuf();
    file.close();
    
    return loadModelFromString(buffer.str());
}

bool UrdfProcessor::loadModelFromString(const std::string& urdf_string) {
    std::shared_ptr<urdf::ModelInterface> model_interface = urdf::parseURDF(urdf_string);
    
    if (!model_interface) {
        std::cerr << "Failed to parse URDF string" << std::endl;
        return false;
    }
    
    robot_model_ = model_interface;
    
    return true;
}

bool UrdfProcessor::computeLinkTransforms(
    const std::map<std::string, double>& joint_positions,
    std::map<std::string, Eigen::Matrix4d>& link_transforms) const {
    
    if (!robot_model_) {
        std::cerr << "No robot model loaded" << std::endl;
        return false;
    }
    
    link_transforms.clear();
    
    auto root_link = robot_model_->getRoot();
    if (!root_link) {
        std::cerr << "No root link found in robot model" << std::endl;
        return false;
    }
    
    Eigen::Matrix4d root_transform = Eigen::Matrix4d::Identity();
    link_transforms[root_link->name] = root_transform;
    
    computeLinkTransformsRecursive(root_link->name, link_transforms, joint_positions);
    
    return true;
}

std::map<std::string, Eigen::Matrix4d> UrdfProcessor::getLinkTransforms(
    const std::map<std::string, double>& joint_positions) const {
    
    std::map<std::string, Eigen::Matrix4d> link_transforms;
    computeLinkTransforms(joint_positions, link_transforms);
    return link_transforms;
}

void UrdfProcessor::computeLinkTransformsRecursive(
    const std::string& parent_link_name,
    std::map<std::string, Eigen::Matrix4d>& link_transforms,
    const std::map<std::string, double>& joint_positions) const {
    
    auto parent_transform_it = link_transforms.find(parent_link_name);
    if (parent_transform_it == link_transforms.end()) {
        return;
    }
    
    auto parent_transform = parent_transform_it->second;
    
    for (const auto& joint_entry : robot_model_->joints_) {
        auto child_joint = joint_entry.second;
        if (!child_joint) continue;
        
        if (child_joint->parent_link_name == parent_link_name) {
            auto child_link = robot_model_->getLink(child_joint->child_link_name);
            if (!child_link) continue;
            
            double joint_pos = 0.0;
            auto joint_pos_it = joint_positions.find(child_joint->name);
            if (joint_pos_it != joint_positions.end()) {
                joint_pos = joint_pos_it->second;
            }
            
            Eigen::Matrix4d joint_transform = Eigen::Matrix4d::Identity();
            
            if (child_joint->type == urdf::Joint::REVOLUTE || child_joint->type == urdf::Joint::CONTINUOUS) {
                Eigen::Vector3d axis(
                    child_joint->axis.x,
                    child_joint->axis.y,
                    child_joint->axis.z
                );
                
                Eigen::Matrix3d rotation = Eigen::AngleAxisd(joint_pos, axis.normalized()).matrix();
                joint_transform.block<3,3>(0,0) = rotation;
            } else if (child_joint->type == urdf::Joint::PRISMATIC) {
                Eigen::Vector3d axis(
                    child_joint->axis.x,
                    child_joint->axis.y,
                    child_joint->axis.z
                );
                
                Eigen::Vector3d translation = axis.normalized() * joint_pos;
                joint_transform.block<3,1>(0,3) = translation;
            }
            
            Eigen::Vector3d xyz(
                child_joint->parent_to_joint_origin_transform.position.x,
                child_joint->parent_to_joint_origin_transform.position.y,
                child_joint->parent_to_joint_origin_transform.position.z
            );
            
            Eigen::Quaterniond quat(
                child_joint->parent_to_joint_origin_transform.rotation.w,
                child_joint->parent_to_joint_origin_transform.rotation.x,
                child_joint->parent_to_joint_origin_transform.rotation.y,
                child_joint->parent_to_joint_origin_transform.rotation.z
            );
            Eigen::Matrix3d rot = quat.matrix();
            
            Eigen::Matrix4d origin_transform = Eigen::Matrix4d::Identity();
            origin_transform.block<3,3>(0,0) = rot;
            origin_transform.block<3,1>(0,3) = xyz;
            
            Eigen::Matrix4d child_transform = parent_transform * origin_transform * joint_transform;
            
            link_transforms[child_link->name] = child_transform;
            
            computeLinkTransformsRecursive(child_link->name, link_transforms, joint_positions);
        }
    }
}

} // namespace arm_robot