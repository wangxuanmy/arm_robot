#ifndef COLLISION_DETECTOR_H
#define COLLISION_DETECTOR_H

#include "urdf_processor.h"
#include <fcl/broadphase/broadphase_collision_manager.h>
#include <fcl/geometry/shape/box.h>
#include <fcl/geometry/shape/sphere.h>
#include <fcl/geometry/shape/cylinder.h>
#include <fcl/narrowphase/collision_object.h>
#include <fcl/math/bv/OBBRSS.h>

#include <Eigen/Dense>
#include <vector>
#include <map>
#include <memory>
#include <string>

namespace arm_robot {

class CollisionDetector {
private:
    std::shared_ptr<UrdfProcessor> urdf_processor_;
    
    // FCL碰撞对象管理器
    std::shared_ptr<fcl::BroadPhaseCollisionManagerd> manager_;
    
    // 存储链接名称到碰撞对象的映射
    std::map<std::string, std::shared_ptr<fcl::CollisionObjectd>> link_collision_objs_;
    
    // 存储链接名称到碰撞对象索引的映射
    std::map<std::string, size_t> link_to_collision_obj_idx_;

public:
    CollisionDetector(std::shared_ptr<UrdfProcessor> processor);
    
    // Setup collision geometries from URDF model
    bool setupCollisionGeometries();
    
    // Calculate forward kinematics and update transforms
    bool updateTransforms(const std::map<std::string, double>& joint_positions);
    
    // Check collision between two specific links
    bool checkCollisionBetweenLinks(const std::string& link1_name, 
                                   const std::string& link2_name, 
                                   const std::map<std::string, double>& joint_positions);
    
    // Check collision with environment obstacles
    bool checkEnvironmentCollision(const std::map<std::string, double>& joint_positions,
                                  const std::vector<std::vector<double>>& obstacles);
    
    // Check self collision
    bool checkSelfCollision(const std::map<std::string, double>& joint_positions);
    
    // Get transforms of all links after FK calculation
    std::map<std::string, Eigen::Matrix4d> getLinkTransforms(const std::map<std::string, double>& joint_positions) const;
};

} // namespace arm_robot

#endif // COLLISION_DETECTOR_H