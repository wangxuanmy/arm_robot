#include "collision_detector.h"
#include <iostream>
#include <fcl/geometry/shape/box.h>
#include <fcl/geometry/shape/sphere.h>
#include <fcl/geometry/shape/cylinder.h>
#include <fcl/narrowphase/collision.h>
#include <fcl/narrowphase/distance.h>
#include <fcl/broadphase/broadphase_collision_manager.h>
#include <fcl/broadphase/broadphase_dynamic_AABB_tree.h>

namespace arm_robot {

CollisionDetector::CollisionDetector(std::shared_ptr<UrdfProcessor> processor) 
    : urdf_processor_(processor) {
    manager_ = std::make_shared<fcl::DynamicAABBTreeCollisionManagerd>();
}

bool CollisionDetector::setupCollisionGeometries() {
    if (!urdf_processor_) {
        std::cerr << "No URDF processor provided" << std::endl;
        return false;
    }

    auto robot_model = urdf_processor_->getModel();
    if (!robot_model) {
        std::cerr << "No robot model loaded in processor" << std::endl;
        return false;
    }

    // Clear existing collision objects
    link_collision_objs_.clear();
    link_to_collision_obj_idx_.clear();
    manager_->clear();

    // Iterate through all links in the URDF model
    for (const auto& link_pair : robot_model->links_) {
        const auto& link = link_pair.second;
        
        if (!link->collision) continue;  // Skip links without collision geometry
        
        std::shared_ptr<fcl::CollisionGeometryd> geom;
        
        // Create collision geometry based on URDF shape type
        switch (link->collision->geometry->type) {
            case urdf::Geometry::SPHERE: {
                auto sphere = std::dynamic_pointer_cast<urdf::Sphere>(link->collision->geometry);
                geom = std::shared_ptr<fcl::Sphered>(new fcl::Sphered(sphere->radius));
                break;
            }
            case urdf::Geometry::BOX: {
                auto box = std::dynamic_pointer_cast<urdf::Box>(link->collision->geometry);
                geom = std::shared_ptr<fcl::Boxd>(new fcl::Boxd(
                    box->dim.x, 
                    box->dim.y, 
                    box->dim.z
                ));
                break;
            }
            case urdf::Geometry::CYLINDER: {
                auto cylinder = std::dynamic_pointer_cast<urdf::Cylinder>(link->collision->geometry);
                geom = std::shared_ptr<fcl::Cylinderd>(new fcl::Cylinderd(
                    cylinder->radius, 
                    cylinder->length
                ));
                break;
            }
            case urdf::Geometry::MESH: {
                // For mesh, we'll approximate with a bounding box
                auto mesh = std::dynamic_pointer_cast<urdf::Mesh>(link->collision->geometry);
                
                // Create a rough bounding box for the mesh
                // In a real implementation, you would load the actual mesh
                geom = std::shared_ptr<fcl::Boxd>(new fcl::Boxd(0.1, 0.1, 0.1));
                break;
            }
            default:
                std::cerr << "Unsupported geometry type for link: " << link->name << std::endl;
                continue;
        }
        
        // Create collision object
        auto collision_obj = std::shared_ptr<fcl::CollisionObjectd>(new fcl::CollisionObjectd(geom));
        
        // Store in our mappings
        link_collision_objs_[link->name] = collision_obj;
        link_to_collision_obj_idx_[link->name] = link_collision_objs_.size() - 1;
        
        // Add to collision manager
        manager_->registerObject(collision_obj.get());
    }
    
    // Initialize the collision manager
    manager_->setup();
    
    return true;
}

bool CollisionDetector::updateTransforms(const std::map<std::string, double>& joint_positions) {
    if (!urdf_processor_) {
        std::cerr << "No URDF processor provided" << std::endl;
        return false;
    }
    
    // Get the transforms from URDF processor
    auto link_transforms = urdf_processor_->getLinkTransforms(joint_positions);
    
    // Update the collision objects' transforms
    for (const auto& transform_pair : link_transforms) {
        const std::string& link_name = transform_pair.first;
        const Eigen::Matrix4d& transform = transform_pair.second;
        
        // Find the corresponding collision object
        auto collision_obj_it = link_collision_objs_.find(link_name);
        if (collision_obj_it != link_collision_objs_.end()) {
            auto collision_obj = collision_obj_it->second;
            
            // Convert Eigen matrix to FCL transform
            fcl::Transform3d fcl_transform;
            fcl_transform.matrix() = transform.cast<double>();
            
            // Set the new transform
            collision_obj->setTransform(fcl_transform);
            collision_obj->computeAABB();  // Recompute AABB after transform change
        }
    }
    
    // Update the collision manager
    manager_->update();
    
    return true;
}

bool CollisionDetector::checkCollisionBetweenLinks(const std::string& link1_name, 
                                                  const std::string& link2_name, 
                                                  const std::map<std::string, double>& joint_positions) {
    // Update transforms first
    if (!updateTransforms(joint_positions)) {
        std::cerr << "Failed to update transforms" << std::endl;
        return false;
    }
    
    // Get the collision objects for the specified links
    auto obj1_it = link_collision_objs_.find(link1_name);
    auto obj2_it = link_collision_objs_.find(link2_name);
    
    if (obj1_it == link_collision_objs_.end()) {
        std::cerr << "Link " << link1_name << " not found in collision objects" << std::endl;
        return false;
    }
    
    if (obj2_it == link_collision_objs_.end()) {
        std::cerr << "Link " << link2_name << " not found in collision objects" << std::endl;
        return false;
    }
    
    auto obj1 = obj1_it->second;
    auto obj2 = obj2_it->second;
    
    // Perform collision test between the two objects
    fcl::CollisionRequestd request;
    fcl::CollisionResultd result;
    
    fcl::collide(obj1.get(), obj2.get(), request, result);
    
    return result.isCollision();
}

bool CollisionDetector::checkEnvironmentCollision(const std::map<std::string, double>& joint_positions,
                                                const std::vector<std::vector<double>>& obstacles) {
    // Update transforms first
    if (!updateTransforms(joint_positions)) {
        std::cerr << "Failed to update transforms" << std::endl;
        return false;
    }
    
    // Process environment obstacles
    for (const auto& obstacle : obstacles) {
        if (obstacle.size() < 4) {
            std::cerr << "Invalid obstacle definition (need at least x, y, z, radius)" << std::endl;
            continue;
        }
        
        // Create a sphere to represent the obstacle
        auto obstacle_geom = std::shared_ptr<fcl::Sphered>(new fcl::Sphered(obstacle[3]));  // radius
        fcl::Transform3d obstacle_tf;
        obstacle_tf.translation() = fcl::Vector3d(obstacle[0], obstacle[1], obstacle[2]);  // x, y, z
        
        auto obstacle_obj = std::shared_ptr<fcl::CollisionObjectd>(new fcl::CollisionObjectd(obstacle_geom, obstacle_tf));
        
        // Check collision with all robot links
        std::vector<fcl::CollisionObjectd*> coll_objects;
        manager_->getObjects(coll_objects);
        
        for (auto obj : coll_objects) {
            fcl::CollisionRequestd request;
            fcl::CollisionResultd result;
            
            fcl::collide(obj, obstacle_obj.get(), request, result);
            
            if (result.isCollision()) {
                return true;  // Found a collision
            }
        }
    }
    
    return false;  // No collision detected
}

bool CollisionDetector::checkSelfCollision(const std::map<std::string, double>& joint_positions) {
    // Update transforms first
    if (!updateTransforms(joint_positions)) {
        std::cerr << "Failed to update transforms" << std::endl;
        return false;
    }
    
    // Create a temporary collision request and result
    fcl::CollisionRequestd request;
    fcl::CollisionResultd result;
    
    // For self-collision, we need to perform pairwise checks between all objects
    std::vector<fcl::CollisionObjectd*> coll_objects;
    manager_->getObjects(coll_objects);
    
    // Check all pairs of objects for collision
    for (size_t i = 0; i < coll_objects.size(); ++i) {
        for (size_t j = i + 1; j < coll_objects.size(); ++j) {
            fcl::collide(coll_objects[i], coll_objects[j], request, result);
            if (result.isCollision()) {
                return true;  // Found a collision
            }
        }
    }
    
    return false;  // No collision detected
}

std::map<std::string, Eigen::Matrix4d> CollisionDetector::getLinkTransforms(
    const std::map<std::string, double>& joint_positions) const {
    
    if (!urdf_processor_) {
        std::cerr << "No URDF processor provided" << std::endl;
        return {};
    }
    
    return urdf_processor_->getLinkTransforms(joint_positions);
}

} // namespace arm_robot