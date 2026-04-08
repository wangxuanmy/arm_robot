#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <map>

#include <Eigen/Dense>
#include <Eigen/Geometry>

// Include JSON support
#include <json/json.h>

// Include URDF parser
#include <urdf_model/model.h>
#include <urdf_parser/urdf_parser.h>

// Include project headers
#include "arm_utils/darwin.h"
#include "arm_utils/body_base.h"

using namespace Eigen;

/**
 * @brief DH to URDF Converter - 将DH参数定义的关节关系转换为URDF格式
 * 
 * 该工具将DH模型中的关节关系转换为URDF格式，确保两种模型具有相同的运动学特性
 */
class DhToUrdfConverter {
private:
    std::shared_ptr<arm_robot::Human> robot_dh_;
    std::shared_ptr<urdf::ModelInterface> urdf_model_;
    std::string config_path_;
    std::string urdf_path_;
    std::string output_urdf_path_;
    std::vector<std::string> joints_name_;
    std::map<std::string, int> joint_dh_index_;
    
    // 存储关节映射关系
    std::map<std::string, int> joint_name_to_dh_index_;
    std::vector<std::string> joint_names_ordered_;  // 按照DH顺序排列的关节名

public:
    DhToUrdfConverter(const std::string& config_path, const std::string& urdf_path, const std::string& output_urdf_path)
        : config_path_(config_path), urdf_path_(urdf_path), output_urdf_path_(output_urdf_path) {
        // 加载配置文件
        loadConfig(config_path_);
        
        // 创建机器人模型
        initRobot();
        
        // 加载URDF模型
        if (!loadUrdfModel(urdf_path_)) {
            throw std::runtime_error("Failed to load URDF model from: " + urdf_path_);
        }
        
        // 建立关节名称到DH索引的映射
        establishJointMapping();
    }

    void loadConfig(const std::string& json_config_path) {
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
        
        // Extract DH parameters
        const Json::Value& dhParams = root["dh_params"];
        
        const Json::Value& bodyDh = dhParams["body"];
        std::vector<std::vector<double>> dh_params_body_;
        for (int i = 0; i < bodyDh.size(); ++i) {
            std::vector<double> dh_row;
            for (int j = 0; j < bodyDh[i].size(); ++j) {
                dh_row.push_back(bodyDh[i][j].asDouble());
            }
            dh_params_body_.push_back(dh_row);
        }
        
        const Json::Value& leftHandDh = dhParams["left_hand"];
        std::vector<std::vector<double>> dh_params_left_hand_;
        for (int i = 0; i < leftHandDh.size(); ++i) {
            std::vector<double> dh_row;
            for (int j = 0; j < leftHandDh[i].size(); ++j) {
                dh_row.push_back(leftHandDh[i][j].asDouble());
            }
            dh_params_left_hand_.push_back(dh_row);
        }
        
        const Json::Value& rightHandDh = dhParams["right_hand"];
        std::vector<std::vector<double>> dh_params_right_hand_;
        for (int i = 0; i < rightHandDh.size(); ++i) {
            std::vector<double> dh_row;
            for (int j = 0; j < rightHandDh[i].size(); ++j) {
                dh_row.push_back(rightHandDh[i][j].asDouble());
            }
            dh_params_right_hand_.push_back(dh_row);
        }

        std::vector<std::vector<double>> dh_params_head_;
        if(dhParams.isMember("head")){
            const Json::Value& headDh = dhParams["head"];
            for (int i = 0; i < headDh.size(); ++i) {
                std::vector<double> dh_row;
                for (int j = 0; j < headDh[i].size(); ++j) {
                    dh_row.push_back(headDh[i][j].asDouble());
                }
                dh_params_head_.push_back(dh_row);
            }
        }
        
        // Extract joint limits
        const Json::Value& jointMin = root["joint_min"];
        const Json::Value& jointMax = root["joint_max"];
        std::vector<double> joint_min_, joint_max_;
        for (int i = 0; i < jointMin.size(); ++i) {
            joint_min_.push_back(jointMin[i].asDouble());
        }
        for (int i = 0; i < jointMax.size(); ++i) {
            joint_max_.push_back(jointMax[i].asDouble());
        }
        
        // Extract enabled joints
        const Json::Value& enableJoint = root["enable_joint"];
        std::vector<int> enable_joint;
        for (int i = 0; i < enableJoint.size(); ++i) {
            enable_joint.push_back(enableJoint[i].asInt());
        }

        // Extract joint names
        const Json::Value& jointNames = root["joint_names"];
        std::vector<std::string> joint_names;
        for (int i = 0; i < jointNames.size(); ++i) {
            joint_names.push_back(jointNames[i].asString());
        }

        for (int i = 0; i < enable_joint.size(); ++i) {
            joints_name_.push_back(joint_names[enable_joint[i]]);
        }
        
        
        // Extract initial offsets
        const Json::Value& initOffset = root["init_offset"];
        std::vector<std::vector<double>> init_offset_;
        for (int i = 0; i < initOffset.size(); ++i) {
            std::vector<double> offset_row;
            for (int j = 0; j < initOffset[i].size(); ++j) {
                offset_row.push_back(initOffset[i][j].asDouble());
            }
            init_offset_.push_back(offset_row);
        }
        
        // Extract arm types
        const Json::Value& armTypes = root["arm_type"];
        std::vector<std::string> arm_types_;
        for (int i = 0; i < armTypes.size(); ++i) {
            arm_types_.push_back(armTypes[i].asString());
        }

        // Create theta limits matrix
        Eigen::MatrixXd theta_limit_input(enable_joint.size(), 2);
        for (int i = 0; i < enable_joint.size(); ++i) {
            theta_limit_input(i, 0) = joint_min_[enable_joint[i]];
            theta_limit_input(i, 1) = joint_max_[enable_joint[i]];
        }
        
        // Convert init_offset_ to Eigen::MatrixXd format for base offsets
        Eigen::MatrixXd base_offset_matrix;
        if (!init_offset_.empty()) {
            base_offset_matrix.resize(init_offset_.size(), 6);
            for (size_t i = 0; i < init_offset_.size(); ++i) {
                for (int j = 0; j < 6; ++j) {
                    base_offset_matrix(i, j) = init_offset_[i][j];
                }
            }
        } 

        // Create robot instance with base offset and arm types
        robot_dh_ = std::make_shared<arm_robot::Human>(
            dh_params_body_,
            dh_params_left_hand_,
            dh_params_right_hand_,
            dh_params_head_,
            theta_limit_input,
            base_offset_matrix,
            arm_types_
        );
    }

    void initRobot() {
        // Robot initialization happens in loadConfig
    }

    bool loadUrdfModel(const std::string& urdf_path) {
        std::ifstream stream(urdf_path);
        if (!stream) {
            std::cerr << "Could not find URDF file: " << urdf_path << std::endl;
            return false;
        }
        
        std::stringstream sstr;
        sstr << stream.rdbuf();
        std::string urdf_string = sstr.str();
        
        urdf_model_ = urdf::parseURDF(urdf_string);
        if (!urdf_model_) {
            std::cerr << "Could not parse URDF file: " << urdf_path << std::endl;
            return false;
        }
        
        std::cout << "Loaded URDF model with " << urdf_model_->joints_.size() << " joints" << std::endl;
        return true;
    }

    void establishJointMapping() {
        // 按照URDF中出现的顺序记录可映射的关节
        for (const auto& joint_pair : urdf_model_->joints_) {
            auto joint = joint_pair.second;
            // 对于旋转关节才进行映射
            if (joint->type == urdf::Joint::REVOLUTE || joint->type == urdf::Joint::CONTINUOUS) {
                int dh_idx = joint_names_ordered_.size();
                if (dh_idx < robot_dh_->getNumJoints()) {
                    joint_name_to_dh_index_[joint->name] = dh_idx;
                    joint_names_ordered_.push_back(joint->name);
                }
            }
        }
        
        std::cout << "Established mapping for " << joint_name_to_dh_index_.size() << " joints" << std::endl;
    }

    /**
     * @brief 将DH模型的关节关系转换为URDF格式
     */
    bool convertDhToUrdf() {
        if (!robot_dh_ || !urdf_model_) {
            std::cerr << "Robot model or URDF model not loaded" << std::endl;
            return false;
        }

        std::cout << "Converting DH model to URDF with " << robot_dh_->getNumJoints() << " joints..." << std::endl;

        // 设置零位姿态
        std::vector<double> zero_pose(robot_dh_->getNumJoints(), 0.0);
        robot_dh_->flashTheta(zero_pose);

        // 获取DH模型中所有关节的变换矩阵（相对于基座）
        std::vector<Matrix4d> dh_tf = robot_dh_->getAllTf();
        int num_joints = robot_dh_->getNumJoints();
        std::cout << "Got " << num_joints << " transforms from DH model" << dh_tf.size() << std::endl;

        int boyd_joint_num = robot_dh_->getPart("body")->getNumJoints();
        int left_hand_joint_num = robot_dh_->getPart("left")->getNumJoints();
        int right_hand_joint_num = robot_dh_->getPart("right")->getNumJoints();
        int head_joint_num = robot_dh_->getPart("head")->getNumJoints();
        std::vector<int> add_vec = {boyd_joint_num, left_hand_joint_num, right_hand_joint_num, head_joint_num};

        int index = 1;
        int add_index = 0;
        int next_add_index = add_vec[add_index];
        ++add_index;
        for (auto name : joints_name_){
            joint_dh_index_[name] = index;
            ++index;
            if(index > next_add_index){
                next_add_index = index + add_vec[add_index];
                ++index;
                ++add_index;
            }
        }
        
        for (auto name : joints_name_) { 
            std::cout << name << " " << joint_dh_index_[name] << std::endl;
        }

        // 执行转换
        return writeConvertedUrdf(dh_tf);
    }

    bool replaceFirst(std::string &s, const std::string &from, const std::string &to) {
        size_t pos = s.find(from);  // 查找位置
        if (pos == std::string::npos) return false; // 没找到
        
        s.replace(pos, from.length(), to);    // 替换
        return true;
    }

private:
    /**
     * @brief 将转换后的URDF写入文件
     */
    bool writeConvertedUrdf(const std::vector<Matrix4d>& dh_tf) {
        std::ofstream output_file(output_urdf_path_);
        if (!output_file.is_open()) {
            std::cerr << "Cannot open output file: " << output_urdf_path_ << std::endl;
            return false;
        }

        // 写入完整的URDF文件，保留原始结构
        output_file << "<?xml version=\"1.0\" ?>" << std::endl;
        output_file << "<!-- Converted URDF based on DH parameters -->" << std::endl;
        output_file << "<!-- This URDF maintains the same kinematic relationships as the DH model -->" << std::endl;
        output_file << "<robot name=\"" << urdf_model_->getName() << "_converted\">" << std::endl;
        
        // 输出所有链接，保留原始几何信息
        for (const auto& link_pair : urdf_model_->links_) {
            auto link = link_pair.second;
            output_file << "  <link name=\"" << link->name << "\">" << std::endl;
            
            // 输出视觉元素
            if (link->visual) {
                output_file << "    <visual>" << std::endl;
                
                // 输出origin标签（如果存在变换）
                if (link->visual->origin.position.x != 0 || link->visual->origin.position.y != 0 || 
                    link->visual->origin.position.z != 0 ||
                    link->visual->origin.rotation.x != 0 || link->visual->origin.rotation.y != 0 || 
                    link->visual->origin.rotation.z != 0) {
                    output_file << "      <origin xyz=\"" 
                              << link->visual->origin.position.x << " " 
                              << link->visual->origin.position.y << " " 
                              << link->visual->origin.position.z << "\" ";
                    output_file << "rpy=\"" 
                              << link->visual->origin.rotation.x << " " 
                              << link->visual->origin.rotation.y << " " 
                              << link->visual->origin.rotation.z << "\" />" << std::endl;
                }
                
                if (link->visual->geometry) {
                    output_file << "      <geometry>" << std::endl;
                    
                    // 输出几何形状定义
                    switch (link->visual->geometry->type) {
                        case urdf::Geometry::MESH: {
                            urdf::Mesh* mesh = dynamic_cast<urdf::Mesh*>(link->visual->geometry.get());
                            output_file << "        <mesh filename=\"" << mesh->filename << "\"";
                            if (mesh->scale.x != 1.0 || mesh->scale.y != 1.0 || mesh->scale.z != 1.0) {
                                output_file << " scale=\"" << mesh->scale.x << " " << mesh->scale.y << " " << mesh->scale.z << "\"";
                            }
                            output_file << " />" << std::endl;
                            break;
                        }
                        case urdf::Geometry::BOX: {
                            urdf::Box* box = dynamic_cast<urdf::Box*>(link->visual->geometry.get());
                            output_file << "        <box size=\"" << box->dim.x << " " << box->dim.y << " " << box->dim.z << "\" />" << std::endl;
                            break;
                        }
                        case urdf::Geometry::CYLINDER: {
                            urdf::Cylinder* cyl = dynamic_cast<urdf::Cylinder*>(link->visual->geometry.get());
                            output_file << "        <cylinder radius=\"" << cyl->radius << "\" length=\"" << cyl->length << "\" />" << std::endl;
                            break;
                        }
                        case urdf::Geometry::SPHERE: {
                            urdf::Sphere* sph = dynamic_cast<urdf::Sphere*>(link->visual->geometry.get());
                            output_file << "        <sphere radius=\"" << sph->radius << "\" />" << std::endl;
                            break;
                        }
                    }
                    
                    output_file << "      </geometry>" << std::endl;
                }
                
                // 输出原始材质定义（如果存在）
                if (link->visual->material) {
                    output_file << "      <material name=\"" << link->visual->material->name << "\">" << std::endl;
                    output_file << "        <color rgba=\"" 
                              << link->visual->material->color.r << " "
                              << link->visual->material->color.g << " "
                              << link->visual->material->color.b << " "
                              << link->visual->material->color.a << "\" />" << std::endl;
                    if (!link->visual->material->texture_filename.empty()) {
                        output_file << "        <texture filename=\"" 
                                  << link->visual->material->texture_filename << "\" />" << std::endl;
                    }
                    output_file << "      </material>" << std::endl;
                }
                
                output_file << "    </visual>" << std::endl;
            }
            
            // 输出碰撞元素
            if (link->collision) {
                output_file << "    <collision>" << std::endl;
                
                // 输出origin标签（如果存在变换）
                if (link->collision->origin.position.x != 0 || link->collision->origin.position.y != 0 || 
                    link->collision->origin.position.z != 0 ||
                    link->collision->origin.rotation.x != 0 || link->collision->origin.rotation.y != 0 || 
                    link->collision->origin.rotation.z != 0) {
                    output_file << "      <origin xyz=\"" 
                              << link->collision->origin.position.x << " " 
                              << link->collision->origin.position.y << " " 
                              << link->collision->origin.position.z << "\" ";
                    output_file << "rpy=\"" 
                              << link->collision->origin.rotation.x << " " 
                              << link->collision->origin.rotation.y << " " 
                              << link->collision->origin.rotation.z << "\" />" << std::endl;
                }
                
                if (link->collision->geometry) {
                    output_file << "      <geometry>" << std::endl;
                    
                    // 输出几何形状定义
                    switch (link->collision->geometry->type) {
                        case urdf::Geometry::MESH: {
                            urdf::Mesh* mesh = dynamic_cast<urdf::Mesh*>(link->collision->geometry.get());
                            output_file << "        <mesh filename=\"" << mesh->filename << "\"";
                            if (mesh->scale.x != 1.0 || mesh->scale.y != 1.0 || mesh->scale.z != 1.0) {
                                output_file << " scale=\"" << mesh->scale.x << " " << mesh->scale.y << " " << mesh->scale.z << "\"";
                            }
                            output_file << " />" << std::endl;
                            break;
                        }
                        case urdf::Geometry::BOX: {
                            urdf::Box* box = dynamic_cast<urdf::Box*>(link->collision->geometry.get());
                            output_file << "        <box size=\"" << box->dim.x << " " << box->dim.y << " " << box->dim.z << "\" />" << std::endl;
                            break;
                        }
                        case urdf::Geometry::CYLINDER: {
                            urdf::Cylinder* cyl = dynamic_cast<urdf::Cylinder*>(link->collision->geometry.get());
                            output_file << "        <cylinder radius=\"" << cyl->radius << "\" length=\"" << cyl->length << "\" />" << std::endl;
                            break;
                        }
                        case urdf::Geometry::SPHERE: {
                            urdf::Sphere* sph = dynamic_cast<urdf::Sphere*>(link->collision->geometry.get());
                            output_file << "        <sphere radius=\"" << sph->radius << "\" />" << std::endl;
                            break;
                        }
                    }
                    
                    output_file << "      </geometry>" << std::endl;
                }
                
                output_file << "    </collision>" << std::endl;
            }
            
            // 输出惯性数据，保留原始URDF的惯性属性
            if (link->inertial) {
                output_file << "    <inertial>" << std::endl;
                output_file << "      <mass value=\"" << link->inertial->mass << "\" />" << std::endl;
                output_file << "      <inertia ixx=\"" << link->inertial->ixx 
                          << "\" ixy=\"" << link->inertial->ixy 
                          << "\" ixz=\"" << link->inertial->ixz 
                          << "\" iyy=\"" << link->inertial->iyy 
                          << "\" iyz=\"" << link->inertial->iyz 
                          << "\" izz=\"" << link->inertial->izz << "\" />" << std::endl;
                output_file << "    </inertial>" << std::endl;
            } else {
                // 如果原始URDF中没有惯性数据，则添加默认值
                output_file << "    <inertial>" << std::endl;
                output_file << "      <mass value=\"0.01\" />" << std::endl;
                output_file << "      <inertia ixx=\"0.0001\" ixy=\"0\" ixz=\"0\" iyy=\"0.0001\" iyz=\"0\" izz=\"0.0001\" />" << std::endl;
                output_file << "    </inertial>" << std::endl;
            }
            
            output_file << "  </link>" << std::endl;
        }



        
        // 从base_link开始，遍历所有链接，并输出
        std::string current_link_name = "base_link";
        std::vector<std::string> sub_link_names;
        std::map<std::string, Matrix4d> urdf_joint_transforms;
        std::map<std::string, std::string> link2joint;
        while (true) { 
            for (const auto& joint_pair : urdf_model_->joints_) {
                auto joint = joint_pair.second;
                if (joint->parent_link_name == current_link_name) {
                    sub_link_names.push_back(joint->child_link_name);
                    std::cout << "Processing joint: " << joint->name << " par:" << joint->parent_link_name << std::endl;

                    output_file << "  <joint name=\"" << joint->name << "\" type=\"";
        
                    switch (joint->type) {
                        case urdf::Joint::FIXED:
                            output_file << "fixed";
                            break;
                        case urdf::Joint::REVOLUTE:
                            output_file << "revolute";
                            break;
                        case urdf::Joint::CONTINUOUS:
                            output_file << "continuous";
                            break;
                        case urdf::Joint::PRISMATIC:
                            output_file << "prismatic";
                            break;
                        default:
                            output_file << "fixed";  // 默认为fixed
                            break;
                    }
                    
                    output_file << "\">" << std::endl;
                    output_file << "    <parent link=\"" << joint->parent_link_name << "\" />" << std::endl;
                    output_file << "    <child link=\"" << joint->child_link_name << "\" />" << std::endl;
                    if (joint_dh_index_.find(joint->name) != joint_dh_index_.end()) {
                        int dh_idx = joint_dh_index_[joint->name];
                        
                        // 计算相对于父关节的变换
                        Matrix4d now_dh_tf = dh_tf[dh_idx];  // DH模型中当前关节相对于base的变换

                        // 父joint需要把父link的link换成joint
                        std::string parent_joint_name = link2joint[joint->parent_link_name];
    

                        std::cout << "cover " << joint->name << " parent " << parent_joint_name <<  std::endl;

                        // check now_dh_tf
                        // std::cout << "now_dh_tf " << now_dh_tf << std::endl;


                        Eigen::Quaterniond quat(
                            joint->parent_to_joint_origin_transform.rotation.w,
                            joint->parent_to_joint_origin_transform.rotation.x,
                            joint->parent_to_joint_origin_transform.rotation.y,
                            joint->parent_to_joint_origin_transform.rotation.z
                        );
                        
                        Matrix4d parent_urdf_tf;
                        if(current_link_name == "base_link"){
                            // 单位阵
                            parent_urdf_tf = Matrix4d::Identity();
                        }
                        else{
                            parent_urdf_tf = urdf_joint_transforms[parent_joint_name];
                        }

                        std::cout << "parent_urdf_tf " << parent_urdf_tf << std::endl;

                        Matrix4d target_tf = now_dh_tf;
                        
                        // 计算当前关节相对于父关节的变换
                        Matrix4d rel_transform = parent_urdf_tf.inverse() * target_tf;

                        rel_transform.block<3, 3>(0, 0) = quat.toRotationMatrix();

                        // 姿态用urdf里的
                        urdf_joint_transforms[joint->name] = parent_urdf_tf * rel_transform;

                        
                        // 提取平移部分（x, y, z）
                        double x = rel_transform(0, 3);
                        double y = rel_transform(1, 3);
                        double z = rel_transform(2, 3);
                        
                        // 提取旋转部分并转换为RPY
                        // Eigen::Matrix3d rot = rel_transform.block<3, 3>(0, 0);
                        // Eigen::Vector3d rpy = rotationMatrixToRpy(rot);
                        
                        output_file << "    <origin xyz=\"" 
                                  << x << " "
                                  << y << " "
                                  << z << "\" ";
        
                        Eigen::Vector3d rpy = quat.toRotationMatrix().eulerAngles(2, 1, 0); // RPY = XYZ extrinsic rotation
                        output_file << "rpy=\"" 
                                  << rpy[2] << " "
                                  << rpy[1] << " "
                                  << rpy[0] << "\" />" << std::endl;
                    } else {
                        
                        // 不在DH模型中的关节，使用原始参数
                        // 首先将四元数转换为欧拉角
                        Eigen::Quaterniond quat(
                            joint->parent_to_joint_origin_transform.rotation.w,
                            joint->parent_to_joint_origin_transform.rotation.x,
                            joint->parent_to_joint_origin_transform.rotation.y,
                            joint->parent_to_joint_origin_transform.rotation.z
                            
                        );
                        Eigen::Vector3d rpy = quat.toRotationMatrix().eulerAngles(2, 1, 0); // RPY = XYZ extrinsic rotation
                        
                        output_file << "    <origin xyz=\"" 
                                  << joint->parent_to_joint_origin_transform.position.x << " "
                                  << joint->parent_to_joint_origin_transform.position.y << " "
                                  << joint->parent_to_joint_origin_transform.position.z << "\" ";
                        output_file << "rpy=\"" 
                                  << rpy[2] << " "
                                  << rpy[1] << " "
                                  << rpy[0] << "\" />" << std::endl;

                        std::string parent_joint_name = link2joint[joint->parent_link_name];
                        Matrix4d tf = Matrix4d::Identity();
                        tf.block<3, 3>(0, 0) = quat.toRotationMatrix();
                        tf.block<3, 1>(0, 3) = Vector3d(joint->parent_to_joint_origin_transform.position.x,
                                                      joint->parent_to_joint_origin_transform.position.y,
                                                      joint->parent_to_joint_origin_transform.position.z);
                        urdf_joint_transforms[joint->name] = urdf_joint_transforms[parent_joint_name] * tf;
                    }

                    link2joint[joint->child_link_name] = joint->name;
                    
                    // 如果是旋转关节，输出轴向（保持不变）
                    if (joint->type == urdf::Joint::REVOLUTE || joint->type == urdf::Joint::CONTINUOUS || 
                        joint->type == urdf::Joint::PRISMATIC) {
                        output_file << "    <axis xyz=\"" 
                                  << joint->axis.x << " "
                                  << joint->axis.y << " "
                                  << joint->axis.z << "\" />" << std::endl;
                                  
                        // 输出限制（如果有的话）
                        if (joint->limits) {
                            output_file << "    <limit lower=\"" << joint->limits->lower << "\" "
                                      << "upper=\"" << joint->limits->upper << "\" "
                                      << "effort=\"" << joint->limits->effort << "\" "
                                      << "velocity=\"" << joint->limits->velocity << "\" />" << std::endl;
                        }
                    }
                    output_file << "  </joint>" << std::endl;
                }
            }
            if (sub_link_names.empty()) {
                break;
            }
            current_link_name = sub_link_names[0];
            sub_link_names.erase(sub_link_names.begin());
        }
        
        output_file << "</robot>" << std::endl;
        
        std::cout << "DH-to-URDF converted URDF written to: " << output_urdf_path_ << std::endl;
        return true;
    }

    /**
     * @brief 将旋转矩阵转换为RPY（Roll-Pitch-Yaw）角度
     */
    Eigen::Vector3d rotationMatrixToRpy(const Eigen::Matrix3d& rot) {
        double roll, pitch, yaw;

        // 检查俯仰角是否接近±90度（万向锁条件）
        pitch = std::atan2(-rot(2, 0), std::sqrt(rot(0, 0)*rot(0, 0) + rot(1, 0)*rot(1, 0)));

        if (std::abs(pitch - M_PI/2) < 1e-6) {
            // 俯仰角为+90度
            roll = 0;
            yaw = std::atan2(rot(0, 1), rot(1, 1));
        } else if (std::abs(pitch + M_PI/2) < 1e-6) {
            // 俯仰角为-90度
            roll = 0;
            yaw = std::atan2(-rot(0, 1), -rot(1, 1));
        } else {
            // 一般情况
            roll = std::atan2(rot(2, 1), rot(2, 2));
            yaw = std::atan2(rot(1, 0), rot(0, 0));
        }

        return Eigen::Vector3d(roll, pitch, yaw);
    }
    
    /**
     * @brief 从URDF模型中计算指定关节相对于基座的绝对位置
     */
    Matrix4d getAbsoluteTransformFromUrdf(const std::string& joint_name) {
        // 从指定关节向上追溯到根节点，累积变换
        auto joint = urdf_model_->getJoint(joint_name);
        if (!joint) {
            std::cerr << "Joint not found in URDF: " << joint_name << std::endl;
            return Matrix4d::Identity();
        }

        // 从指定关节到根节点的路径
        Matrix4d cumulative_transform = Matrix4d::Identity();
        
        // 递归地向上追溯，直到到达根节点
        std::shared_ptr<const urdf::Link> current_link = urdf_model_->getLink(joint->child_link_name);
        
        // 从当前链接一直追溯到根节点
        while (current_link && current_link->getParent() != nullptr) {
            auto parent_joint = current_link->parent_joint;
            if (parent_joint) {
                // 获取当前链接到其父链接的变换（即父关节的变换）
                Matrix4d joint_transform = getTransformFromUrdfPose(parent_joint->parent_to_joint_origin_transform);
                
                // 将当前变换左乘到累积变换上
                cumulative_transform = joint_transform * cumulative_transform;
            }
            
            // 移动到父链接
            current_link = current_link->getParent();
        }

        return cumulative_transform;
    }
    
    /**
     * @brief 将URDF Pose转换为齐次变换矩阵
     */
    Matrix4d getTransformFromUrdfPose(const urdf::Pose& pose) {
        Matrix4d transform = Matrix4d::Identity();
        
        // 设置旋转部分
        double qx = pose.rotation.x;
        double qy = pose.rotation.y;
        double qz = pose.rotation.z;
        double qw = pose.rotation.w;
        
        // 创建四元数并转换为旋转矩阵
        Quaterniond q(qw, qx, qy, qz);
        q.normalize();
        transform.block<3, 3>(0, 0) = q.toRotationMatrix();
        
        // 设置平移部分
        transform(0, 3) = pose.position.x;
        transform(1, 3) = pose.position.y;
        transform(2, 3) = pose.position.z;
        
        return transform;
    }
};

int main(int argc, char** argv) {
    if (argc < 4) {
        std::cerr << "Usage: " << argv[0] << " <config_path> <input_urdf_path> <output_urdf_path>" << std::endl;
        return -1;
    }

    std::string config_path = argv[1];
    std::string input_urdf_path = argv[2];
    std::string output_urdf_path = argv[3];

    try {
        DhToUrdfConverter converter(config_path, input_urdf_path, output_urdf_path);
        if (converter.convertDhToUrdf()) {
            std::cout << "DH-to-URDF conversion completed successfully!" << std::endl;
            return 0;
        } else {
            std::cerr << "DH-to-URDF conversion failed!" << std::endl;
            return -1;
        }
    } catch (const std::exception& e) {
        std::cerr << "Error during DH-to-URDF conversion: " << e.what() << std::endl;
        return -1;
    }
}