#ifndef COORD_H
#define COORD_H

#include <iostream>
#include <vector>  // 添加缺失的vector头文件
#include <Eigen/Dense>
#include <cmath>

namespace coord{
    // 定义geometry_msgs风格的数据结构
    struct Quaternion {
        double x, y, z, w;
    };

    struct Pose {
        struct {
            double x, y, z;
        } position;
        Quaternion orientation;
    };

    // 欧拉角转四元数 (单位：弧度)
    Quaternion euler_to_quaternion(double roll, double pitch, double yaw);

    // 四元数转欧拉角 (单位：弧度)
    void quaternion_to_euler(const Quaternion& quat, 
                             double& roll, double& pitch, double& yaw);

    // 欧拉角转旋转矩阵
    Eigen::Matrix3d euler_to_rotation_matrix(double roll, double pitch, double yaw);

    // 四元数转旋转矩阵
    Eigen::Matrix3d quaternion_to_rotation_matrix(const Quaternion& quat);

    // 旋转矩阵转欧拉角
    void rotation_matrix_to_euler(const Eigen::Matrix3d& matrix,
                                 double& roll, double& pitch, double& yaw);

    // 旋转矩阵转四元数
    Quaternion rotation_matrix_to_quaternion(const Eigen::Matrix3d& matrix);

    // pose 转 齐次矩阵
    Eigen::Matrix4d pose_to_matrix(const Pose& pose);

    // 齐次矩阵 转 pose
    Pose matrix_to_pose(const Eigen::Matrix4d& matrix);

    // 欧拉角和坐标 转 齐次矩阵
    Eigen::Matrix4d euler_and_coord_to_matrix(double roll, double pitch, double yaw,
                                            double x, double y, double z);

    // 测试转换是否正确的函数
    void test_conversions();

} // namespace coord

#endif // COORD_HPP