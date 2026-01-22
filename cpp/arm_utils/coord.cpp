#include "coord.h"
#include <Eigen/Dense>
#include <cmath>

namespace coord {
    
// 欧拉角转四元数 (单位：弧度)
Quaternion euler_to_quaternion(double roll, double pitch, double yaw) {
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);

    Quaternion q;
    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;
    return q;
}

// 四元数转欧拉角 (单位：弧度)
void quaternion_to_euler(const Quaternion& quat, 
                         double& roll, double& pitch, double& yaw) {
    double sinr_cosp = 2 * (quat.w * quat.x + quat.y * quat.z);
    double cosr_cosp = 1 - 2 * (quat.x * quat.x + quat.y * quat.y);
    roll = atan2(sinr_cosp, cosr_cosp);

    double sinp = 2 * (quat.w * quat.y - quat.z * quat.x);
    if (fabs(sinp) >= 1)
        pitch = copysign(M_PI / 2, sinp);
    else
        pitch = asin(sinp);

    double siny_cosp = 2 * (quat.w * quat.z + quat.x * quat.y);
    double cosy_cosp = 1 - 2 * (quat.y * quat.y + quat.z * quat.z);
    yaw = atan2(siny_cosp, cosy_cosp);
}

// 欧拉角转旋转矩阵
Eigen::Matrix3d euler_to_rotation_matrix(double roll, double pitch, double yaw) {
    Eigen::Matrix3d mat;
    
    double cr = cos(roll), sr = sin(roll);
    double cp = cos(pitch), sp = sin(pitch);
    double cy = cos(yaw), sy = sin(yaw);

    mat << cp * cy, sr * sp * cy - cr * sy, cr * sp * cy + sr * sy,
           cp * sy, sr * sp * sy + cr * cy, cr * sp * sy - sr * cy,
          -sp,     sr * cp,               cr * cp;
          
    return mat;
}

// 四元数转旋转矩阵
Eigen::Matrix3d quaternion_to_rotation_matrix(const Quaternion& quat) {
    Eigen::Matrix3d mat;
    
    double qx = quat.x, qy = quat.y, qz = quat.z, qw = quat.w;
    double qx2 = qx * qx, qy2 = qy * qy, qz2 = qz * qz;
    double qw2 = qw * qw;
    double qxqy = qx * qy, qxqz = qx * qz, qxqw = qx * qw;
    double qyqz = qy * qz, qyqw = qy * qw, qzqw = qz * qw;

    mat << qw2 + qx2 - qy2 - qz2, 2*(qxqy - qzqw), 2*(qxqz + qyqw),
           2*(qxqy + qzqw), qw2 - qx2 + qy2 - qz2, 2*(qyqz - qxqw),
           2*(qxqz - qyqw), 2*(qyqz + qxqw), qw2 - qx2 - qy2 + qz2;
           
    return mat;
}

// 旋转矩阵转欧拉角
void rotation_matrix_to_euler(const Eigen::Matrix3d& matrix,
                             double& roll, double& pitch, double& yaw) {
    pitch = atan2(-matrix(2, 0), sqrt(matrix(0, 0)*matrix(0, 0) + matrix(1, 0)*matrix(1, 0)));
    
    if (fabs(pitch - M_PI/2.) < 1e-6) {
        yaw = 0;
        roll = atan2(matrix(0, 1), matrix(1, 1));
    } else if (fabs(pitch + M_PI/2.) < 1e-6) {
        yaw = 0;
        roll = atan2(-matrix(0, 1), -matrix(1, 1));
    } else {
        yaw = atan2(matrix(1, 0)/cos(pitch), matrix(0, 0)/cos(pitch));
        roll = atan2(matrix(2, 1)/cos(pitch), matrix(2, 2)/cos(pitch));
    }
}

// 旋转矩阵转四元数
Quaternion rotation_matrix_to_quaternion(const Eigen::Matrix3d& matrix) {
    Quaternion q;
    double trace = matrix(0, 0) + matrix(1, 1) + matrix(2, 2);
    
    if (trace > 0) {
        double s = sqrt(trace + 1.0) * 2; // S=4*qw
        q.w = 0.25 * s;
        q.x = (matrix(2, 1) - matrix(1, 2)) / s;
        q.y = (matrix(0, 2) - matrix(2, 0)) / s;
        q.z = (matrix(1, 0) - matrix(0, 1)) / s;
    } else if ((matrix(0, 0) > matrix(1, 1)) && (matrix(0, 0) > matrix(2, 2))) {
        double s = sqrt(1.0 + matrix(0, 0) - matrix(1, 1) - matrix(2, 2)) * 2; // S=4*qx
        q.w = (matrix(2, 1) - matrix(1, 2)) / s;
        q.x = 0.25 * s;
        q.y = (matrix(0, 1) + matrix(1, 0)) / s;
        q.z = (matrix(0, 2) + matrix(2, 0)) / s;
    } else if (matrix(1, 1) > matrix(2, 2)) {
        double s = sqrt(1.0 + matrix(1, 1) - matrix(0, 0) - matrix(2, 2)) * 2; // S=4*qy
        q.w = (matrix(0, 2) - matrix(2, 0)) / s;
        q.x = (matrix(0, 1) + matrix(1, 0)) / s;
        q.y = 0.25 * s;
        q.z = (matrix(1, 2) + matrix(2, 1)) / s;
    } else {
        double s = sqrt(1.0 + matrix(2, 2) - matrix(0, 0) - matrix(1, 1)) * 2; // S=4*qz
        q.w = (matrix(1, 0) - matrix(0, 1)) / s;
        q.x = (matrix(0, 2) + matrix(2, 0)) / s;
        q.y = (matrix(1, 2) + matrix(2, 1)) / s;
        q.z = 0.25 * s;
    }
    
    return q;
}

// pose 转 齐次矩阵
Eigen::Matrix4d pose_to_matrix(const Pose& pose) {
    // 1. 初始化4x4单位矩阵（避免未初始化的垃圾值）
    Eigen::Matrix4d mat = Eigen::Matrix4d::Identity();
    
    // 2. 获取旋转矩阵（Eigen::Matrix3d）
    Eigen::Matrix3d rot_mat = quaternion_to_rotation_matrix(pose.orientation);
    
    // 3. 填充旋转部分（前3行前3列）
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            mat(i, j) = rot_mat(i, j);
        }
    }
    
    // 4. 填充平移部分（前3行第4列）
    mat(0, 3) = pose.position.x;
    mat(1, 3) = pose.position.y;
    mat(2, 3) = pose.position.z;
    
    // 5. 最后一行固定为 [0, 0, 0, 1]（已由Identity()初始化）
    return mat;
}

// 齐次矩阵 转 pose
Pose matrix_to_pose(const Eigen::Matrix4d& matrix) {
    Eigen::Matrix3d rot_mat;
    for(int i = 0; i < 3; i++){
        for(int j = 0; j < 3; j++){
            rot_mat(i, j) = matrix(i, j);
        }
    }
    Pose pose;
    coord::Quaternion quat = rotation_matrix_to_quaternion(rot_mat);
    pose.orientation = quat;
    pose.position.x = matrix(0, 3);
    pose.position.y = matrix(1, 3);
    pose.position.z = matrix(2, 3);
    return pose;
}

// 欧拉角和坐标 转 齐次矩阵
Eigen::Matrix4d euler_and_coord_to_matrix(double roll, double pitch, double yaw,
                                        double x, double y, double z) {
    Eigen::Matrix3d rot_mat = euler_to_rotation_matrix(roll, pitch, yaw);
    // 1. 初始化4x4单位矩阵（避免未初始化的垃圾值）
    Eigen::Matrix4d mat = Eigen::Matrix4d::Identity();
    
    // 3. 填充旋转部分（前3行前3列）
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            mat(i, j) = rot_mat(i, j);
        }
    }
    
    // 4. 填充平移部分（前3行第4列）
    mat(0, 3) = x;
    mat(1, 3) = y;
    mat(2, 3) = z;
    
    // 5. 最后一行固定为 [0, 0, 0, 1]（已由Identity()初始化）
    return mat;
}

// 测试转换是否正确的函数
void test_conversions() {
    // 测试用例：选择几个典型角度（单位：弧度）
    struct TestCase {
        double roll;
        double pitch;
        double yaw;
        std::string name;
    };

    std::vector<TestCase> test_cases = {
        {0, 0, 0, "零角度"},
        {M_PI/4, 0, 0, "45度滚转"},
        {0, M_PI/4, 0, "45度俯仰"},
        {0, 0, M_PI/4, "45度偏航"},
        {M_PI/6, M_PI/3, M_PI/2, "复合角度"}
    };

    // 允许的最大误差（由于浮点计算）
    const double EPS = 1e-6;

    for (const auto& tc : test_cases) {
        std::cout << "\n测试用例: " << tc.name << std::endl;
        std::cout << "原始欧拉角: roll=" << tc.roll << ", pitch=" << tc.pitch << ", yaw=" << tc.yaw << std::endl;

        // 1. 测试欧拉角 ↔ 四元数 转换
        auto quat = euler_to_quaternion(tc.roll, tc.pitch, tc.yaw);
        double r1, p1, y1;
        quaternion_to_euler(quat, r1, p1, y1);
        
        double err_roll = fabs(r1 - tc.roll);
        double err_pitch = fabs(p1 - tc.pitch);
        double err_yaw = fabs(y1 - tc.yaw);
        
        std::cout << "欧拉角→四元数→欧拉角误差: " 
                  << "roll=" << err_roll << ", "
                  << "pitch=" << err_pitch << ", "
                  << "yaw=" << err_yaw << " | "
                  << (err_roll < EPS && err_pitch < EPS && err_yaw < EPS ? "通过" : "失败") << std::endl;

        // 2. 测试欧拉角 ↔ 旋转矩阵 转换
        Eigen::Matrix3d rot_mat = euler_to_rotation_matrix(tc.roll, tc.pitch, tc.yaw);
        double r2, p2, y2;
        rotation_matrix_to_euler(rot_mat, r2, p2, y2);
        
        err_roll = fabs(r2 - tc.roll);
        err_pitch = fabs(p2 - tc.pitch);
        err_yaw = fabs(y2 - tc.yaw);
        
        std::cout << "欧拉角→旋转矩阵→欧拉角误差: " 
                  << "roll=" << err_roll << ", "
                  << "pitch=" << err_pitch << ", "
                  << "yaw=" << err_yaw << " | "
                  << (err_roll < EPS && err_pitch < EPS && err_yaw < EPS ? "通过" : "失败") << std::endl;

        // 3. 测试四元数 ↔ 旋转矩阵 转换
        Eigen::Matrix3d rot_mat2 = quaternion_to_rotation_matrix(quat);
        auto quat2 = rotation_matrix_to_quaternion(rot_mat2);
        
        // 四元数误差（考虑四元数的双重表示：q和-q表示同一旋转）
        double err_quat = std::min(
            fabs(quat.x - quat2.x) + fabs(quat.y - quat2.y) + fabs(quat.z - quat2.z) + fabs(quat.w - quat2.w),
            fabs(quat.x + quat2.x) + fabs(quat.y + quat2.y) + fabs(quat.z + quat2.z) + fabs(quat.w + quat2.w)
        );
        
        std::cout << "四元数→旋转矩阵→四元数误差: " << err_quat << " | "
                  << (err_quat < EPS ? "通过" : "失败") << std::endl;
    }


    // 测试 euler_and_coord_to_matrix 函数
    std::cout << "\n测试 euler_and_coord_to_matrix 函数" << std::endl;
    Eigen::Matrix4d matrix = euler_and_coord_to_matrix(0.5, 0.3, 0.1, 2.0, 3.0, 4.0);
    std::cout << matrix << std::endl;
    std::cout << "齐次矩阵转pose再转回齐次矩阵: " << std::endl << pose_to_matrix(matrix_to_pose(matrix)) << std::endl;
}
} // namespace arm_robot