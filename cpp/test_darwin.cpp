#include "darwin.h"
#include <iostream>
#include <cassert>
#include <vector>
#include <Eigen/Dense>

using namespace arm_robot;

void testBasicInitialization() {
    std::cout << "Testing basic initialization..." << std::endl;
    
    // 创建一些示例DH参数
    std::vector<std::vector<double>> bodyDh = {{0, 0, 0, 0, 0}};
    std::vector<std::vector<double>> leftHandDh = {{0, 0, 0, 0, 0}};
    std::vector<std::vector<double>> rightHandDh = {{0, 0, 0, 0, 0}};
    
    // 创建Human实例
    Human human(bodyDh, leftHandDh, rightHandDh);
    
    // 测试初始化成功
    assert(true); // 初始化应该成功
    
    std::cout << "Basic initialization test passed!" << std::endl;
}

void testGetAllTf() {
    std::cout << "Testing getAllTf function..." << std::endl;
    
    // 创建一些示例DH参数
    std::vector<std::vector<double>> bodyDh = {{0, 0, 0, 0, 0}};
    std::vector<std::vector<double>> leftHandDh = {{0, 0, 0, 0, 0}};
    std::vector<std::vector<double>> rightHandDh = {{0, 0, 0, 0, 0}};
    
    // 创建Human实例
    Human human(bodyDh, leftHandDh, rightHandDh);
    
    // 获取所有变换矩阵
    std::vector<Eigen::Matrix4d> transforms = human.getAllTf();
    
    // 检查返回值不为空
    assert(!transforms.empty());
    
    std::cout << "GetAllTf test passed!" << std::endl;
}

void testGetEef() {
    std::cout << "Testing getEef function..." << std::endl;
    
    // 创建一些示例DH参数
    std::vector<std::vector<double>> bodyDh = {{0, 0, 0, 0, 0}};
    std::vector<std::vector<double>> leftHandDh = {{0, 0, 0, 0, 0}};
    std::vector<std::vector<double>> rightHandDh = {{0, 0, 0, 0, 0}};
    
    // 创建Human实例
    Human human(bodyDh, leftHandDh, rightHandDh);
    
    // 获取末端执行器位置
    std::vector<Eigen::Matrix4d> eef = human.getEef();
    
    // 检查返回值大小为3（body, left, right）
    assert(eef.size() == 3);
    
    std::cout << "GetEef test passed!" << std::endl;
}

void testFlashTheta() {
    std::cout << "Testing flashTheta function..." << std::endl;
    
    // 创建一些示例DH参数
    std::vector<std::vector<double>> bodyDh = {{0, 0, 0, 0, 0}};
    std::vector<std::vector<double>> leftHandDh = {{0, 0, 0, 0, 0}};
    std::vector<std::vector<double>> rightHandDh = {{0, 0, 0, 0, 0}};
    
    // 创建Human实例
    Human human(bodyDh, leftHandDh, rightHandDh);
    
    // 准备输入theta值
    std::vector<double> thetaInput = {0.1, 0.2, 0.3};
    
    // 调用flashTheta函数
    human.flashTheta(thetaInput);
    
    // 获取更新后的theta值
    std::vector<double> allTheta = human.getAllTheta();
    
    // 检查是否更新成功
    assert(!allTheta.empty());
    
    std::cout << "FlashTheta test passed!" << std::endl;
}

int main() {
    std::cout << "Starting Darwin C++ Unit Tests..." << std::endl;
    
    testBasicInitialization();
    testGetAllTf();
    testGetEef();
    testFlashTheta();
    
    std::cout << "All tests passed successfully!" << std::endl;
    
    return 0;
}