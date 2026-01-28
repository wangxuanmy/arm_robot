#include <iostream>
#include <string>
#include <thread>
#include <chrono>
#include <vector>
#include "arm_utils/protected_lfu_cache.h"

int main() {
    std::cout << "Testing ProtectedLFUCache...zz" << std::endl;

    // 创建一个最大容量为3，保护期为2秒的缓存
    ProtectedLFUCache<int, std::string> cache(3, std::chrono::seconds(2));

    // 测试基本操作
    std::cout << "\n1. Testing basic operations:" << std::endl;
    cache.put(1, "one");
    cache.put(2, "two");
    cache.put(3, "three");

    std::cout << "Cache size: " << cache.size() << std::endl;
    std::cout << "Key 1 contains: " << cache.contains(1) << std::endl;

    try {
        std::cout << "Value of key 1: " << cache.get(1) << std::endl;
    } catch (const std::exception& e) {
        std::cout << "Exception: " << e.what() << std::endl;
    }

    // 测试大小限制
    std::cout << "\n2. Testing size limit:" << std::endl;
    cache.put(4, "four");  // 这应该触发淘汰机制
    std::cout << "Cache size after adding 4th item: " << cache.size() << std::endl;

    // 检查哪些键仍然存在
    std::cout << "Keys still in cache: ";
    auto keys = cache.keys();
    for (const auto& key : keys) {
        std::cout << key << " ";
    }
    std::cout << std::endl;

    // 测试访问频率的影响
    std::cout << "\n3. Testing access frequency influence:" << std::endl;
    
    // 清空缓存重新开始
    cache.clear();
    cache.put(1, "one");
    cache.put(2, "two");
    cache.put(3, "three");
    
    // 频繁访问键2和3
    for (int i = 0; i < 5; ++i) {
        try {
            cache.get(1);
            cache.get(3);
        } catch (const std::exception& e) {
            std::cout << "Exception during get: " << e.what() << std::endl;
        }
    }
    
    // 添加第四个元素，由于访问频率更高，键2和3应该保留
    cache.put(4, "four");
    
    std::cout << "After accessing keys 2 and 3 frequently, and adding key 4:" << std::endl;
    std::cout << "Cache size: " << cache.size() << std::endl;
    keys = cache.keys();
    std::cout << "Keys in cache: ";
    for (const auto& key : keys) {
        std::cout << key << " ";
    }
    std::cout << std::endl;


    // 测试访问频率的影响
    std::cout << "\n3. Testing access frequency influence:" << std::endl;
    
    // 清空缓存重新开始
    cache.clear();
    cache.put(1, "one");
    cache.put(2, "two");
    cache.put(3, "three");
    
    // 频繁访问键2和3
    for (int i = 0; i < 5; ++i) {
        try {
            cache.get(1);
            cache.get(3);
        } catch (const std::exception& e) {
            std::cout << "Exception during get: " << e.what() << std::endl;
        }
    }

     // 等待保护期结束
     std::cout << "Waiting for protection period to end..." << std::endl;
     std::this_thread::sleep_for(std::chrono::milliseconds(3100)); // 2.1秒
     
    
    // 添加第四个元素，由于访问频率更高，键2和3应该保留
    cache.put(4, "four");
    
    std::cout << "After accessing keys 2 and 3 frequently, and adding key 4:" << std::endl;
    std::cout << "Cache size: " << cache.size() << std::endl;
    keys = cache.keys();
    std::cout << "Keys in cache: ";
    for (const auto& key : keys) {
        std::cout << key << " ";
    }
    std::cout << std::endl;

    // 验证键1被淘汰，而经常访问的键2和3以及新键4被保留
    bool key1Present = cache.contains(1);
    bool key2Present = cache.contains(2);
    bool key3Present = cache.contains(3);
    bool key4Present = cache.contains(4);
    
    std::cout << "Key 1 (should be absent): " << key1Present << std::endl;
    std::cout << "Key 2 (should be present): " << key2Present << std::endl;
    std::cout << "Key 3 (should be present): " << key3Present << std::endl;
    std::cout << "Key 4 (should be present): " << key4Present << std::endl;

    // 测试保护期功能
    std::cout << "\n4. Testing protection period:" << std::endl;
    cache.clear();
    
    // 添加一些项目
    cache.put(10, "ten");
    cache.put(11, "eleven");
    cache.put(12, "twelve");
    
    // 频繁访问这些项目
    for (int i = 0; i < 10; ++i) {
        try {
            cache.get(10);
            cache.get(11);
            cache.get(12);
        } catch (const std::exception& e) {
            std::cout << "Exception during get: " << e.what() << std::endl;
        }
    }

    
    // 等待保护期结束
    std::cout << "Waiting for protection period to end..." << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(2100)); // 2.1秒
    
    // 现在添加新项目，之前频繁访问但受保护期保护的项目可能会被替换
    cache.put(13, "thirteen");
    
    std::cout << "After waiting for protection period and adding new item:" << std::endl;
    keys = cache.keys();
    std::cout << "Keys in cache: ";
    for (const auto& key : keys) {
        std::cout << key << " ";
    }
    std::cout << std::endl;

    // 测试获取创建时间
    std::cout << "\n5. Testing creation time:" << std::endl;
    cache.clear();
    auto start_time = std::chrono::steady_clock::now();
    cache.put(20, "twenty");
    auto creation_time = cache.getCreateTime(20);
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
        creation_time.time_since_epoch() - start_time.time_since_epoch());
    std::cout << "Item created at (relative): " << duration.count() << "ms from start" << std::endl;

    std::cout << "\nTest completed successfully!" << std::endl;
    
    return 0;
}