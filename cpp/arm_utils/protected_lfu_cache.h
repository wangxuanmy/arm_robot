#ifndef PROTECTED_LFU_CACHE_H
#define PROTECTED_LFU_CACHE_H

#include <unordered_map>
#include <unordered_set>
#include <set>
#include <chrono>
#include <mutex>
#include <algorithm>
#include <functional>
#include <stdexcept>
#include <memory>
#include <limits>

/**
 * @brief 一个带保护期的LFU缓存实现
 * 
 * 特性：
 * 1. 大小限制：缓存有最大容量限制
 * 2. LFU策略：优先淘汰访问频率最低的项
 * 3. 保护期机制：新加入的缓存项在保护期内不会被淘汰
 */
template<typename KeyType, typename ValueType>
class ProtectedLFUCache {
public:
    using TimePoint = std::chrono::steady_clock::time_point;
    using Duration = std::chrono::seconds;

    /**
     * @brief 内部用于排序的时间键对
     */
    struct TimeKeyPair {
        TimePoint time;
        KeyType key;
        
        bool operator<(const TimeKeyPair& other) const {
            if (time != other.time) {
                return time < other.time;
            }
            // 如果时间相同，使用键的比较确保唯一顺序
            return key < other.key;
        }
    };

    /**
     * @brief 构造函数
     * @param maxsize 缓存最大容量
     * @param protect_duration 新项保护期，默认60秒
     */
    explicit ProtectedLFUCache(size_t maxsize, 
                              std::chrono::seconds protect_duration = std::chrono::seconds(60))
        : maxsize_(maxsize), protect_duration_(protect_duration) {}

    /**
     * @brief 设置缓存项
     * @param key 键
     * @param value 值
     */
    void put(const KeyType& key, const ValueType& value) {
        std::lock_guard<std::mutex> lock(mutex_);
        
        // 如果是新键，记录创建时间并初始化访问计数为1

        if (data_.find(key) == data_.end()) {
            create_times_[key] = std::chrono::steady_clock::now();
            counts_[key] = 1;  // 新项初始化访问计数为1
            time_sorted_keys_.insert({create_times_[key], key});
        }
        
        // 设置值
        data_[key] = value;
        
        // 检查是否需要淘汰项
        evict_();
    }

    /**
     * @brief 获取缓存项
     * @param key 键
     * @return 值的引用
     * @throws std::out_of_range 如果键不存在
     */
    const ValueType& get(const KeyType& key) {
        std::lock_guard<std::mutex> lock(mutex_);
        
        auto it = data_.find(key);
        if (it == data_.end()) {
            throw std::out_of_range("Key not found in cache");
        }
        
        // 增加访问计数
        counts_[key]++;
        return it->second;
    }

    /**
     * @brief 检查键是否存在
     * @param key 键
     * @return 是否存在
     */
    bool contains(const KeyType& key) const {
        std::lock_guard<std::mutex> lock(mutex_);
        return data_.find(key) != data_.end();
    }

    /**
     * @brief 获取缓存大小
     * @return 缓存项数量
     */
    size_t size() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return data_.size();
    }

    /**
     * @brief 获取所有键
     * @return 所有键的集合
     */
    std::unordered_set<KeyType> keys() const {
        std::lock_guard<std::mutex> lock(mutex_);
        std::unordered_set<KeyType> result;
        for (const auto& pair : data_) {
            result.insert(pair.first);
        }
        return result;
    }

    /**
     * @brief 清空缓存
     */
    void clear() {
        std::lock_guard<std::mutex> lock(mutex_);
        data_.clear();
        counts_.clear();
        create_times_.clear();
        time_sorted_keys_.clear();
    }

    /**
     * @brief 获取指定键的创建时间
     * @param key 键
     * @return 创建时间点
     */
    TimePoint getCreateTime(const KeyType& key) const {
        std::lock_guard<std::mutex> lock(mutex_);
        auto it = create_times_.find(key);
        if (it != create_times_.end()) {
            return it->second;
        }
        return TimePoint{};  // 返回默认构造的时间点
    }

private:
    size_t maxsize_;
    Duration protect_duration_;
    mutable std::mutex mutex_;

    std::unordered_map<KeyType, ValueType> data_;
    std::unordered_map<KeyType, size_t> counts_;
    std::unordered_map<KeyType, TimePoint> create_times_;
    
    // 按时间排序的键集合，用于快速查找保护期临界点
    std::set<TimeKeyPair> time_sorted_keys_;

    /**
     * @brief 淘汰缓存项
     * 
     * 淘汰策略：
     * 1. 优先淘汰保护期外访问频率最低的项
     * 2. 如果所有项都在保护期内，则淘汰最旧的项
     */
    void evict_() {
        // 只有当缓存超过最大容量时才进行淘汰
        if (data_.size() <= maxsize_) {
            return;
        }

        auto now = std::chrono::steady_clock::now();
        // 计算保护期临界时间点
        auto cutoff_time = now - protect_duration_;

        // 使用lower_bound快速找到保护期临界点
        TimeKeyPair cutoff_pair{cutoff_time, KeyType{}};
        auto cutoff_it = time_sorted_keys_.lower_bound(cutoff_pair);

        KeyType key_to_evict;

        // 检查是否在保护期外有项
        if (cutoff_it != time_sorted_keys_.begin()) {
            // 存在保护期外的项，从这些项中找访问次数最少的
            // 注意：cutoff_it指向第一个>=cutoff_time的项，所以之前的项都是保护期外的
            KeyType candidate_key = KeyType{};
            size_t min_count = std::numeric_limits<size_t>::max();
            
            // 遍历保护期外的项，找到访问次数最少的
            for (auto it = time_sorted_keys_.begin(); it != cutoff_it; ++it) {
                KeyType key = it->key;
                if (counts_[key] < min_count) {
                    min_count = counts_[key];
                    candidate_key = key;
                }
            }

            key_to_evict = candidate_key;
        } else {
            // 所有项都在保护期内，选择最旧的项淘汰
            if (!time_sorted_keys_.empty()) {
                // 最旧的项在时间排序集合的开头
                auto it = time_sorted_keys_.begin();
                key_to_evict = it->key;
            } else {
                return;  // 没有任何键可以淘汰
            }
        }

        // 为了删除，我们需要先获取时间值，因为之后数据会被删除
        TimePoint eviction_time = create_times_[key_to_evict];
        
        // 淘汰选中的键
        data_.erase(key_to_evict);
        counts_.erase(key_to_evict);
        create_times_.erase(key_to_evict);
        
        // 从时间排序集合中移除对应的项
        TimeKeyPair to_remove{eviction_time, key_to_evict};
        time_sorted_keys_.erase(to_remove);

        // 如果仍然超过大小限制，递归调用继续淘汰
        if (data_.size() > maxsize_) {
            evict_();
        }
    }
};

#endif // PROTECTED_LFU_CACHE_H