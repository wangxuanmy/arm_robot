import time
import threading
from collections import defaultdict


class ProtectedLFUCache:
    """
    一个带保护期的LFU缓存实现
    
    特性：
    1. 大小限制：缓存有最大容量限制
    2. LFU策略：优先淘汰访问频率最低的项
    3. 保护期机制：新加入的缓存项在保护期内不会被淘汰
    
    实现细节：
    - 不依赖cachetools.LFUCache
    - 自实现LFU算法和缓存管理
    - 添加了创建时间和保护期管理
    - 支持线程安全操作
    """

    def __init__(self, maxsize, protect_seconds=60):
        """
        初始化缓存
        
        Args:
            maxsize: 缓存最大容量
            protect_seconds: 新项保护期（秒），默认60秒
        """
        self.maxsize = maxsize
        self.protect_seconds = protect_seconds
        
        # 缓存存储
        self._data = {}  # 实际数据 {key: value}
        self._counts = defaultdict(int)  # 访问计数 {key: count}
        self._create_times = {}  # 创建时间 {key: timestamp}
        
        # 线程锁
        self._lock = threading.RLock()

    def __setitem__(self, key, value):
        """
        设置缓存项
        
        Args:
            key: 键
            value: 值
        """
        with self._lock:
            # 如果是新键，则记录创建时间
            if key not in self._data:
                self._create_times[key] = time.time()
                
            # 设置值
            self._data[key] = value
            
            # 检查是否需要淘汰项
            self._evict()

    def __getitem__(self, key):
        """
        获取缓存项
        
        Args:
            key: 键
            
        Returns:
            值
            
        Raises:
            KeyError: 键不存在
        """
        with self._lock:
            # 增加访问计数
            self._counts[key] += 1
            return self._data[key]

    def __delitem__(self, key):
        """
        删除缓存项
        
        Args:
            key: 要删除的键
        """
        with self._lock:
            del self._data[key]
            self._counts.pop(key, None)
            self._create_times.pop(key, None)

    def __contains__(self, key):
        """
        检查键是否存在
        
        Args:
            key: 键
            
        Returns:
            bool: 键是否存在
        """
        return key in self._data

    def __len__(self):
        """
        获取缓存大小
        
        Returns:
            int: 缓存项数量
        """
        return len(self._data)

    def keys(self):
        """
        获取所有键
        
        Returns:
            list: 所有键的列表
        """
        return list(self._data.keys())

    def items(self):
        """
        获取所有键值对
        
        Returns:
            list: 所有键值对的列表
        """
        with self._lock:
            return list(self._data.items())

    def clear(self):
        """
        清空缓存
        """
        with self._lock:
            self._data.clear()
            self._counts.clear()
            self._create_times.clear()

    def _evict(self):
        """
        淘汰缓存项
        
        淘汰策略：
        1. 优先淘汰保护期外访问频率最低的项
        2. 如果所有项都在保护期内，则淘汰最旧的项
        """
        # 只有当缓存超过最大容量时才进行淘汰
        if len(self._data) <= self.maxsize:
            return

        with self._lock:
            now = time.time()
            all_keys = list(self._data.keys())
            
            # 分离保护期内和保护期外的项
            protected_keys = []
            evictable_keys = []
            
            for key in all_keys:
                create_time = self._create_times.get(key, 0)
                if now - create_time <= self.protect_seconds:
                    protected_keys.append(key)
                else:
                    evictable_keys.append(key)
            
            # 如果有保护期外的项，从中选择访问次数最少的项淘汰
            if evictable_keys:
                # 找到访问次数最少的项
                min_count = min(self._counts[key] for key in evictable_keys)
                candidates = [key for key in evictable_keys if self._counts[key] == min_count]
                
                # 如果有多个候选，选择最旧的
                key_to_evict = min(candidates, key=lambda k: self._create_times[k])
                del self[key_to_evict]
            else:
                # 所有项都在保护期内，选择最旧的项淘汰
                oldest_key = min(protected_keys if protected_keys else all_keys, 
                               key=lambda k: self._create_times[k])
                del self[oldest_key]
            
            # 如果仍然超过大小限制，继续淘汰
            if len(self._data) > self.maxsize:
                self._evict()

    def get_create_time(self, key):
        """
        获取指定键的创建时间（用于测试）
        
        Args:
            key: 键
            
        Returns:
            float: 创建时间戳
        """
        return self._create_times.get(key, 0)


# 单元测试
if __name__ == "__main__":
    import unittest

    class TestProtectedLFUCache(unittest.TestCase):
        """ProtectedLFUCache的单元测试"""

        def setUp(self):
            """测试前准备"""
            self.cache = ProtectedLFUCache(maxsize=3, protect_seconds=1)

        def test_basic_operations(self):
            """测试基本操作"""
            # 添加项
            self.cache['a'] = 1
            self.assertEqual(len(self.cache), 1)
            self.assertEqual(self.cache['a'], 1)
            
            # 更新项
            self.cache['a'] = 2
            self.assertEqual(len(self.cache), 1)
            self.assertEqual(self.cache['a'], 2)
            
            # 删除项
            del self.cache['a']
            self.assertEqual(len(self.cache), 0)
            with self.assertRaises(KeyError):
                _ = self.cache['a']

        def test_size_limit(self):
            """测试大小限制"""
            # 添加超过限制的项
            self.cache['a'] = 1
            self.cache['b'] = 2
            self.cache['c'] = 3
            self.cache['d'] = 4  # 应该触发淘汰
            
            # 确保缓存大小不超过限制
            self.assertEqual(len(self.cache), 3)

        def test_protect_period(self):
            """测试保护期"""
            # 添加项
            self.cache['a'] = 1  # 将被保护
            self.cache['b'] = 2
            self.cache['c'] = 3
            
            # 频繁访问b和c
            for _ in range(5):
                _ = self.cache['b']
                _ = self.cache['c']
            
            # 等待保护期过后
            time.sleep(1.1)
            
            # 添加新项应该淘汰保护期外访问次数较少的a而不是b或c
            self.cache['d'] = 4
            
            # 检查结果
            self.assertNotIn('a', self.cache)  # a应该被淘汰
            self.assertIn('b', self.cache)
            self.assertIn('c', self.cache)
            self.assertIn('d', self.cache)

        def test_rapid_insertion_during_protect_period(self):
            """测试保护期内快速插入数据的行为"""
            # 创建一个保护期较长的缓存用于此测试
            cache = ProtectedLFUCache(maxsize=3, protect_seconds=3)
            
            # 添加初始数据
            cache['a'] = 1
            time.sleep(0.01)  # 确保创建时间不同
            cache['b'] = 2
            time.sleep(0.01)
            cache['c'] = 3
            
            # 频繁访问所有项，确保它们都有较高的访问频率
            for _ in range(5):
                _ = cache['a']
                _ = cache['b']
                _ = cache['c']
            
            # 等待保护期过后
            time.sleep(3.1)
            
            # 快速添加3个新项（在短时间内添加，确保它们都在保护期内）
            start_time = time.time()
            cache['d'] = 4
            cache['e'] = 5
            cache['f'] = 6
            end_time = time.time()
            
            # 确保添加操作在很短的时间内完成
            self.assertLess(end_time - start_time, 1.0)
            
            # 检查结果 - 应该只保留最后3个（d, e, f），因为它们受保护期保护
            self.assertNotIn('a', cache)
            self.assertNotIn('b', cache)
            self.assertNotIn('c', cache)
            self.assertIn('d', cache)
            self.assertIn('e', cache)
            self.assertIn('f', cache)

        def test_clear(self):
            """测试清空缓存"""
            self.cache['a'] = 1
            self.cache['b'] = 2
            self.cache.clear()
            
            self.assertEqual(len(self.cache), 0)
            self.assertEqual(len(self.cache._create_times), 0)

        def test_get_create_time(self):
            """测试获取创建时间"""
            before = time.time()
            self.cache['a'] = 1
            after = time.time()
            
            create_time = self.cache.get_create_time('a')
            
            self.assertGreaterEqual(create_time, before)
            self.assertLessEqual(create_time, after)

        def test_items_method(self):
            """测试items方法"""
            # 添加项
            self.cache['a'] = 1
            self.cache['b'] = 2
            
            # 获取所有键值对
            items = self.cache.items()
            
            # 检查结果
            self.assertIn(('a', 1), items)
            self.assertIn(('b', 2), items)
            self.assertEqual(len(items), 2)

    # 运行测试
    unittest.main(argv=['first-arg-is-ignored'], exit=False, verbosity=2)