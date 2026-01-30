# 编译器设置
CXX = g++
CXXFLAGS = -std=c++17 -Wall -Wextra -g -O2 -fPIC
LDFLAGS = -ljsoncpp -lurdfdom_model -lurdfdom_world -lorocos-kdl -lfcl -lconsole_bridge

# 目录设置
SRC_DIR = ./arm_utils
PLANNER_DIR = ./plan
BUILD_DIR = build
TEST_DIR = .

# 库设置
EIGEN_INCLUDE = /usr/include/eigen3
JSONCPP_INCLUDE = $(shell pkg-config --cflags jsoncpp)
PYBIND11_INCLUDE = $(shell python3 -m pybind11 --includes)

# URDF/FCL Includes - Updated for ROS Humble
URDF_INCLUDE = /opt/ros/humble/include
KDL_INCLUDE = /usr/include/orocos-kdl
FCL_INCLUDE = /usr/include/fcl
KDL_PARSER_INCLUDE1 = /opt/ros/humble/include
KDL_PARSER_INCLUDE2 = /opt/ros/humble/include/kdl_parser
KDL_PARSER_LIB_DIR = -L/opt/ros/humble/lib
KDL_PARSER_LIB = -lkdl_parser

# 额外的ROS库
ADDITIONAL_ROS_LIBS = -lurdf -lrcutils -lrosidl_typesupport_cpp -lbuiltin_interfaces__rosidl_typesupport_cpp -lgeometry_msgs__rosidl_typesupport_cpp

# 头文件路径
INCLUDES = -I$(SRC_DIR) -I$(PLANNER_DIR) -I$(EIGEN_INCLUDE) -I$(URDF_INCLUDE) -I$(KDL_INCLUDE) -I$(FCL_INCLUDE) -I$(KDL_PARSER_INCLUDE1) -I$(KDL_PARSER_INCLUDE2) $(JSONCPP_INCLUDE) $(PYBIND11_INCLUDE)

# 源文件
ARM_UTILS_SOURCES = $(wildcard $(SRC_DIR)/*.cpp)
PLAN_SOURCES = $(wildcard $(PLANNER_DIR)/*.cpp) PathPlanner3D.cpp  # 添加PathPlanner3D.cpp
ROBOT_CTRL_SOURCES = robot_ctrl.cpp
URDF_PROCESSOR_SOURCES = urdf_processor.cpp
COLLISION_DETECTOR_SOURCES = collision_detector.cpp
EXAMPLE_SOURCES = example_robot_ctrl.cpp
CACHE_TEST_SOURCES = test_protected_lfu_cache.cpp
URDF_LOAD_TEST_SOURCES = test_urdf_load.cpp
SIMPLE_URDF_TEST_SOURCES = simple_urdf_test.cpp
COLLISION_DETECTION_TEST_SOURCES = test_collision_detection.cpp

# 对象文件
ARM_UTILS_OBJECTS = $(ARM_UTILS_SOURCES:.cpp=.o)
PLAN_OBJECTS = $(PLAN_SOURCES:.cpp=.o)
ROBOT_CTRL_OBJECTS = $(ROBOT_CTRL_SOURCES:.cpp=.o)
URDF_PROCESSOR_OBJECTS = $(URDF_PROCESSOR_SOURCES:.cpp=.o)
COLLISION_DETECTOR_OBJECTS = $(COLLISION_DETECTOR_SOURCES:.cpp=.o)
EXAMPLE_OBJECTS = $(EXAMPLE_SOURCES:.cpp=.o)
CACHE_TEST_OBJECTS = $(CACHE_TEST_SOURCES:.cpp=.o)
URDF_LOAD_TEST_OBJECTS = $(URDF_LOAD_TEST_SOURCES:.cpp=.o)
SIMPLE_URDF_TEST_OBJECTS = $(SIMPLE_URDF_TEST_OBJECTS:.cpp=.o)
COLLISION_DETECTION_TEST_OBJECTS = $(COLLISION_DETECTION_TEST_OBJECTS:.cpp=.o)

# 在BUILD_DIR中放置对象文件
ARM_UTILS_BUILD_OBJS = $(addprefix $(BUILD_DIR)/, $(notdir $(ARM_UTILS_OBJECTS)))
PLAN_BUILD_OBJS = $(addprefix $(BUILD_DIR)/, $(notdir $(PLAN_OBJECTS)))
ROBOT_CTRL_BUILD_OBJS = $(addprefix $(BUILD_DIR)/, $(notdir $(ROBOT_CTRL_OBJECTS)))
URDF_PROCESSOR_BUILD_OBJS = $(addprefix $(BUILD_DIR)/, $(notdir $(URDF_PROCESSOR_OBJECTS)))
COLLISION_DETECTOR_BUILD_OBJS = $(addprefix $(BUILD_DIR)/, $(notdir $(COLLISION_DETECTOR_OBJECTS)))
EXAMPLE_BUILD_OBJS = $(addprefix $(BUILD_DIR)/, $(notdir $(EXAMPLE_OBJECTS)))
CACHE_TEST_BUILD_OBJS = $(addprefix $(BUILD_DIR)/, $(notdir $(CACHE_TEST_OBJECTS)))
URDF_LOAD_TEST_BUILD_OBJS = $(addprefix $(BUILD_DIR)/, $(notdir $(URDF_LOAD_TEST_OBJECTS)))
SIMPLE_URDF_TEST_BUILD_OBJS = $(addprefix $(BUILD_DIR)/, $(notdir $(SIMPLE_URDF_TEST_OBJECTS)))
COLLISION_DETECTION_TEST_BUILD_OBJS = $(addprefix $(BUILD_DIR)/, $(notdir $(COLLISION_DETECTION_TEST_OBJECTS)))

# 可执行文件名称
TARGET_LIB = libdarwin.a
TARGET_PLANNER_LIB = libplanner.a
TARGET_ROBOT_CTRL_LIB = librobotctrl.a
TARGET_URDF_PROCESSOR_LIB = liburdfprocessor.a
TARGET_COLLISION_DETECTOR_LIB = libcollisiondetector.a
TARGET_TEST = test_darwin
TARGET_ASTAR_TEST = test_astar3d
TARGET_PATH_SMOOTHER_TEST = test_path_smoother
TARGET_FULL_TEST = test_planner_full
TARGET_EXE = darwin_example
TARGET_ROBOT_CTRL_EXE = example_robot_ctrl
TARGET_PYBIND = arm_robot_cpp.so
TARGET_CACHE_TEST = test_protected_lfu_cache
TARGET_URDF_COLLISION_EXE = test_urdf_collision
TARGET_URDF_LOAD_TEST = test_urdf_load
TARGET_SIMPLE_URDF_TEST = simple_urdf_test
TARGET_COLLISION_DETECTION_TEST = test_collision_detection

.PHONY: all clean test example pybind planner_test path_smoother_test full_test robot_ctrl_example cache_test urdf_collision_test urdf_load_test simple_urdf_test collision_detection_test

# 主要目标
all: $(TARGET_LIB) $(TARGET_PLANNER_LIB) $(TARGET_ROBOT_CTRL_LIB) $(TARGET_URDF_PROCESSOR_LIB) $(TARGET_COLLISION_DETECTOR_LIB) example robot_ctrl_example

# 创建构建目录
$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

# 编译所有库
$(TARGET_LIB): $(BUILD_DIR) $(ARM_UTILS_BUILD_OBJS)
	ar rcs $@ $(ARM_UTILS_BUILD_OBJS)
	ranlib $@

$(TARGET_PLANNER_LIB): $(BUILD_DIR) $(PLAN_BUILD_OBJS)
	ar rcs $@ $(PLAN_BUILD_OBJS)
	ranlib $@

$(TARGET_ROBOT_CTRL_LIB): $(BUILD_DIR) $(ROBOT_CTRL_BUILD_OBJS)
	ar rcs $@ $(ROBOT_CTRL_BUILD_OBJS)
	ranlib $@

$(TARGET_URDF_PROCESSOR_LIB): $(BUILD_DIR) $(URDF_PROCESSOR_BUILD_OBJS)
	ar rcs $@ $(URDF_PROCESSOR_BUILD_OBJS)
	ranlib $@

$(TARGET_COLLISION_DETECTOR_LIB): $(BUILD_DIR) $(COLLISION_DETECTOR_BUILD_OBJS)
	ar rcs $@ $(COLLISION_DETECTOR_BUILD_OBJS)
	ranlib $@

# 编译对象文件到build目录
$(BUILD_DIR)/%.o: $(SRC_DIR)/%.cpp | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

$(BUILD_DIR)/%.o: $(PLANNER_DIR)/%.cpp | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

$(BUILD_DIR)/%.o: %.cpp | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

# 编译示例程序
example: $(TARGET_LIB) $(TARGET_EXE)

$(TARGET_EXE): $(BUILD_DIR)/test_darwin.o $(TARGET_LIB)
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ $< -L. -ldarwin

# 编译RobotCtrl示例程序
robot_ctrl_example: $(TARGET_ROBOT_CTRL_LIB) $(TARGET_PLANNER_LIB) $(TARGET_LIB) $(TARGET_ROBOT_CTRL_EXE)

$(TARGET_ROBOT_CTRL_EXE): $(EXAMPLE_BUILD_OBJS) $(TARGET_ROBOT_CTRL_LIB) $(TARGET_PLANNER_LIB) $(TARGET_LIB)
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ $< -L. -lrobotctrl -lplanner -ldarwin $(LDFLAGS)

# 编译并运行AStar测试
planner_test: $(TARGET_PLANNER_LIB) $(TARGET_ASTAR_TEST)
	./$(TARGET_ASTAR_TEST)

$(TARGET_ASTAR_TEST): $(BUILD_DIR)/test_astar3d.o $(TARGET_PLANNER_LIB)
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ $< -L. -lplanner

# 编译并运行路径平滑测试
path_smoother_test: $(TARGET_PLANNER_LIB) $(TARGET_PATH_SMOOTHER_TEST)
	./$(TARGET_PATH_SMOOTHER_TEST)

$(TARGET_PATH_SMOOTHER_TEST): $(BUILD_DIR)/test_path_smoother.o $(TARGET_PLANNER_LIB)
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ $< -L. -lplanner

# 编译并运行完整测试
full_test: $(TARGET_PLANNER_LIB) $(TARGET_FULL_TEST)
	./$(TARGET_FULL_TEST)

$(TARGET_FULL_TEST): $(BUILD_DIR)/test_planner_full.o $(TARGET_PLANNER_LIB)
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ $< -L. -lplanner

# 编译并运行缓存测试
cache_test: $(TARGET_LIB) $(TARGET_CACHE_TEST)
	./$(TARGET_CACHE_TEST)

$(TARGET_CACHE_TEST): $(CACHE_TEST_BUILD_OBJS) $(TARGET_LIB)
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ $< -L. -ldarwin

# 编译并运行URDF碰撞检测测试
urdf_collision_test: $(TARGET_COLLISION_DETECTOR_LIB) $(TARGET_URDF_COLLISION_EXE)
	./$(TARGET_URDF_COLLISION_EXE)

$(TARGET_URDF_COLLISION_EXE): $(BUILD_DIR)/test_urdf_collision.o $(TARGET_COLLISION_DETECTOR_LIB)
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ $< -L. -lcollisiondetector $(LDFLAGS)

# 编译并运行URDF加载测试
urdf_load_test: $(TARGET_URDF_PROCESSOR_LIB) $(TARGET_URDF_LOAD_TEST)
	./$(TARGET_URDF_LOAD_TEST)

$(TARGET_URDF_LOAD_TEST): $(URDF_LOAD_TEST_BUILD_OBJS) $(TARGET_URDF_PROCESSOR_LIB)
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ $< -L. -lurdfprocessor $(LDFLAGS) $(KDL_PARSER_LIB_DIR) $(KDL_PARSER_LIB) $(ADDITIONAL_ROS_LIBS)

# 编译并运行简化版URDF测试
simple_urdf_test: $(TARGET_URDF_PROCESSOR_LIB) $(TARGET_SIMPLE_URDF_TEST)
	./$(TARGET_SIMPLE_URDF_TEST)

$(TARGET_SIMPLE_URDF_TEST): $(BUILD_DIR)/simple_urdf_test.o $(TARGET_URDF_PROCESSOR_LIB)
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ $< -L. -lurdfprocessor -ljsoncpp -lurdfdom_model -lurdfdom_world -lorocos-kdl -lfcl

# 编译并运行碰撞检测测试
collision_detection_test: $(TARGET_COLLISION_DETECTOR_LIB) $(TARGET_URDF_PROCESSOR_LIB) $(TARGET_COLLISION_DETECTION_TEST)
	./$(TARGET_COLLISION_DETECTION_TEST)

$(TARGET_COLLISION_DETECTION_TEST): $(BUILD_DIR)/test_collision_detection.o $(TARGET_COLLISION_DETECTOR_LIB) $(TARGET_URDF_PROCESSOR_LIB)
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ $< -L. -lcollisiondetector -lurdfprocessor -ljsoncpp -lurdfdom_model -lurdfdom_world -lorocos-kdl -lfcl $(KDL_PARSER_LIB_DIR) $(KDL_PARSER_LIB) $(ADDITIONAL_ROS_LIBS)

# 编译并运行测试
test: $(TARGET_LIB) $(TARGET_TEST)
	./$(TARGET_TEST)

$(TARGET_TEST): $(BUILD_DIR)/test_darwin.o $(TARGET_LIB)
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ $< -L. -lplanner -ldarwin

# 编译Python绑定
pybind: $(TARGET_LIB) $(TARGET_PLANNER_LIB) $(TARGET_PYBIND)

$(TARGET_PYBIND): pybind_interface.cpp $(TARGET_LIB) $(PLAN_BUILD_OBJS)
	$(CXX) $(CXXFLAGS) $(INCLUDES) -shared -o $@ $< -L. -ldarwin -lplanner $(LDFLAGS)

# 清理
clean:
	rm -rf $(BUILD_DIR) $(TARGET_LIB) $(TARGET_PLANNER_LIB) $(TARGET_ROBOT_CTRL_LIB) $(TARGET_URDF_PROCESSOR_LIB) $(TARGET_COLLISION_DETECTOR_LIB) $(TARGET_TEST) $(TARGET_ASTAR_TEST) $(TARGET_PATH_SMOOTHER_TEST) $(TARGET_FULL_TEST) $(TARGET_EXE) $(TARGET_ROBOT_CTRL_EXE) $(TARGET_PYBIND) $(TARGET_CACHE_TEST) $(TARGET_URDF_COLLISION_EXE) $(TARGET_URDF_LOAD_TEST) $(TARGET_SIMPLE_URDF_TEST) $(TARGET_COLLISION_DETECTION_TEST) *.csv

# 依赖关系
$(ARM_UTILS_BUILD_OBJS): $(wildcard $(SRC_DIR)/*.hpp)
$(PLAN_BUILD_OBJS): $(wildcard $(PLANNER_DIR)/*.h) PathPlanner3D.h
$(URDF_PROCESSOR_BUILD_OBJS): urdf_processor.h
$(COLLISION_DETECTOR_BUILD_OBJS): collision_detector.h urdf_processor.h