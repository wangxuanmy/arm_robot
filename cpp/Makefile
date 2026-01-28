# 编译器设置
CXX = g++
CXXFLAGS = -std=c++17 -Wall -Wextra -g -O2 -fPIC
LDFLAGS = -ljsoncpp

# 目录设置
SRC_DIR = ./arm_utils
PLANNER_DIR = ./plan
BUILD_DIR = build
TEST_DIR = .

# 库设置
EIGEN_INCLUDE = /usr/include/eigen3
JSONCPP_INCLUDE = $(shell pkg-config --cflags jsoncpp)
PYBIND11_INCLUDE = $(shell python3 -m pybind11 --includes)

# 头文件路径
INCLUDES = -I$(SRC_DIR) -I$(PLANNER_DIR) -I$(EIGEN_INCLUDE) $(JSONCPP_INCLUDE) $(PYBIND11_INCLUDE)

# 源文件
ARM_UTILS_SOURCES = $(wildcard $(SRC_DIR)/*.cpp)
PLAN_SOURCES = $(wildcard $(PLANNER_DIR)/*.cpp)
ROBOT_CTRL_SOURCES = robot_ctrl.cpp
EXAMPLE_SOURCES = example_robot_ctrl.cpp

# 对象文件
ARM_UTILS_OBJECTS = $(ARM_UTILS_SOURCES:.cpp=.o)
PLAN_OBJECTS = $(PLAN_SOURCES:.cpp=.o)
ROBOT_CTRL_OBJECTS = $(ROBOT_CTRL_SOURCES:.cpp=.o)
EXAMPLE_OBJECTS = $(EXAMPLE_SOURCES:.cpp=.o)

# 在BUILD_DIR中放置对象文件
ARM_UTILS_BUILD_OBJS = $(addprefix $(BUILD_DIR)/, $(notdir $(ARM_UTILS_OBJECTS)))
PLAN_BUILD_OBJS = $(addprefix $(BUILD_DIR)/, $(notdir $(PLAN_OBJECTS)))
ROBOT_CTRL_BUILD_OBJS = $(addprefix $(BUILD_DIR)/, $(notdir $(ROBOT_CTRL_OBJECTS)))
EXAMPLE_BUILD_OBJS = $(addprefix $(BUILD_DIR)/, $(notdir $(EXAMPLE_OBJECTS)))

# 可执行文件名称
TARGET_LIB = libdarwin.a
TARGET_PLANNER_LIB = libplanner.a
TARGET_ROBOT_CTRL_LIB = librobotctrl.a
TARGET_TEST = test_darwin
TARGET_ASTAR_TEST = test_astar3d
TARGET_PATH_SMOOTHER_TEST = test_path_smoother
TARGET_FULL_TEST = test_planner_full
TARGET_EXE = darwin_example
TARGET_ROBOT_CTRL_EXE = example_robot_ctrl
TARGET_PYBIND = arm_robot_cpp.so

.PHONY: all clean test example pybind planner_test path_smoother_test full_test robot_ctrl_example

# 主要目标
all: $(TARGET_LIB) $(TARGET_PLANNER_LIB) $(TARGET_ROBOT_CTRL_LIB) example robot_ctrl_example

# 创建构建目录
$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

# 编译所有库
$(TARGET_LIB): $(BUILD_DIR) $(ARM_UTILS_BUILD_OBJS)
	ar rcs $@ $^
	ranlib $@

$(TARGET_PLANNER_LIB): $(BUILD_DIR) $(PLAN_BUILD_OBJS)
	ar rcs $@ $^
	ranlib $@

$(TARGET_ROBOT_CTRL_LIB): $(BUILD_DIR) $(ROBOT_CTRL_BUILD_OBJS)
	ar rcs $@ $^
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

# 编译并运行测试
test: $(TARGET_LIB) $(TARGET_TEST)
	./$(TARGET_TEST)

$(TARGET_TEST): $(BUILD_DIR)/test_darwin.o $(TARGET_LIB)
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ $< -L. -ldarwin

# 编译Python绑定
pybind: $(TARGET_LIB) $(TARGET_PLANNER_LIB) $(TARGET_PYBIND)

$(TARGET_PYBIND): pybind_interface.cpp $(TARGET_LIB) $(TARGET_PLANNER_LIB)
	$(CXX) $(CXXFLAGS) $(INCLUDES) -shared -o $@ $< -L. -ldarwin -lplanner

# 清理
clean:
	rm -rf $(BUILD_DIR) $(TARGET_LIB) $(TARGET_PLANNER_LIB) $(TARGET_ROBOT_CTRL_LIB) $(TARGET_TEST) $(TARGET_ASTAR_TEST) $(TARGET_PATH_SMOOTHER_TEST) $(TARGET_FULL_TEST) $(TARGET_EXE) $(TARGET_ROBOT_CTRL_EXE) $(TARGET_PYBIND) *.csv

# 依赖关系
$(ARM_UTILS_BUILD_OBJS): $(wildcard $(SRC_DIR)/*.hpp)
$(PLAN_BUILD_OBJS): $(wildcard $(PLANNER_DIR)/*.h)