# 编译器设置
CXX = g++
CXXFLAGS = -std=c++17 -Wall -Wextra -g -O2 -fPIC
LDFLAGS = 

# 目录设置
SRC_DIR = ./arm_utils
PLANNER_DIR = .
BUILD_DIR = build
TEST_DIR = .

# 库设置
EIGEN_INCLUDE = /usr/include/eigen3
PYBIND11_INCLUDE = $(shell python3 -m pybind11 --includes)

# 头文件路径
INCLUDES = -I$(SRC_DIR) -I$(PLANNER_DIR) -I$(EIGEN_INCLUDE) $(PYBIND11_INCLUDE)

# 源文件和对象文件
SOURCES = $(wildcard $(SRC_DIR)/*.cpp)
PLANNER_SOURCES = $(wildcard $(PLANNER_DIR)/Map3D.cpp $(PLANNER_DIR)/AStar3D.cpp $(PLANNER_DIR)/PathSmoother.cpp $(PLANNER_DIR)/PathPlanner3D.cpp)
# 排除测试文件和Python绑定文件
CPP_SOURCES = $(filter-out $(TEST_DIR)/test_%.cpp $(TEST_DIR)/test_astar3d.cpp $(TEST_DIR)/test_path_smoother.cpp $(TEST_DIR)/test_planner_full.cpp $(SRC_DIR)/pybind_interface.cpp, $(SOURCES))
OBJECTS = $(patsubst $(SRC_DIR)/%.cpp,$(BUILD_DIR)/%.o,$(CPP_SOURCES))
PLANNER_OBJECTS = $(patsubst %.cpp,$(BUILD_DIR)/%.o,$(PLANNER_SOURCES))

# 可执行文件名称
TARGET_LIB = libdarwin.a
TARGET_PLANNER_LIB = libplanner.a
TARGET_TEST = test_darwin
TARGET_ASTAR_TEST = test_astar3d
TARGET_PATH_SMOOTHER_TEST = test_path_smoother
TARGET_FULL_TEST = test_planner_full
TARGET_EXE = darwin_example
TARGET_PYBIND = arm_robot_cpp.so

.PHONY: all clean test example pybind planner_test path_smoother_test full_test

# 主要目标
all: $(TARGET_LIB) $(TARGET_PLANNER_LIB) example

# 创建构建目录
$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

# 编译库文件
$(TARGET_LIB): $(OBJECTS) | $(BUILD_DIR)
	ar rcs $@ $^
	ranlib $@

# 编译规划库文件
$(TARGET_PLANNER_LIB): $(PLANNER_OBJECTS) | $(BUILD_DIR)
	ar rcs $@ $^
	ranlib $@

# 编译对象文件
$(BUILD_DIR)/%.o: $(SRC_DIR)/%.cpp | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

$(BUILD_DIR)/%.o: %.cpp | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

# 编译示例程序
example: $(TARGET_LIB) $(TARGET_EXE)

$(TARGET_EXE): $(TARGET_LIB) $(BUILD_DIR)/test_darwin.o
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ $(BUILD_DIR)/test_darwin.o -L. -ldarwin

# 编译并运行AStar测试
planner_test: $(TARGET_PLANNER_LIB) $(TARGET_ASTAR_TEST)
	./$(TARGET_ASTAR_TEST)

$(TARGET_ASTAR_TEST): $(TARGET_PLANNER_LIB) $(BUILD_DIR)/test_astar3d.o
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ $(BUILD_DIR)/test_astar3d.o -L. -lplanner

# 编译并运行路径平滑测试
path_smoother_test: $(TARGET_PLANNER_LIB) $(TARGET_PATH_SMOOTHER_TEST)
	./$(TARGET_PATH_SMOOTHER_TEST)

$(TARGET_PATH_SMOOTHER_TEST): $(TARGET_PLANNER_LIB) $(BUILD_DIR)/test_path_smoother.o
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ $(BUILD_DIR)/test_path_smoother.o -L. -lplanner

# 编译并运行完整测试
full_test: $(TARGET_PLANNER_LIB) $(TARGET_FULL_TEST)
	./$(TARGET_FULL_TEST)

$(TARGET_FULL_TEST): $(TARGET_PLANNER_LIB) $(BUILD_DIR)/test_planner_full.o
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ $(BUILD_DIR)/test_planner_full.o -L. -lplanner

# 编译并运行测试
test: $(TARGET_LIB) $(TARGET_TEST)
	./$(TARGET_TEST)

$(TARGET_TEST): $(TARGET_LIB) $(BUILD_DIR)/test_darwin.o
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ $(BUILD_DIR)/test_darwin.o -L. -lplanner -ldarwin

# 编译Python绑定
pybind: $(TARGET_LIB) $(TARGET_PYBIND)

$(TARGET_PYBIND): $(TARGET_LIB) pybind_interface.cpp $(PLANNER_SOURCES)
	$(CXX) $(CXXFLAGS) $(INCLUDES) -shared -o $@ pybind_interface.cpp -L. -ldarwin $(PLANNER_SOURCES)

# 清理
clean:
	rm -rf $(BUILD_DIR) $(TARGET_LIB) $(TARGET_PLANNER_LIB) $(TARGET_TEST) $(TARGET_ASTAR_TEST) $(TARGET_PATH_SMOOTHER_TEST) $(TARGET_FULL_TEST) $(TARGET_EXE) $(TARGET_PYBIND) *.csv

# 依赖关系
$(OBJECTS): $(wildcard $(SRC_DIR)/*.hpp)
$(PLANNER_OBJECTS): $(wildcard $(PLANNER_DIR)/*.h)