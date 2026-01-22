# 编译器设置
CXX = g++
CXXFLAGS = -std=c++14 -Wall -Wextra -g -O2 -fPIC
LDFLAGS = 

# 目录设置
SRC_DIR = ./arm_utils
BUILD_DIR = build
TEST_DIR = .

# 库设置
EIGEN_INCLUDE = /usr/include/eigen3
PYBIND11_INCLUDE = $(shell python3 -m pybind11 --includes)

# 头文件路径
INCLUDES = -I$(SRC_DIR) -I$(EIGEN_INCLUDE) $(PYBIND11_INCLUDE)

# 源文件和对象文件
SOURCES = $(wildcard $(SRC_DIR)/*.cpp)
# 排除测试文件和Python绑定文件
CPP_SOURCES = $(filter-out $(TEST_DIR)/test_%.cpp $(SRC_DIR)/pybind_interface.cpp, $(SOURCES))
OBJECTS = $(patsubst $(SRC_DIR)/%.cpp,$(BUILD_DIR)/%.o,$(CPP_SOURCES))

# 可执行文件名称
TARGET_LIB = libdarwin.a
TARGET_TEST = test_darwin
TARGET_EXE = darwin_example
TARGET_PYBIND = arm_robot_cpp.so

.PHONY: all clean test example pybind

# 主要目标
all: $(TARGET_LIB) $(TARGET_EXE)

# 创建构建目录
$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

# 编译库文件
$(TARGET_LIB): $(OBJECTS) | $(BUILD_DIR)
	ar rcs $@ $^
	ranlib $@

# 编译对象文件
$(BUILD_DIR)/%.o: $(SRC_DIR)/%.cpp | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

# 编译示例程序
$(TARGET_EXE): $(TARGET_LIB) $(BUILD_DIR)/darwin.o
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ $(SRC_DIR)/darwin.cpp -L. -ldarwin

# 编译并运行测试
test: $(TARGET_LIB) $(TARGET_TEST)
	./$(TARGET_TEST)

$(TARGET_TEST): $(TARGET_LIB) $(BUILD_DIR)/test_darwin.o
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ $(BUILD_DIR)/test_darwin.o -L. -ldarwin

# 编译Python绑定
pybind: $(TARGET_LIB) $(TARGET_PYBIND)

$(TARGET_PYBIND): $(TARGET_LIB) pybind_interface.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) -shared -o $@ pybind_interface.cpp -L. -ldarwin

# 清理
clean:
	rm -rf $(BUILD_DIR) $(TARGET_LIB) $(TARGET_TEST) $(TARGET_EXE) $(TARGET_PYBIND)

# 依赖关系
$(OBJECTS): $(wildcard $(SRC_DIR)/*.hpp)