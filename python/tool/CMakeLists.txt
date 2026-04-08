cmake_minimum_required(VERSION 3.8)
project(robot_ctrl_standalone)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

# Find dependencies
find_package(Eigen3 REQUIRED)
find_package(PkgConfig REQUIRED)

# Use pkg-config to find jsoncpp
pkg_check_modules(JSONCPP jsoncpp)
if(NOT JSONCPP_FOUND)
    message(FATAL_ERROR "jsoncpp library not found")
endif()

# Find FCL for collision detection
find_package(fcl REQUIRED)
find_package(urdfdom REQUIRED)
find_package(console_bridge REQUIRED)

# Include directories
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/../include
  ${CMAKE_CURRENT_SOURCE_DIR}/../include/arm_utils
  ${CMAKE_CURRENT_SOURCE_DIR}/../include/std_robot_ctrl
  ${CMAKE_CURRENT_SOURCE_DIR}/../include/plan
  ${JSONCPP_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${FCL_INCLUDE_DIRS}
  ${console_bridge_INCLUDE_DIRS}
  ${urdfdom_INCLUDE_DIRS}
)

# Define source files
set(SRC_FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/../src/robot_ctrl.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../src/collision_detector.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../src/urdf_processor.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../src/arm_utils/darwin.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../src/arm_utils/body_base.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../src/arm_utils/coord.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../src/arm_utils/dh.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../src/arm_utils/quaternion_utils.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../src/plan/Map3D.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../src/plan/AStar3D.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../src/plan/PathSmoother.cpp
)

# Main robot control library
add_library(robot_ctrl_lib STATIC ${SRC_FILES})
target_include_directories(robot_ctrl_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(robot_ctrl_lib
  ${JSONCPP_LIBRARIES}
  Eigen3::Eigen
  ${FCL_LIBRARIES}
  ${console_bridge_LIBRARIES}
  ${urdfdom_LIBRARIES}
)





# DH to URDF converter executable
add_executable(dh_to_urdf_converter src/dh_to_urdf_converter.cpp)
target_link_libraries(dh_to_urdf_converter 
  robot_ctrl_lib
  ${urdfdom_LIBRARIES}
  ${console_bridge_LIBRARIES}
)


# Install
install(TARGETS 
  dh_to_urdf_converter
  DESTINATION lib/${PROJECT_NAME})