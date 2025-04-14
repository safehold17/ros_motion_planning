cmake_minimum_required(VERSION 3.0.2)
project(common)

get_filename_component(PROJECT_ROOT_DIR ${CMAKE_SOURCE_DIR} DIRECTORY)
include(${PROJECT_ROOT_DIR}/3rd/conanbuildinfo.cmake)
conan_basic_setup()
# 设置 Conan 依赖的环境（如库路径、包含路径等）。
# Conan 是 C++ 的包管理器，这里用于管理项目的第三方库依赖。

find_package(Eigen3 REQUIRED)
# Eigen3 是一个 C++ 的线性代数库，这里用于数学计算。

find_package(catkin REQUIRED COMPONENTS
  roscpp
  costmap_2d
)
# costmap_2d：ROS 导航栈中的 2D 代价地图库，可能用于路径规划或避障。

catkin_package(
	INCLUDE_DIRS   include
	LIBRARIES      ${PROJECT_NAME}
	CATKIN_DEPENDS roscpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)
#  包含路径

file(GLOB util_src_files src/util/*.cpp)
file(GLOB math_src_files_cpp src/math/*.cpp)
file(GLOB geometry_src_files src/geometry/*.cpp)
file(GLOB geometry_curve_src_files src/geometry/curve/*.cpp)
#  查找当前目录下所有 .cpp 文件，将路径列表存储到 SOURCES 变量中
#  源文件收集


add_library(${PROJECT_NAME}
  ${util_src_files}
  ${math_src_files_cpp}
  ${geometry_src_files}
  ${geometry_curve_src_files}
)
#  add_library：创建一个名为 common（由 ${PROJECT_NAME} 定义）的库，包含所有收集的源文件。
#  构建库

add_dependencies(${PROJECT_NAME} 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)
#  依赖声明

target_include_directories(${PROJECT_NAME} PUBLIC
  ${CONAN_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${CONAN_LIBS}
)

# install
install(DIRECTORY include/${PROJECT_NAME}/
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
	FILES_MATCHING PATTERN "*.h"
	PATTERN ".svn" EXCLUDE
)
#  安装 include/common/ 目录下的所有 .h 头文件到 Catkin 的标准头文件目录

install(TARGETS ${PROJECT_NAME}
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

# 安装规则