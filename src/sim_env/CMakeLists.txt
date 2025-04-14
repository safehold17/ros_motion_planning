# 用于构建一个机器人仿真环境，包括多机器人导航、Gazebo 仿真和 RViz 可视化

cmake_minimum_required(VERSION 3.0.2)  # 指定 CMake 的最低版本要求为 3.0.2
project(sim_env)
find_package(catkin REQUIRED COMPONENTS  # ROS 生态系统中已有的标准或第三方功能包
  roscpp
  rospy
  std_msgs
  message_generation
  controller_manager  # 管理机器人控制器
  gazebo_ros
  rviz
)

catkin_package(
  CATKIN_DEPENDS urdf xacro
)

# CATKIN_DEPENDS urdf xacro：声明运行时依赖于 urdf 和 xacro 两个功能包。
#     urdf：用于加载机器人描述文件（URDF，Unified Robot Description Format），定义机器人的物理结构和运动学。
#     xacro：URDF 的扩展工具，支持宏和参数化，简化复杂机器人模型的定义。




# Catkin 是 ROS（机器人操作系统，Robot Operating System）官方推荐的构建系统，用于组织、编译和管理 ROS 功能包（packages）。
# Catkin 是一个工具链，帮助开发者：
# 编译 ROS 功能包中的代码（C++、Python 等）。
# 管理功能包之间的依赖关系。
# 安装资源文件（如 URDF、地图、配置文件）。
# 集成仿真工具（如 Gazebo、RViz）。
