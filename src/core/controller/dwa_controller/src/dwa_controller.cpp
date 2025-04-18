
#include <Eigen/Core>
#include <cmath>

#include <ros/console.h>
#include <tf2/utils.h>
#include <nav_msgs/Path.h>
#include <pluginlib/class_list_macros.h>
#include <base_local_planner/goal_functions.h>

#include "controller/dwa_controller.h"

// register this planner as a BaseLocalPlanner plugin
PLUGINLIB_EXPORT_CLASS(rmp::controller::DWAController, nav_core::BaseLocalPlanner)

namespace rmp
{
namespace controller
{
void DWAController::reconfigureCB(dwa_controller::DWAControllerConfig& config, uint32_t level)
  // 由 ROS 动态重配置机制生成，用于在运行时传递用户设置的参数值
  // level 可用于优化，仅处理发生变化的参数
{
  if (setup_ && config.restore_defaults)
  {
    config = default_config_;
    config.restore_defaults = false;
  }
  if (!setup_)
  {
    default_config_ = config;
    setup_ = true;
  }
  // setup_的初始化在后面
  // default_config_的初始值在哪里？？？

  // update generic local planner params
  base_local_planner::LocalPlannerLimits limits;
  // limits 是一个临时的 LocalPlannerLimits 对象，通过从 config 复制通用参数（如速度、加速度、容差）创建
  limits.max_vel_trans = config.max_vel_trans;
  limits.min_vel_trans = config.min_vel_trans;
  limits.max_vel_x = config.max_vel_x;
  limits.min_vel_x = config.min_vel_x;
  limits.max_vel_y = config.max_vel_y;
  limits.min_vel_y = config.min_vel_y;
  limits.max_vel_theta = config.max_vel_theta;
  limits.min_vel_theta = config.min_vel_theta;
  limits.acc_lim_x = config.acc_lim_x;
  limits.acc_lim_y = config.acc_lim_y;
  limits.acc_lim_theta = config.acc_lim_theta;
  limits.acc_lim_trans = config.acc_lim_trans;
  limits.xy_goal_tolerance = config.xy_goal_tolerance;
  limits.yaw_goal_tolerance = config.yaw_goal_tolerance;
  limits.prune_plan = config.prune_plan;
  limits.trans_stopped_vel = config.trans_stopped_vel;
  limits.theta_stopped_vel = config.theta_stopped_vel;
  planner_util_.reconfigureCB(limits, config.restore_defaults);

  // update dwa specific configuration
  dp_->reconfigure(config);
}
// 通过 ROS 的动态重配置机制，允许用户在运行时调整参数

DWAController::DWAController() : initialized_(false), odom_helper_("odom"), setup_(false)
{
}
// 参数初始化

void DWAController::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros)
{
  if (!isInitialized())
  {
    ros::NodeHandle private_nh("~/" + name);
    g_plan_pub_ = private_nh.advertise<nav_msgs::Path>("global_plan", 1);
    l_plan_pub_ = private_nh.advertise<nav_msgs::Path>("local_plan", 1);
    // "global_plan" 和 "local_plan" 是 ROS 中的主题名称，用于发布全局和局部路径规划结果
    tf_ = tf;
    costmap_ros_ = costmap_ros;
    costmap_ros_->getRobotPose(current_pose_);

    // make sure to update the costmap we'll use for this cycle
    costmap_2d::Costmap2D* costmap = costmap_ros_->getCostmap();
    //获取 costmap
    
    planner_util_.initialize(tf, costmap, costmap_ros_->getGlobalFrameID());

    // create the actual planner that we'll use.. it'll configure itself from the parameter server
    dp_ = boost::shared_ptr<DWA>(new DWA(name, &planner_util_));
    // 创建一个 DWA 对象，初始化 DWA 算法的核心逻辑
    // 使用 boost::shared_ptr 管理 DWA 对象的内存，赋值给 dp_
    // 传递 name 和 planner_util_ 作为构造函数参数，设置 DWA 的命名空间和数据访问接口

    if (private_nh.getParam("odom_topic", odom_topic_))
    {
      odom_helper_.setOdomTopic(odom_topic_);
    }

    initialized_ = true;

    ROS_INFO("Using local controller: %s", name.c_str());

    dsrv_ = new dynamic_reconfigure::Server<dwa_controller::DWAControllerConfig>(private_nh);
    dynamic_reconfigure::Server<dwa_controller::DWAControllerConfig>::CallbackType cb =
        boost::bind(&DWAController::reconfigureCB, this, _1, _2);
    dsrv_->setCallback(cb);
    // 设置动态重配置服务器，允许运行时调整 DWAController 的参数
    // 通过 reconfigureCB，将用户修改的参数应用于规划器（planner_util_ 和 dp_）
  }
  else
  {
    ROS_WARN("This planner has already been initialized, doing nothing.");
  }
}
// 初始化与 ROS 的接口，如 tf（坐标变换缓冲区）、costmap（代价地图）等。
// 创建 DWA 对象（dp_），这是 DWA 算法的核心实现。
// 设置动态重配置服务器，绑定 reconfigureCB 回调函数。

bool DWAController::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan)
{
  if (!isInitialized())
  {
    ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
    return false;
  }
  // when we get a new plan, we also want to clear any latch we may have on goal tolerances
  latchedStopRotateController_.resetLatching();
  // 调用 LatchedStopRotateController 的 resetLatching 方法，重置目标容差的锁存状态
  // 确保在接收新全局路径时，规划器重新评估目标是否接近，而不是依赖旧的锁存状态
  // 锁存状态用于记录机器人是否已经接近目标（在目标容差范围内），并“锁定”这一状态以避免反复调整或震荡行为

  ROS_INFO("Got new plan");
  return dp_->setPlan(orig_global_plan);
}
// 接收全局路径（orig_global_plan），并传递给 dp_->setPlan。
// 重置目标容差的锁存状态（latchedStopRotateController_.resetLatching()），确保新路径生效。

bool DWAController::isGoalReached()
{
  if (!isInitialized())
  {
    ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
    return false;
  }
  if (!costmap_ros_->getRobotPose(current_pose_))
  {
    ROS_ERROR("Could not get robot pose");
    return false;
  }

  if (latchedStopRotateController_.isGoalReached(&planner_util_, odom_helper_, current_pose_))
  {
    ROS_INFO("Goal reached");
    return true;
  }
  else
  {
    return false;
  }
}

void DWAController::publishLocalPlan(std::vector<geometry_msgs::PoseStamped>& path)
{
  base_local_planner::publishPlan(path, l_plan_pub_);
}

void DWAController::publishGlobalPlan(std::vector<geometry_msgs::PoseStamped>& path)
{
  base_local_planner::publishPlan(path, g_plan_pub_);
}

DWAController::~DWAController()
{
  // make sure to clean things up
  delete dsrv_;
}

bool DWAController::dwaComputeVelocityCommands(geometry_msgs::PoseStamped& global_pose, geometry_msgs::Twist& cmd_vel)
// cmd_vel表示控制机器人移动的速度指令
{
  // dynamic window sampling approach to get useful velocity commands
  if (!isInitialized())
  {
    ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
    return false;
  }

  geometry_msgs::PoseStamped robot_vel;
  odom_helper_.getRobotVel(robot_vel);

  /* For timing uncomment
  struct timeval start, end;
  double start_t, end_t, t_diff;
  gettimeofday(&start, NULL);
  */

  // compute what trajectory to drive along
  geometry_msgs::PoseStamped drive_cmds;
  drive_cmds.header.frame_id = costmap_ros_->getBaseFrameID();
  // 指定坐标系为机器人的基座坐标系 ID

  // call with updated footprint
  base_local_planner::Trajectory path = dp_->findBestPath(global_pose, robot_vel, drive_cmds);
  // ROS_ERROR("Best: %.2f, %.2f, %.2f, %.2f", path.xv_, path.yv_, path.thetav_, path.cost_);

  /* For timing uncomment
  gettimeofday(&end, NULL);
  start_t = start.tv_sec + double(start.tv_usec) / 1e6;
  end_t = end.tv_sec + double(end.tv_usec) / 1e6;
  t_diff = end_t - start_t;
  ROS_INFO("Cycle time: %.9f", t_diff);
  */

  // pass along drive commands
  cmd_vel.linear.x = drive_cmds.pose.position.x;
  cmd_vel.linear.y = drive_cmds.pose.position.y;
  cmd_vel.angular.z = tf2::getYaw(drive_cmds.pose.orientation);
  // 速度为什么能等于位置？？？

  // if we cannot move... tell someone
  std::vector<geometry_msgs::PoseStamped> local_plan;
  if (path.cost_ < 0)
  {
    ROS_DEBUG_NAMED("dwa_local_planner",
                    "The dwa local planner failed to find a valid plan, cost functions discarded all candidates. This "
                    "can mean there is an obstacle too close to the robot.");
    local_plan.clear();
    publishLocalPlan(local_plan);
    // 清空当前的局部路径
    // 向系统发布一个空的局部路径
    return false;
  }

  ROS_DEBUG_NAMED("dwa_local_planner", "A valid velocity command of (%.2f, %.2f, %.2f) was found for this cycle.",
                  cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z);

  // Fill out the local plan
  for (unsigned int i = 0; i < path.getPointsSize(); ++i)
  {
    double p_x, p_y, p_th;
    path.getPoint(i, p_x, p_y, p_th);

    geometry_msgs::PoseStamped p;
    p.header.frame_id = costmap_ros_->getGlobalFrameID();
    p.header.stamp = ros::Time::now();
    p.pose.position.x = p_x;
    p.pose.position.y = p_y;
    p.pose.position.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0, 0, p_th);
    tf2::convert(q, p.pose.orientation);
    local_plan.push_back(p);
  }

  // publish information to the visualizer

  publishLocalPlan(local_plan);
  return true;
}
// 获取机器人当前位姿和速度。
// 调用 dp_->findBestPath 生成最佳轨迹。
// 将最佳轨迹的速度命令转换为 cmd_vel（线速度和角速度）。
// 发布局部路径供可视化。

bool DWAController::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
// cmd_vel 在函数调用时是一个空的或未初始化的变量，函数的目的是通过 DWA 算法计算速度命令并填充到 cmd_vel 中
{
  // dispatches to either dwa sampling control or stop and rotate control, depending on whether we have been close
  // enough to goal
  if (!costmap_ros_->getRobotPose(current_pose_))
  {
    ROS_ERROR("Could not get robot pose");
    return false;
  }
  std::vector<geometry_msgs::PoseStamped> transformed_plan;
  if (!planner_util_.getLocalPlan(current_pose_, transformed_plan))
  {
    ROS_ERROR("Could not get local plan");
    return false;
  }

  // if the global plan passed in is empty... we won't do anything
  if (transformed_plan.empty())
  // 检查是否为空
  {
    ROS_WARN_NAMED("dwa_local_planner", "Received an empty transformed plan.");
    return false;
  }
  ROS_DEBUG_NAMED("dwa_local_planner", "Received a transformed plan with %zu points.", transformed_plan.size());
  // .size() 返回其中包含的元素个数

  // update plan in dwa_planner even if we just stop and rotate, to allow checkTrajectory
  dp_->updatePlanAndLocalCosts(current_pose_, transformed_plan, costmap_ros_->getRobotFootprint());

  if (latchedStopRotateController_.isPositionReached(&planner_util_, current_pose_))
  {
    // publish an empty plan because we've reached our goal position
    std::vector<geometry_msgs::PoseStamped> local_plan;
    std::vector<geometry_msgs::PoseStamped> transformed_plan;
    publishGlobalPlan(transformed_plan);
    publishLocalPlan(local_plan);
    // 创建并发布空的全局路径和局部路径
    base_local_planner::LocalPlannerLimits limits = planner_util_.getCurrentLimits();
    return latchedStopRotateController_.computeVelocityCommandsStopRotate(
        cmd_vel, limits.getAccLimits(), dp_->getSimPeriod(), &planner_util_, odom_helper_, current_pose_,
        boost::bind(&DWA::checkTrajectory, dp_, _1, _2, _3));
        // 绑定 DWA（动态窗口法）规划器的 checkTrajectory 方法，用于验证生成的轨迹是否可行
        // 如果机器人位置正确但朝向不匹配目标朝向，会生成旋转命令调整朝向
  }
  else
  {
    bool isOk = dwaComputeVelocityCommands(current_pose_, cmd_vel);
    if (isOk)
    {
      publishGlobalPlan(transformed_plan);
    }
    else
    {
      ROS_WARN_NAMED("dwa_local_planner", "DWA planner failed to produce path.");
      std::vector<geometry_msgs::PoseStamped> empty_plan;
      publishGlobalPlan(empty_plan);
    }
    return isOk;
  }
}
// 根据机器人是否接近目标，决定使用 DWA 算法还是停止旋转控制。
// 如果接近目标（isPositionReached 为真），调用 latchedStopRotateController_ 生成旋转命令调整朝向。
// 否则，调用 dwaComputeVelocityCommands 执行 DWA 算法。

};  // namespace controller
}  // namespace rmp
