
#include <base_local_planner/goal_functions.h>
#include <cmath>

// for computing path distance
#include <queue>

#include <angles/angles.h>

#include <ros/ros.h>
#include <tf2/utils.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include "controller/dwa.h"

namespace rmp
{
namespace controller
{
void DWA::reconfigure(dwa_controller::DWAControllerConfig& config)
{
  boost::mutex::scoped_lock l(configuration_mutex_);
  // 确保 DWA::reconfigure 函数的执行是线程安全的。在多线程环境中，多个线程可能同时尝试调用 reconfigure 修改配置参数
  // 如果没有锁保护，可能会导致数据不一致

  generator_.setParameters(config.sim_time, config.sim_granularity, config.angular_sim_granularity, config.use_dwa,
                           sim_period_);

  double resolution = planner_util_->getCostmap()->getResolution();
  path_distance_bias_ = resolution * config.path_distance_bias;
  // 机器人轨迹与全局路径（global plan）的接近程度
  // pdistscale used for both path and alignment, set  forward_point_distance to zero to discard alignment
  path_costs_.setScale(path_distance_bias_);
  alignment_costs_.setScale(path_distance_bias_);

  goal_distance_bias_ = resolution * config.goal_distance_bias;
  // 机器人轨迹与局部目标点（local goal，通常是全局路径的子目标）的接近程度
  goal_costs_.setScale(goal_distance_bias_);
  goal_front_costs_.setScale(goal_distance_bias_);

  occdist_scale_ = config.occdist_scale;
  // 障碍物距离缩放因子（Obstacle Distance Scale）
  obstacle_costs_.setScale(occdist_scale_);

  stop_time_buffer_ = config.stop_time_buffer;
  // config 是 dwa_controller::DWAControllerConfig 结构体的一个实例，stop_time_buffer 是其中的一个字段，类型为 double
  // 在DWAController.cfg中定义了 stop_time_buffer 的默认值为 0.2
  // 这个参数用于设置在计算机器人轨迹时，考虑的时间缓冲区大小

  oscillation_costs_.setOscillationResetDist(config.oscillation_reset_dist, config.oscillation_reset_angle);
  forward_point_distance_ = config.forward_point_distance;
  goal_front_costs_.setXShift(forward_point_distance_);
  // 调用 goal_front_costs_ 对象的 setXShift方法
  alignment_costs_.setXShift(forward_point_distance_);

  // obstacle costs can vary due to scaling footprint feature
  obstacle_costs_.setParams(config.max_vel_trans, config.max_scaling_factor, config.scaling_speed);

  twirling_costs_.setScale(config.twirling_scale);

  int vx_samp, vy_samp, vth_samp;
  vx_samp = config.vx_samples;
  vy_samp = config.vy_samples;
  vth_samp = config.vth_samples;

  if (vx_samp <= 0)
  {
    ROS_WARN(
        "You've specified that you don't want any samples in the x dimension. We'll at least assume that you want to "
        "sample one value... so we're going to set vx_samples to 1 instead");
    vx_samp = 1;
    config.vx_samples = vx_samp;
  }

  if (vy_samp <= 0)
  {
    ROS_WARN(
        "You've specified that you don't want any samples in the y dimension. We'll at least assume that you want to "
        "sample one value... so we're going to set vy_samples to 1 instead");
    vy_samp = 1;
    config.vy_samples = vy_samp;
  }

  if (vth_samp <= 0)
  {
    ROS_WARN(
        "You've specified that you don't want any samples in the th dimension. We'll at least assume that you want to "
        "sample one value... so we're going to set vth_samples to 1 instead");
    vth_samp = 1;
    config.vth_samples = vth_samp;
  }

  vsamples_[0] = vx_samp;
  vsamples_[1] = vy_samp;
  vsamples_[2] = vth_samp;
}

DWA::DWA(std::string name, base_local_planner::LocalPlannerUtil* planner_util)
  // 初始化成员变量，调用MapGridCostFunction构造函数的参数
  : planner_util_(planner_util)
  , obstacle_costs_(planner_util->getCostmap())
  // 这个指针允许访问代价地图的数据
  , path_costs_(planner_util->getCostmap())
  , goal_costs_(planner_util->getCostmap(), 0.0, 0.0, true)
  , goal_front_costs_(planner_util->getCostmap(), 0.0, 0.0, true)
  , alignment_costs_(planner_util->getCostmap())
{
  ros::NodeHandle private_nh("~/" + name);

  goal_front_costs_.setStopOnFailure(false);
  alignment_costs_.setStopOnFailure(false);

  // Assuming this planner is being run within the navigation stack, we can
  // just do an upward search for the frequency at which its being run. This
  // also allows the frequency to be overwritten locally.
  std::string controller_frequency_param_name;
  if (!private_nh.searchParam("controller_frequency", controller_frequency_param_name))
  // searchParam用于在参数服务器中搜索指定参数（controller_frequency）并返回其完整名称
  // controller_frequency_param_name用于储存参数的完成路径
  {
    sim_period_ = 0.05; //如果没有配置参数，规划器默认周期为0.05秒
  }
  else
  {
    double controller_frequency = 0; //声明一个变量以便读取参数
    private_nh.param(controller_frequency_param_name, controller_frequency, 20.0);
    // param 是一个函数，ROS（Robot Operating System）中 ros::NodeHandle 类提供的一个模板方法，用于从参数服务器中读取参数值。
    if (controller_frequency > 0)
    {
      sim_period_ = 1.0 / controller_frequency;
    }
    else
    {
      ROS_WARN("A controller_frequency less than 0 has been set. Ignoring the parameter, assuming a rate of 20Hz");
      sim_period_ = 0.05;
    }
  }
  ROS_INFO("Sim period is set to %.2f", sim_period_);

  oscillation_costs_.resetOscillationFlags();

  bool sum_scores;
  private_nh.param("sum_scores", sum_scores, false);
  // private_nh 是一个私有的 ROS 节点句柄，通常用于读取特定于该节点的参数（例如，~sum_scores）
  obstacle_costs_.setSumScores(sum_scores);
  // 如果 sum_scores 为 true，障碍物成本函数在评分轨迹时会累加轨迹上所有点的障碍物成本
  // 如果 sum_scores 为 false，障碍物成本函数通常只考虑轨迹中最大的障碍物成本

  private_nh.param("publish_cost_grid_pc", publish_cost_grid_pc_, false);
  map_viz_.initialize(name, planner_util->getGlobalFrame(),
                      boost::bind(&DWA::getCellCosts, this, _1, _2, _3, _4, _5, _6));
                      // 使用 boost::bind 将 DWA 类的 getCellCosts 方法绑定为回调函数，供 MapGridVisualizer 调用
                      // this: 绑定当前 DWA 对象的实例，确保 getCellCosts 方法能在正确的对象上下文调用
                      // _1, _2, _3, _4, _5, _6: 占位符，表示 getCellCosts 方法的六个参数
  // 这段代码初始化了一个代价网格可视化工具 map_viz_，用于在 RViz 或其他可视化工具中显示局部规划器的代价分布

  private_nh.param("global_frame_id", frame_id_, std::string("odom"));

  traj_cloud_pub_ = private_nh.advertise<sensor_msgs::PointCloud2>("trajectory_cloud", 1);
  // 发布器 发布类型 主题名称 队列大小
  private_nh.param("publish_traj_pc", publish_traj_pc_, false);

  // set up all the cost functions that will be applied in order
  // (any function returning negative values will abort scoring, so the order can improve performance)
  std::vector<base_local_planner::TrajectoryCostFunction*> critics;
  // 评分函数列表
  critics.push_back(&oscillation_costs_);  // discards oscillating motions (assisgns cost -1)
  critics.push_back(&obstacle_costs_);     // discards trajectories that move into obstacles
  critics.push_back(&goal_front_costs_);   // prefers trajectories that make the nose go towards (local) nose goal
  critics.push_back(&alignment_costs_);    // prefers trajectories that keep the robot nose on nose path
  critics.push_back(&path_costs_);         // prefers trajectories on global path
  critics.push_back(&goal_costs_);      // prefers trajectories that go towards (local) goal, based on wave propagation
  critics.push_back(&twirling_costs_);  // optionally prefer trajectories that don't spin

  // trajectory generatorsSimpleScoredSamplingPlanner
  std::vector<base_local_planner::TrajectorySampleGenerator*> generator_list;
  // TrajectorySampleGenerator 是 ROS 导航栈 base_local_planner 包中的一个抽象基类
  // 负责根据速度样本（vx_samples、vy_samples、vth_samples）生成机器人可能的轨迹
  generator_list.push_back(&generator_);

  scored_sampling_planner_ = base_local_planner::(generator_list, critics);

  private_nh.param("cheat_factor", cheat_factor_, 1.0);
}

// used for visualization only, total_costs are not really total costs
bool DWA::getCellCosts(int cx, int cy, float& path_cost, float& goal_cost, float& occ_cost, float& total_cost)
{
  path_cost = path_costs_.getCellCosts(cx, cy);
  goal_cost = goal_costs_.getCellCosts(cx, cy);
  occ_cost = planner_util_->getCostmap()->getCost(cx, cy); // 返回的值是基于该网格单元与障碍物的距离计算的成本
  if (path_cost == path_costs_.obstacleCosts() || path_cost == path_costs_.unreachableCellCosts() ||
      occ_cost >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
      // costmap_2d::INSCRIBED_INFLATED_OBSTACLE（值为 253）
  {
    return false;
  }

  total_cost = path_distance_bias_ * path_cost + goal_distance_bias_ * goal_cost + occdist_scale_ * occ_cost;
  return true;
}

bool DWA::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan)
{
  oscillation_costs_.resetOscillationFlags(); 
  // 清除振荡检测的内部标志，使振荡成本函数恢复到初始状态，不受之前的运动历史影响
  return planner_util_->setPlan(orig_global_plan);
  // setPlan 函数是 DWA 局部规划器与全局规划器交互的关键接口，用于接收和设置新的全局路径
}

/**
 * This function is used when other strategies are to be applied,
 * but the cost functions for obstacles are to be reused.
 */
bool DWA::checkTrajectory(Eigen::Vector3f pos, Eigen::Vector3f vel, Eigen::Vector3f vel_samples)
{
  oscillation_costs_.resetOscillationFlags();
  // 重置振荡检测的标志
  base_local_planner::Trajectory traj;
  // 定义一个空的 Trajectory 对象 traj，用于存储生成的轨迹
  geometry_msgs::PoseStamped goal_pose = global_plan_.back();
  Eigen::Vector3f goal(goal_pose.pose.position.x, goal_pose.pose.position.y, tf2::getYaw(goal_pose.pose.orientation));
  // goal是一个向量
  base_local_planner::LocalPlannerLimits limits = planner_util_->getCurrentLimits();
  generator_.initialise(pos, vel, goal, &limits, vsamples_);
  generator_.generateTrajectory(pos, vel, vel_samples, traj);
  double cost = scored_sampling_planner_.scoreTrajectory(traj, -1);
  // traj：表示一条候选轨迹，包含机器人在未来一段时间内的位置、速度和方向信息
  // -1：这是一个索引参数，表示不对轨迹进行特定的采样点限制（即对整个轨迹进行评分）
  // if the trajectory is a legal one... the check passes
  if (cost >= 0)
  {
    return true;
  }
  ROS_WARN("Invalid Trajectory %f, %f, %f, cost: %f", vel_samples[0], vel_samples[1], vel_samples[2], cost);

  // otherwise the check fails
  return false;
}

void DWA::updatePlanAndLocalCosts(const geometry_msgs::PoseStamped& global_pose,
                                  const std::vector<geometry_msgs::PoseStamped>& new_plan,
                                  const std::vector<geometry_msgs::Point>& footprint_spec)
{
  global_plan_.resize(new_plan.size());
  for (unsigned int i = 0; i < new_plan.size(); ++i)
  {
    global_plan_[i] = new_plan[i];
  }

  obstacle_costs_.setFootprint(footprint_spec);

  // costs for going away from path
  path_costs_.setTargetPoses(global_plan_);

  // costs for not going towards the local goal as much as possible
  goal_costs_.setTargetPoses(global_plan_);

  // alignment costs
  geometry_msgs::PoseStamped goal_pose = global_plan_.back();

  Eigen::Vector3f pos(global_pose.pose.position.x, global_pose.pose.position.y,
                      tf2::getYaw(global_pose.pose.orientation));
  double sq_dist = (pos[0] - goal_pose.pose.position.x) * (pos[0] - goal_pose.pose.position.x) +
                   (pos[1] - goal_pose.pose.position.y) * (pos[1] - goal_pose.pose.position.y);

  // we want the robot nose to be drawn to its final position
  // (before robot turns towards goal orientation), not the end of the
  // path for the robot center. Choosing the final position after
  // turning towards goal orientation causes instability when the
  // robot needs to make a 180 degree turn at the end
  std::vector<geometry_msgs::PoseStamped> front_global_plan = global_plan_;
  // 通过复制 global_plan_ 初始化
  double angle_to_goal = atan2(goal_pose.pose.position.y - pos[1], goal_pose.pose.position.x - pos[0]);
  front_global_plan.back().pose.position.x =
      front_global_plan.back().pose.position.x + forward_point_distance_ * cos(angle_to_goal);
  front_global_plan.back().pose.position.y =
      front_global_plan.back().pose.position.y + forward_point_distance_ * sin(angle_to_goal);

  goal_front_costs_.setTargetPoses(front_global_plan);

  // keeping the nose on the path
  if (sq_dist > forward_point_distance_ * forward_point_distance_ * cheat_factor_)
  {
    alignment_costs_.setScale(path_distance_bias_);
    // costs for robot being aligned with path (nose on path, not ju
    alignment_costs_.setTargetPoses(global_plan_);
  }
  else
  {
    // once we are close to goal, trying to keep the nose close to anything destabilizes behavior.
    alignment_costs_.setScale(0.0);
  }
}

/*
 * given the current state of the robot, find a good trajectory
 */
base_local_planner::Trajectory DWA::findBestPath(const geometry_msgs::PoseStamped& global_pose,
                                                 const geometry_msgs::PoseStamped& global_vel,
                                                 geometry_msgs::PoseStamped& drive_velocities)
{
  // make sure that our configuration doesn't change mid-run
  boost::mutex::scoped_lock l(configuration_mutex_);

  Eigen::Vector3f pos(global_pose.pose.position.x, global_pose.pose.position.y,
                      tf2::getYaw(global_pose.pose.orientation));
  Eigen::Vector3f vel(global_vel.pose.position.x, global_vel.pose.position.y, tf2::getYaw(global_vel.pose.orientation));
  geometry_msgs::PoseStamped goal_pose = global_plan_.back();
  Eigen::Vector3f goal(goal_pose.pose.position.x, goal_pose.pose.position.y, tf2::getYaw(goal_pose.pose.orientation));
  base_local_planner::LocalPlannerLimits limits = planner_util_->getCurrentLimits();

  // prepare cost functions and generators for this run
  generator_.initialise(pos, vel, goal, &limits, vsamples_);

  result_traj_.cost_ = -7;
  // 任意负值，表示无效轨迹
  // find best trajectory by sampling and scoring the samples
  std::vector<base_local_planner::Trajectory> all_explored;
  scored_sampling_planner_.findBestTrajectory(result_traj_, &all_explored);

  if (publish_traj_pc_)
  {
    sensor_msgs::PointCloud2 traj_cloud;
    traj_cloud.header.frame_id = frame_id_;
    traj_cloud.header.stamp = ros::Time::now();

    sensor_msgs::PointCloud2Modifier cloud_mod(traj_cloud);
    // 点云修改器
    cloud_mod.setPointCloud2Fields(5, "x", 1, sensor_msgs::PointField::FLOAT32, "y", 1,
                                   sensor_msgs::PointField::FLOAT32, "z", 1, sensor_msgs::PointField::FLOAT32, "theta",
                                   1, sensor_msgs::PointField::FLOAT32, "cost", 1, sensor_msgs::PointField::FLOAT32);
    // 调用 setPointCloud2Fields 方法为点云定义了 5 个字段（x, y, z, theta, cost）
    unsigned int num_points = 0;
    for (std::vector<base_local_planner::Trajectory>::iterator t = all_explored.begin(); t != all_explored.end(); ++t)
    {
      if (t->cost_ < 0)
        continue;
      num_points += t->getPointsSize();
      // 如果轨迹有效（cost_ >= 0），调用 t->getPointsSize() 获取该轨迹包含的点数，并将其累加到 num_points 中
    }

    cloud_mod.resize(num_points);
    sensor_msgs::PointCloud2Iterator<float> iter_x(traj_cloud, "x");
    // 点云迭代器 iter_x，"x" 表示要访问的字段名称
    for (std::vector<base_local_planner::Trajectory>::iterator t = all_explored.begin(); t != all_explored.end(); ++t)
    {
      if (t->cost_ < 0)
        continue;
      // Fill out the plan
      for (unsigned int i = 0; i < t->getPointsSize(); ++i)
      {
        double p_x, p_y, p_th;
        // 获取轨迹点的坐标和角度
        t->getPoint(i, p_x, p_y, p_th);
        iter_x[0] = p_x;
        iter_x[1] = p_y;
        iter_x[2] = 0.0;
        iter_x[3] = p_th;
        iter_x[4] = t->cost_;
        ++iter_x;
      }
    }
    traj_cloud_pub_.publish(traj_cloud);
  }

  // verbose publishing of point clouds
  if (publish_cost_grid_pc_)
  {
    // we'll publish the visualization of the costs to rviz before returning our best trajectory
    map_viz_.publishCostCloud(planner_util_->getCostmap());
  }

  // debrief stateful scoring functions
  oscillation_costs_.updateOscillationFlags(pos, &result_traj_, planner_util_->getCurrentLimits().min_vel_trans);
  // oscillation_costs_是一个类的对象，更新的值在这个类中

  // if we don't have a legal trajectory, we'll just command zero
  if (result_traj_.cost_ < 0)
  {
    drive_velocities.pose.position.x = 0;
    drive_velocities.pose.position.y = 0;
    drive_velocities.pose.position.z = 0;
    drive_velocities.pose.orientation.w = 1;
    drive_velocities.pose.orientation.x = 0;
    drive_velocities.pose.orientation.y = 0;
    drive_velocities.pose.orientation.z = 0;
  }
  // w 是四元数的标量部分（实部），通常与旋转的角度有关
  // x, y, z 是矢量部分（虚部），表示旋转轴的方向
  else
  {
    drive_velocities.pose.position.x = result_traj_.xv_;
    drive_velocities.pose.position.y = result_traj_.yv_;
    drive_velocities.pose.position.z = 0;
    tf2::Quaternion q;
    q.setRPY(0, 0, result_traj_.thetav_);
    // 这里不应该输入角度吗，thetav是角速度吧？？？
    tf2::convert(q, drive_velocities.pose.orientation);
    // 将一个 tf2::Quaternion 类型的对象 q 转换为 drive_velocities.pose.orientation 所需的类型，通常是 ROS 中用于表示姿态的类型
  }

  return result_traj_;
}
};  // namespace controller
}  // namespace rmp