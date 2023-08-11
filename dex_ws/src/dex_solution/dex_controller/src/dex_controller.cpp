// Copyright 2023 Dexory

#include "dex_controller/dex_controller.hpp"

#include <nav2_util/node_utils.hpp>
#include <nav2_util/geometry_utils.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

using namespace nav2_costmap_2d;  //NOLINT

namespace dex_controller
{

DexController::DexController() : regulated_pp_(std::make_unique<nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController>())
{
    RCLCPP_INFO(
    logger_,
    "Consutructor");
}

void DexController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent, std::string name,
  const std::shared_ptr<tf2_ros::Buffer> tf,
  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{

  node_ = parent;
  tf_ = tf;
  costmap_ros_ = costmap_ros;
  name_ = name;
  global_frame_ = costmap_ros_->getGlobalFrameID();

    RCLCPP_INFO(
    logger_,
    "Reached");

  auto node = parent.lock();

      RCLCPP_INFO(
    logger_,
    "Reached 2");


  // Configure parameters
  double yaw_kp, yaw_ki, yaw_kd;

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".yaw_kp", rclcpp::ParameterValue(1.4));

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".yaw_ki", rclcpp::ParameterValue(0.001));
  
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".yaw_kd", rclcpp::ParameterValue(0.01));

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".yaw_error_threshold", rclcpp::ParameterValue(0.05));

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".pca_point_no", rclcpp::ParameterValue(5));

  node->get_parameter(name + ".yaw_kp", yaw_kp);
  node->get_parameter(name + ".yaw_ki", yaw_ki);
  node->get_parameter(name + ".yaw_kd", yaw_kd);

  node->get_parameter(name + ".yaw_error_threshold", yaw_error_threshold_);
  node->get_parameter(name + ".pca_point_no", pca_point_no_);

  yaw_pid_.setGains(yaw_kp, yaw_ki, yaw_kd);

  RCLCPP_INFO(
    logger_,
    "Reached 3");

      RCLCPP_INFO(
    logger_,
    "is null regulator_pp_? %d", regulated_pp_.get() == nullptr);

    std::string expired = parent.expired() ? "true" : "false";
    RCLCPP_INFO(
    logger_,
    expired.c_str());

  // Configure main tracking controller: regulated pure pursuit
  regulated_pp_->configure(parent, name_, tf_, costmap_ros_);

   RCLCPP_INFO(
    logger_,
    "Reached 4");
  
  RCLCPP_INFO(
    logger_,
    "Configured DexController");

}

void DexController::cleanup() 
{
  regulated_pp_->cleanup();
}

void DexController::activate() 
{
  regulated_pp_->activate();
}

void DexController::deactivate() 
{
  regulated_pp_->deactivate();
}

geometry_msgs::msg::TwistStamped DexController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped& pose, const geometry_msgs::msg::Twist& speed/*speed*/,
  nav2_core::GoalChecker* goal_checker)
{
  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header.stamp = node_.lock()->now();
  cmd_vel.header.frame_id = global_frame_;

  if (goal_checker != nullptr && goal_checker->isGoalReached(pose.pose, global_plan_.poses.back().pose, speed))
  {
    RCLCPP_INFO(
      logger_,
      "Goal reached!");
    return cmd_vel;
  }

  if(pre_plan_turning_)
  { 
    const auto twist_cmd = alignWithTrajectory(pose, speed);
    cmd_vel.twist = twist_cmd;
    return cmd_vel;
  }
  else
  {
    cmd_vel = regulated_pp_->computeVelocityCommands(pose, speed, goal_checker);
  }

  return cmd_vel;
}


void DexController::setGlobalPlanOrientations(nav_msgs::msg::Path& global_plan)
{
  for (auto it = global_plan.poses.begin(); it != global_plan.poses.end() - 1; ++it) 
  {
    auto& current_pose = *it;
    const auto& next_pose = *(it + 1);

    const auto current_yaw = std::atan2(next_pose.pose.position.y - current_pose.pose.position.y, next_pose.pose.position.x - current_pose.pose.position.x);

    current_pose.pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), current_yaw));
  }
}

geometry_msgs::msg::Twist DexController::alignWithTrajectory(const geometry_msgs::msg::PoseStamped& pose, const geometry_msgs::msg::Twist& speed)
{
  geometry_msgs::msg::Twist twist_cmd;

  const auto current_yaw = tf2::getYaw(pose.pose.orientation);

  double target_yaw;

  if (std::abs(current_yaw - target_yaw_) > M_PI)
  {
    target_yaw = (target_yaw_ > 0 ? target_yaw_ - 2*M_PI : target_yaw_ + 2*M_PI);
  }
  else
  {
    target_yaw = target_yaw_;
  }

  const auto current_yaw_dot = speed.angular.z;
  const auto dt = calculateTimeDiff(pose);

  yaw_pid_.setSetpoint(target_yaw); // Should ideally be in the setPlan method

  const auto theta_velocity_cmd = yaw_pid_.update(current_yaw, current_yaw_dot, dt, logger_);

  if (abs(target_yaw - current_yaw) < yaw_error_threshold_)
    pre_plan_turning_ = false;

  twist_cmd.linear.x = 0;
  twist_cmd.linear.y = 0;
  twist_cmd.linear.z = 0;
  twist_cmd.angular.z = theta_velocity_cmd;

  return twist_cmd;
}



double DexController::calculateTimeDiff(const geometry_msgs::msg::PoseStamped& new_pose) noexcept
{
  // Could be set also as a ros2 param and the dt is just assumed to be correct from that
  // Could run into issue if user provided dt is incorrect though
  const auto new_time = new_pose.header.stamp.sec + new_pose.header.stamp.nanosec*1e-9;
  const double dt = (previous_time_ == 0 ? 0 : new_time - previous_time_);
  previous_time_ = new_time;
  return dt;
}


double DexController::principleComponentAnalysis(const std::vector<Eigen::Vector3d>& points)
{
  Eigen::MatrixXd data(points.size(), 3);
  Eigen::Vector3d mean = {0, 0, 0};

  for (size_t i = 0; i < points.size(); ++i) 
  {
      data.row(i) = points[i];
      mean += points[i];
  }
  mean /= points.size();

  // Center the data
  for (size_t i = 0; i < points.size(); ++i) {
      data -= mean;
  }

  // Compute the covariance matrix
  Eigen::MatrixXd covariance = data.transpose() * data / points.size();

  // Compute eigenvectors and eigenvalues
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> solver(covariance);

  int maxIndex;
  solver.eigenvalues().real().maxCoeff(&maxIndex);
  Eigen::Vector3d direction = solver.eigenvectors().real().col(maxIndex).normalized();

  // Make sure the direction is pointing in the same direction as vector from first to last
  Eigen::Vector3d first_to_last {points.back()[0] - points[0][0], points.back()[1] - points[0][1], 0};
  if (direction.dot(first_to_last) < 0)
  {
    direction *= -1;
  }

  RCLCPP_INFO(
  logger_,
  "Direction: %f, %f", direction(0), direction(1));

  double theta = std::atan2(direction.y(), direction.x());

  return theta;
}


void DexController::setPlan(const nav_msgs::msg::Path& path) 
{
  RCLCPP_INFO(
  logger_,
  "New plan set");

  global_plan_ = path;

  setGlobalPlanOrientations(global_plan_);

  pre_plan_turning_ = true;
  std::vector<Eigen::Vector3d> points;

  const size_t max_val = path.poses.size() > 5 ? 5 : path.poses.size();

  for (size_t i = 0; i<max_val; i++)
  {
    points.push_back(toPoint(path.poses[i]));
  }

  target_yaw_ = principleComponentAnalysis(points); // Could also just use orientation of first pose, but this is slightly more interesting 


  yaw_pid_.reset();
  previous_time_ = 0;
  regulated_pp_->setPlan(global_plan_);
}

Eigen::Vector3d DexController::toPoint(const geometry_msgs::msg::PoseStamped& pose)
{
  Eigen::Vector3d point;
  tf2::fromMsg(pose.pose.position, point);
  return point;
}

void DexController::setSpeedLimit(const double & speed_limit, const bool & percentage) 
{
  RCLCPP_INFO(
  logger_,
  "New speed limit set");
  regulated_pp_->setSpeedLimit(speed_limit, percentage);
}

}  // namespace dex_controller

// Register this controller as a nav2_core plugin
PLUGINLIB_EXPORT_CLASS(dex_controller::DexController, nav2_core::Controller)