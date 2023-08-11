// Copyright 2023 Dexory

#ifndef DEX_CONTROLLER__DEX_CONTROLLER_HPP_
#define DEX_CONTROLLER__DEX_CONTROLLER_HPP_

#include <string>
#include <memory>

#include "geometry_msgs/msg/pose2_d.hpp"
#include "nav2_core/controller.hpp"
#include "nav_msgs/msg/path.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"

#include <dex_controller/simple_pid.hpp>
#include <nav2_regulated_pure_pursuit_controller/regulated_pure_pursuit_controller.hpp>
#include <vector>
#include <Eigen/Dense>


namespace dex_controller
{

/**
 * @class dex_controller::DexController
 * @brief Dex controller plugin
 */
class DexController : public nav2_core::Controller
{
public:
  /**
   * @brief Constructor for dex_controller::DexController
   */
  DexController();

  /**
   * @brief Destructor for dex_controller::DexController
   */
  ~DexController() override = default;

  /**
    * @brief Configure controller on bringup
    * @param parent WeakPtr to node
    * @param name Name of plugin
    * @param tf TF buffer to use
    * @param costmap_ros Costmap2DROS object of environment
    */
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
    const std::shared_ptr<tf2_ros::Buffer> tf,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  /**
   * @brief Cleanup controller state machine
   */
  void cleanup() override;

  /**
   * @brief Activate controller state machine
   */
  void activate() override;

  /**
   * @brief Deactivate controller state machine
   */
  void deactivate() override;

  /**
   * @brief Compute the best command given the current pose and velocity,
   * with possible debug information
   *
   * Same as above computeVelocityCommands, but with debug results.
   * If the results pointer is not null, additional information about the twists
   * evaluated will be in results after the call.
   *
   * @param pose      Current robot pose
   * @param velocity  Current robot velocity
   * @param goal_checker  Ptr to the goal checker for this task in case useful in computing commands
   * @return          Best command
   */
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & speed,
    nav2_core::GoalChecker * goal_checker) override;

  /**
   * @brief nav2_core setPlan - Sets the global plan
   * @param path The global plan
   */
  void setPlan(const nav_msgs::msg::Path & path) override;

  /**
   * @brief Limits the maximum linear speed of the robot.
   * @param speed_limit expressed in absolute value (in m/s)
   * or in percentage from maximum robot speed.
   * @param percentage Setting speed limit in percentage if true
   * or in absolute values in false case.
   */
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;


protected:
  /**
   * @brief Compute the best command given the current pose and velocity
   *
   * @param pose      Current robot pose
   * @param velocity  Current robot velocity
   * @return          Best command
   */
  void setGlobalPlanOrientations(nav_msgs::msg::Path& global_plan);
  
  /**
    * @brief Aligns the robot with the trajectory
    * @param pose Current robot pose
    * @param speed Current robot speed
    * @return Best command
  */
  geometry_msgs::msg::Twist alignWithTrajectory(const geometry_msgs::msg::PoseStamped& pose, const geometry_msgs::msg::Twist& speed);

  /**
   * @brief Calculates the time difference between the current pose and the new pose
   * @param new_pose The new pose
   * @return The time difference
  */
  double calculateTimeDiff(const geometry_msgs::msg::PoseStamped& new_pose) noexcept;

  /**
   * @brief Performs principle component analysis on the given points to estimate the orientation of the initial trajectory section
   * @param points The points to perform PCA on
   * @return The estimated orientation in radians
  */
  double principleComponentAnalysis(const std::vector<Eigen::Vector3d>& points);

  /**
   * @brief Converts a PoseStamped to a Vector3d
   * @param pose The pose to convert
   * @return The converted pose
  */
  Eigen::Vector3d toPoint(const geometry_msgs::msg::PoseStamped& pose);

  rclcpp::Logger logger_{rclcpp::get_logger("DexController")};
  nav_msgs::msg::Path global_plan_;

  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::string name_;
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::string global_frame_;


  SimplePID<double> yaw_pid_;
  double yaw_error_threshold_;
  int pca_point_no_; // Number of points to use for PCA
  double previous_time_ = 0;
  bool pre_plan_turning_;
  double target_yaw_;

  std::unique_ptr<nav2_core::Controller> regulated_pp_;
};


}  // namespace dex_controller

#endif  // DEX_CONTROLLER__DEX_CONTROLLER_HPP_
