#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <dex_controller/dex_controller.hpp>

namespace dex_controller
{

class DexoryControllerTest : public DexController
{
    public:

        DexoryControllerTest() : DexController() {};

        void setGlobalPlanOrientations(nav_msgs::msg::Path& global_plan) 
        {
            return setGlobalPlanOrientations(global_plan);
        }

        nav_msgs::msg::Path getGlobalPlan() const noexcept
        {
            return getGlobalPlan();
        }

        geometry_msgs::msg::Twist alignWithTrajectory(const geometry_msgs::msg::PoseStamped& pose, const geometry_msgs::msg::Twist& speed)
        {
            return alignWithTrajectory(pose, speed);
        }

        bool isPrePlanTurning() const noexcept
        {
            return pre_plan_turning_;
        }

        double calculateTimeDiff(const geometry_msgs::msg::PoseStamped& new_pose) noexcept
        {
            return calculateTimeDiff(new_pose);
        }

        double principleComponentAnalysis(const std::vector<Eigen::Vector3d>& points)
        {
            return principleComponentAnalysis(points);
        }
};


} // namespace dex_controller

TEST(setGlobalPlanOrientations, orientation_1)
{
    auto ctrl = std::make_shared<dex_controller::DexoryControllerTest>();
    auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("DexControllerTest");

    std::string name = "PathFollower";
    auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>("fake_costmap");
    ctrl->configure(node, name, tf, costmap);

    nav_msgs::msg::Path global_plan = nav_msgs::msg::Path();

    auto first_waypoint = geometry_msgs::msg::PoseStamped();
    first_waypoint.pose.position.x = 1;
    first_waypoint.pose.position.y = 1;

    auto second_waypoint = geometry_msgs::msg::PoseStamped();
    second_waypoint.pose.position.x = 1;
    second_waypoint.pose.position.y = 2;


    global_plan.poses.push_back(first_waypoint);
    global_plan.poses.push_back(second_waypoint);

    ctrl->setGlobalPlanOrientations(global_plan);
    EXPECT_DOUBLE_EQ(tf2::getYaw(ctrl->getGlobalPlan().poses[0].pose.orientation), 0.0);

}

TEST(alignWithTrajectory, StillTurning)
{
        
    dex_controller::DexoryControllerTest dexory_controller_test;

    auto ctrl = std::make_shared<dex_controller::DexoryControllerTest>();
    auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("DexControllerTest");

    std::string name = "DexControllerTest";
    auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>("fake_costmap");
    std::cout << "Test 1" << std::endl;
    ctrl->configure(node, name, tf, costmap);
    std::cout << "Test 2" << std::endl;

    ctrl->activate();

    

    nav_msgs::msg::Path global_plan = nav_msgs::msg::Path();


    auto first_waypoint = geometry_msgs::msg::PoseStamped();
    first_waypoint.pose.position.x = 1;
    first_waypoint.pose.position.y = 1;

    auto second_waypoint = geometry_msgs::msg::PoseStamped();
    second_waypoint.pose.position.x = 1;
    second_waypoint.pose.position.y = 2;

    global_plan.poses.push_back(first_waypoint);
    global_plan.poses.push_back(second_waypoint);

    
    ctrl->setPlan(global_plan);

    auto current_pose = geometry_msgs::msg::PoseStamped();
    current_pose.pose.position.x = 0;
    current_pose.pose.position.y = 0;

    tf2::Quaternion q;
    q.setRPY(0, 0, 0.01); 
    current_pose.pose.orientation = tf2::toMsg(q);

    geometry_msgs::msg::Twist current_twist;

    ctrl->alignWithTrajectory(current_pose, current_twist);
    EXPECT_EQ(ctrl->isPrePlanTurning(), true);
}

TEST(alignWithTrajectory, StopTurning)
{
    auto ctrl = std::make_shared<dex_controller::DexoryControllerTest>();
    auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("DexControllerTest");

    std::string name = "PathFollower";
    auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>("fake_costmap");
    ctrl->configure(node, name, tf, costmap);
    nav_msgs::msg::Path global_plan = nav_msgs::msg::Path();

    auto first_waypoint = geometry_msgs::msg::PoseStamped();
    first_waypoint.pose.position.x = 1;
    first_waypoint.pose.position.y = 1;

    auto second_waypoint = geometry_msgs::msg::PoseStamped();
    second_waypoint.pose.position.x = 1;
    second_waypoint.pose.position.y = 2;


    global_plan.poses.push_back(first_waypoint);
    global_plan.poses.push_back(second_waypoint);

    ctrl->setGlobalPlanOrientations(global_plan);
    

    auto current_pose = geometry_msgs::msg::PoseStamped();
    current_pose.pose.position.x = 0;
    current_pose.pose.position.y = 0;

    tf2::Quaternion q;
    q.setRPY(0, 0, 0.5); 
    current_pose.pose.orientation = tf2::toMsg(q);

    geometry_msgs::msg::Twist current_twist;

    ctrl->alignWithTrajectory(current_pose, current_twist);
    EXPECT_EQ(ctrl->isPrePlanTurning(), false);
}

TEST(calculateTimeDiff, DefaultInitialTime)
{
    auto ctrl = std::make_shared<dex_controller::DexoryControllerTest>();
    auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("DexControllerTest");

    std::string name = "PathFollower";
    auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>("fake_costmap");
    ctrl->configure(node, name, tf, costmap);

    geometry_msgs::msg::PoseStamped new_pose = geometry_msgs::msg::PoseStamped();
    new_pose.header.stamp = rclcpp::Time(5);
    const auto dt = ctrl->calculateTimeDiff(new_pose);
    EXPECT_DOUBLE_EQ(dt, 5.0);
}


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}