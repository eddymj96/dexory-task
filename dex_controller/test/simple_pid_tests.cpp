#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <dex_controller/simple_pid.hpp>


TEST(SimplePID, update_kp)
{
    dex_controller::SimplePID<double> pid;
    pid.setGains(1.0, 0.0, 0.0);
    pid.setSetpoint(0.5);
    const auto val = pid.update(0.3, 0.0, 0.1);

    EXPECT_DOUBLE_EQ(val, 0.2);
}

TEST(SimplePID, update_ki)
{
    dex_controller::SimplePID<double> pid;
    pid.setGains(0.0, 1.0, 0.0);
    pid.setSetpoint(0.5);
    pid.update(0.3, 0.0, 0.1);
    const auto val = pid.update(0.3, 0.0, 0.1);

    EXPECT_DOUBLE_EQ(val, 0.04);
}

TEST(SimplePID, update_kd)
{
    dex_controller::SimplePID<double> pid;
    pid.setGains(0.0, 0.0, 1.0);
    pid.setSetpoint(0.5);
    const auto val = pid.update(0.3, 0.0, 0.1);

    EXPECT_DOUBLE_EQ(val, 0.0);
}

TEST(SimplePID, reset)
{
    dex_controller::SimplePID<double> pid;
    pid.setGains(0.0, 1.0, 0.0);
    pid.setSetpoint(0.5);
    pid.update(0.3, 0.0, 0.1);
    pid.update(0.3, 0.0, 0.1);
    pid.reset();
    const auto val = pid.update(0.3, 0.0, 0.1);

    EXPECT_DOUBLE_EQ(val, 0.02);
}

TEST(SimplePID, wind_up_limit)
{
    dex_controller::SimplePID<double> pid;
    pid.setGains(0.0, 1.0, 0.0);
    pid.setSetpoint(10);
    pid.setWindUpLimit(500);
    const auto val = pid.update(5, 0.0, 10000);
    EXPECT_DOUBLE_EQ(val, 500);
}


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}