

#include <gtest/gtest.h>
#include <tf2_eigen/tf2_eigen.hpp>

#include <moveit_servo/datatypes.hpp>
#include <moveit_servo/utils.hpp>


TEST(ServoUtilsUnitTests, validVector)
{
    Eigen::VectorXd valid_vector(7);
    valid_vector << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    EXPECT_TRUE(moveit_servo::isValidCommand(valid_vector));
}

TEST(ServoUtilsUnitTests, invalidVector)
{
    Eigen::VectorXd invalid_vector(6);
    invalid_vector << 0.0, 0.0, 0.0, 0.0, std::nan("");
    EXPECT_FALSE(moveit_servo::isValidCommand(invalid_vector));
}

TEST(ServoUtilsUnitTests, validTwist)
{
    Eigen::VectorXd valid_vector(6);
    valid_vector << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    moveit_servo::Twist valid_twist{"panda_link0", valid_vector};
    EXPECT_TRUE(moveit_servo::isValidCommand(valid_twist));
}

TEST(ServoUtilsUnitTests, emptyTwistFrame)
{
    Eigen::VectorXd invalid_vector(6);
    invalid_vector << 0.0, 0.0, 0.0, 0.0, std::nan("");
    moveit_servo::Twist invalid_twist;
    invalid_twist.velocities = invalid_vector;
    EXPECT_FALSE(moveit_servo::isValidCommand(invalid_twist));
}

TEST(ServoUtilsUnitTests, invalidTwistVelocities)
{
    Eigen::VectorXd invalid_vector(6);
    invalid_vector << 0.0, 0.0, 0.0, 0.0, 0.0, std::nan("");
    moveit_servo::Twist invalid_twist {"panda_link0", invalid_vector};
    EXPECT_FALSE(moveit_servo::isValidCommand(invalid_twist));
}

TEST(ServoUtilsUnitTests, validIsometry)
{
    Eigen::Isometry3d valid_isometry;
    valid_isometry.setIdentity();
    EXPECT_TRUE(moveit_servo::isValidCommand(valid_isometry));
}

TEST(ServoUtilsUnitTests, invalidIsometry)
{
    Eigen::Isometry3d invalid_isometry;
    invalid_isometry.setIdentity();
    invalid_isometry.translation().z() = std::nan("");
    EXPECT_FALSE(moveit_servo::isValidCommand(invalid_isometry));
}

TEST(ServoUtilsUnitTests, validPose)
{ 
    Eigen::Isometry3d valid_isometry;
    valid_isometry.setIdentity();
    moveit_servo::Pose valid_pose{"panda_link0", valid_isometry};
    EXPECT_TRUE(moveit_servo::isValidCommand(valid_pose));
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    ::testing::InitGoogleTest(&argc, argv);
    int result = RUN_ALL_TESTS();
    rclcpp::shutdown();
    return result;
}