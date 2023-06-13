
#include <limits>
#include <gtest/gtest.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <moveit/utils/robot_model_test_utils.h>

#include <moveit_servo/datatypes.hpp>
#include <moveit_servo/utils.hpp>

namespace
{
    class ServoCppUnitTests: public testing::Test
    {
        protected:

            void SetUp() override
            {
                robot_model_ = moveit::core::loadTestingRobotModel("panda");
                joint_model_group_ = robot_model_->getJointModelGroup("panda_arm");
            }

            moveit::core::RobotModelConstPtr robot_model_;
            const moveit::core::JointModelGroup* joint_model_group_;
    };
}

TEST_F(ServoCppUnitTests, validVector)
{
    Eigen::VectorXd valid_vector(7);
    valid_vector << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    EXPECT_TRUE(moveit_servo::isValidCommand(valid_vector));
}

TEST_F(ServoCppUnitTests, invalidVector)
{
    Eigen::VectorXd invalid_vector(6);
    invalid_vector << 0.0, 0.0, 0.0, 0.0, std::nan("");
    EXPECT_FALSE(moveit_servo::isValidCommand(invalid_vector));
}

TEST_F(ServoCppUnitTests, validTwist)
{
    Eigen::VectorXd valid_vector(6);
    valid_vector << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    moveit_servo::Twist valid_twist{"panda_link0", valid_vector};
    EXPECT_TRUE(moveit_servo::isValidCommand(valid_twist));
}

TEST_F(ServoCppUnitTests, emptyTwistFrame)
{
    Eigen::VectorXd invalid_vector(6);
    invalid_vector << 0.0, 0.0, 0.0, 0.0, std::nan("");
    moveit_servo::Twist invalid_twist;
    invalid_twist.velocities = invalid_vector;
    EXPECT_FALSE(moveit_servo::isValidCommand(invalid_twist));
}

TEST_F(ServoCppUnitTests, invalidTwistVelocities)
{
    Eigen::VectorXd invalid_vector(6);
    invalid_vector << 0.0, 0.0, 0.0, 0.0, 0.0, std::nan("");
    moveit_servo::Twist invalid_twist {"panda_link0", invalid_vector};
    EXPECT_FALSE(moveit_servo::isValidCommand(invalid_twist));
}

TEST_F(ServoCppUnitTests, validIsometry)
{
    Eigen::Isometry3d valid_isometry;
    valid_isometry.setIdentity();
    EXPECT_TRUE(moveit_servo::isValidCommand(valid_isometry));
}

TEST_F(ServoCppUnitTests, invalidIsometry)
{
    Eigen::Isometry3d invalid_isometry;
    invalid_isometry.setIdentity();
    invalid_isometry.translation().z() = std::nan("");
    EXPECT_FALSE(moveit_servo::isValidCommand(invalid_isometry));
}

TEST_F(ServoCppUnitTests, validPose)
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