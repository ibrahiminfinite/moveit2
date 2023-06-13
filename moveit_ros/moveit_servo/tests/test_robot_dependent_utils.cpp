
#include <gtest/gtest.h>
#include <tf2_eigen/tf2_eigen.hpp>

#include "servo_cpp_fixture.hpp"
#include <moveit_servo/utils.hpp>

namespace
{

    TEST_F(ServoCppFixture, testGetEndEffectorFrame)
    {

        Eigen::Isometry3d ee_pose = servo_test_instance_->getEndEffectorPose();
        EXPECT_TRUE(moveit_servo::isValidCommand(ee_pose));

    }

}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    ::testing::InitGoogleTest(&argc, argv);
    int result = RUN_ALL_TESTS();
    rclcpp::shutdown();
    return result;
}
