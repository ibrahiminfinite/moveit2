
#include <gtest/gtest.h>
#include <tf2_eigen/tf2_eigen.hpp>

#include <moveit_servo/servo.hpp>
#include <moveit_servo/datatypes.hpp>
#include <moveit_servo/utils.hpp>

class ServoCppFixture : public testing::Test
{
protected:
  void SetUp() override
  {
    // Create a node to be given to Servo.
    servo_test_node_ = std::make_shared<rclcpp::Node>("moveit_servo_test");
    // Create a Servo object for testing.
    const std::string servo_param_namespace = "moveit_servo_test";
    servo_param_listener_ = std::make_shared<servo::ParamListener>(servo_test_node_, servo_param_namespace);
    servo_test_instance_ = std::make_shared<moveit_servo::Servo>(servo_test_node_, servo_param_listener_);
  }

  void TearDown() override
  {
    servo_test_instance_.reset();
    servo_param_listener_.reset();
    servo_test_node_.reset();
    rclcpp::shutdown();
  }

  std::shared_ptr<rclcpp::Node> servo_test_node_;
  std::shared_ptr<const servo::ParamListener> servo_param_listener_;
  std::shared_ptr<moveit_servo::Servo> servo_test_instance_;
};
