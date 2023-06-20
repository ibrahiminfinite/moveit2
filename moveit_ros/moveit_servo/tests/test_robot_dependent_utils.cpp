/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, PickNik Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of PickNik Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author    : V Mohammed Ibrahim
   Desc      : Tests for utilities that depend on the robot/ robot state.
   Title     : test_robot_dependent_utils.cpp
   Project   : moveit_servo
   Created   : 06/20/2023
*/

#include <gtest/gtest.h>
#include <tf2_eigen/tf2_eigen.hpp>

#include "servo_cpp_fixture.hpp"
#include <moveit_servo/utils.hpp>

namespace
{
TEST_F(ServoCppFixture, testLeavingSingularity)
{
  moveit::core::RobotStatePtr robot_state = planning_scene_monitor_->getStateMonitor()->getCurrentState();
  auto joint_model_group = robot_state->getJointModelGroup(servo_param_listener_->get_params().move_group_name);
  auto servo_params = servo_param_listener_->get_params();
  Eigen::VectorXd cartesian_delta(6);
  cartesian_delta << 0.005, 0.0, 0.0, 0.0, 0.0, 0.0;

  robot_state->setToDefaultValues();
  robot_state->setVariablePosition("panda_joint1", 0.0);
  robot_state->setVariablePosition("panda_joint2", 0.334);
  robot_state->setVariablePosition("panda_joint3", 0.0);
  robot_state->setVariablePosition("panda_joint4", -1.177);
  robot_state->setVariablePosition("panda_joint5", 0.0);
  robot_state->setVariablePosition("panda_joint6", 1.510);
  robot_state->setVariablePosition("panda_joint7", 0.785);

  auto scaling_result =
      moveit_servo::velocityScalingFactorForSingularity(joint_model_group, robot_state, cartesian_delta, servo_params);
  ASSERT_EQ(scaling_result.second, moveit_servo::StatusCode::DECELERATE_FOR_APPROACHING_SINGULARITY);

  cartesian_delta(0) *= -1;
  robot_state->setVariablePosition("panda_joint1", 0.0);
  robot_state->setVariablePosition("panda_joint2", 0.3458);
  robot_state->setVariablePosition("panda_joint3", 0.0);
  robot_state->setVariablePosition("panda_joint4", -1.1424);
  robot_state->setVariablePosition("panda_joint5", 0.0);
  robot_state->setVariablePosition("panda_joint6", 1.4865);
  robot_state->setVariablePosition("panda_joint7", 0.785);

  scaling_result =
      moveit_servo::velocityScalingFactorForSingularity(joint_model_group, robot_state, cartesian_delta, servo_params);
  ASSERT_EQ(scaling_result.second, moveit_servo::StatusCode::DECELERATE_FOR_LEAVING_SINGULARITY);
}

TEST_F(ServoCppFixture, testApproachingSingularity)
{
  moveit::core::RobotStatePtr robot_state = planning_scene_monitor_->getStateMonitor()->getCurrentState();
  auto joint_model_group = robot_state->getJointModelGroup(servo_param_listener_->get_params().move_group_name);
  auto servo_params = servo_param_listener_->get_params();
  Eigen::VectorXd cartesian_delta(6);
  cartesian_delta << 0.005, 0.0, 0.0, 0.0, 0.0, 0.0;

  auto scaling_result =
      moveit_servo::velocityScalingFactorForSingularity(joint_model_group, robot_state, cartesian_delta, servo_params);
  ASSERT_EQ(scaling_result.second, moveit_servo::StatusCode::NO_WARNING);

  robot_state->setToDefaultValues();
  robot_state->setVariablePosition("panda_joint1", 0.0);
  robot_state->setVariablePosition("panda_joint2", 0.334);
  robot_state->setVariablePosition("panda_joint3", 0.0);
  robot_state->setVariablePosition("panda_joint4", -1.177);
  robot_state->setVariablePosition("panda_joint5", 0.0);
  robot_state->setVariablePosition("panda_joint6", 1.510);
  robot_state->setVariablePosition("panda_joint7", 0.785);

  scaling_result =
      moveit_servo::velocityScalingFactorForSingularity(joint_model_group, robot_state, cartesian_delta, servo_params);
  ASSERT_EQ(scaling_result.second, moveit_servo::StatusCode::DECELERATE_FOR_APPROACHING_SINGULARITY);
}

TEST_F(ServoCppFixture, testHaltForSingularity)
{
  moveit::core::RobotStatePtr robot_state = planning_scene_monitor_->getStateMonitor()->getCurrentState();
  auto joint_model_group = robot_state->getJointModelGroup(servo_param_listener_->get_params().move_group_name);
  auto servo_params = servo_param_listener_->get_params();
  Eigen::VectorXd cartesian_delta(6);
  cartesian_delta << 0.005, 0.0, 0.0, 0.0, 0.0, 0.0;

  auto scaling_result =
      moveit_servo::velocityScalingFactorForSingularity(joint_model_group, robot_state, cartesian_delta, servo_params);
  ASSERT_EQ(scaling_result.second, moveit_servo::StatusCode::NO_WARNING);

  robot_state->setToDefaultValues();
  robot_state->setVariablePosition("panda_joint1", -0.0001);
  robot_state->setVariablePosition("panda_joint2", 0.5690);
  robot_state->setVariablePosition("panda_joint3", 0.0005);
  robot_state->setVariablePosition("panda_joint4", -0.7782);
  robot_state->setVariablePosition("panda_joint5", 0.0);
  robot_state->setVariablePosition("panda_joint6", 1.3453);
  robot_state->setVariablePosition("panda_joint7", 0.7845);

  scaling_result =
      moveit_servo::velocityScalingFactorForSingularity(joint_model_group, robot_state, cartesian_delta, servo_params);
  ASSERT_EQ(scaling_result.second, moveit_servo::StatusCode::HALT_FOR_SINGULARITY);
}

TEST_F(ServoCppFixture, testGetEndEffectorFrame)
{
  Eigen::Isometry3d ee_pose = servo_test_instance_->getEndEffectorPose();
  ASSERT_TRUE(moveit_servo::isValidCommand(ee_pose));
}

TEST_F(ServoCppFixture, testPoseFromCartesianDelta)
{
  moveit::core::RobotStatePtr robot_state = planning_scene_monitor_->getStateMonitor()->getCurrentState();
  auto joint_model_group = robot_state->getJointModelGroup(servo_param_listener_->get_params().move_group_name);
  kinematics::KinematicsBaseConstPtr ik_solver = joint_model_group->getSolverInstance();
  const Eigen::Isometry3d base_to_tip_frame_transform =
      robot_state->getGlobalLinkTransform(ik_solver->getBaseFrame()).inverse() *
      robot_state->getGlobalLinkTransform(ik_solver->getTipFrame());
  ASSERT_TRUE(moveit_servo::isValidCommand(base_to_tip_frame_transform));

  // Pose message for a carteisan delta with only a +45 degree rotation about z.
  Eigen::VectorXd cartesian_delta(6);
  cartesian_delta << 0.0, 0.0, 0.0, 0.0, 0.0, M_PI / 4;
  geometry_msgs::msg::Pose recieved_pose =
      moveit_servo::poseFromCartesianDelta(cartesian_delta, base_to_tip_frame_transform);

  // End effector pose rotated by 45 degree
  Eigen::Isometry3d ee_pose = servo_test_instance_->getEndEffectorPose();
  ee_pose.rotate(Eigen::AngleAxisd(M_PI / 4, Eigen::Vector3d::UnitZ()));
  Eigen::Quaterniond ee_rotation(ee_pose.rotation());
  double ee_pose_z = ee_rotation.z();
  ASSERT_FLOAT_EQ(recieved_pose.orientation.z, ee_pose_z);
}

}  // namespace

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
