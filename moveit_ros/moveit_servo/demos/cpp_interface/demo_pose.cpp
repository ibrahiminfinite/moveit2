/*******************************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2021, PickNik Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

/*      Title     : joint_jog_demo.cpp
 *      Project   : moveit_servo
 *      Created   : 27/05/2023
 *      Author    : V Mohammed Ibrahim
 */

#include <iostream>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <moveit_servo/servo.hpp>
#include <moveit_servo/utils.hpp>

using namespace moveit_servo;

const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_servo.pose_demo");

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  // The servo object expects to get a ROS node.
  auto servo_node = std::make_shared<rclcpp::Node>("servo_node");

  // Get the servo parameters.
  std::string param_namespace = "moveit_servo";
  auto servo_param_listener = std::make_shared<const servo::ParamListener>(servo_node, param_namespace);
  auto servo_params = servo_param_listener->get_params();

  // The publisher to send trajectory message to the robot controller.
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_outgoing_cmd_pub;
  trajectory_outgoing_cmd_pub = servo_node->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      servo_params.command_out_topic, rclcpp::SystemDefaultsQoS());

  // Create the servo object
  auto servo = Servo(servo_node, servo_param_listener);

  // Wait for some time, so that we can actually see when the robot moves.
  // This is just for convenience, should not be used for sync in real application.
  std::this_thread::sleep_for(std::chrono::seconds(3));

  // Frquency at which the commands will be send to robot controller.
  rclcpp::WallRate rate(1.0 / servo_params.publish_period);

  // Set the command type for servo.
  servo.incomingCommandType(CommandType::POSE);

  RCLCPP_INFO_STREAM(LOGGER, servo.getStatusMessage());

  // Get current pose of end-effector, this is in planning frame.
  Eigen::Isometry3d curr_pose = servo.getEndEffectorPose();

  Eigen::Isometry3d target_pose = curr_pose;
  target_pose.translation().x() += 0.1;
  target_pose.translation().y() += 0.1;

  Pose p1{ servo_params.planning_frame, target_pose };

  while (rclcpp::ok() && servo.getStatus() == StatusCode::NO_WARNING)
  {
    curr_pose = servo.getEndEffectorPose();
    if (curr_pose.isApprox(target_pose, 0.001))
    {
      RCLCPP_INFO_STREAM(LOGGER, "REACHED TARGET POSE");
      break;
    }
    auto joint_state = servo.getNextJointState(p1);

    // Send the command to robot controller only if the command was valid.
    if (servo.getStatus() != StatusCode::INVALID)
    {
      auto joint_trajectory = composeTrajectoryMessage(servo_params, joint_state);
      trajectory_outgoing_cmd_pub->publish(joint_trajectory);
    }

    rate.sleep();
  }
  RCLCPP_INFO_STREAM(LOGGER, servo.getStatusMessage());
  rclcpp::shutdown();
}
