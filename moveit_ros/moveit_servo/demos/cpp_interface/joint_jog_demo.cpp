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

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;

  auto servo_node = std::make_shared<rclcpp::Node>("servo_node", options);
  std::string param_namespace = "moveit_servo";
  auto servo_param_listener = std::make_shared<const servo::ParamListener>(servo_node, param_namespace);
  auto servo_params = servo_param_listener->get_params();
  auto servo = moveit_servo::Servo(servo_node, servo_param_listener);

  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_outgoing_cmd_pub_;
  trajectory_outgoing_cmd_pub_ = servo_node->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      servo_params.command_out_topic, rclcpp::SystemDefaultsQoS());

  rclcpp::WallRate rate(1.0 / servo_params.publish_period);
  // Wait for some time, so that we can actually see when the robot moves
  std::this_thread::sleep_for(std::chrono::seconds(5));
  int count = 0;

  while (rclcpp::ok() && servo.getStatus() != moveit_servo::StatusCode::JOINT_BOUND)
  {
    servo.incomingCommandType(moveit_servo::CommandType::JOINT_POSITION);
    moveit_servo::JointVelocity vec(7);
    // Move only the 8th joint
    vec << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0;
    auto joint_state = servo.getNextJointState(vec);
    auto joint_trajectory = moveit_servo::composeTrajectoryMessage(servo_params, joint_state);

    trajectory_outgoing_cmd_pub_->publish(std::move(joint_trajectory));
    count++;
    rate.sleep();
  }

  rclcpp::shutdown();
}
