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

/*      Title     : servo_node_main.cpp
 *      Project   : moveit_servo
 *      Created   : 17/05/2023
 *      Author    : V Mohammed Ibrahim
 */

#include <tf2_eigen/tf2_eigen.hpp>
#include <moveit_servo/servo.hpp>

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;

  auto servo_node = std::make_shared<rclcpp::Node>("servo_node", options);
  auto servo = moveit_servo::Servo(servo_node);

  rclcpp::WallRate rate(1.0 / 0.2);
  while (rclcpp::ok())
  {
    servo.incomingCommandType(moveit_servo::CommandType::JOINT_POSITION);
    moveit_servo::JointVelocity vec(2);
    vec << 1.0, std::nan("NaN");
    servo.getNextJointState(vec);
    servo.incomingCommandType(moveit_servo::CommandType::POSE);
    moveit_servo::Pose pose;
    pose.setIdentity();
    servo.getNextJointState(pose);
    servo.incomingCommandType(moveit_servo::CommandType::TWIST);
    const std::string frame = "panda_link8";
    moveit_servo::Twist twist{ frame, { 1.0, 1.0, 1.0, 1.0, 1.0, 1.0 } };
    servo.getNextJointState(twist);
    rate.sleep();
  }

  rclcpp::shutdown();
}
