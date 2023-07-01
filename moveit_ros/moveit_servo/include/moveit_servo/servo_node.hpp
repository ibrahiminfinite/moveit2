/*******************************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2019, Los Alamos National Security, LLC
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

/*      Title       : servo_node.hpp
 *      Project     : moveit_servo
 *      Created     : 01/07/2023
 *      Author      : Brian O'Neil, Andy Zelenak, Blake Anderson, V Mohammed Ibrahim
 *
 *      Description : The ROS API for Servo
 */

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <control_msgs/msg/joint_jog.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include <moveit_servo/servo.hpp>

namespace moveit_servo
{

class ServoNode
{
public:
  ServoNode(const rclcpp::Node::SharedPtr& node);

private:
  void servoLoop();

  void jointJogCallback(const control_msgs::msg::JointJog::SharedPtr msg);
  void twistCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);

  const rclcpp::Node::SharedPtr node_;
  std::unique_ptr<Servo> servo_;
  std::shared_ptr<servo::ParamListener> servo_param_listener_;
  servo::Params servo_params_;
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;

  std::atomic<bool> new_joint_jog_, new_twist_;
  control_msgs::msg::JointJog latest_joint_jog_;
  geometry_msgs::msg::TwistStamped latest_twist_;
  rclcpp::Subscription<control_msgs::msg::JointJog>::SharedPtr joint_jog_subscriber_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_subscriber_;

  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_publisher_;

  std::thread loop_thread_;
};

}  // namespace moveit_servo
