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

/*      Title       : servo_node.cpp
 *      Project     : moveit_servo
 *      Created     : 01/07/2023
 *      Author      : V Mohammed Ibrahim
 *
 */

#include <moveit_servo/servo_node.hpp>

namespace
{
const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_servo.servo_node");
}

namespace moveit_servo
{

ServoNode::ServoNode(const rclcpp::Node::SharedPtr& node) : node_(node)
{
  servo_param_listener_ = std::make_shared<servo::ParamListener>(node_, "moveit_servo");
  servo_params_ = servo_param_listener_->get_params();

  // Create Servo instance
  servo_ = std::make_unique<Servo>(node_, servo_param_listener_, createPlanningSceneMonitor(node_, servo_params_));

  // Create subscriber for jointjog
  joint_jog_subscriber_ = node_->create_subscription<control_msgs::msg::JointJog>(
      servo_params_.command_out_topic, 10,
      [this](const control_msgs::msg::JointJog::SharedPtr msg) { jointJogCallback(msg); });

  new_joint_jog_ = false;
}

void ServoNode::jointJogCallback(const control_msgs::msg::JointJog::SharedPtr msg)
{
  latest_joint_jog_ = *msg;
  new_joint_jog_ = true;
}

}  // namespace moveit_servo

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("moveit_servo");
  auto servo_node = std::make_unique<moveit_servo::ServoNode>(node);
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
