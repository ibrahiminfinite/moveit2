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

/*      Title     : servo_node.cpp
 *      Project   : moveit_servo
 *      Created   : 12/31/2018
 *      Author    : Andy Zelenak, V Mohammed Ibrahim
 */

#include <moveit_servo/servo_node.h>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_servo.servo_node");

namespace moveit_servo
{
ServoNode::ServoNode(const rclcpp::NodeOptions& options)
  : node_{ std::make_shared<rclcpp::Node>("servo_node", options) }
{
  if (!options.use_intra_process_comms())
  {
    RCLCPP_WARN_STREAM(LOGGER, "Intra-process communication is disabled, consider enabling it by adding: "
                               "\nextra_arguments=[{'use_intra_process_comms' : True}]\nto the Servo composable node "
                               "in the launch file");
  }

  auto servo_node = std::make_shared<rclcpp::Node>("servo_node", options);
  std::string param_namespace = "moveit_servo";
  auto servo_param_listener = std::make_shared<const servo::ParamListener>(servo_node, param_namespace);
  servo_params_ = servo_param_listener->get_params();

  joint_cmd_sub_ = node_->create_subscription<control_msgs::msg::JointJog>(
    servo_params_.joint_command_in_topic, rclcpp::SystemDefaultsQoS(),
    [this](const control_msgs::msg::JointJog::ConstSharedPtr& msg) { return jointCmdCB(msg); });

  // Create Servo
  servo_ = std::make_unique<moveit_servo::Servo>(node_);
}

void ServoCalcs::jointCmdCB(const control_msgs::msg::JointJog::ConstSharedPtr& msg)
{

  joint_velocity_command_ = msg;
  // notify that we have a new input
  has_new_command_ = true;

}


void ServoNode::createServices()
{
  // Set up services for interacting with Servo
  start_servo_service_ = node_->create_service<std_srvs::srv::Trigger>(
      "~/start_servo",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>& request,
             const std::shared_ptr<std_srvs::srv::Trigger::Response>& response) { return startCB(request, response); });
  stop_servo_service_ = node_->create_service<std_srvs::srv::Trigger>(
      "~/stop_servo",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>& request,
             const std::shared_ptr<std_srvs::srv::Trigger::Response>& response) { return stopCB(request, response); });
}

void ServoNode::startCB(const std::shared_ptr<std_srvs::srv::Trigger::Request>& /* unused */,
                        const std::shared_ptr<std_srvs::srv::Trigger::Response>& response)
{
  response->success = true;
}

void ServoNode::stopCB(const std::shared_ptr<std_srvs::srv::Trigger::Request>& /* unused */,
                       const std::shared_ptr<std_srvs::srv::Trigger::Response>& response)
{
  response->success = true;
}
}  // namespace moveit_servo


void ServoNode::servoLoop()
{
    while(rclcpp::ok())
    {
      if(incoming_command_type == CommandType::JointVelocity && has_new_command_)
      {
        JoinVelocity joint_cmd = Eigen::Map<Eigen::VectorXd>(joint_velocity_command_.velocities.data(), 
                                                                joint_velocity_command_.velocities.size())
      }
    }
}

// Register the component with class_loader
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(moveit_servo::ServoNode)
