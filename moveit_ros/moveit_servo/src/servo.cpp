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

/*      Title     : servo_inputs.hpp
 *      Project   : moveit_servo
 *      Created   : 17/05/2023
 *      Author    : Brian O'Neil, Andy Zelenak, Blake Anderson, V Mohammed Ibrahim
 */

#include <rclcpp/rclcpp.hpp>
#include <moveit_servo/servo.hpp>
#include <moveit_servo/utils.hpp>

namespace moveit_servo
{

static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_servo.servo");

Servo::Servo(const rclcpp::Node::SharedPtr& node) : node_(node)
{
  std::string param_namespace = "moveit_servo";
  servo_param_listener_ = std::make_shared<servo::ParamListener>(node_, param_namespace);
  servo_params_ = servo_param_listener_->get_params();
  validateParams(servo_params_);

  // Can set robot_description name from parameters
  std::string robot_description_name = "robot_description";
  node_->get_parameter_or("robot_description_name", robot_description_name, robot_description_name);
  // Set up planning_scene_monitor
  planning_scene_monitor_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
      node_, robot_description_name, "planning_scene_monitor");
  planning_scene_monitor_->startStateMonitor(servo_params_.joint_topic);
  planning_scene_monitor_->startSceneMonitor(servo_params_.monitored_planning_scene_topic);
  planning_scene_monitor_->setPlanningScenePublishingFrequency(25);
  planning_scene_monitor_->getStateMonitor()->enableCopyDynamics(true);
  planning_scene_monitor_->startPublishingPlanningScene(planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE,
                                                        std::string(node_->get_fully_qualified_name()) +
                                                            "/publish_planning_scene");
  if (servo_params_.is_primary_planning_scene_monitor)
  {
    planning_scene_monitor_->providePlanningSceneService();
  }
  else
  {
    planning_scene_monitor_->requestPlanningSceneState();
  }

  current_state_ = planning_scene_monitor_->getStateMonitor()->getCurrentState();
  auto joint_model_group_ = current_state_->getJointModelGroup(servo_params_.move_group_name);
  if (joint_model_group_ == nullptr)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Invalid move group name: `" << servo_params_.move_group_name << '`');
    throw std::runtime_error("Invalid move group name");
  }
  // Get the IK solver for the group
  ik_solver_ = joint_model_group_->getSolverInstance();
  if (!ik_solver_)
  {
    RCLCPP_WARN(
        LOGGER,
        "No kinematics solver instantiated for group '%s'. Will use inverse Jacobian for servo calculations instead.",
        joint_model_group_->getName().c_str());
  }
  else if (!ik_solver_->supportsGroup(joint_model_group_))
  {
    ik_solver_ = nullptr;
    RCLCPP_WARN(LOGGER,
                "The loaded kinematics plugin does not support group '%s'. Will use inverse Jacobian for servo "
                "calculations instead.",
                joint_model_group_->getName().c_str());
  }

  RCLCPP_INFO_STREAM(LOGGER, "SERVO : Initialized");
}

RobotJointState Servo::getNextJointState(const ServoInput& command)
{
  RobotJointState next_joint_state;
  next_joint_state.positions.setZero();
  next_joint_state.velocities.setZero();
  next_joint_state.accelerations.setZero();
  processCommand(command);
  return next_joint_state;
}

Eigen::VectorXd Servo::processCommand(const ServoInput& command)
{
  Eigen::VectorXd next_joint_positions;
  next_joint_positions.setZero();  // This should be set to current position.

  CommandType incomingType = incomingCommandType();

  if (incomingType == CommandType::JOINT_POSITION && command.index() == 0)
  {
    next_joint_positions = processCommand(std::get<JointVelocity>(command));
  }
  else if (incomingType == CommandType::TWIST && command.index() == 1)
  {
    next_joint_positions = processCommand(std::get<Twist>(command));
  }
  else if (incomingType == CommandType::POSE && command.index() == 2)
  {
    next_joint_positions = processCommand(std::get<Pose>(command));
  }
  else
  {
    // PRINT RCLCPP_ERROR
  }
  return next_joint_positions;
}

Eigen::VectorXd Servo::processCommand(const Pose& command)
{
  Eigen::VectorXd rvec(2);
  rvec << 1.0, 1.0;
  return rvec;
}

Eigen::VectorXd Servo::processCommand(const Twist& command)
{
  Eigen::VectorXd rvec(2);
  rvec << 1.0, 1.0;
  return rvec;
}

Eigen::VectorXd Servo::processCommand(const JointVelocity& command)
{
  if (isValidCommand(command))
  {
    std::cout << "Valid " << std::endl;
  }
  Eigen::VectorXd rvec(2);
  rvec << 1.0, 1.0;
  return rvec;
}

CommandType Servo::incomingCommandType()
{
  std::lock_guard<std::mutex> lock(command_type_mutex_);
  return incoming_command_type_;
}

bool Servo::incomingCommandType(const CommandType& command_type)
{
  std::lock_guard<std::mutex> lock(command_type_mutex_);
  incoming_command_type_ = command_type;
  return true;
}

void Servo::validateParams(const servo::Params& servo_params)
{
  bool has_error = false;
  if (servo_params.hard_stop_singularity_threshold <= servo_params.lower_singularity_threshold)
  {
    RCLCPP_ERROR(LOGGER, "Parameter 'hard_stop_singularity_threshold' "
                         "should be greater than 'lower_singularity_threshold.' "
                         "Check the parameters YAML file used to launch this node.");
    has_error = true;
  }

  if (!servo_params.publish_joint_positions && !servo_params.publish_joint_velocities &&
      !servo_params.publish_joint_accelerations)
  {
    RCLCPP_ERROR(LOGGER, "At least one of publish_joint_positions / "
                         "publish_joint_velocities / "
                         "publish_joint_accelerations must be true. "
                         "Check the parameters YAML file used to launch this node.");
    has_error = true;
  }

  if ((servo_params.command_out_type == "std_msgs/Float64MultiArray") && servo_params.publish_joint_positions &&
      servo_params.publish_joint_velocities)
  {
    RCLCPP_ERROR(LOGGER, "When publishing a std_msgs/Float64MultiArray, "
                         "you must select positions OR velocities."
                         "Check the parameters YAML file used to launch this node.");
    has_error = true;
  }

  if (servo_params.scene_collision_proximity_threshold < servo_params.self_collision_proximity_threshold)
  {
    RCLCPP_ERROR(LOGGER, "Parameter 'self_collision_proximity_threshold' should probably be less "
                         "than or equal to 'scene_collision_proximity_threshold'."
                         "Check the parameters YAML file used to launch this node.");
    has_error = true;
  }

  if (has_error)
  {
    throw std::runtime_error("Servo failed to initialize : Invalid parameter values");
  }
}

}  // namespace moveit_servo
