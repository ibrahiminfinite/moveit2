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

  createPlanningSceneMonitor();

  current_state_ = planning_scene_monitor_->getStateMonitor()->getCurrentState();
  joint_model_group_ = current_state_->getJointModelGroup(servo_params_.move_group_name);
  if (joint_model_group_ == nullptr)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Invalid move group name: `" << servo_params_.move_group_name << '`');
    throw std::runtime_error("Invalid move group name");
  }

  setIKSolver();

  RCLCPP_INFO_STREAM(LOGGER, "SERVO : Initialized");
}

void Servo::createPlanningSceneMonitor()
{
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
}

void Servo::setIKSolver()
{
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
}

RobotJointState Servo::getNextJointState(const ServoInput& command)
{
  RobotJointState next_joint_state;
  // Compute the change in joint position due to the incoming command
  Eigen::VectorXd joint_position_delta = jointDeltaFromCommand(command);
  // Apply collision scaling to the delta
  // Compute the next joint positions and apply smoother (should probably be applied to velicity instead)
  // Compute the next joint velocities
  // Enforce joint velocity and position limits
  return next_joint_state;
}

Eigen::VectorXd Servo::jointDeltaFromCommand(const ServoInput& command)
{
  Eigen::VectorXd next_joint_positions;
  next_joint_positions.setZero();  // This should be set to current position.

  CommandType incomingType = incomingCommandType();

  if (incomingType == CommandType::JOINT_POSITION && command.index() == 0)
  {
    next_joint_positions = jointDeltaFromCommand(std::get<JointVelocity>(command));
  }
  else if (incomingType == CommandType::TWIST && command.index() == 1)
  {
    next_joint_positions = jointDeltaFromCommand(std::get<Twist>(command));
  }
  else if (incomingType == CommandType::POSE && command.index() == 2)
  {
    next_joint_positions = jointDeltaFromCommand(std::get<Pose>(command));
  }
  else
  {
    // PRINT RCLCPP_ERROR
  }
  return next_joint_positions;
}

Eigen::VectorXd Servo::jointDeltaFromCommand(const Pose& command)
{
  Eigen::VectorXd rvec(2);
  rvec << 1.0, 1.0;
  return rvec;
}

Eigen::VectorXd Servo::jointDeltaFromCommand(const Twist& command)
{
  Eigen::VectorXd joint_poition_delta;
  Eigen::VectorXd cartesian_position_delta;

  if (isValidCommand(command.velocities))
  {
    Twist transformed_twist = command;

    if (command.frame_id.empty())
    {
      RCLCPP_WARN_STREAM(LOGGER,
                         "No frame specified for command, will use planning_frame: " << servo_params_.planning_frame);
      command.frame_id = servo_params_.planning_frame;
    }
    // Transform the command to the MoveGroup planning frame
    if (command.frame_id != servo_params_.planning_frame)
    {
      // We solve (planning_frame -> base -> cmd.header.frame_id)
      // by computing (base->planning_frame)^-1 * (base->cmd.header.frame_id)
      const Eigen::Isometry3d planning_frame_transfrom =
          current_state_->getGlobalLinkTransform(command.frame_id).inverse() *
          current_state_->getGlobalLinkTransform(servo_params_.planning_frame);

      // Apply the transformation to the command vector
      transformed_twist.frame_id = servo_params_.planning_frame;
      transformed_twist.velocities.head<3>() = planning_frame_transfrom.linear() * command.velocities.head<3>();
      transformed_twist.velocities.tail<3>() = planning_frame_transfrom.linear() * command.velocities.tail<3>();
    }

    // Compute the cartesian position delta based on incoming command
    cartesian_position_delta = transformed_twist.velocities * servo_params_.publish_period;

    return joint_poition_delta;
  }

  return joint_poition_delta;
}

Eigen::VectorXd Servo::jointDeltaFromCommand(const JointVelocity& command)
{
  // Find the target joint position based on the commanded joint velocity
  Eigen::VectorXd joint_poition_delta;
  if (isValidCommand(command))
  {
    joint_poition_delta = command * servo_params_.publish_period;
  }
  else
  {
    RCLCPP_WARN_STREAM(LOGGER, " SERVO : Invalid joint velocity command");
  }
  return joint_poition_delta;
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
