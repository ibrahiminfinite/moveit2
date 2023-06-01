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

/*      Title     : servo.cpp
 *      Project   : moveit_servo
 *      Created   : 17/05/2023
 *      Author    : Brian O'Neil, Andy Zelenak, Blake Anderson, V Mohammed Ibrahim
 */

#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <moveit_servo/servo.hpp>
#include <moveit_servo/utils.hpp>

namespace moveit_servo
{

static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_servo.servo");

Servo::Servo(const rclcpp::Node::SharedPtr& node, std::shared_ptr<const servo::ParamListener>& servo_param_listener)
  : node_(node)
  , servo_param_listener_{ servo_param_listener }
  , smoothing_loader_("moveit_core", "online_signal_smoothing::SmoothingBaseClass")

{
  servo_status_ = StatusCode::NO_WARNING;
  servo_params_ = servo_param_listener_->get_params();

  validateParams(servo_params_);

  createPlanningSceneMonitor();

  // Create the collision checker
  collision_checker_ = std::make_unique<CollisionCheck>(node_, planning_scene_monitor_, servo_param_listener_);
  if (servo_params_.check_collisions)
    collision_checker_->start();

  // Create collision velocity subscriber
  collision_velocity_scale_sub_ = node_->create_subscription<std_msgs::msg::Float64>(
      "~/collision_velocity_scale", rclcpp::SystemDefaultsQoS(),
      [this](const std_msgs::msg::Float64::ConstSharedPtr& msg) { return collisionVelocityScaleCB(msg); });

  current_state_ = planning_scene_monitor_->getStateMonitor()->getCurrentState();
  joint_model_group_ = current_state_->getJointModelGroup(servo_params_.move_group_name);

  // Get necessary information about joints
  joint_names_ = joint_model_group_->getActiveJointModelNames();
  joint_bounds_ = joint_model_group_->getActiveJointModelsBounds();
  num_joints_ = joint_names_.size();

  if (joint_model_group_ == nullptr)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Invalid move group name: `" << servo_params_.move_group_name << '`');
    throw std::runtime_error("Invalid move group name");
  }

  // Load the IK solver if one exists for the robot
  setIKSolver();
  // Load the smoothing plugin
  setSmoothingPlugin();

  RCLCPP_INFO_STREAM(LOGGER, "SERVO : Initialized");
}

void Servo::collisionVelocityScaleCB(const std_msgs::msg::Float64::ConstSharedPtr& msg)
{
  collision_velocity_scale_ = msg->data;
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

void Servo::setSmoothingPlugin()
{
  // Load the smoothing plugin
  try
  {
    smoother_ = smoothing_loader_.createUniqueInstance(servo_params_.smoothing_filter_plugin_name);
  }
  catch (pluginlib::PluginlibException& ex)
  {
    RCLCPP_ERROR(LOGGER, "Exception while loading the smoothing plugin '%s': '%s'",
                 servo_params_.smoothing_filter_plugin_name.c_str(), ex.what());
    std::exit(EXIT_FAILURE);
  }

  // Initialize the smoothing plugin
  if (!smoother_->initialize(node_, planning_scene_monitor_->getRobotModel(), num_joints_))
  {
    RCLCPP_ERROR(LOGGER, "Smoothing plugin could not be initialized");
    std::exit(EXIT_FAILURE);
  }
}

void Servo::updateParams()
{
  if (servo_param_listener_->is_old(servo_params_))
  {
    auto params = servo_param_listener_->get_params();
    if (params.override_velocity_scaling_factor != servo_params_.override_velocity_scaling_factor)
    {
      RCLCPP_INFO_STREAM(LOGGER, "override_velocity_scaling_factor changed to : "
                                     << std::to_string(params.override_velocity_scaling_factor));
    }

    if (params.robot_link_command_frame != servo_params_.robot_link_command_frame)
    {
      if (current_state_->knowsFrameTransform(params.robot_link_command_frame))
      {
        RCLCPP_INFO_STREAM(LOGGER, "robot_link_command_frame changed to : " << params.robot_link_command_frame);
      }
      else
      {
        RCLCPP_ERROR_STREAM(LOGGER, "Failed to change robot_link_command_frame. Passed frame '"
                                        << params.robot_link_command_frame
                                        << "' is unknown, will keep using old command frame.");
        // Replace frame in new param set with old frame value
        params.robot_link_command_frame = servo_params_.robot_link_command_frame;
      }
    }
    servo_params_ = params;
  }
}

sensor_msgs::msg::JointState Servo::getNextJointState(const ServoInput& command)
{
  // Update the parameters
  if (servo_params_.enable_parameter_update)
  {
    updateParams();
  }

  // Set status to clear
  servo_status_ = moveit_servo::StatusCode::NO_WARNING;

  // Joint position and veloctiy variables.
  // The smoother needs a std::vector input, so we create std::vector's but make Eigen::Map for cleaner computation.
  std::vector<double> current_joint_pos(num_joints_), next_joint_pos(num_joints_);
  Eigen::Map<Eigen::VectorXd> current_joint_positions(current_joint_pos.data(), current_joint_pos.size());
  Eigen::Map<Eigen::VectorXd> next_joint_positions(next_joint_pos.data(), next_joint_pos.size());
  Eigen::VectorXd current_joint_velocities(num_joints_), next_joint_velocities(num_joints_);

  // Update current robot state as reported by planning scene monitor
  current_state_ = planning_scene_monitor_->getStateMonitor()->getCurrentState();
  current_state_->copyJointGroupPositions(joint_model_group_, current_joint_pos);
  current_state_->copyJointGroupVelocities(joint_model_group_, current_joint_velocities);

  // Update filter state
  smoother_->reset(current_joint_pos);

  // Compute the change in joint position due to the incoming command
  Eigen::VectorXd joint_position_delta = jointDeltaFromCommand(command);

  // Apply collision scaling to the joint position delta
  if (collision_velocity_scale_ > 0 && collision_velocity_scale_ < 1)
  {
    servo_status_ = StatusCode::DECELERATE_FOR_COLLISION;
  }
  else if (collision_velocity_scale_ == 0)
  {
    servo_status_ = StatusCode::HALT_FOR_COLLISION;
  }
  joint_position_delta *= collision_velocity_scale_;

  // Compute the next joint positions based on the joint position deltas
  next_joint_positions = current_joint_positions + joint_position_delta;

  // TODO : apply filtering to the velocity instead of position
  // Apply smoothing to the positions
  smoother_->doSmoothing(next_joint_pos);

  // Compute velocities based on smoothed joint positions
  next_joint_velocities = (next_joint_positions - current_joint_positions) / servo_params_.publish_period;

  // Enforce joint limits
  std::vector<int> joint_idxs_to_halt;
  double min_scaling_factor = servo_params_.override_velocity_scaling_factor;
  // If override value is close to zero, user is not overriding the scaling
  if (min_scaling_factor < 0.01)
  {
    double bounded_vel;
    std::vector<double> velocity_scaling_factors;  // The allowable fraction of computed veclocity

    for (size_t i = 0; i < joint_bounds_.size(); i++)
    {
      const auto joint_bound = (*joint_bounds_[i])[0];
      if (joint_bound.velocity_bounded_ && next_joint_velocities[i] != 0.0)
      {
        // Find the ratio of clamped velocity to original velocity
        bounded_vel = std::clamp(next_joint_velocities[i], joint_bound.min_velocity_, joint_bound.max_velocity_);
        velocity_scaling_factors.push_back(bounded_vel / next_joint_velocities[i]);
      }
    }
    // Find the lowest scaling factor, this helps preserve cartesian motion.
    min_scaling_factor = *std::min_element(velocity_scaling_factors.begin(), velocity_scaling_factors.end());
  }

  // Scale down the velocity
  next_joint_velocities *= min_scaling_factor;

  // Adjust joint position based on scaled down velocity
  next_joint_positions = current_joint_positions + (next_joint_velocities * servo_params_.publish_period);

  // Check if any joints are going past joint position limits
  for (size_t i = 0; i < joint_bounds_.size(); i++)
  {
    const auto joint_bound = (*joint_bounds_[i])[0];
    if (joint_bound.position_bounded_)
    {
      const bool negative_bound =
          next_joint_velocities[i] < 0 &&
          next_joint_positions[i] < (joint_bound.min_position_ + servo_params_.joint_limit_margin);
      const bool positive_bound =
          next_joint_velocities[i] > 0 &&
          next_joint_positions[i] > (joint_bound.max_position_ - servo_params_.joint_limit_margin);
      if (negative_bound || positive_bound)
      {
        RCLCPP_WARN_STREAM(LOGGER, " Joint position limit on joint " << joint_names_[i]);
        joint_idxs_to_halt.push_back(i);
      }
    }
  }

  // Apply halting if any joints need to be halted.
  if (!joint_idxs_to_halt.empty())
  {
    servo_status_ = moveit_servo::StatusCode::JOINT_BOUND;

    const bool all_joint_halt =
        incomingCommandType() == CommandType::JOINT_JOG && servo_params_.halt_all_joints_in_joint_mode;

    if (all_joint_halt)
    {
      next_joint_positions = current_joint_positions;
      next_joint_velocities.setZero();
    }
    else
    {
      // Halt only the joints that are out of bounds
      for (const int idx : joint_idxs_to_halt)
      {
        next_joint_positions[idx] = current_joint_positions[idx];
        next_joint_velocities[idx] = 0.0;
      }
    }
  }

  // next state
  sensor_msgs::msg::JointState next_joint_state;
  next_joint_state.name = joint_names_;
  next_joint_state.position.resize(num_joints_);
  next_joint_state.velocity.resize(num_joints_);
  for (size_t i = 0; i < num_joints_; i++)
  {
    next_joint_state.position[i] = next_joint_positions[i];
    next_joint_state.velocity[i] = next_joint_velocities[i];
  }

  return next_joint_state;
}

Eigen::VectorXd Servo::jointDeltaFromCommand(const ServoInput& command)
{
  Eigen::VectorXd next_joint_positions(num_joints_);
  next_joint_positions.setZero();  // This should be set to current position.

  CommandType incomingType = incomingCommandType();

  if (incomingType == CommandType::JOINT_JOG && command.index() == 0)
  {
    next_joint_positions = jointDeltaFromCommand(std::get<JointJog>(command));
  }
  else if (incomingType == CommandType::TWIST && command.index() == 1)
  {
    next_joint_positions = jointDeltaFromCommand(std::get<Twist>(command));
  }
  // else if (incomingType == CommandType::POSE && command.index() == 2)
  // {
  //   next_joint_positions = jointDeltaFromCommand(std::get<Pose>(command));
  // }
  else
  {
    servo_status_ = StatusCode::INVALID;
  }
  return next_joint_positions;
}

Eigen::VectorXd Servo::jointDeltaFromCommand(const JointJog& command)
{
  // Find the target joint position based on the commanded joint velocity
  Eigen::VectorXd joint_poition_delta(num_joints_);
  joint_poition_delta.setZero();

  if (isValidCommand(command))
  {
    // The incoming command should be in rad/s
    joint_poition_delta = command * servo_params_.publish_period;
  }
  else
  {
    RCLCPP_WARN_STREAM(LOGGER, " SERVO : Invalid joint velocity command");
  }
  return joint_poition_delta;
}

Eigen::VectorXd Servo::jointDeltaFromCommand(const Twist& command)
{
  Eigen::VectorXd joint_position_delta(num_joints_);
  Eigen::VectorXd cartesian_position_delta;

  std::string command_frame = command.frame_id;
  const std::string planning_frame = servo_params_.planning_frame;

  if (isValidCommand(command.velocities))
  {
    Twist transformed_twist = command;

    if (command_frame.empty())
    {
      RCLCPP_WARN_STREAM(LOGGER,
                         "No frame specified for command, will use planning_frame: " << servo_params_.planning_frame);
      command_frame = planning_frame;
    }
    // Transform the command to the MoveGroup planning frame
    if (command.frame_id != planning_frame)
    {
      // We solve (planning_frame -> base -> cmd.header.frame_id)
      // by computing (base->planning_frame)^-1 * (base->cmd.header.frame_id)
      const Eigen::Isometry3d planning_frame_transfrom =
          current_state_->getGlobalLinkTransform(command_frame).inverse() *
          current_state_->getGlobalLinkTransform(planning_frame);

      // Apply the transformation to the command vector
      transformed_twist.frame_id = planning_frame;
      transformed_twist.velocities.head<3>() = planning_frame_transfrom.linear() * command.velocities.head<3>();
      transformed_twist.velocities.tail<3>() = planning_frame_transfrom.linear() * command.velocities.tail<3>();
    }

    // Compute the cartesian position delta based on incoming command, assumed to be in m/s
    cartesian_position_delta = transformed_twist.velocities * servo_params_.publish_period;

    Eigen::MatrixXd jacobian = current_state_->getJacobian(joint_model_group_);
    Eigen::JacobiSVD<Eigen::MatrixXd> svd =
        Eigen::JacobiSVD<Eigen::MatrixXd>(jacobian, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::MatrixXd matrix_s = svd.singularValues().asDiagonal();
    Eigen::MatrixXd pseudo_inverse = svd.matrixV() * matrix_s.inverse() * svd.matrixU().transpose();

    // Compute the required change in joint angles.
    if (ik_solver_)
    {
      // Use robot's IK solver to get joint position delta.
      joint_position_delta = detlaFromIkSolver(cartesian_position_delta);
    }
    else
    {
      // Robot does not have an IK solver, use inverse Jacobian to compute IK.
      joint_position_delta = pseudo_inverse * cartesian_position_delta;
    }

    // Apply velocity scaling for singularity.
    joint_position_delta *= moveit_servo::velocityScalingFactorForSingularity(
        joint_model_group_, current_state_, cartesian_position_delta, servo_params_, servo_status_);
  }
  return joint_position_delta;
}

Eigen::VectorXd Servo::detlaFromIkSolver(Eigen::VectorXd cartesian_position_delta)
{
  Eigen::VectorXd delta_theta(num_joints_);
  std::vector<double> current_joint_positions(num_joints_);

  current_state_->copyJointGroupPositions(joint_model_group_, current_joint_positions);

  const Eigen::Isometry3d base_to_tip_frame_transform =
      current_state_->getGlobalLinkTransform(ik_solver_->getBaseFrame()).inverse() *
      current_state_->getGlobalLinkTransform(ik_solver_->getTipFrame());

  geometry_msgs::msg::Pose next_pose = poseFromCartesianDelta(cartesian_position_delta, base_to_tip_frame_transform);

  // setup for IK call
  std::vector<double> solution(num_joints_);
  moveit_msgs::msg::MoveItErrorCodes err;
  kinematics::KinematicsQueryOptions opts;
  opts.return_approximate_solution = true;
  if (ik_solver_->searchPositionIK(next_pose, current_joint_positions, servo_params_.publish_period / 2.0, solution,
                                   err, opts))
  {
    // find the difference in joint positions that will get us to the desired pose
    for (size_t i = 0; i < num_joints_; ++i)
    {
      delta_theta.coeffRef(i) = solution.at(i) - current_joint_positions[i];
    }
  }
  else
  {
    RCLCPP_WARN(LOGGER, "Could not find IK solution for requested motion, got error code %d", err.val);
  }

  return delta_theta;
}

StatusCode Servo::getStatus()
{
  return servo_status_;
}

CommandType Servo::incomingCommandType()
{
  return incoming_command_type_;
}

void Servo::incomingCommandType(const CommandType& command_type)
{
  incoming_command_type_ = command_type;
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
