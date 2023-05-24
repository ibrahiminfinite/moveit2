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

#pragma once

#include <variant>
#include <tf2_eigen/tf2_eigen.hpp>
#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit_servo_lib_parameters.hpp>
#include <moveit_servo/collision_monitor.hpp>

namespace moveit_servo
{

// Datatypes used by servo

typedef Eigen::Isometry3d Pose;
typedef Eigen::VectorXd JointVelocity;
struct Twist
{
  std::string frame_id;
  Eigen::Vector<double, 6> velocities;
};

typedef std::variant<JointVelocity, Twist, Pose> ServoInput;

struct RobotJointState
{
  Eigen::VectorXd positions;
  Eigen::VectorXd velocities;
  Eigen::VectorXd accelerations;
};

enum class CommandType
{
  JOINT_POSITION = 0,
  TWIST,
  POSE
};

class Servo
{
public:
  Servo(const rclcpp::Node::SharedPtr& node);

  RobotJointState getNextJointState(const ServoInput& command);

  bool incomingCommandType(const CommandType& command_type);
  CommandType incomingCommandType();

  Eigen::VectorXd jointDeltaFromCommand(const ServoInput& command);
  Eigen::VectorXd jointDeltaFromCommand(const JointVelocity& command);
  Eigen::VectorXd jointDeltaFromCommand(const Twist& command);
  Eigen::VectorXd jointDeltaFromCommand(const Pose& command);

  void validateParams(const servo::Params& servo_params);
  void createPlanningSceneMonitor();
  void setIKSolver();

private:
  const rclcpp::Node::SharedPtr node_;

  std::mutex command_type_mutex_;
  CommandType incoming_command_type_;

  servo::Params servo_params_;
  std::shared_ptr<const servo::ParamListener> servo_param_listener_;

  moveit::core::RobotStatePtr current_state_;
  const moveit::core::JointModelGroup* joint_model_group_;
  kinematics::KinematicsBaseConstPtr ik_solver_ = nullptr;
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;

  std::unique_ptr<CollisionCheck> collision_checker_;
};

}  // namespace moveit_servo
