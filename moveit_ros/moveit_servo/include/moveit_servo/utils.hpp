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

/*      Title     : utils.hpp
 *      Project   : moveit_servo
 *      Created   : 17/05/2023
 *      Author    : Andy Zelenak, V Mohammed Ibrahim
 */

#pragma once

#include <control_toolbox/pid.hpp>

#include <tf2_eigen/tf2_eigen.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include <moveit/robot_model/joint_model_group.h>
#include <moveit/robot_state/robot_state.h>

#include <moveit_servo/datatypes.hpp>
#include <moveit_servo/status_codes.hpp>
#include <moveit_servo_lib_parameters.hpp>

namespace moveit_servo
{

/**
 * \brief Checks if a transform exists to the given frame.
 * @param current_state The current robot state.
 * @param frame_name The name of the frame for which we want to know if transform exists.
 * @return True if transform exists, else false
 */
bool transformExists(const moveit::core::RobotStatePtr& current_state, const std::string& frame_name);

/**
 * \brief Checks if a given command is valid.
 * @param command The command to be checked.
 * @return True if the command is valid, else False.
 */
bool isValidCommand(const Eigen::VectorXd& command);

/**
 * \brief Create a pose message for the provided change in cartesian position.
 * @param delta_x The change in cartesian position.
 * @param base_to_tip_frame_transform The transformation from robot base to ee frame.
 * @return The pose message.
 */
geometry_msgs::msg::Pose poseFromCartesianDelta(const Eigen::VectorXd& delta_x,
                                                const Eigen::Isometry3d& base_to_tip_frame_transform);

/**
 * \brief Create a trajectory message from given joint state.
 * @param servo_params The configuration used by servo, required for setting some field of the trajectory message.
 * @param joint_state The joint state to be added into the trajectory.
 * @return The trajectory message.
 */
trajectory_msgs::msg::JointTrajectory composeTrajectoryMessage(const servo::Params& servo_params,
                                                               const KinematicState& joint_state);

/**
 * \brief Computes scaling factor for velocity when the robot is near a singularity.
 * @param joint_model_group The joint model group of the robot, used for fetching the Jacobian.
 * @param current_state The current state of the robot, used for singularity look ahead.
 * @param target_delta_x The vector containing the required change in cartesian position.
 * @param servo_params The servo parameters, contains the singularity thresholds.
 * @return The velocity scaling factor and the reason for scaling.
 */
std::pair<double, StatusCode>
velocityScalingFactorForSingularity(const moveit::core::JointModelGroup* joint_model_group,
                                    const moveit::core::RobotStatePtr& current_state,
                                    const Eigen::VectorXd& target_delta_x, const servo::Params& servo_params);

/**
 * \brief Apply velocity scaling based on joint limits.
 * @param velocities The commanded velocities.
 * @param joint_bounds The bounding information for the robot joints.
 * @param scaling_override The user defined velocity scaling override.
 * @return The velocity scaling factor.
 */
double velocityScalingFactor(const Eigen::VectorXd& velocities, const moveit::core::JointBoundsVector& joint_bounds,
                             double scaling_override);

/**
 * \brief Finds the joints that are going past allowable joint limits.
 * @param positions The joints positions.
 * @param velocities The current commanded velocities.
 * @param joint_bounds The bounding information for the robot joints.
 * @param margin The buffer before the actual limit.
 * @return The joints that are violating the specified position limits.
 */
std::vector<int> jointsToHalt(const Eigen::VectorXd& positions, const Eigen::VectorXd& velocities,
                              const moveit::core::JointBoundsVector& joint_bounds, double margin);

/**
 * \brief Creates the PID controllers used for pose tracking based on config provided in servo parameters.
 * The parameters can be updated dynamically.
 * @param servo_params The servo parameters, used to get pid information.
 */
std::map<std::string, control_toolbox::Pid> createControllers(const servo::Params& servo_params);

}  // namespace moveit_servo
