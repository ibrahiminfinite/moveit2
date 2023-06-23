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

/*      Title     : utils.cpp
 *      Project   : moveit_servo
 *      Created   : 05/17/2023
 *      Author    : Andy Zelenak, V Mohammed Ibrahim
 */

#include <moveit_servo/utils.hpp>
#include <moveit_servo/datatypes.hpp>

namespace
{
// The threshold above which `override_velocity_scaling_factor` will be used instead of computing the scaling from joint bounds.
const double SCALING_OVERRIDE_THRESHOLD = 0.01;
}  // namespace

namespace moveit_servo
{

bool isValidCommand(const Eigen::VectorXd& command)
{
  // returns true only if there are no nan values.
  return (command.array() == command.array()).all();
}

bool isValidCommand(const Eigen::Isometry3d& command)
{
  bool is_valid_rotation = true;
  Eigen::Matrix3d identity, rotation;
  identity.setIdentity();
  rotation = command.linear();
  // checks rotation, will fail if there is nan
  is_valid_rotation = identity.isApprox(rotation.inverse() * rotation);
  // Command is not vald if there is Nan
  const bool not_nan = isValidCommand(command.translation());
  return is_valid_rotation && not_nan;
}

bool isValidCommand(const Twist& command)
{
  return !command.frame_id.empty() && isValidCommand(command.velocities);
}

bool isValidCommand(const Pose& command)
{
  return !command.frame_id.empty() && isValidCommand(command.pose);
}

trajectory_msgs::msg::JointTrajectory composeTrajectoryMessage(const servo::Params& servo_params,
                                                               const KinematicState& joint_state)
{
  // When a joint_trajectory_controller receives a new command, a stamp of 0 indicates "begin immediately"
  // See http://wiki.ros.org/joint_trajectory_controller#Trajectory_replacement

  trajectory_msgs::msg::JointTrajectory joint_trajectory;
  joint_trajectory.header.stamp = rclcpp::Time(0);
  joint_trajectory.header.frame_id = servo_params.planning_frame;
  joint_trajectory.joint_names = joint_state.joint_names;

  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.time_from_start = rclcpp::Duration::from_seconds(servo_params.publish_period);

  // Set the fields of trajectory point based on which fields are requested.
  // Some controllers check that acceleration data is non-empty, even if accelerations are not used
  // Send all zeros (joint_state.accelerations is a vector of all zeros).
  point.positions = servo_params.publish_joint_positions ? joint_state.positions : point.positions;
  point.velocities = servo_params.publish_joint_velocities ? joint_state.velocities : point.velocities;
  point.accelerations = servo_params.publish_joint_accelerations ? joint_state.accelerations : point.accelerations;

  joint_trajectory.points.push_back(point);
  return joint_trajectory;
}

std::pair<double, StatusCode>
velocityScalingFactorForSingularity(const moveit::core::JointModelGroup* joint_model_group,
                                    const moveit::core::RobotStatePtr& current_state,
                                    const Eigen::VectorXd& target_delta_x, const servo::Params& servo_params)
{
  // We need to send information back about if we are halting, moving away or towards the singularity.
  StatusCode servo_status = StatusCode::NO_WARNING;

  // Get the thresholds.
  const double lower_singularity_threshold = servo_params.lower_singularity_threshold;
  const double hard_stop_singularity_threshold = servo_params.hard_stop_singularity_threshold;
  const double leaving_singularity_threshold_multiplier = servo_params.leaving_singularity_threshold_multiplier;

  // Get size of total controllable dimensions.
  size_t dims = target_delta_x.size();

  // Get the current Jacobian and compute SVD
  Eigen::JacobiSVD<Eigen::MatrixXd> current_svd = Eigen::JacobiSVD<Eigen::MatrixXd>(
      current_state->getJacobian(joint_model_group), Eigen::ComputeThinU | Eigen::ComputeThinV);
  Eigen::MatrixXd matrix_s = current_svd.singularValues().asDiagonal();

  // Compute pseudo inverse
  Eigen::MatrixXd pseudo_inverse = current_svd.matrixV() * matrix_s.inverse() * current_svd.matrixU().transpose();

  // Get the singular vector corresponding to least singular value.
  // This vector represents the least responsive dimension. By convention this is the last column of the matrix U.
  // The sign of the singular vector from result of SVD is not reliable, so we need to do extra checking to make sure of
  // the sign. See R. Bro, "Resolving the Sign Ambiguity in the Singular Value Decomposition".
  Eigen::VectorXd vector_towards_singularity = current_svd.matrixU().col(dims - 1);

  // Compute the current condition number. The ratio of max and min singular values.
  // By convention these are the first and last element of the diagonal.
  const double current_condition_number = current_svd.singularValues()(0) / current_svd.singularValues()(dims - 1);

  // Take a small step in the direction of vector_towards_singularity
  double scale = 100;
  Eigen::VectorXd delta_x = vector_towards_singularity / scale;

  // Compute the new joint angles if we take the small step delta_x
  Eigen::VectorXd next_joint_angles;
  current_state->copyJointGroupPositions(joint_model_group, next_joint_angles);
  next_joint_angles += pseudo_inverse * delta_x;

  // Compute the Jacobian SVD for the new robot state.
  current_state->setJointGroupPositions(joint_model_group, next_joint_angles);
  Eigen::JacobiSVD<Eigen::MatrixXd> next_svd = Eigen::JacobiSVD<Eigen::MatrixXd>(
      current_state->getJacobian(joint_model_group), Eigen::ComputeThinU | Eigen::ComputeThinV);

  // Compute condition number for the new Jacobian.
  const double next_condition_number = next_svd.singularValues()(0) / next_svd.singularValues()(dims - 1);

  // If the condition number has increased, we are moving towards singularity and the direction of the
  // vector_towards_singularity is correct. If the condition number has decreased, it means the sign of
  // vector_towards_singularity needs to be flipped.
  if (next_condition_number <= current_condition_number)
  {
    vector_towards_singularity *= -1;
  }

  // Double check the direction using dot product.
  const bool moving_towards_singularity = vector_towards_singularity.dot(target_delta_x) > 0;

  // Compute upper condition variable threshold based on if we are moving towards or away from singularity.
  // See https://github.com/ros-planning/moveit2/pull/620#issuecomment-1201418258 for visual explanation.
  double upper_threshold;
  if (moving_towards_singularity)
  {
    upper_threshold = hard_stop_singularity_threshold;
  }
  else
  {
    const double threshold_size = (hard_stop_singularity_threshold - lower_singularity_threshold);
    upper_threshold = lower_singularity_threshold + (threshold_size * leaving_singularity_threshold_multiplier);
  }

  // Compute the scale based on the current condition number.
  double velocity_scale = 1.0;
  const bool is_above_lower_limit = current_condition_number > lower_singularity_threshold;
  const bool is_below_hard_stop_limit = current_condition_number < hard_stop_singularity_threshold;
  if (is_above_lower_limit && is_below_hard_stop_limit)
  {
    velocity_scale -=
        (current_condition_number - lower_singularity_threshold) / (upper_threshold - lower_singularity_threshold);

    servo_status = moving_towards_singularity ? StatusCode::DECELERATE_FOR_APPROACHING_SINGULARITY :
                                                StatusCode::DECELERATE_FOR_LEAVING_SINGULARITY;
  }
  // If condition number has crossed hard stop limit, halt the robot.
  else if (!is_below_hard_stop_limit)
  {
    servo_status = StatusCode::HALT_FOR_SINGULARITY;
    velocity_scale = 0.0;
  }

  return std::make_pair(velocity_scale, servo_status);
}

double velocityScalingFactor(const Eigen::VectorXd& velocities, const moveit::core::JointBoundsVector& joint_bounds,
                             double scaling_override)
{
  // If override value is close to zero, user is not overriding the scaling
  if (scaling_override < SCALING_OVERRIDE_THRESHOLD)
  {
    scaling_override = 1.0;  // Set to no scaling.
    double bounded_vel;
    std::vector<double> velocity_scaling_factors;  // The allowable fraction of computed veclocity

    for (size_t i = 0; i < joint_bounds.size(); i++)
    {
      const auto joint_bound = (*joint_bounds[i])[0];
      if (joint_bound.velocity_bounded_ && velocities(i) != 0.0)
      {
        // Find the ratio of clamped velocity to original velocity
        bounded_vel = std::clamp(velocities(i), joint_bound.min_velocity_, joint_bound.max_velocity_);
        velocity_scaling_factors.push_back(bounded_vel / velocities(i));
      }
    }
    // Find the lowest scaling factor, this helps preserve Cartesian motion.
    scaling_override = velocity_scaling_factors.empty() ?
                           scaling_override :
                           *std::min_element(velocity_scaling_factors.begin(), velocity_scaling_factors.end());
  }

  return scaling_override;
}

std::vector<int> jointsToHalt(const Eigen::VectorXd& positions, const Eigen::VectorXd& velocities,
                              const moveit::core::JointBoundsVector& joint_bounds, double margin)
{
  std::vector<int> joint_idxs_to_halt;
  for (size_t i = 0; i < joint_bounds.size(); i++)
  {
    const auto joint_bound = (*joint_bounds[i])[0];
    if (joint_bound.position_bounded_)
    {
      const bool negative_bound = velocities[i] < 0 && positions[i] < (joint_bound.min_position_ + margin);
      const bool positive_bound = velocities[i] > 0 && positions[i] > (joint_bound.max_position_ - margin);
      if (negative_bound || positive_bound)
      {
        joint_idxs_to_halt.push_back(i);
      }
    }
  }
  return joint_idxs_to_halt;
}

/** \brief Helper function for converting Eigen::Isometry3d to geometry_msgs/TransformStamped **/
geometry_msgs::msg::TransformStamped convertIsometryToTransform(const Eigen::Isometry3d& eigen_tf,
                                                                const std::string& parent_frame,
                                                                const std::string& child_frame)
{
  geometry_msgs::msg::TransformStamped output = tf2::eigenToTransform(eigen_tf);
  output.header.frame_id = parent_frame;
  output.child_frame_id = child_frame;
  return output;
}

planning_scene_monitor::PlanningSceneMonitorPtr createPlanningSceneMonitor(const rclcpp::Node::SharedPtr& node,
                                                                           const servo::Params& servo_params)
{
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor;
  // Can set robot_description name from parameters
  std::string robot_description_name = "robot_description";
  node->get_parameter_or("robot_description_name", robot_description_name, robot_description_name);
  // Set up planning_scene_monitor
  planning_scene_monitor = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(node, robot_description_name,
                                                                                          "planning_scene_monitor");
  planning_scene_monitor->startStateMonitor(servo_params.joint_topic);
  planning_scene_monitor->startSceneMonitor(servo_params.monitored_planning_scene_topic);
  planning_scene_monitor->setPlanningScenePublishingFrequency(25);
  planning_scene_monitor->getStateMonitor()->enableCopyDynamics(true);
  planning_scene_monitor->startPublishingPlanningScene(planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE,
                                                       std::string(node->get_fully_qualified_name()) +
                                                           "/publish_planning_scene");

  return planning_scene_monitor;
}

}  // namespace moveit_servo
