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
 *      Created   : 17/05/2023
 *      Author    : V Mohammed Ibrahim
 */

#include <moveit_servo/utils.hpp>

namespace moveit_servo
{

bool isValidCommand(Eigen::VectorXd command)
{
  bool isValid = true;
  for (const double& val : command)
  {
    if (std::isnan(val))
    {
      isValid = false;
      break;
    }
  }
  return isValid;
}

geometry_msgs::msg::Pose poseFromCartesianDelta(const Eigen::VectorXd& delta_x,
                                                const Eigen::Isometry3d& base_to_tip_frame_transform)
{
  // get a transformation matrix with the desired position change &
  // get a transformation matrix with desired orientation change
  Eigen::Isometry3d tf_pos_delta(Eigen::Isometry3d::Identity());
  tf_pos_delta.translate(Eigen::Vector3d(delta_x[0], delta_x[1], delta_x[2]));

  Eigen::Isometry3d tf_rot_delta(Eigen::Isometry3d::Identity());
  Eigen::Quaterniond q = Eigen::AngleAxisd(delta_x[3], Eigen::Vector3d::UnitX()) *
                         Eigen::AngleAxisd(delta_x[4], Eigen::Vector3d::UnitY()) *
                         Eigen::AngleAxisd(delta_x[5], Eigen::Vector3d::UnitZ());
  tf_rot_delta.rotate(q);

  // Find the new tip link position without newly applied rotation
  const Eigen::Isometry3d tf_no_new_rot = tf_pos_delta * base_to_tip_frame_transform;

  // we want the rotation to be applied in the requested reference frame,
  // but we want the rotation to be about the EE point in space, not the origin.
  // So, we need to translate to origin, rotate, then translate back
  // Given T = transformation matrix from origin -> EE point in space (translation component of tf_no_new_rot)
  // and T' as the opposite transformation, EE point in space -> origin (translation only)
  // apply final transformation as T * R * T' * tf_no_new_rot
  const Eigen::Matrix<double, 3, 1> tf_translation = tf_no_new_rot.translation();
  Eigen::Isometry3d tf_neg_translation = Eigen::Isometry3d::Identity();  // T'
  tf_neg_translation(0, 3) = -tf_translation(0, 0);
  tf_neg_translation(1, 3) = -tf_translation(1, 0);
  tf_neg_translation(2, 3) = -tf_translation(2, 0);
  Eigen::Isometry3d tf_pos_translation = Eigen::Isometry3d::Identity();  // T
  tf_pos_translation(0, 3) = tf_translation(0, 0);
  tf_pos_translation(1, 3) = tf_translation(1, 0);
  tf_pos_translation(2, 3) = tf_translation(2, 0);

  // T * R * T' * tf_no_new_rot
  return tf2::toMsg(tf_pos_translation * tf_rot_delta * tf_neg_translation * tf_no_new_rot);
}

trajectory_msgs::msg::JointTrajectory composeTrajectoryMessage(const servo::Params& servo_params,
                                                               sensor_msgs::msg::JointState& joint_state)
{
  // When a joint_trajectory_controller receives a new command, a stamp of 0 indicates "begin immediately"
  // See http://wiki.ros.org/joint_trajectory_controller#Trajectory_replacement

  trajectory_msgs::msg::JointTrajectory joint_trajectory;
  joint_trajectory.header.stamp = rclcpp::Time(0);
  joint_trajectory.header.frame_id = servo_params.planning_frame;
  joint_trajectory.joint_names = joint_state.name;

  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.time_from_start = rclcpp::Duration::from_seconds(servo_params.publish_period);
  if (servo_params.publish_joint_positions)
    point.positions = joint_state.position;
  if (servo_params.publish_joint_velocities)
    point.velocities = joint_state.velocity;
  if (servo_params.publish_joint_accelerations)
  {
    // I do not know of a robot that takes acceleration commands.
    // However, some controllers check that this data is non-empty.
    // Send all zeros, for now.
    std::vector<double> acceleration(7);
    point.accelerations = acceleration;
  }
  joint_trajectory.points.push_back(point);

  return joint_trajectory;
}

}  // namespace moveit_servo
