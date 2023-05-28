
#pragma once

#include <tf2_eigen/tf2_eigen.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include <moveit_servo_lib_parameters.hpp>

namespace moveit_servo
{
bool isValidCommand(Eigen::VectorXd command);

geometry_msgs::msg::Pose poseFromCartesianDelta(const Eigen::VectorXd& delta_x,
                                                const Eigen::Isometry3d& base_to_tip_frame_transform);

trajectory_msgs::msg::JointTrajectory composeTrajectoryMessage(const servo::Params& servo_params,
                                                               sensor_msgs::msg::JointState& joint_state);

}  // namespace moveit_servo
