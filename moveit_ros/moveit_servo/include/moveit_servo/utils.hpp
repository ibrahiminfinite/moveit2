
#pragma once

#include <tf2_eigen/tf2_eigen.hpp>

namespace moveit_servo
{
bool isValidCommand(Eigen::VectorXd command);
geometry_msgs::msg::Pose poseFromCartesianDelta(const Eigen::VectorXd& delta_x,
                                                const Eigen::Isometry3d& base_to_tip_frame_transform);
}  // namespace moveit_servo
