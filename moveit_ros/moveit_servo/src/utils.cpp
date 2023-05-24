
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

}  // namespace moveit_servo
