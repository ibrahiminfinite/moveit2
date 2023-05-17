
#pragma once

#include <moveit_servo/servo.hpp>

namespace moveit_servo
{
   
    Eigen::VectorXd processJointPositionCommand(ServoInput command);

} // namespace moveit_servo
