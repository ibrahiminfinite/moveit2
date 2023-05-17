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

namespace moveit_servo
{
Servo::Servo()
{
}

Eigen::MatrixXd Servo::getNextJointState(ServoInput command)
{
  Eigen::Matrix<double, 3, 2> next_joint_state;
  next_joint_state.setZero();
  // next_joint_state.row(0) =
  processCommand(command);

  return next_joint_state;
}

Eigen::VectorXd Servo::processCommand(ServoInput command)
{
  Eigen::VectorXd next_joint_positions;
  next_joint_positions.setZero();  // This should be set to current position.
  if (incomingCommandType() == ServoCommandType::JOINT_POSITION && command.index() == 0)
  {
    next_joint_positions = processJointPositionCommand(std::get<JointPosition>(command));
  }
  else if (incomingCommandType() == ServoCommandType::TWIST && command.index() == 1)
  {
    next_joint_positions = processTwistCommand(std::get<Twist>(command));
  }
  else if (incomingCommandType() == ServoCommandType::POSE && command.index() == 2)
  {
    next_joint_positions = processPoseCommand(std::get<Pose>(command));
  }
  else
  {
    // PRINT RCLCPP_ERROR
  }
  return next_joint_positions;
}

Eigen::VectorXd Servo::processPoseCommand(Pose command)
{
  std::cout << "Got Pose " << std::endl;
  Eigen::VectorXd rvec(2);
  rvec << 1.0, 1.0;
  return rvec;
}

Eigen::VectorXd Servo::processTwistCommand(Twist command)
{
  std::cout << "Got Twist " << std::endl;
  Eigen::VectorXd rvec(2);
  rvec << 1.0, 1.0;
  return rvec;
}

Eigen::VectorXd Servo::processJointPositionCommand(JointPosition command)
{
  std::cout << "Got Joint Position " << std::endl;
  Eigen::VectorXd rvec(2);
  rvec << 1.0, 1.0;
  return rvec;
}

ServoCommandType Servo::incomingCommandType()
{
  std::lock_guard<std::mutex> lock(command_type_mutex_);
  return incoming_command_type_;
}

bool Servo::incomingCommandType(ServoCommandType command_type)
{
  std::lock_guard<std::mutex> lock(command_type_mutex_);
  incoming_command_type_ = command_type;
  return true;
}
}  // namespace moveit_servo
