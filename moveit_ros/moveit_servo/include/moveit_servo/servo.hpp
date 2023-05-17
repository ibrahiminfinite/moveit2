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

namespace moveit_servo
{

typedef Eigen::Isometry3d Pose;
typedef Eigen::VectorXd JointVelocity;
typedef Eigen::Vector<double, 6> Twist;
typedef std::variant<JointVelocity, Twist, Pose> ServoInput;

enum class ServoCommandType
{
  JOINT_POSITION = 0,
  TWIST,
  POSE
};

class Servo
{
public:
  Servo();

  Eigen::MatrixXd getNextJointState(ServoInput command);

  bool incomingCommandType(ServoCommandType command_type);
  ServoCommandType incomingCommandType();

  Eigen::VectorXd processCommand(ServoInput command);
  Eigen::VectorXd processCommand(JointVelocity command);
  Eigen::VectorXd processCommand(Twist command);
  Eigen::VectorXd processCommand(Pose command);

private:
  std::mutex command_type_mutex_;
  ServoCommandType incoming_command_type_;
};

}  // namespace moveit_servo
