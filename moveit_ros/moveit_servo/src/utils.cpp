
#include <moveit_servo/utils.hpp>

namespace moveit_servo
{

bool isValidCommand(Eigen::VectorXd command)
{
  bool isValid = true;
  for (const double& velocity : command)
  {
    if (std::isnan(velocity))
    {
      isValid = false;
    }
  }
  return isValid;
}

}  // namespace moveit_servo
