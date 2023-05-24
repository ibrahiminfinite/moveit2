
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

}  // namespace moveit_servo
