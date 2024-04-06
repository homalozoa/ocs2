#include "CartPoleUrdf.hpp"
#include "gtest/gtest.h"
#include "ocs2_pinocchio_interface/PinocchioInterface.hpp"
#include "ocs2_pinocchio_interface/urdf.hpp"

TEST(testPinocchioInterface, buildFromXml)
{
  auto pinocchio = ocs2::getPinocchioInterfaceFromUrdfString(cartPoleUrdf);
  std::cout << pinocchio;
}
