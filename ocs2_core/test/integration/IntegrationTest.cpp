// Copyright 2024 Homalozoa. All rights reserved.
// Copyright 2020 Farbod Farshidian. All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Farbod nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <memory>

#include "gtest/gtest.h"
#include "ocs2_core/control/LinearController.hpp"
#include "ocs2_core/dynamics/LinearSystemDynamics.hpp"
#include "ocs2_core/integration/Integrator.hpp"

std::unique_ptr<ocs2::OdeBase> getSystem(ocs2::LinearController & controller)
{
  ocs2::matrix_t A(2, 2);
  A << -2, -1,  // clang-format off
        1,  0;  // clang-format on
  ocs2::matrix_t B(2, 1);
  B << 1, 0;

  auto sys = std::make_unique<ocs2::LinearSystemDynamics>(std::move(A), std::move(B));
  sys->setController(&controller);

  return std::move(sys);
}

void testSecondOrderSystem(ocs2::IntegratorType integrator_type)
{
  const ocs2::scalar_t t0 = 0.0;
  const ocs2::scalar_t t1 = 10.0;
  const ocs2::scalar_t dt = 0.05;
  const ocs2::vector_t x0 = ocs2::vector_t::Zero(2);

  const ocs2::scalar_array_t cntTimeStamp{0, 10};
  const ocs2::vector_array_t uff(2, ocs2::vector_t::Ones(1));
  const ocs2::matrix_array_t k(2, ocs2::matrix_t::Zero(1, 2));
  ocs2::LinearController controller(cntTimeStamp, uff, k);
  auto sys = getSystem(controller);

  std::unique_ptr<ocs2::IntegratorBase> integrator = newIntegrator(integrator_type);

  // Adaptive time integrator
  ocs2::scalar_array_t timeTrajectory;
  ocs2::vector_array_t stateTrajectory;
  auto observer = ocs2::Observer(&stateTrajectory, &timeTrajectory);
  integrator->integrateAdaptive(*sys, observer, x0, t0, t1);

  EXPECT_NEAR(timeTrajectory.front(), t0, 1e-6);
  EXPECT_NEAR(timeTrajectory.back(), t1, 1e-6);
  EXPECT_TRUE(stateTrajectory.front().isApprox(x0, 1e-6));
  EXPECT_NEAR(stateTrajectory.back()(0), 0.0, 1e-3);
  EXPECT_NEAR(stateTrajectory.back()(1), 1.0, 1e-3);

  // Equidistant time integrator
  stateTrajectory.clear();
  timeTrajectory.clear();
  observer = ocs2::Observer(&stateTrajectory, &timeTrajectory);
  integrator->integrateConst(*sys, observer, x0, t0, t1, dt);

  EXPECT_NEAR(timeTrajectory.front(), t0, 1e-6);
  EXPECT_NEAR(timeTrajectory.back(), t1, 1e-6);
  EXPECT_TRUE(stateTrajectory.front().isApprox(x0, 1e-6));
  EXPECT_NEAR(stateTrajectory.back()(0), 0.0, 1e-3);
  EXPECT_NEAR(stateTrajectory.back()(1), 1.0, 1e-3);

  // Integrator with given time trajectory
  stateTrajectory.clear();
  observer = ocs2::Observer(&stateTrajectory);
  integrator->integrateTimes(*sys, observer, x0, timeTrajectory.begin(), timeTrajectory.end());

  EXPECT_NEAR(stateTrajectory.back()(0), 0.0, 1e-3);
  EXPECT_NEAR(stateTrajectory.back()(1), 1.0, 1e-3);
}

TEST(IntegrationTest, SecondOrderSystem_ODE45)
{
  testSecondOrderSystem(ocs2::IntegratorType::ODE45);
}

TEST(IntegrationTest, SecondOrderSystem_ODE45_OCS2)
{
  testSecondOrderSystem(ocs2::IntegratorType::ODE45_OCS2);
}

TEST(IntegrationTest, SecondOrderSystem_AdamsBashfort)
{
  testSecondOrderSystem(ocs2::IntegratorType::ADAMS_BASHFORTH);
}

#if (BOOST_VERSION / 100000 == 1 && BOOST_VERSION / 100 % 1000 > 55)

TEST(IntegrationTest, SecondOrderSystem_AdamsBashfortMoulton)
{
  testSecondOrderSystem(IntegratorType::ADAMS_BASHFORTH_MOULTON);
}

#endif

TEST(IntegrationTest, integratorType_from_string)
{
  ocs2::IntegratorType type = ocs2::integrator_type::fromString("ODE45");
  EXPECT_EQ(type, ocs2::IntegratorType::ODE45);
}

TEST(IntegrationTest, integratorType_to_string)
{
  std::string name = ocs2::integrator_type::toString(ocs2::IntegratorType::ODE45);
  EXPECT_EQ(name, "ODE45");
}
