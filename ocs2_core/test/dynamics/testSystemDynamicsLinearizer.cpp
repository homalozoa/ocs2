// Copyright 2024 Homalozoa. All rights reserved.
// Copyright 2020 Michael Spieler. All rights reserved.
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
#include <random>

#include "gtest/gtest.h"
#include "ocs2_core/Types.hpp"
#include "ocs2_core/dynamics/LinearSystemDynamics.hpp"
#include "ocs2_core/dynamics/SystemDynamicsLinearizer.hpp"

const ocs2::scalar_t TOLERANCE = 1e-5;
const ocs2::scalar_t EPSILON = 1e-10;

static bool derivativeChecker(
  ocs2::SystemDynamicsBase & nonlinearSystem, ocs2::SystemDynamicsBase & reference,
  ocs2::scalar_t tolerance, ocs2::scalar_t t, const ocs2::vector_t & x, const ocs2::vector_t & u);

/**
 * Pendulum system, \fn$ \theta = 0 \fn$ is upright
 */
class PendulumSystem final : public ocs2::SystemDynamicsBase
{
public:
  PendulumSystem() : SystemDynamicsBase() {}
  ~PendulumSystem() override = default;
  PendulumSystem * clone() const override { return new PendulumSystem(*this); }

  ocs2::vector_t computeFlowMap(
    ocs2::scalar_t t, const ocs2::vector_t & x, const ocs2::vector_t & u,
    const ocs2::PreComputation &) override
  {
    ocs2::vector_t dfdt(2);
    dfdt << x(1), sin(x(0)) + 0.1 * u(0);
    return dfdt;
  }

  ocs2::VectorFunctionLinearApproximation linearApproximation(
    ocs2::scalar_t t, const ocs2::vector_t & x, const ocs2::vector_t & u,
    const ocs2::PreComputation &) override
  {
    ocs2::VectorFunctionLinearApproximation linearDynamics;
    linearDynamics.f = computeFlowMap(t, x, u, ocs2::PreComputation());
    linearDynamics.dfdx.resize(2, 2);
    linearDynamics.dfdx << 0, 1,  // clang-format off
                           cos(x(0)), 0;  // clang-format on
    linearDynamics.dfdu.resize(2, 1);
    linearDynamics.dfdu << 0, 0.1;
    return linearDynamics;
  }
};

TEST(testSystemDynamicsLinearizer, testDerivativeChecker)
{
  ocs2::scalar_t time = 0.0;
  ocs2::vector_t state = ocs2::vector_t::Zero(2);
  ocs2::vector_t input = ocs2::vector_t::Zero(1);

  ocs2::matrix_t A(2, 2);
  ocs2::matrix_t B(2, 1);

  A << 0.6, 1.2, -0.8, 3.4;
  B << 1, 1;
  ocs2::LinearSystemDynamics linSys(A, B);

  A(0, 0) = 0;
  B(0, 0) = 0;
  ocs2::LinearSystemDynamics alteredSys(A, B);

  ASSERT_FALSE(derivativeChecker(linSys, alteredSys, TOLERANCE, time, state, input));
}

TEST(testSystemDynamicsLinearizer, testLinearSystem)
{
  ocs2::scalar_t time = 0.0;
  ocs2::vector_t state = ocs2::vector_t::Zero(2);
  ocs2::vector_t input = ocs2::vector_t::Zero(1);

  ocs2::matrix_t A(2, 2);
  A << 0.6, 1.2, -0.8, 3.4;
  ocs2::matrix_t B(2, 1);
  B << 1, 1;
  ocs2::LinearSystemDynamics linSys(A, B);

  ocs2::SystemDynamicsLinearizer linearizedSys(
    std::unique_ptr<ocs2::ControlledSystemBase>(linSys.clone()), /*doubleSidedDerivative=*/true,
    /*isSecondOrderSystem=*/false, EPSILON);

  ASSERT_TRUE(derivativeChecker(linSys, linearizedSys, TOLERANCE, time, state, input));
}

TEST(testSystemDynamicsLinearizer, testPendulum)
{
  std::srand((unsigned int)0);  // Seed repeatably

  const size_t divisions = 1000;
  const ocs2::scalar_t maxDeg = 180.0;
  constexpr ocs2::scalar_t toRads = M_PI / 180.0;
  const ocs2::scalar_t t = 0;

  ocs2::matrix_t testStates(2, divisions);
  testStates.row(0).setLinSpaced(
    0, toRads * maxDeg);        // initial starting points between upright and down
  testStates.row(1).setZero();  // Zero initial starting velocity
  ocs2::vector_t input = ocs2::vector_t::Random(1);

  PendulumSystem nonLinSys;
  ocs2::SystemDynamicsLinearizer linearizedSys(
    std::unique_ptr<ocs2::ControlledSystemBase>(nonLinSys.clone()), /*doubleSidedDerivative=*/true,
    /*isSecondOrderSystem=*/false, EPSILON);

  for (auto i = 0; i < divisions; ++i) {
    ASSERT_TRUE(
      derivativeChecker(nonLinSys, linearizedSys, TOLERANCE, t, testStates.col(i), input));
  }
}

static bool derivativeChecker(
  ocs2::SystemDynamicsBase & sys1, ocs2::SystemDynamicsBase & sys2, ocs2::scalar_t tolerance,
  ocs2::scalar_t t, const ocs2::vector_t & x, const ocs2::vector_t & u)
{
  auto derivatives1 = sys1.linearApproximation(t, x, u, ocs2::PreComputation());
  auto derivatives2 = sys2.linearApproximation(t, x, u, ocs2::PreComputation());
  ocs2::scalar_t A_error = (derivatives1.dfdx - derivatives2.dfdx).lpNorm<Eigen::Infinity>();
  ocs2::scalar_t B_error = (derivatives1.dfdu - derivatives2.dfdu).lpNorm<Eigen::Infinity>();
  return tolerance > std::fmax(A_error, B_error);
}
