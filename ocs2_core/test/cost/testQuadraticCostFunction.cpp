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

#include "gtest/gtest.h"
#include "ocs2_core/cost/QuadraticStateCost.hpp"
#include "ocs2_core/cost/QuadraticStateInputCost.hpp"

class testQuadraticCost : public testing::Test
{
protected:
  testQuadraticCost()
  {
    Q_ << 2, 1, 1, 2;
    Qf_ << 1, 0, 0, 1;
    R_ << 2;
    P_ << 1, 1;

    xNominal_.setRandom(2);
    uNominal_.setRandom(1);
    targetTrajectories_ = ocs2::TargetTrajectories({0.0}, {xNominal_}, {uNominal_});

    x_.setRandom(2);
    u_.setRandom(1);

    ocs2::vector_t dx = x_ - xNominal_;
    ocs2::vector_t du = u_ - uNominal_;

    expectedCost_ = 0.5 * dx.dot(Q_ * dx) + 0.5 * du.dot(R_ * du) + du.dot(P_ * dx);
    expectedFinalCost_ = 0.5 * dx.dot(Qf_ * dx);
  }

  const ocs2::scalar_t PRECISION = 1e-9;

  ocs2::matrix_t Q_{2, 2};
  ocs2::matrix_t Qf_{2, 2};
  ocs2::matrix_t R_{1, 1};
  ocs2::matrix_t P_{1, 2};
  ocs2::scalar_t t_ = 0.0;
  ocs2::vector_t x_;
  ocs2::vector_t u_;
  ocs2::vector_t xNominal_;
  ocs2::vector_t uNominal_;
  ocs2::TargetTrajectories targetTrajectories_;
  const ocs2::PreComputation preComputation_;

  ocs2::scalar_t expectedCost_;
  ocs2::scalar_t expectedFinalCost_;
};

TEST_F(testQuadraticCost, StateInputCostValue)
{
  ocs2::QuadraticStateInputCost costFunction(Q_, R_, P_);

  auto L = costFunction.getValue(t_, x_, u_, targetTrajectories_, preComputation_);
  EXPECT_NEAR(L, expectedCost_, PRECISION);
}

TEST_F(testQuadraticCost, StateInputCostApproximation)
{
  ocs2::QuadraticStateInputCost costFunction(Q_, R_, P_);

  auto L = costFunction.getQuadraticApproximation(t_, x_, u_, targetTrajectories_, preComputation_);

  ocs2::vector_t dx = x_ - xNominal_;
  ocs2::vector_t du = u_ - uNominal_;
  EXPECT_NEAR(L.f, expectedCost_, PRECISION);
  EXPECT_TRUE(L.dfdx.isApprox(Q_ * dx + P_.transpose() * du, PRECISION));
  EXPECT_TRUE(L.dfdu.isApprox(R_ * du + P_ * dx, PRECISION));
  EXPECT_TRUE(L.dfdxx.isApprox(Q_, PRECISION));
  EXPECT_TRUE(L.dfdux.isApprox(P_, PRECISION));
  EXPECT_TRUE(L.dfduu.isApprox(R_, PRECISION));
}

TEST_F(testQuadraticCost, StateInputCostClone)
{
  ocs2::QuadraticStateInputCost costFunction(Q_, R_, P_);
  auto costFunctionClone = std::unique_ptr<ocs2::StateInputCost>(costFunction.clone());

  auto L = costFunction.getValue(t_, x_, u_, targetTrajectories_, preComputation_);
  auto Lclone = costFunctionClone->getValue(t_, x_, u_, targetTrajectories_, preComputation_);
  EXPECT_NEAR(L, Lclone, PRECISION);
}

TEST_F(testQuadraticCost, StateCostValue)
{
  ocs2::QuadraticStateCost costFunction(Qf_);

  auto L = costFunction.getValue(t_, x_, targetTrajectories_, preComputation_);
  EXPECT_NEAR(L, expectedFinalCost_, PRECISION);
}

TEST_F(testQuadraticCost, StateCostApproximation)
{
  ocs2::QuadraticStateCost costFunction(Qf_);

  auto Phi = costFunction.getQuadraticApproximation(t_, x_, targetTrajectories_, preComputation_);

  ocs2::vector_t dx = x_ - xNominal_;
  EXPECT_NEAR(Phi.f, expectedFinalCost_, PRECISION);
  EXPECT_TRUE(Phi.dfdx.isApprox(Qf_ * dx, PRECISION));
  EXPECT_TRUE(Phi.dfdxx.isApprox(Qf_, PRECISION));
}

TEST_F(testQuadraticCost, StateCostClone)
{
  ocs2::QuadraticStateCost costFunction(Qf_);
  auto costFunctionClone = std::unique_ptr<ocs2::StateCost>(costFunction.clone());

  auto L = costFunction.getValue(t_, x_, targetTrajectories_, preComputation_);
  auto Lclone = costFunctionClone->getValue(t_, x_, targetTrajectories_, preComputation_);
  EXPECT_NEAR(L, Lclone, PRECISION);
}
