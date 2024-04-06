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

#include "gtest/gtest.h"
#include "ocs2_core/Types.hpp"
#include "ocs2_core/dynamics/LinearSystemDynamics.hpp"
#include "ocs2_core/dynamics/SystemDynamicsBase.hpp"

class DummyPreComputation final : public ocs2::PreComputation
{
public:
  DummyPreComputation() = default;
  ocs2::PreComputation * clone() const override { return new DummyPreComputation(*this); }

  void request(
    ocs2::RequestSet request, ocs2::scalar_t t, const ocs2::vector_t & x,
    const ocs2::vector_t & u) override
  {
    lastRequest = request;
  }

  void requestPreJump(ocs2::RequestSet request, ocs2::scalar_t t, const ocs2::vector_t & x) override
  {
    lastRequest = request;
  }

  static void reset() { lastRequest = ocs2::RequestSet(static_cast<ocs2::Request>(0)); }

  static ocs2::RequestSet lastRequest;
};

ocs2::RequestSet DummyPreComputation::lastRequest = ocs2::RequestSet(static_cast<ocs2::Request>(0));

class DummySystem final : public ocs2::SystemDynamicsBase
{
public:
  DummySystem() : ocs2::SystemDynamicsBase(DummyPreComputation()) {}
  ~DummySystem() override = default;
  DummySystem * clone() const override { return new DummySystem(*this); }

  ocs2::vector_t computeFlowMap(
    ocs2::scalar_t t, const ocs2::vector_t & x, const ocs2::vector_t & u,
    const ocs2::PreComputation &) override
  {
    ocs2::vector_t dfdt(2);
    dfdt << x(1), u(0);
    return dfdt;
  }

  ocs2::VectorFunctionLinearApproximation linearApproximation(
    ocs2::scalar_t t, const ocs2::vector_t & x, const ocs2::vector_t & u,
    const ocs2::PreComputation & preComp) override
  {
    ocs2::VectorFunctionLinearApproximation linearDynamics;
    linearDynamics.f = computeFlowMap(t, x, u, preComp);
    linearDynamics.dfdx.resize(2, 2);
    linearDynamics.dfdx << 0, 1,  // clang-format off
                           0, 0;  // clang-format on
    linearDynamics.dfdu.resize(2, 1);
    linearDynamics.dfdu << 0, 1;
    return linearDynamics;
  }

  using ControlledSystemBase::computeFlowMap;
  using SystemDynamicsBase::linearApproximation;
};

TEST(testSystemDynamicsPreComputation, testIntermediateCallback)
{
  DummySystem system;

  const ocs2::scalar_t t = 0.0;
  const ocs2::vector_t x = ocs2::vector_t::Zero(2);
  const ocs2::vector_t u = ocs2::vector_t::Zero(1);

  DummyPreComputation::reset();
  EXPECT_FALSE(DummyPreComputation::lastRequest.contains(ocs2::Request::Dynamics));
  EXPECT_FALSE(DummyPreComputation::lastRequest.contains(ocs2::Request::Approximation));

  const auto flowMap = system.ControlledSystemBase::computeFlowMap(t, x, u);
  EXPECT_TRUE(DummyPreComputation::lastRequest.contains(ocs2::Request::Dynamics));
  EXPECT_FALSE(DummyPreComputation::lastRequest.contains(ocs2::Request::Approximation));

  DummyPreComputation::reset();
  EXPECT_FALSE(DummyPreComputation::lastRequest.contains(ocs2::Request::Dynamics));
  EXPECT_FALSE(DummyPreComputation::lastRequest.contains(ocs2::Request::Approximation));
  const auto flowMapApproximation = system.SystemDynamicsBase::linearApproximation(t, x, u);
  EXPECT_TRUE(DummyPreComputation::lastRequest.contains(ocs2::Request::Dynamics));
  EXPECT_TRUE(DummyPreComputation::lastRequest.contains(ocs2::Request::Approximation));
}

TEST(testSystemDynamicsPreComputation, testPreJumpCallback)
{
  DummySystem system;

  const ocs2::scalar_t t = 0.0;
  const ocs2::vector_t x = ocs2::vector_t::Zero(2);

  DummyPreComputation::reset();
  EXPECT_FALSE(DummyPreComputation::lastRequest.contains(ocs2::Request::Dynamics));
  EXPECT_FALSE(DummyPreComputation::lastRequest.contains(ocs2::Request::Approximation));

  const auto jumpMap = system.computeJumpMap(t, x);
  EXPECT_TRUE(DummyPreComputation::lastRequest.contains(ocs2::Request::Dynamics));
  EXPECT_FALSE(DummyPreComputation::lastRequest.contains(ocs2::Request::Approximation));

  DummyPreComputation::reset();
  EXPECT_FALSE(DummyPreComputation::lastRequest.contains(ocs2::Request::Dynamics));
  EXPECT_FALSE(DummyPreComputation::lastRequest.contains(ocs2::Request::Approximation));

  const auto jumpMapApproximation = system.jumpMapLinearApproximation(t, x);
  EXPECT_TRUE(DummyPreComputation::lastRequest.contains(ocs2::Request::Dynamics));
  EXPECT_TRUE(DummyPreComputation::lastRequest.contains(ocs2::Request::Approximation));
}

TEST(testSystemDynamicsPreComputation, testPreComputationRequestLogic)
{
  constexpr ocs2::RequestSet a = ocs2::Request::Cost + ocs2::Request::Constraint;

  // Test for individual flags
  EXPECT_TRUE(a.contains(ocs2::Request::Cost));
  EXPECT_TRUE(a.contains(ocs2::Request::Constraint));
  EXPECT_FALSE(a.contains(ocs2::Request::Approximation));
  EXPECT_FALSE(a.contains(ocs2::Request::SoftConstraint));
  EXPECT_FALSE(a.contains(ocs2::Request::Dynamics));
  // a.contains(int(42)); // this should not compile
  // a.contains(Request(Request::Cost)); // this should not compile

  constexpr ocs2::RequestSet b = ocs2::Request::Cost + ocs2::Request::Approximation;
  constexpr ocs2::RequestSet c = a + b;  // union

  // Test for subset
  EXPECT_TRUE(c.containsAll(a));
  EXPECT_TRUE(c.containsAll(b));
  EXPECT_TRUE(c.containsAll(c));
  EXPECT_FALSE(a.containsAll(b));
  EXPECT_FALSE(b.containsAll(a));

  // Test for non-empty intersection
  EXPECT_TRUE(c.containsAny(a));
  EXPECT_TRUE(c.containsAny(b));
  EXPECT_TRUE(c.containsAny(c));
  EXPECT_TRUE(a.containsAny(b));
  EXPECT_TRUE(b.containsAny(a));

  EXPECT_FALSE(c.contains(ocs2::Request::Dynamics));
  EXPECT_FALSE(c.contains(ocs2::Request::SoftConstraint));

  // Add same flag twice
  EXPECT_TRUE(
    (ocs2::Request::Dynamics + ocs2::Request::Dynamics).contains(ocs2::Request::Dynamics));
}
