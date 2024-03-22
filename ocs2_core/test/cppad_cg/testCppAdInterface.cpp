// Copyright 2020 Ruben Grandia. All rights reserved.
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

#include "commonFixture.hpp"
#include "gtest/gtest.h"

class CppAdInterfaceNoParameterFixture : public ocs2::CommonCppAdNoParameterFixture
{
};
class CppAdInterfaceParameterizedFixture : public ocs2::CommonCppAdParameterizedFixture
{
};

TEST_F(CppAdInterfaceNoParameterFixture, testModelGeneration)
{
  ocs2::CppAdInterface adInterface(funImpl, variableDim_, "testModelWithoutParameters");

  adInterface.createModels(ocs2::CppAdInterface::ApproximationOrder::Second, true);
  ocs2::vector_t x = ocs2::vector_t::Random(variableDim_);

  ASSERT_TRUE(adInterface.getFunctionValue(x).isApprox(testFun(x)));
  ASSERT_TRUE(adInterface.getJacobian(x).isApprox(testJacobian(x)));
  ASSERT_TRUE(adInterface.getHessian(0, x).isApprox(testHessian(x)));

  const auto gnApproximation = adInterface.getGaussNewtonApproximation(x);
  ASSERT_DOUBLE_EQ(gnApproximation.f, 0.5 * testFun(x).squaredNorm());
  ASSERT_TRUE(gnApproximation.dfdx.isApprox(testJacobian(x).transpose() * testFun(x)));
  ASSERT_TRUE(gnApproximation.dfdxx.isApprox(testJacobian(x).transpose() * testJacobian(x)));
}

TEST_F(CppAdInterfaceParameterizedFixture, testModelGeneration)
{
  ocs2::CppAdInterface adInterface(funImpl, variableDim_, parameterDim_, "testModelWithParameters");

  adInterface.createModels(ocs2::CppAdInterface::ApproximationOrder::Second, true);
  ocs2::vector_t x = ocs2::vector_t::Random(variableDim_);
  ocs2::vector_t p = ocs2::vector_t::Random(parameterDim_);

  ASSERT_TRUE(adInterface.getFunctionValue(x, p).isApprox(testFun(x, p)));
  ASSERT_TRUE(adInterface.getJacobian(x, p).isApprox(testJacobian(x, p)));
  ASSERT_TRUE(adInterface.getHessian(0, x, p).isApprox(testHessian(0, x, p)));
  ASSERT_TRUE(adInterface.getHessian(1, x, p).isApprox(testHessian(1, x, p)));

  const auto gnApproximation = adInterface.getGaussNewtonApproximation(x, p);
  ASSERT_DOUBLE_EQ(gnApproximation.f, 0.5 * testFun(x, p).squaredNorm());
  ASSERT_TRUE(gnApproximation.dfdx.isApprox(testJacobian(x, p).transpose() * testFun(x, p)));
  ASSERT_TRUE(gnApproximation.dfdxx.isApprox(testJacobian(x, p).transpose() * testJacobian(x, p)));
}

TEST_F(CppAdInterfaceParameterizedFixture, loadIfAvailable)
{
  ocs2::CppAdInterface adInterface(
    funImpl, variableDim_, parameterDim_, "testModelLoadIfAvailable");

  adInterface.loadModelsIfAvailable(ocs2::CppAdInterface::ApproximationOrder::Second, true);
  ocs2::vector_t x = ocs2::vector_t::Random(variableDim_);
  ocs2::vector_t p = ocs2::vector_t::Random(parameterDim_);

  ASSERT_TRUE(adInterface.getFunctionValue(x, p).isApprox(testFun(x, p)));
  ASSERT_TRUE(adInterface.getJacobian(x, p).isApprox(testJacobian(x, p)));
  ASSERT_TRUE(adInterface.getHessian(0, x, p).isApprox(testHessian(0, x, p)));
  ASSERT_TRUE(adInterface.getHessian(1, x, p).isApprox(testHessian(1, x, p)));

  const auto gnApproximation = adInterface.getGaussNewtonApproximation(x, p);
  ASSERT_DOUBLE_EQ(gnApproximation.f, 0.5 * testFun(x, p).squaredNorm());
  ASSERT_TRUE(gnApproximation.dfdx.isApprox(testJacobian(x, p).transpose() * testFun(x, p)));
  ASSERT_TRUE(gnApproximation.dfdxx.isApprox(testJacobian(x, p).transpose() * testJacobian(x, p)));
}
