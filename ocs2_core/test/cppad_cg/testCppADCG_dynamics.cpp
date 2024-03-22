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
#include <functional>
#include <iostream>

#include "LinearSystemDynamicsAD.hpp"
#include "boost/filesystem.hpp"
#include "gtest/gtest.h"
#include "ocs2_core/dynamics/LinearSystemDynamics.hpp"
#include "ocs2_core/test/testTools.hpp"

class testCppADCG_dynamicsFixture : public ::testing::Test
{
public:
  const size_t stateDim_ = 4;
  const size_t inputDim_ = 2;

  testCppADCG_dynamicsFixture() { create(); }

  void create()
  {
    // Define dynamics parameters
    ocs2::matrix_t A = ocs2::matrix_t::Random(stateDim_, stateDim_);
    ocs2::matrix_t B = ocs2::matrix_t::Random(stateDim_, inputDim_);
    ocs2::matrix_t G = ocs2::matrix_t::Random(stateDim_, stateDim_);

    linearSystem_.reset(new ocs2::LinearSystemDynamics(A, B, G));

    // boost::filesystem::path filePath(__FILE__);
    std::string libraryFolder = "/tmp/testCppADCG_generated";
    adLinearSystem_.reset(new ocs2::LinearSystemDynamicsAD(A, B, G));

    adLinearSystem_->initialize(
      stateDim_, inputDim_, "testCppADCG_dynamics", libraryFolder, true, true);
  }

  std::unique_ptr<ocs2::LinearSystemDynamics> linearSystem_;
  std::unique_ptr<ocs2::LinearSystemDynamicsAD> adLinearSystem_;
};

void checkSystemDynamics(
  const size_t numTests, ocs2::SystemDynamicsBase * const linearSystem1,
  ocs2::SystemDynamicsBase * const linearSystem2, bool & success, size_t stateDim_,
  size_t inputDim_)
{
  ocs2::vector_t x;
  ocs2::vector_t u;
  const ocs2::scalar_t t = 0;
  const ocs2::scalar_t precision = 1e-9;

  success = true;
  for (size_t it = 0; it < numTests && success; it++) {
    x.setRandom(stateDim_);
    u.setRandom(inputDim_);

    ocs2::VectorFunctionLinearApproximation f1 = linearSystem1->linearApproximation(t, x, u);
    ocs2::VectorFunctionLinearApproximation f2 = linearSystem2->linearApproximation(t, x, u);

    if (!isApprox(f1, f2, precision)) {
      std::cout << "systemDynamic1:\n" << f1 << std::endl;
      std::cout << "systemDynamic2:\n" << f2 << std::endl;
      success = false;
    }

    ocs2::VectorFunctionLinearApproximation g1 = linearSystem1->jumpMapLinearApproximation(t, x);
    ocs2::VectorFunctionLinearApproximation g2 = linearSystem2->jumpMapLinearApproximation(t, x);
    if (!isApprox(g1, g2, precision)) {
      std::cout << "jumpMap1:\n" << g1.dfdx << std::endl;
      std::cout << "jumpMap2:\n" << g2.dfdx << std::endl;
      success = false;
    }
  }
}

TEST_F(testCppADCG_dynamicsFixture, system_dynamics_test)
{
  bool success;
  checkSystemDynamics(
    100, linearSystem_.get(), adLinearSystem_.get(), success, stateDim_, inputDim_);

  ASSERT_TRUE(success);
}

TEST_F(testCppADCG_dynamicsFixture, clone_test)
{
  std::unique_ptr<ocs2::SystemDynamicsBase> adLinearSystemPtr(adLinearSystem_->clone());
  bool success;
  checkSystemDynamics(
    100, linearSystem_.get(), adLinearSystemPtr.get(), success, stateDim_, inputDim_);

  ASSERT_TRUE(success);
}

TEST_F(testCppADCG_dynamicsFixture, multithread_test)
{
  std::unique_ptr<ocs2::SystemDynamicsBase> linearSystemPtr(linearSystem_->clone());
  std::unique_ptr<ocs2::SystemDynamicsBase> adLinearSystemPtr(adLinearSystem_->clone());

  bool success = false;
  std::thread thread1(
    checkSystemDynamics, 10000, linearSystem_.get(), adLinearSystem_.get(), std::ref(success),
    stateDim_, inputDim_);

  bool successClone = false;
  std::thread thread2(
    checkSystemDynamics, 10000, linearSystemPtr.get(), adLinearSystemPtr.get(),
    std::ref(successClone), stateDim_, inputDim_);

  if (thread1.joinable()) {
    thread1.join();
  }
  if (thread2.joinable()) {
    thread2.join();
  }

  ASSERT_TRUE(success && successClone);
}
