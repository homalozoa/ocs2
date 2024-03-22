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
#include "ocs2_core/automatic_differentiation/CppAdSparsity.hpp"

class SparsityFixture : public ocs2::CommonCppAdParameterizedFixture
{
};

TEST_F(SparsityFixture, extractSparsity)
{
  auto jacSparsity = ocs2::cppad_sparsity::getJacobianSparsityPattern(*fun_);
  ASSERT_EQ(jacSparsity, jacobianSparsity_);

  auto hessSparsity = ocs2::cppad_sparsity::getHessianSparsityPattern(*fun_);
  ASSERT_EQ(hessSparsity, hessianSparsity_);
}

TEST(CppAdSparsity, jacobianSparsity)
{
  auto sparsity = ocs2::cppad_sparsity::getJacobianVariableSparsity(1, 2);
  ocs2::cppad_sparsity::SparsityPattern trueSparsity{{0, 1}};
  ASSERT_EQ(sparsity, trueSparsity);
}

TEST(CppAdSparsity, sparsityIntersection)
{
  ocs2::cppad_sparsity::SparsityPattern sparsity0{{0, 1, 2}, {0, 1, 2}, {0, 2}};
  ocs2::cppad_sparsity::SparsityPattern sparsity1{{0, 1, 2}, {0, 2}, {0, 1, 2}};
  ocs2::cppad_sparsity::SparsityPattern sparsity_intersection{{0, 1, 2}, {0, 2}, {0, 2}};

  // All intersections should reduce to the minimal case, i.e. sparsity_intersection
  ASSERT_EQ(sparsity_intersection, ocs2::cppad_sparsity::getIntersection(sparsity0, sparsity1));
  ASSERT_EQ(
    sparsity_intersection, ocs2::cppad_sparsity::getIntersection(sparsity_intersection, sparsity1));
  ASSERT_EQ(
    sparsity_intersection, ocs2::cppad_sparsity::getIntersection(sparsity_intersection, sparsity0));
  ASSERT_EQ(
    sparsity_intersection,
    ocs2::cppad_sparsity::getIntersection(sparsity_intersection, sparsity_intersection));
}

TEST(CppAdSparsity, hessianSparsity)
{
  // H = [1 0; 0 0]
  auto sparsity = ocs2::cppad_sparsity::getHessianVariableSparsity(1, 1);
  ocs2::cppad_sparsity::SparsityPattern trueSparsity{{0}, {}};
  ASSERT_EQ(sparsity, trueSparsity);

  // H = [1 1 0; 0 1 0; 0 0 0]
  auto sparsityDiagonal = ocs2::cppad_sparsity::getHessianVariableSparsity(2, 1);
  ocs2::cppad_sparsity::SparsityPattern trueSparsityDiagonal{{0, 1}, {1}, {}};
  ASSERT_EQ(sparsityDiagonal, trueSparsityDiagonal);
}
