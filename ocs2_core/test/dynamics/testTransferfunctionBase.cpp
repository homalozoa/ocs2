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

#include "gtest/gtest.h"
#include "ocs2_core/dynamics/TransferFunctionBase.hpp"

TEST(testTransferFunctionBase, noDelay)
{
  Eigen::VectorXd num, den;
  Eigen::MatrixXd A, B, C, D;

  num.resize(2);
  den.resize(2);
  num << 0.01, 2.0;
  den << 0.2, 3.0;
  ocs2::TransferFunctionBase tf1(num, den);
  tf1.getStateSpace(A, B, C, D);

  // Matlab
  // h = tf([0.01, 2.0], [0.2, 3.0])
  // [A, B, C, D] = tf2ss(h.num{:}, h.den{:})
  ASSERT_DOUBLE_EQ(A(0), -15);
  ASSERT_DOUBLE_EQ(B(0), 1);
  ASSERT_DOUBLE_EQ(C(0), 9.25);
  ASSERT_DOUBLE_EQ(D(0), 0.05);

  num.resize(2);
  den.resize(4);
  num << 2.0, 4.0;
  den << 0.2, 3.0, 0.3, 6.0;
  ocs2::tf2ss(num, den, A, B, C, D, 0.0, false);

  // Matlab
  // h = tf([2.0, 4.0], [0.2, 3.0, 0.3, 6.0])
  // [A, B, C, D] = tf2ss(h.num{:}, h.den{:})
  ASSERT_DOUBLE_EQ(A(0, 0), -15);
  ASSERT_DOUBLE_EQ(A(0, 1), -1.5);
  ASSERT_DOUBLE_EQ(A(0, 2), -30.0);
  ASSERT_DOUBLE_EQ(A(1, 0), 1.0);
  ASSERT_DOUBLE_EQ(A(1, 1), 0.0);
  ASSERT_DOUBLE_EQ(A(1, 2), 0.0);
  ASSERT_DOUBLE_EQ(A(2, 0), 0.0);
  ASSERT_DOUBLE_EQ(A(2, 1), 1.0);
  ASSERT_DOUBLE_EQ(A(2, 2), 0.0);
  ASSERT_DOUBLE_EQ(B(0), 1.0);
  ASSERT_DOUBLE_EQ(C(0), 0.0);
  ASSERT_DOUBLE_EQ(C(1), 10.0);
  ASSERT_DOUBLE_EQ(C(2), 20.0);
  ASSERT_DOUBLE_EQ(D(0), 0.0);
}

TEST(testTransferFunctionBase, withDelay)
{
  Eigen::VectorXd num, den;
  num.resize(2);
  den.resize(2);
  num << 0.01, 2.0;
  den << 0.2, 3.0;
  double delay = 0.25;
  ocs2::TransferFunctionBase tf1(num, den, delay, false);
  Eigen::MatrixXd A, B, C, D;
  tf1.getStateSpace(A, B, C, D);

  // Matlab
  // h = tf([0.01, 2.0], [0.2, 3.0])*tf([-0.5*0.25, 1], [0.5*0.25, 1]);
  // [A, B, C, D] = tf2ss(h.num{:}, h.den{:})
  ASSERT_DOUBLE_EQ(A(0, 0), -23.0);
  ASSERT_DOUBLE_EQ(A(0, 1), -120.0);
  ASSERT_DOUBLE_EQ(A(1, 0), 1.0);
  ASSERT_DOUBLE_EQ(A(1, 1), 0.0);
  ASSERT_DOUBLE_EQ(B(0), 1.0);
  ASSERT_DOUBLE_EQ(C(0), -8.45);
  ASSERT_DOUBLE_EQ(C(1), 86.0);
  ASSERT_DOUBLE_EQ(D(0), -0.05);
}
