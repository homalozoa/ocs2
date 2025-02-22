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


#include <iostream>

#include "eigen3/Eigen/Dense"
#include "gtest/gtest.h"
#include "ocs2_core/misc/LinearInterpolation.hpp"

TEST(testLinearInterpolation, testInterpolation)
{
  using Data_T = Eigen::Matrix<double, 2, 1>;

  // Create data
  std::vector<double> t = {0.0, 1.0, 2.0, 3.0, 3.0, 4.0};
  std::vector<Data_T, Eigen::aligned_allocator<Data_T>> v;
  for (auto & t_k : t) {
    v.emplace_back(t_k * Data_T::Ones());
  }

  std::cout << "TEST DATA" << std::endl;
  for (int k = 0; k < t.size(); k++) {
    std::cout << "time: " << t[k] << " v: " << v[k].transpose() << std::endl;
  }

  // Test function
  auto test_interpolation = [&](double time, int index, double value) {
    const auto indexAlpha = ocs2::LinearInterpolation::timeSegment(time, t);
    const auto v_t = ocs2::LinearInterpolation::interpolate(indexAlpha, v);
    const auto foundIndex = indexAlpha.first;
    const auto alpha = indexAlpha.second;
    std::cout << "time: " << time << " index: " << foundIndex << " v: " << v_t.transpose()
              << " alpha: " << alpha << std::endl;
    EXPECT_EQ(foundIndex, index);
    EXPECT_DOUBLE_EQ(v_t(0), value);
    ASSERT_TRUE(std::isfinite(alpha));
    ASSERT_GE(alpha, 0.0);
    ASSERT_LE(alpha, 1.0);
  };

  // Before start
  test_interpolation(-1.0, 0, 0.0);
  // At start
  test_interpolation(t[0], 0, t[0]);
  // First interval - off center
  test_interpolation(0.25 * t[0] + 0.75 * t[1], 0, 0.25 * t[0] + 0.75 * t[1]);
  // First interval
  test_interpolation(0.5 * t[0] + 0.5 * t[1], 0, 0.5 * t[0] + 0.5 * t[1]);
  // Boundary to second interval
  test_interpolation(t[1], 0, t[1]);
  // Event time
  test_interpolation(t[3], 2, t[3]);
  // Last interval
  test_interpolation(0.5 * t[4] + 0.5 * t[5], 4, 0.5 * t[4] + 0.5 * t[5]);
  // At End
  test_interpolation(t[5], 4, t[5]);
  // Beyond end
  test_interpolation(t[5] + 1.0, 4, t[5]);
}

TEST(testLinearInterpolation, testEventTimeInterpolation)
{
  {  // Only an event
    std::vector<double> time{1.0, 1.0};
    const auto indexAlpha = ocs2::LinearInterpolation::timeSegment(1.0, time);
    ASSERT_EQ(indexAlpha.first, 0);
    ASSERT_TRUE(std::isfinite(indexAlpha.second));
    ASSERT_GE(indexAlpha.second, 0.0);
    ASSERT_LE(indexAlpha.second, 1.0);
  }

  {  // Short time between samples
    constexpr auto eps = ocs2::numeric_traits::weakEpsilon<ocs2::scalar_t>();
    std::vector<double> time{0.0, 1.0, 1.0 + eps, 2.0};
    // Close to 1.0
    const auto indexAlphaLhs = ocs2::LinearInterpolation::timeSegment(1.0 + 0.1 * eps, time);
    ASSERT_EQ(indexAlphaLhs.first, 1);
    ASSERT_EQ(indexAlphaLhs.second, 1.0);
    // Close to 1.0 + eps
    const auto indexAlphaRhs = ocs2::LinearInterpolation::timeSegment(1.0 + 0.9 * eps, time);
    ASSERT_EQ(indexAlphaRhs.first, 1);
    ASSERT_EQ(indexAlphaRhs.second, 0.0);
  }
}

TEST(testLinearInterpolation, testSizeOneTime)
{
  using Data_T = Eigen::Matrix<double, 2, 1>;

  // Create data
  std::vector<double> t = {1.0};
  std::vector<Data_T, Eigen::aligned_allocator<Data_T>> v;
  for (auto & t_k : t) {
    v.emplace_back(t_k * Data_T::Ones());
  }

  std::cout << "TEST DATA" << std::endl;
  for (int k = 0; k < t.size(); k++) {
    std::cout << "time: " << t[k] << " v: " << v[k].transpose() << std::endl;
  }

  // Test function
  auto test_interpolation = [&](double time, int index, double value) {
    const auto indexAlpha = ocs2::LinearInterpolation::timeSegment(time, t);
    const auto v_t = ocs2::LinearInterpolation::interpolate(indexAlpha, v);
    const auto foundIndex = indexAlpha.first;
    const auto alpha = indexAlpha.second;
    std::cout << "time: " << time << " index: " << foundIndex << " v: " << v_t.transpose()
              << " alpha: " << alpha << std::endl;
    EXPECT_EQ(foundIndex, index);
    EXPECT_DOUBLE_EQ(v_t(0), value);
    ASSERT_TRUE(std::isfinite(alpha));
    ASSERT_GE(alpha, 0.0);
    ASSERT_LE(alpha, 1.0);
  };

  // Before time
  test_interpolation(-1.0, 0, 1.0);
  // Beyond time
  test_interpolation(2.0, 0, 1.0);
}

TEST(testLinearInterpolation, testDifferentEigenSizes)
{
  using Data_T = Eigen::MatrixXd;
  std::vector<double> times = {0.0, 1.0};
  std::vector<Data_T, Eigen::aligned_allocator<Data_T>> data = {
    Data_T::Zero(2, 3), Data_T::Ones(4, 4)};

  Data_T result;

  result = ocs2::LinearInterpolation::interpolate(0.4, times, data);
  EXPECT_TRUE(result.isApprox(data[0]));

  result = ocs2::LinearInterpolation::interpolate(0.6, times, data);
  EXPECT_TRUE(result.isApprox(data[1]));

  result = ocs2::LinearInterpolation::interpolate(-0.1, times, data);
  EXPECT_TRUE(result.isApprox(data[0]));

  result = ocs2::LinearInterpolation::interpolate(1.1, times, data);
  EXPECT_TRUE(result.isApprox(data[1]));
}
