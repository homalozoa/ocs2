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


#include "gtest/gtest.h"
#include "ocs2_core/control/LinearController.hpp"

TEST(testLinearController, testSerialization)
{
  ocs2::scalar_array_t time = {0.0, 1.0};
  ocs2::vector_array_t bias = {ocs2::vector_t::Random(2), ocs2::vector_t::Random(2)};
  ocs2::matrix_array_t gain = {ocs2::matrix_t::Random(2, 3), ocs2::matrix_t::Random(2, 3)};
  ocs2::LinearController controller(time, bias, gain);

  std::vector<std::vector<float>> data(2);
  std::vector<std::vector<float> *> dataPtr{&data[0], &data[1]};
  std::vector<std::vector<float> const *> dataPtrConst{&data[0], &data[1]};

  controller.flatten(time, dataPtr);

  auto controllerOut = ocs2::LinearController::unFlatten({3, 3}, {2, 2}, time, dataPtrConst);

  for (int k = 0; k < time.size(); k++) {
    EXPECT_NEAR(controller.timeStamp_[k], controllerOut.timeStamp_[k], 1e-6);
    EXPECT_TRUE(controller.gainArray_[k].isApprox(controllerOut.gainArray_[k], 1e-6));
    EXPECT_TRUE(controller.biasArray_[k].isApprox(controllerOut.biasArray_[k], 1e-6));
  }
}
