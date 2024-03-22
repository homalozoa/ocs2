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
#include "ocs2_core/thread_support/BufferedValue.hpp"

TEST(testBufferedValue, basicSetGet)
{
  // initialize
  const std::string initialValue{"init"};
  ocs2::BufferedValue<std::string> bufferedValue(initialValue);
  ASSERT_EQ(bufferedValue.get(), initialValue);

  // set buffer with copy
  const std::string updatedValue{"update"};
  bufferedValue.setBuffer(updatedValue);
  ASSERT_EQ(bufferedValue.get(), initialValue);

  // update
  const bool isUpdated = bufferedValue.updateFromBuffer();
  ASSERT_TRUE(isUpdated);
  ASSERT_EQ(bufferedValue.get(), updatedValue);

  // update twice is false
  const bool isUpdatedTwice = bufferedValue.updateFromBuffer();
  ASSERT_FALSE(isUpdatedTwice);

  // set buffer with r-value
  bufferedValue.setBuffer("update again");
  ASSERT_EQ(bufferedValue.get(), updatedValue);

  // update again
  const bool isUpdatedAgain = bufferedValue.updateFromBuffer();
  ASSERT_TRUE(isUpdatedAgain);
  ASSERT_EQ(bufferedValue.get(), "update again");
}

namespace
{
/**
 * Move only class that counts the amount of times it has been moved.
 */
class MoveCounter
{
public:
  MoveCounter() : count_(0) {}
  MoveCounter(const MoveCounter &) = delete;
  MoveCounter & operator=(const MoveCounter &) = delete;
  MoveCounter(MoveCounter && other) : count_(other.count_ + 1) {}
  MoveCounter & operator=(MoveCounter && other)
  {
    count_ = other.count_ + 1;
    return *this;
  }

  int getCount() const { return count_; }

private:
  int count_;
};
}  // unnamed namespace

TEST(testBufferedValue, moveCount)
{
  // 1 move on creation
  ocs2::BufferedValue<MoveCounter> bufferedValue(MoveCounter{});
  ASSERT_EQ(bufferedValue.get().getCount(), 1);

  /*
   * A new value can be set with two moves:
   *  - 1 into the buffer.
   *  - 1 from the buffer to the active value.
   */
  MoveCounter newCounter{};
  bufferedValue.setBuffer(std::move(newCounter));
  const bool isUpdated = bufferedValue.updateFromBuffer();
  ASSERT_TRUE(isUpdated);
  ASSERT_EQ(bufferedValue.get().getCount(), 2);
}
