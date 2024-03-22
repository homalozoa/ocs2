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
#include "ocs2_core/misc/Lookup.hpp"

TEST(testLookup, findIndexInTimeArray)
{
  // Normal case
  std::vector<double> timeArray{-1.0, 2.0, 3.0};
  ASSERT_EQ(ocs2::lookup::findIndexInTimeArray(timeArray, -2.0), 0);
  ASSERT_EQ(ocs2::lookup::findIndexInTimeArray(timeArray, -1.0), 0);
  ASSERT_EQ(ocs2::lookup::findIndexInTimeArray(timeArray, 0.0), 1);
  ASSERT_EQ(ocs2::lookup::findIndexInTimeArray(timeArray, 2.0), 1);
  ASSERT_EQ(ocs2::lookup::findIndexInTimeArray(timeArray, 2.5), 2);
  ASSERT_EQ(ocs2::lookup::findIndexInTimeArray(timeArray, 3.0), 2);
  ASSERT_EQ(ocs2::lookup::findIndexInTimeArray(timeArray, 4.0), 3);

  // With repetitions
  std::vector<double> timeArrayRepeated{-1.0, 2.0, 2.0, 2.0, 3.0};
  ASSERT_EQ(ocs2::lookup::findIndexInTimeArray(timeArrayRepeated, 1.9), 1);
  ASSERT_EQ(ocs2::lookup::findIndexInTimeArray(timeArrayRepeated, 2.0), 1);
  ASSERT_EQ(ocs2::lookup::findIndexInTimeArray(timeArrayRepeated, 2.1), 4);

  // Single Time
  std::vector<double> timeArraySingle{1.0};
  ASSERT_EQ(ocs2::lookup::findIndexInTimeArray(timeArraySingle, 0.0), 0);
  ASSERT_EQ(ocs2::lookup::findIndexInTimeArray(timeArraySingle, 1.0), 0);
  ASSERT_EQ(ocs2::lookup::findIndexInTimeArray(timeArraySingle, 2.0), 1);

  // empty time
  std::vector<double> timeArrayEmpty;
  ASSERT_EQ(ocs2::lookup::findIndexInTimeArray(timeArrayEmpty, -1.0), 0);
  ASSERT_EQ(ocs2::lookup::findIndexInTimeArray(timeArrayEmpty, 0.0), 0);
  ASSERT_EQ(ocs2::lookup::findIndexInTimeArray(timeArrayEmpty, 1.0), 0);
}

TEST(testLookup, findIndexInTimeArray_precision_lowNumbers)
{
  std::vector<double> timeArray{0.0};
  double tQuery = timeArray.front();

  // Test next representable number in both directions
  double tQueryMinus = std::nextafter(tQuery, std::numeric_limits<double>::lowest());
  double tQueryPlus = std::nextafter(tQuery, std::numeric_limits<double>::max());

  // Make sure the tested values have the correct ordering
  ASSERT_TRUE(tQueryMinus <= timeArray.front());
  ASSERT_TRUE(tQuery <= timeArray.front());
  ASSERT_TRUE(tQueryPlus > timeArray.front());

  // Test lookup
  ASSERT_EQ(ocs2::lookup::findIndexInTimeArray(timeArray, tQuery), 0);
  ASSERT_EQ(ocs2::lookup::findIndexInTimeArray(timeArray, tQueryMinus), 0);
  ASSERT_EQ(ocs2::lookup::findIndexInTimeArray(timeArray, tQueryPlus), 1);
}

TEST(testLookup, findIndexInTimeArray_precision_highNumbers)
{
  std::vector<double> timeArray{10000.0};
  double tQuery = timeArray.front();

  // Test next representable number in both directions
  double tQueryMinus = std::nextafter(tQuery, std::numeric_limits<double>::lowest());
  double tQueryPlus = std::nextafter(tQuery, std::numeric_limits<double>::max());

  // Make sure the tested values have the correct ordering
  ASSERT_TRUE(tQueryMinus <= timeArray.front());
  ASSERT_TRUE(tQuery <= timeArray.front());
  ASSERT_TRUE(tQueryPlus > timeArray.front());

  // Test lookup
  ASSERT_EQ(ocs2::lookup::findIndexInTimeArray(timeArray, tQuery), 0);
  ASSERT_EQ(ocs2::lookup::findIndexInTimeArray(timeArray, tQueryMinus), 0);
  ASSERT_EQ(ocs2::lookup::findIndexInTimeArray(timeArray, tQueryPlus), 1);
}

TEST(testLookup, findIntervalInTimeArray)
{
  // Normal case
  std::vector<double> timeArray{-1.0, 2.0, 3.0};
  ASSERT_EQ(ocs2::lookup::findIntervalInTimeArray(timeArray, -2.0), -1);
  ASSERT_EQ(ocs2::lookup::findIntervalInTimeArray(timeArray, -1.0), -1);
  ASSERT_EQ(ocs2::lookup::findIntervalInTimeArray(timeArray, 0.0), 0);
  ASSERT_EQ(ocs2::lookup::findIntervalInTimeArray(timeArray, 2.0), 0);
  ASSERT_EQ(ocs2::lookup::findIntervalInTimeArray(timeArray, 2.5), 1);
  ASSERT_EQ(ocs2::lookup::findIntervalInTimeArray(timeArray, 3.0), 1);
  ASSERT_EQ(ocs2::lookup::findIntervalInTimeArray(timeArray, 4.0), 2);

  // With repetitions
  std::vector<double> timeArrayRepeated{-1.0, 2.0, 2.0, 2.0, 3.0};
  ASSERT_EQ(ocs2::lookup::findIntervalInTimeArray(timeArrayRepeated, 1.9), 0);
  ASSERT_EQ(ocs2::lookup::findIntervalInTimeArray(timeArrayRepeated, 2.0), 0);
  ASSERT_EQ(ocs2::lookup::findIntervalInTimeArray(timeArrayRepeated, 2.1), 3);

  // Single Time
  std::vector<double> timeArraySingle{1.0};
  ASSERT_EQ(ocs2::lookup::findIntervalInTimeArray(timeArraySingle, 0.0), -1);
  ASSERT_EQ(ocs2::lookup::findIntervalInTimeArray(timeArraySingle, 1.0), -1);
  ASSERT_EQ(ocs2::lookup::findIntervalInTimeArray(timeArraySingle, 2.0), 0);

  // empty time
  std::vector<double> timeArrayEmpty;
  ASSERT_EQ(ocs2::lookup::findIntervalInTimeArray(timeArrayEmpty, -1.0), 0);
  ASSERT_EQ(ocs2::lookup::findIntervalInTimeArray(timeArrayEmpty, 0.0), 0);
  ASSERT_EQ(ocs2::lookup::findIntervalInTimeArray(timeArrayEmpty, 1.0), 0);
}

TEST(testLookup, findActiveIntervalInTimeArray)
{
  // Normal case
  std::vector<double> timeArray{-1.0, 2.0, 3.0};
  ASSERT_EQ(ocs2::lookup::findActiveIntervalInTimeArray(timeArray, -2.0), -1);
  ASSERT_EQ(ocs2::lookup::findActiveIntervalInTimeArray(timeArray, -1.0), 0);
  ASSERT_EQ(ocs2::lookup::findActiveIntervalInTimeArray(timeArray, 0.0), 0);
  ASSERT_EQ(ocs2::lookup::findActiveIntervalInTimeArray(timeArray, 2.0), 0);
  ASSERT_EQ(ocs2::lookup::findActiveIntervalInTimeArray(timeArray, 2.5), 1);
  ASSERT_EQ(ocs2::lookup::findActiveIntervalInTimeArray(timeArray, 3.0), 1);
  ASSERT_EQ(ocs2::lookup::findActiveIntervalInTimeArray(timeArray, 4.0), 2);

  // With repetitions
  std::vector<double> timeArrayRepeated{-1.0, 2.0, 2.0, 2.0, 3.0};
  ASSERT_EQ(ocs2::lookup::findActiveIntervalInTimeArray(timeArrayRepeated, 1.9), 0);
  ASSERT_EQ(ocs2::lookup::findActiveIntervalInTimeArray(timeArrayRepeated, 2.0), 0);
  ASSERT_EQ(ocs2::lookup::findActiveIntervalInTimeArray(timeArrayRepeated, 2.1), 3);

  // Single Time
  std::vector<double> timeArraySingle{1.0};
  ASSERT_EQ(ocs2::lookup::findActiveIntervalInTimeArray(timeArraySingle, 0.0), -1);
  ASSERT_EQ(ocs2::lookup::findActiveIntervalInTimeArray(timeArraySingle, 1.0), 0);
  ASSERT_EQ(ocs2::lookup::findActiveIntervalInTimeArray(timeArraySingle, 2.0), 0);

  // empty time
  std::vector<double> timeArrayEmpty;
  ASSERT_EQ(ocs2::lookup::findActiveIntervalInTimeArray(timeArrayEmpty, -1.0), 0);
  ASSERT_EQ(ocs2::lookup::findActiveIntervalInTimeArray(timeArrayEmpty, 0.0), 0);
  ASSERT_EQ(ocs2::lookup::findActiveIntervalInTimeArray(timeArrayEmpty, 1.0), 0);
}

TEST(testLookup, findBoundedActiveIntervalInTimeArray)
{
  // Normal case
  std::vector<double> timeArray{-1.0, 2.0, 3.0};
  ASSERT_ANY_THROW(
    ocs2::lookup::findBoundedActiveIntervalInTimeArray(timeArray, -2.0));  // throws on -1
  ASSERT_EQ(ocs2::lookup::findBoundedActiveIntervalInTimeArray(timeArray, -1.0), 0);
  ASSERT_EQ(ocs2::lookup::findBoundedActiveIntervalInTimeArray(timeArray, 0.0), 0);
  ASSERT_EQ(ocs2::lookup::findBoundedActiveIntervalInTimeArray(timeArray, 2.0), 0);
  ASSERT_EQ(ocs2::lookup::findBoundedActiveIntervalInTimeArray(timeArray, 2.5), 1);
  ASSERT_EQ(ocs2::lookup::findBoundedActiveIntervalInTimeArray(timeArray, 3.0), 1);
  ASSERT_ANY_THROW(
    ocs2::lookup::findBoundedActiveIntervalInTimeArray(timeArray, 4.0));  // throws on 2

  // With repetitions
  std::vector<double> timeArrayRepeated{-1.0, 2.0, 2.0, 2.0, 3.0};
  ASSERT_EQ(ocs2::lookup::findBoundedActiveIntervalInTimeArray(timeArrayRepeated, 1.9), 0);
  ASSERT_EQ(ocs2::lookup::findBoundedActiveIntervalInTimeArray(timeArrayRepeated, 2.0), 0);
  ASSERT_EQ(ocs2::lookup::findBoundedActiveIntervalInTimeArray(timeArrayRepeated, 2.1), 3);

  // Single Time -> always throws
  std::vector<double> timeArraySingle{1.0};
  ASSERT_ANY_THROW(ocs2::lookup::findBoundedActiveIntervalInTimeArray(timeArraySingle, 0.0));
  ASSERT_ANY_THROW(ocs2::lookup::findBoundedActiveIntervalInTimeArray(timeArraySingle, 1.0));
  ASSERT_ANY_THROW(ocs2::lookup::findBoundedActiveIntervalInTimeArray(timeArraySingle, 2.0));

  // empty time -> always throws
  std::vector<double> timeArrayEmpty;
  ASSERT_ANY_THROW(ocs2::lookup::findBoundedActiveIntervalInTimeArray(timeArrayEmpty, -1.0));
  ASSERT_ANY_THROW(ocs2::lookup::findBoundedActiveIntervalInTimeArray(timeArrayEmpty, 0.0));
  ASSERT_ANY_THROW(ocs2::lookup::findBoundedActiveIntervalInTimeArray(timeArrayEmpty, 1.0));
}
