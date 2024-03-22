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

#include <string>

#include "boost/filesystem.hpp"
#include "gtest/gtest.h"
#include "ocs2_core/misc/LoadStdVectorOfPair.hpp"

namespace
{
const std::string getDataFolder()
{
  static std::string dataFolder =
    boost::filesystem::path(__FILE__).parent_path().generic_string() + "/data/";
  return dataFolder;
}
}  // namespace

TEST(testLoadPair, loadStringPair)
{
  std::vector<std::pair<std::string, std::string>> loadVector;
  ocs2::loadData::loadStdVectorOfPair(
    getDataFolder() + "/pairVectors.info", "stringPairs", loadVector);

  EXPECT_EQ(loadVector.size(), 2);
  EXPECT_EQ(loadVector[0].first, "s1");
  EXPECT_EQ(loadVector[0].second, "s2");
  EXPECT_EQ(loadVector[1].first, "s3");
  EXPECT_EQ(loadVector[1].second, "s4");
}

TEST(testLoadPair, loadSizePair)
{
  std::vector<std::pair<size_t, size_t>> loadVector;
  ocs2::loadData::loadStdVectorOfPair(
    getDataFolder() + "/pairVectors.info", "sizePairs", loadVector);

  EXPECT_EQ(loadVector.size(), 2);
  EXPECT_EQ(loadVector[0].first, 1);
  EXPECT_EQ(loadVector[0].second, 2);
  EXPECT_EQ(loadVector[1].first, 3);
  EXPECT_EQ(loadVector[1].second, 4);
}

TEST(testLoadPair, loadStringScalarPair)
{
  std::vector<std::pair<std::string, ocs2::scalar_t>> loadVector;
  ocs2::loadData::loadStdVectorOfPair(
    getDataFolder() + "/pairVectors.info", "stringScalarPairs", loadVector);

  EXPECT_EQ(loadVector.size(), 2);
  EXPECT_EQ(loadVector[0].first, "s1");
  EXPECT_DOUBLE_EQ(loadVector[0].second, 2.1);
  EXPECT_EQ(loadVector[1].first, "s3");
  EXPECT_DOUBLE_EQ(loadVector[1].second, 4.3);
}

TEST(testLoadPair, loadStringSizePair)
{
  std::vector<std::pair<std::string, size_t>> loadVector;
  ocs2::loadData::loadStdVectorOfPair(
    getDataFolder() + "/pairVectors.info", "stringSizePairs", loadVector);

  EXPECT_EQ(loadVector.size(), 2);
  EXPECT_EQ(loadVector[0].first, "s1");
  EXPECT_EQ(loadVector[0].second, 2);
  EXPECT_EQ(loadVector[1].first, "s3");
  EXPECT_EQ(loadVector[1].second, 4);
}
