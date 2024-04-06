//
// Created by rgrandia on 17.03.20.
//

#include <gtest/gtest.h>

#include "ocs2_switched_model_interface/logic/SingleLegLogic.h"

using namespace switched_model;

TEST(TestFootPlanner, extractPhases_SinglePhase) {
  std::vector<double> event_times = {};
  std::vector<bool> contactFlags;

  // Single stance phase
  contactFlags = {true};
  auto contactPhases = extractContactTimings(event_times, contactFlags);
  ASSERT_TRUE(std::isnan(contactPhases.front().start));
  ASSERT_TRUE(std::isnan(contactPhases.front().end));

  // Single swing phase
  contactFlags = {false};
  contactPhases = extractContactTimings(event_times, contactFlags);
  ASSERT_TRUE(contactPhases.empty());
}

TEST(TestFootPlanner, extractPhases_MergePhases) {
  std::vector<double> event_times = {0.5};
  std::vector<bool> contactFlags;

  // Single stance phase
  contactFlags = {true, true};
  auto contactPhases = extractContactTimings(event_times, contactFlags);
  ASSERT_TRUE(std::isnan(contactPhases.front().start));
  ASSERT_TRUE(std::isnan(contactPhases.front().end));

  // Single swing phase
  contactFlags = {false, false};
  contactPhases = extractContactTimings(event_times, contactFlags);
  ASSERT_TRUE(contactPhases.empty());
}

TEST(TestFootPlanner, extractPhases_singleSwitch) {
  std::vector<double> event_times;
  std::vector<bool> contactFlags;

  // Switched phase
  event_times = {0.5};
  contactFlags = {false, true};
  auto contactPhases = extractContactTimings(event_times, contactFlags);
  ASSERT_EQ(contactPhases.size(), 1);
  ASSERT_DOUBLE_EQ(contactPhases.front().start, event_times.front());
  ASSERT_TRUE(std::isnan(contactPhases.front().end));

  // change switching direction
  contactFlags = {true, false};
  contactPhases = extractContactTimings(event_times, contactFlags);
  ASSERT_EQ(contactPhases.size(), 1);
  ASSERT_TRUE(std::isnan(contactPhases.front().start));
  ASSERT_DOUBLE_EQ(contactPhases.front().end, event_times.front());
}

TEST(TestFootPlanner, extractPhases_multiSwitch_startContact) {
  std::vector<double> event_times;
  std::vector<bool> contactFlags;

  event_times = {1.0, 2.0, 3.0, 4.0};
  contactFlags = {true, true, false, false, true};
  auto contactPhases = extractContactTimings(event_times, contactFlags);
  ASSERT_EQ(contactPhases.size(), 2);
  // contact phase 1
  ASSERT_TRUE(std::isnan(contactPhases[0].start));
  ASSERT_DOUBLE_EQ(contactPhases[0].end, event_times[1]);
  // contact phase 2
  ASSERT_DOUBLE_EQ(contactPhases[1].start, event_times[3]);
  ASSERT_TRUE(std::isnan(contactPhases[1].end));
}

TEST(TestFootPlanner, extractPhases_multiSwitch_startSwing) {
  std::vector<double> event_times;
  std::vector<bool> contactFlags;

  event_times = {1.0, 2.0, 3.0, 4.0};
  contactFlags = {false, false, true, true, false};
  auto contactPhases = extractContactTimings(event_times, contactFlags);
  ASSERT_EQ(contactPhases.size(), 1);
  // contact phase 1
  ASSERT_DOUBLE_EQ(contactPhases[0].start, event_times[1]);
  ASSERT_DOUBLE_EQ(contactPhases[0].end, event_times[3]);
}
