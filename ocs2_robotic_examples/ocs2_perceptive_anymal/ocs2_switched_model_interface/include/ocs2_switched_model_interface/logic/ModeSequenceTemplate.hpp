#pragma once

#include <iostream>
#include <vector>

#include <ocs2_msgs/mode_schedule.h>

#include <ocs2_core/reference/ModeSchedule.h>

#include "ocs2_switched_model_interface/core/MotionPhaseDefinition.h"
#include "ocs2_switched_model_interface/core/SwitchedModel.h"
#include "ocs2_switched_model_interface/logic/Gait.h"

namespace switched_model {

/**
 * ModeSequenceTemplate describes a periodic sequence of modes. It is defined by
 * - switching times (size N+1), where the first time is 0, and the last time denotes the period of the cycle
 * - mode_sequence (size N), indicating the mode between the switching times.
 */
struct ModeSequenceTemplate {
  /**
   * Constructor for a ModeSequenceTemplate. The number of modes must be greater than zero (N > 0)
   * @param [in] switchingTimesInput : switching times of size N + 1
   * @param [in] mode_sequence_input : mode sequence of size N
   */
  ModeSequenceTemplate(std::vector<scalar_t> switchingTimesInput, std::vector<size_t> mode_sequence_input)
      : switchingTimes(std::move(switchingTimesInput)), mode_sequence(std::move(mode_sequence_input)) {
    assert(!mode_sequence.empty());
    assert(switchingTimes.size() == mode_sequence.size() + 1);
  }

  /**
   * Defined as [t_0=0, t_1, .., t_n, t_(n+1)=T], where T is the overall duration
   * of the template logic. t_1 to t_n are the event moments.
   */
  std::vector<scalar_t> switchingTimes;

  /**
   * Defined as [sys_0, sys_n], are the switching systems IDs. Here sys_i is
   * active in period [t_i, t_(i+1)]
   */
  std::vector<size_t> mode_sequence;
};

/** Swap two modesequence templates */
inline void swap(ModeSequenceTemplate& lh, ModeSequenceTemplate& rh) {
  lh.switchingTimes.swap(rh.switchingTimes);
  lh.mode_sequence.swap(rh.mode_sequence);
}

/** Print the modesequence template */
std::ostream& operator<<(std::ostream& stream, const ModeSequenceTemplate& mode_sequenceTemplate);

/** Convert mode sequence template to ROS message */
ocs2_msgs::mode_schedule createModeSequenceTemplateMsg(const ModeSequenceTemplate& mode_sequenceTemplate);

/** Convert ROS message to mode sequence template */
ModeSequenceTemplate readModeSequenceTemplateMsg(const ocs2_msgs::mode_schedule& mode_scheduleMsg);

/** Converts a mode sequence template to a gait */
Gait toGait(const ModeSequenceTemplate& mode_sequenceTemplate);

/**
 * Load a modesequence template from file.  The template needs to be declared as:
 *
 * topicName
 * {
 *   mode_sequence
 *   {
 *     [0]     mode0
 *     [1]     mode1
 *   }
 *   switchingTimes
 *   {
 *     [0]     0.0
 *     [1]     t1
 *     [2]     T
 *   }
 * }
 */
ModeSequenceTemplate loadModeSequenceTemplate(const std::string& filename, const std::string& topicName, bool verbose = true);

/**
 * Load a mode schedule template from file.  The schedule needs to be declared as:
 *
 * topicName
 * {
 *   mode_sequence
 *   {
 *     [0]     mode0
 *     [1]     mode1
 *     [2]     mode2
 *   }
 *   event_times
 *   {
 *     [0]     t0
 *     [1]     t1
 *   }
 * }
 */
ocs2::ModeSchedule loadModeSchedule(const std::string& filename, const std::string& topicName, bool verbose);

}  // namespace switched_model
