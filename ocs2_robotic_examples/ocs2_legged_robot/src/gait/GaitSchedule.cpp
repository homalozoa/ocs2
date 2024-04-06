/******************************************************************************
Copyright (c) 2021, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

 * Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

 * Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#include "ocs2_legged_robot/gait/GaitSchedule.h"

namespace ocs2 {
namespace legged_robot {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
GaitSchedule::GaitSchedule(ModeSchedule initModeSchedule, ModeSequenceTemplate initModeSequenceTemplate, scalar_t phaseTransitionStanceTime)
    : mode_schedule_(std::move(initModeSchedule)),
      mode_sequenceTemplate_(std::move(initModeSequenceTemplate)),
      phaseTransitionStanceTime_(phaseTransitionStanceTime) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void GaitSchedule::insertModeSequenceTemplate(const ModeSequenceTemplate& mode_sequenceTemplate, scalar_t startTime, scalar_t finalTime) {
  mode_sequenceTemplate_ = mode_sequenceTemplate;
  auto& event_times = mode_schedule_.event_times;
  auto& mode_sequence = mode_schedule_.mode_sequence;

  // find the index on which the new gait should be added
  const size_t index = std::lower_bound(event_times.begin(), event_times.end(), startTime) - event_times.begin();

  // delete the old logic from the index
  if (index < event_times.size()) {
    event_times.erase(event_times.begin() + index, event_times.end());
    mode_sequence.erase(mode_sequence.begin() + index + 1, mode_sequence.end());
  }

  // add an intermediate stance phase
  scalar_t phaseTransitionStanceTime = phaseTransitionStanceTime_;
  if (!mode_sequence.empty() && mode_sequence.back() == ModeNumber::STANCE) {
    phaseTransitionStanceTime = 0.0;
  }

  if (phaseTransitionStanceTime > 0.0) {
    event_times.push_back(startTime);
    mode_sequence.push_back(ModeNumber::STANCE);
  }

  // tile the mode sequence template from startTime+phaseTransitionStanceTime to finalTime.
  tileModeSequenceTemplate(startTime + phaseTransitionStanceTime, finalTime);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ModeSchedule GaitSchedule::getModeSchedule(scalar_t lowerBoundTime, scalar_t upperBoundTime) {
  auto& event_times = mode_schedule_.event_times;
  auto& mode_sequence = mode_schedule_.mode_sequence;
  const size_t index = std::lower_bound(event_times.begin(), event_times.end(), lowerBoundTime) - event_times.begin();

  if (index > 0) {
    // delete the old logic from index and set the default start phase to stance
    event_times.erase(event_times.begin(), event_times.begin() + index - 1);  // keep the one before the last to make it stance
    mode_sequence.erase(mode_sequence.begin(), mode_sequence.begin() + index - 1);

    // set the default initial phase
    mode_sequence.front() = ModeNumber::STANCE;
  }

  // Start tiling at time
  const auto tilingStartTime = event_times.empty() ? upperBoundTime : event_times.back();

  // delete the last default stance phase
  event_times.erase(event_times.end() - 1, event_times.end());
  mode_sequence.erase(mode_sequence.end() - 1, mode_sequence.end());

  // tile the template logic
  tileModeSequenceTemplate(tilingStartTime, upperBoundTime);
  return mode_schedule_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void GaitSchedule::tileModeSequenceTemplate(scalar_t startTime, scalar_t finalTime) {
  auto& event_times = mode_schedule_.event_times;
  auto& mode_sequence = mode_schedule_.mode_sequence;
  const auto& templateTimes = mode_sequenceTemplate_.switchingTimes;
  const auto& templateModeSequence = mode_sequenceTemplate_.mode_sequence;
  const size_t numTemplateSubsystems = mode_sequenceTemplate_.mode_sequence.size();

  // If no template subsystem is defined, the last subsystem should continue for ever
  if (numTemplateSubsystems == 0) {
    return;
  }

  if (!event_times.empty() && startTime <= event_times.back()) {
    throw std::runtime_error("The initial time for template-tiling is not greater than the last event time.");
  }

  // add a initial time
  event_times.push_back(startTime);

  // concatenate from index
  while (event_times.back() < finalTime) {
    for (size_t i = 0; i < templateModeSequence.size(); i++) {
      mode_sequence.push_back(templateModeSequence[i]);
      scalar_t deltaTime = templateTimes[i + 1] - templateTimes[i];
      event_times.push_back(event_times.back() + deltaTime);
    }  // end of i loop
  }    // end of while loop

  // default final phase
  mode_sequence.push_back(ModeNumber::STANCE);
}

}  // namespace legged_robot
}  // namespace ocs2
