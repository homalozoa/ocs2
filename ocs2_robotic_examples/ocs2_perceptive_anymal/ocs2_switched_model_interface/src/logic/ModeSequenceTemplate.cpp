/******************************************************************************
Copyright (c) 2017, Farbod Farshidian. All rights reserved.

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

#include "ocs2_switched_model_interface/logic/ModeSequenceTemplate.h"

#include <ocs2_core/misc/Display.h>
#include <ocs2_core/misc/LoadData.h>

namespace switched_model {

std::ostream& operator<<(std::ostream& stream, const ModeSequenceTemplate& mode_sequenceTemplate) {
  stream << "Template switching times: {" << ocs2::to_delimited_str(mode_sequenceTemplate.switchingTimes) << "}\n";
  stream << "Template mode sequence:   {" << ocs2::to_delimited_str(mode_sequenceTemplate.mode_sequence) << "}\n";
  return stream;
}

ModeSequenceTemplate loadModeSequenceTemplate(const std::string& filename, const std::string& topicName, bool verbose) {
  std::vector<scalar_t> switchingTimes;
  ocs2::loadData::loadStdVector(filename, topicName + ".switchingTimes", switchingTimes, verbose);

  std::vector<std::string> mode_sequenceString;
  ocs2::loadData::loadStdVector(filename, topicName + ".mode_sequence", mode_sequenceString, verbose);

  if (switchingTimes.empty() || mode_sequenceString.empty()) {
    throw std::runtime_error("[loadModeSequenceTemplate] failed to load : " + topicName + " from " + filename);
  }

  // convert the mode name to mode enum
  std::vector<size_t> mode_sequence;
  mode_sequence.reserve(mode_sequenceString.size());
  for (const auto& modeName : mode_sequenceString) {
    mode_sequence.push_back(string2ModeNumber(modeName));
  }

  return {switchingTimes, mode_sequence};
}

ocs2_msgs::mode_schedule createModeSequenceTemplateMsg(const ModeSequenceTemplate& mode_sequenceTemplate) {
  ocs2_msgs::mode_schedule mode_scheduleMsg;
  mode_scheduleMsg.event_times.assign(mode_sequenceTemplate.switchingTimes.begin(), mode_sequenceTemplate.switchingTimes.end());
  mode_scheduleMsg.mode_sequence.assign(mode_sequenceTemplate.mode_sequence.begin(), mode_sequenceTemplate.mode_sequence.end());
  return mode_scheduleMsg;
}

ModeSequenceTemplate readModeSequenceTemplateMsg(const ocs2_msgs::mode_schedule& mode_scheduleMsg) {
  std::vector<scalar_t> switchingTimes(mode_scheduleMsg.event_times.begin(), mode_scheduleMsg.event_times.end());
  std::vector<size_t> mode_sequence(mode_scheduleMsg.mode_sequence.begin(), mode_scheduleMsg.mode_sequence.end());
  return {switchingTimes, mode_sequence};
}

Gait toGait(const ModeSequenceTemplate& mode_sequenceTemplate) {
  const auto startTime = mode_sequenceTemplate.switchingTimes.front();
  const auto endTime = mode_sequenceTemplate.switchingTimes.back();
  Gait gait;
  gait.duration = endTime - startTime;
  // Events: from time -> phase
  gait.eventPhases.reserve(mode_sequenceTemplate.switchingTimes.size());
  std::for_each(mode_sequenceTemplate.switchingTimes.begin() + 1, mode_sequenceTemplate.switchingTimes.end() - 1,
                [&](scalar_t eventTime) { gait.eventPhases.push_back((eventTime - startTime) / gait.duration); });
  // Modes:
  gait.mode_sequence = mode_sequenceTemplate.mode_sequence;
  assert(isValidGait(gait));
  return gait;
}

ocs2::ModeSchedule loadModeSchedule(const std::string& filename, const std::string& topicName, bool verbose) {
  std::vector<scalar_t> event_times;
  ocs2::loadData::loadStdVector(filename, topicName + ".event_times", event_times, verbose);

  std::vector<std::string> mode_sequenceString;
  ocs2::loadData::loadStdVector(filename, topicName + ".mode_sequence", mode_sequenceString, verbose);

  if (mode_sequenceString.empty()) {
    throw std::runtime_error("[loadModeSchedule] failed to load : " + topicName + " from " + filename);
  }

  // convert the mode name to mode enum
  std::vector<size_t> mode_sequence;
  mode_sequence.reserve(mode_sequenceString.size());
  for (const auto& modeName : mode_sequenceString) {
    mode_sequence.push_back(string2ModeNumber(modeName));
  }

  return {event_times, mode_sequence};
}

}  // namespace switched_model
