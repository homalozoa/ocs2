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

#pragma once

#include <vector>

#include <ocs2_msgs/mode_schedule.h>

#include <ocs2_legged_robot/gait/ModeSequenceTemplate.h>

namespace ocs2 {
namespace legged_robot {

/** Convert mode sequence template to ROS message */
inline ocs2_msgs::mode_schedule createModeSequenceTemplateMsg(const ModeSequenceTemplate& mode_sequenceTemplate) {
  ocs2_msgs::mode_schedule mode_scheduleMsg;
  mode_scheduleMsg.event_times.assign(mode_sequenceTemplate.switchingTimes.begin(), mode_sequenceTemplate.switchingTimes.end());
  mode_scheduleMsg.mode_sequence.assign(mode_sequenceTemplate.mode_sequence.begin(), mode_sequenceTemplate.mode_sequence.end());
  return mode_scheduleMsg;
}

/** Convert ROS message to mode sequence template */
inline ModeSequenceTemplate readModeSequenceTemplateMsg(const ocs2_msgs::mode_schedule& mode_scheduleMsg) {
  std::vector<scalar_t> switchingTimes(mode_scheduleMsg.event_times.begin(), mode_scheduleMsg.event_times.end());
  std::vector<size_t> mode_sequence(mode_scheduleMsg.mode_sequence.begin(), mode_scheduleMsg.mode_sequence.end());
  return {switchingTimes, mode_sequence};
}

}  // namespace legged_robot
}  // namespace ocs2
