// Copyright 2024 Homalozoa. All rights reserved.
// Copyright 2020 Farbod Farshidian. All rights reserved.
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

#include "ocs2_core/reference/ModeSchedule.hpp"

#include "ocs2_core/misc/Display.hpp"
#include "ocs2_core/misc/Lookup.hpp"
#include "ocs2_core/misc/Numerics.hpp"

namespace ocs2
{

ModeSchedule::ModeSchedule(
  std::vector<scalar_t> event_times_input, std::vector<size_t> mode_sequence_input)
: event_times(std::move(event_times_input)), mode_sequence(std::move(mode_sequence_input))
{
  assert(!mode_sequence.empty());
  assert(event_times.size() + 1 == mode_sequence.size());
}

size_t ModeSchedule::modeAtTime(scalar_t time) const
{
  const auto ind = lookup::findIndexInTimeArray(event_times, time);
  return mode_sequence[ind];
}

void swap(ModeSchedule & lh, ModeSchedule & rh)
{
  lh.event_times.swap(rh.event_times);
  lh.mode_sequence.swap(rh.mode_sequence);
}

std::ostream & operator<<(std::ostream & stream, const ModeSchedule & mode_schedule)
{
  stream << "event times:   {" << to_delimited_str(mode_schedule.event_times) << "}\n";
  stream << "mode sequence: {" << to_delimited_str(mode_schedule.mode_sequence) << "}\n";
  return stream;
}

size_t getNumberOfPrecedingEvents(
  const scalar_array_t & timeTrajectory, const size_array_t & postEventIndices, scalar_t eventTime)
{
  // the case of empty time trajectory and eventTime smaller or equal to the initial time
  if (timeTrajectory.empty() || eventTime <= timeTrajectory.front()) {
    return 0;
  }

  const auto eventIndexItr =
    std::find_if(postEventIndices.cbegin(), postEventIndices.cend(), [&](size_t postEventInd) {
      return numerics::almost_eq(eventTime, timeTrajectory[postEventInd - 1]);
    });

  // if the given time did not match any event time but it is smaller than the final time
  if (eventIndexItr == postEventIndices.cend() && eventTime < timeTrajectory.back()) {
    throw std::runtime_error(
      "[getNumberOfPrecedingEvents] The requested time is within the time trajectory but it is not "
      "marked as an event!");
  }

  return std::distance(postEventIndices.cbegin(), eventIndexItr);
}

std::pair<scalar_t, scalar_t> findIntersectionToExtendableInterval(
  const scalar_array_t & timeTrajectory, const scalar_array_t & event_times,
  const std::pair<scalar_t, scalar_t> & timePeriod)
{
  // no interpolation: a bit before initial time
  const std::pair<scalar_t, scalar_t> emptyInterpolatableInterval{
    timePeriod.first, timePeriod.first - 1e-4};

  if (timeTrajectory.empty() || timePeriod.first > timePeriod.second) {
    return emptyInterpolatableInterval;

  } else {
    const auto pastEventItr = std::find_if(
      event_times.crbegin(), event_times.crend(),
      [&](const scalar_t & te) { return te <= timeTrajectory.front(); });
    const auto initialTime = (pastEventItr != event_times.crend())
                               ? std::max(*pastEventItr, timePeriod.first)
                               : timePeriod.first;

    const auto nextEventItr =
      std::lower_bound(event_times.cbegin(), event_times.cend(), timeTrajectory.back());
    const auto finalTime = (nextEventItr != event_times.cend())
                             ? std::min(*nextEventItr, timePeriod.second)
                             : timePeriod.second;

    return (initialTime < finalTime) ? std::make_pair(initialTime, finalTime)
                                     : emptyInterpolatableInterval;
  }
}

}  // namespace ocs2
